#include "ks_scheduler.h"

#include <set>
#include <map>

#include "path_finder/sipp_solver.h"
#include "utilities.h"
#include "ks_scheduler_common.h"

namespace ks {

using namespace std;

namespace {
// Debug/validation functions.
void PrintMissionInfo(const vector<RobotInfo> &robot_info) {
  for (const auto &r : robot_info) {
    cout << "robot id: " << r.id << " ";
    if (r.has_mission) {
      if (r.mission.is_internal) {
        cout << "internal mission";
      } else {
        cout << r.mission.wms_mission.to_string();
      }
    } else {
      cout << "has no mission.";
    }
    cout << endl;
  }
}

void ValidatePlan(const vector<RobotInfo> &init_status,
                  const vector<vector<ActionWithTime>> &plan,
                  int new_plan_start_time) {
  map<Location, IntervalSet> loc_to_intervals;
  int robot_count = init_status.size();
  for (int robot_id = 0; robot_id < robot_count; robot_id++) {
    if (plan[robot_id].empty()) {
      loc_to_intervals[init_status[robot_id].pos.loc].AddInterval(new_plan_start_time, robot_id);
      continue;
    }

    Position pos = GetPostionAtTime(plan[robot_id], new_plan_start_time);
    int start_time_ms = new_plan_start_time;

    for (ActionWithTime awt : plan[robot_id]) {
      if (awt.start_time_ms < new_plan_start_time) {
        continue;
      }

      if (awt.action != Action::MOVE) {
        ApplyActionOnPosition(awt.action, &pos);
        assert(pos == awt.end_pos);
        continue;
      }
      loc_to_intervals[pos.loc].AddInterval(
          start_time_ms, awt.start_time_ms + kBufferDurationMs, robot_id);
      ApplyActionOnPosition(awt.action, &pos);
      start_time_ms = awt.end_time_ms;
    }
    loc_to_intervals[pos.loc].AddInterval(start_time_ms, robot_id);
  }
}

}

void KsScheduler::AddMission(WmsMission mission) {
  lock_guard<mutex> lock(mutex_io_up_);
//  cout << "received wms mission: " << mission.id
//       << " from: " << mission.pick_from.loc.to_string()
//       << " to: " << mission.drop_to.loc.to_string()
//       << " shelf id: " << mission.shelf_id << endl;
  mq_from_wms_.push(mission);
}

void KsScheduler::ReportActionDone(CommandReport r) {
  lock_guard<mutex> lock(mutex_io_down_);
  mq_from_simulator_.push(r);
}

void KsScheduler::Init(KsWmsApi *wms_p, KsSimulatorApi *simulator_p) {
  wms_p_ = wms_p;
  simulator_p_ = simulator_p;

  robot_manager_.Init();

  const std::vector<Location> &shelf_storage_points = ks_map_.GetShelfStoragePoints();
  // The way shelves are initialized should consistent among modules.
  for (int i = 0; i < ks_map_.shelf_count_; i++) {
    shelf_manager_.AddMapping(i, shelf_storage_points[i]);
  }

  sipp_p_ = new SippSolver(ks_map_);

  acked_action_count_ = 0;
}

void KsScheduler::Run() {
  while (true) {
    SleepMS(kScheduleIntervalMs);

    mutex_io_up_.lock();
    while (!mq_from_wms_.empty()) {
      WmsMission m = mq_from_wms_.front();
      mq_from_wms_.pop();
      missions_from_wms_.push_back(m);
    }

    mutex_io_up_.unlock();

    mutex_.lock();
    // TODO: develop a mode that can do/use global replanning.
    robot_manager_.AssignMissions(&missions_from_wms_, action_graph_.GetCurrentPlan());

    // TODO: rewrite this section.
    // Make a copy of current state.
    vector<RobotInfo> tmp_robot_info = robot_manager_.GetRobotInfo();
    ShelfManager tmp_shelf_manager(shelf_manager_);
    vector<vector<ActionWithTime>> remaining_plan(robot_count_);
    vector<int> replied_action_index(robot_count_);
    int start_time_ms;
    action_graph_.Cut(&tmp_robot_info, &tmp_shelf_manager, &remaining_plan, start_time_ms);
    replied_action_index = action_graph_.GetRepliedActionIndex();
    mutex_.unlock();

    // Do not lock during SippSolver::FindPath and KsActionGraph::BuildTwoWayAdjList(which can be time consuming).
    PfResponse resp = sipp_p_->FindPath(
        {tmp_robot_info, remaining_plan, start_time_ms},
        &tmp_shelf_manager);

    set<Edge> new_edges = GetNewDependency(resp.plan, remaining_plan, replied_action_index);

    // TODO: validate the dependency graph in a separate thread.
    // For validation only.
//    for (int i = 0; i < robot_count_; i++) {
//      if (!resp.plan[i].empty()) {
//        remaining_plan[i] = resp.plan[i];
//      }
//    }
//    ValidatePlan(tmp_robot_info, remaining_plan, start_time_ms);
//    set<Edge> edge_set = BuildDependencyOld(remaining_plan);

    mutex_.lock();
    action_graph_.SetPlan(resp.plan, new_edges);
    mutex_.unlock();
  }
}

void KsScheduler::AdgRunner() {
  while (true) {
    mutex_io_down_.lock();
    std::queue<CommandReport> tmp_robot_report = mq_from_simulator_;
    while (!mq_from_simulator_.empty()) {
      mq_from_simulator_.pop();
    }
    mutex_io_down_.unlock();

    mutex_.lock();
    while (!tmp_robot_report.empty()) {
      acked_action_count_++;
      CommandReport report = tmp_robot_report.front();
      tmp_robot_report.pop();
      // Robot manager and action graph need to be updated in sync.
      action_graph_.UpdateRobotStatus(report.robot_id, report.action);
      auto rtn = robot_manager_.UpdateRobotStatus(report.robot_id, report.action, &shelf_manager_);
      if (rtn) {
        const MissionReport &r = rtn.value();
        if (r.type == MissionReportType::PICKUP_DONE) {
          cout << "mission: " << r.mission.id << " pickup done. by robot " << report.robot_id << endl;
        } else {
          cout << "mission: " << r.mission.id << " drop down done. by robot " << report.robot_id << endl;
        }
        wms_p_->ReportMissionStatus(r);
      }
    }

    // TODO: remove the use of robot info once the cut is fully implemented.
    const vector<vector<Action>> &commands = action_graph_.GetCommands(
        robot_manager_.GetRobotInfo(), acked_action_count_);
    mutex_.unlock();

    for (int i = 0; i < robot_count_; i++) {
      if (!commands[i].empty()) {
        simulator_p_->AddActions({i, commands[i]});
      }
    }

    SleepMS(kUpdateIntervalMs);
  }
}

}