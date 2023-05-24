#include "independence_detection.h"

#include <set>

#include "od_astar.h"
#include "utils.h"

using namespace std;

namespace mapf {
namespace id_od_astar {

namespace {
void JoinActionSeq(ActionSequencePerRobot &joined_actions,
                   const ActionSequencePerTime &partial_actions, const set<int> &robot_ids) {
  vector<int> robot_id_seq;
  for (int id : robot_ids) {
    robot_id_seq.push_back(id);
  }
  for (int i = 0; i < robot_id_seq.size(); i++) {
    for (int j = 0; j < partial_actions.size(); j++) {
      joined_actions[robot_id_seq[i]].push_back(partial_actions[j][i]);
    }
  }
}

vector<Action> GetCurrentStepAction(const ActionSequencePerRobot &actions, int step) {
  int robot_count = actions.size();
  vector<Action> rtn;
  for (int i = 0; i < robot_count; i++) {
    if (step < actions[i].size()) {
      rtn.push_back(actions[i][step]);
    } else {
      rtn.push_back(Action::WAIT);
    }
  }
  return rtn;
}

ActionSequencePerTime RobotBasedActionsToTimeBasedActions(const ActionSequencePerRobot &actions) {
  int robot_count = actions.size();
  int actions_max_len = 0;
  for (int i = 0; i < robot_count; i++) {
    actions_max_len = max(actions_max_len, (int) actions[i].size());
  }

  ActionSequencePerTime rtn(actions_max_len, vector<Action>(robot_count));
  for (int time_step = 0; time_step < actions_max_len; time_step++) {
    for (int robot_id = 0; robot_id < robot_count; robot_id++) {
      if (time_step < actions[robot_id].size()) {
        rtn[time_step][robot_id] = actions[robot_id][time_step];
      } else {
        rtn[time_step][robot_id] = Action::WAIT;
      }
    }
  }
  return rtn;
}
}

IndependenceDetection::IndependenceDetection() {
  solver_ = new OdAstar();
}

SearchResult IndependenceDetection::Solve(
    MapfMap *map_p, const std::vector<Task> &tasks, uint64_t timelimit_ms) {
  start_time_point_ms_ = GetCurrentTimeSinceEpochMS();
  time_limit_ms_ = timelimit_ms;
  robot_count_ = tasks.size();
  tasks_ = tasks;

  uf_.Init(robot_count_);
  map<int, ActionSequencePerTime> group_to_sol;
  for (int i = 0; i < robot_count_; i++) {
    group_to_sol[i] = solver_->Solve(map_p, {tasks[i]}, timelimit_ms).actions;
  }

  while (true) {
    if (IsElapsedTimeLongerThan(start_time_point_ms_, time_limit_ms_)) {
      break;
    }

    // Key is the parent of the group, value is the robots in this group.
    map<int, set<int>> group_to_elem = uf_.GetGroupIdToElem();

    // Join all the solutions.
    ActionSequencePerRobot joined_actions(robot_count_);
    for (const auto &map_it : group_to_elem) {
      JoinActionSeq(joined_actions, group_to_sol[map_it.first], map_it.second);
    }

    auto conflict = GetFirstConflict(tasks, joined_actions);
    if (conflict.has_value()) {
      uf_.Union(conflict.value().first, conflict.value().second);
      int cur_parent = uf_.Find(conflict.value().first);
      set<int> group_to_join = uf_.GetGroup(cur_parent);
      vector<Task> partial_tasks = GetTasks(group_to_join);

      const ActionSequencePerTime &partial_sol = solver_->Solve(map_p, partial_tasks, timelimit_ms).actions;
      group_to_sol[cur_parent] = partial_sol;
    } else {
      // Return success.
      int total_cost = GetTotalCost(joined_actions);
      ActionSequencePerTime rtn = RobotBasedActionsToTimeBasedActions(joined_actions);
      return SearchResult(rtn,
                          true,
                          GetCurrentTimeSinceEpochMS() - start_time_point_ms_,
                          0, total_cost);
    }
  }

  // Return failure.
  return SearchResult({}, false, 0, 0, 0);
}

optional<pair<int, int>> IndependenceDetection::CheckAndApplyActions(vector<Position> &cur_positions,
                                                                     const std::vector<Action> &actions) {
  map<Edge, int> used_edges;
  for (int robot_id = 0; robot_id < robot_count_; robot_id++) {
    Position new_pos = cur_positions[robot_id] + kActionToDelta[(int) actions[robot_id]];
    for (int i = 0; i < robot_id; i++) {
      if (new_pos == cur_positions[i]) {
        return make_pair(i, robot_id);
      }
    }
    if (cur_positions[robot_id] != new_pos) {
      Edge new_edge(cur_positions[robot_id], new_pos);
      if (used_edges.find(new_edge) != used_edges.end()) {
        return make_pair(robot_id, used_edges.at(new_edge));
      }
      used_edges[new_edge] = robot_id;
    }
    cur_positions[robot_id] = new_pos;
  }
  return nullopt;
}

optional<pair<int, int>> IndependenceDetection::GetFirstConflict(
    const std::vector<Task> &tasks, const ActionSequencePerRobot &actions) {
  vector<Position> cur_pos(robot_count_);
  for (int i = 0; i < robot_count_; i++) {
    cur_pos[i] = tasks[i].src;
  }

  int actions_max_len = 0;
  for (const auto &elem : actions) {
    actions_max_len = max(actions_max_len, (int) elem.size());
  }
  for (int step = 0; step < actions_max_len; step++) {
    const vector<Action> &cur_actions = GetCurrentStepAction(actions, step);
    auto rtn = CheckAndApplyActions(cur_pos, cur_actions);
    if (rtn.has_value()) {
      return rtn;
    }
  }
  return nullopt;
}

int IndependenceDetection::GetTotalCost(const ActionSequencePerRobot &actions) {
  int rtn = 0;
  vector<Position> cur_pos;
  for (int robot_id = 0; robot_id < robot_count_; robot_id++) {
    cur_pos.push_back(tasks_[robot_id].src);
  }

  for (int robot_id = 0; robot_id < actions.size(); robot_id++) {
    for (Action action : actions[robot_id]) {
      if (cur_pos[robot_id] == tasks_[robot_id].dest && action == Action::WAIT) {
        // Do nothing.
      } else {
        rtn++;
      }
      cur_pos[robot_id] = cur_pos[robot_id] + kActionToDelta[(int) action];
    }
  }
  return rtn;
}

vector<Task> IndependenceDetection::GetTasks(const std::set<int> &robot_ids) {
  vector<Task> rtn;
  for (int robot_id : robot_ids) {
    rtn.push_back(tasks_[robot_id]);
  }
  return rtn;
}


}  // namespace id_od_astar
}  // namespace mapf