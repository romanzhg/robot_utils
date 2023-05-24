#include "icbs_astar.h"

#include <algorithm>
#include <cassert>
#include <iostream>
#include <set>

#include "utils.h"

using namespace std;

namespace mapf {
namespace icbs {

namespace {
// Debug only.
void PrintConstrains(const set<Constrain> &constrains) {
  cout << "Printing constrains:" << endl;
  for (const auto &c : constrains) {
    cout << c.to_string() << endl;
  }
}
}

void ICbsAstar::Cleanup() {
  while (!open_.empty()) {
    open_.pop();
  }
  closed_.clear();
}

int ICbsAstar::GetHeuristic(const std::vector<Position> &cur_pos) {
  int rtn = 0;
  for (int i = 0; i < robot_count_; i++) {
    rtn += map_p_->GetDistance(cur_pos[i], target_pos_[i]);
  }
  return rtn;
}

// TODO: this function is basically the same as the one in basic_astar.cpp(but
// with the constrain check). Maybe extract them.
bool ICbsAstar::IsActionValid(const std::vector<Position> &cur_positions,
                              const std::set<Edge> &used_edges,
                              int index,
                              Action action,
                              int time_step) {
  const auto &delta = kActionToDelta[(int) action];
  Position new_pos = cur_positions[index] + delta;
  if (!map_p_->IsPointValid(new_pos)) {
    return false;
  }

  // The robot won't move after it reach the dest, need to consider future
  // constrains there.
  if (new_pos == target_pos_[index]) {
    for (const Constrain &c : (*constrains_)) {
      if (c.type != ConstrainType::VERTEX) {
        continue;
      }
      if (time_step <= c.time_step
          && new_pos == c.pos) {
        return false;
      }
    }
  }

  for (const Constrain &c : (*constrains_)) {
    if (c.type == ConstrainType::VERTEX) {
      if (time_step == c.time_step
          && new_pos == c.pos) {
        return false;
      }
    } else {
      // Check edge constrain.
      if (time_step == c.time_step
          && cur_positions[index] == c.edge.first
          && new_pos == c.edge.second) {
        return false;
      }
    }
  }


  // Check with moved robots.
  for (int i = 0; i < index; i++) {
    if (cur_positions[i] == new_pos) {
      return false;
    }
  }

  if (action == Action::WAIT) {
    // Do not check edge reuse for move type WAIT.
    return true;
  } else {
    // No edge reuse.
    Edge edge(new_pos, cur_positions[index]);
    return used_edges.find(edge) == used_edges.end();
  }
}

void ICbsAstar::AstarHelper(const std::vector<Position> &parent_pos,
                            std::vector<Position> &tmp_pos,
                            std::set<Edge> &used_edges,
                            int robot_index,
                            int time_step,
                            int past_cost) {
  if (robot_index == robot_count_) {
    SpatioTemporalPoint stp{time_step, tmp_pos};
    if (closed_.find(stp) != closed_.end()) {
      return;
    }

    int tmp_g_value = past_cost;
    if (g_value_.find(stp) != g_value_.end()) {
      if (tmp_g_value >= g_value_.at(stp)) {
        return;
      }
    }
    g_value_[stp] = tmp_g_value;

    pos_to_prev_[stp] = {time_step - 1, parent_pos};
    open_.push(State(stp, past_cost, GetHeuristic(tmp_pos)));

    return;
  }

  // Fill in moves.
  // Robots that are already at their targets will not move.
  if (parent_pos[robot_index] == target_pos_[robot_index]) {
    // tmp_pos is initialized to be parent_pos, so when a robot is at its destination,
    // just use parent_pos to compare.
    if (!IsActionValid(tmp_pos, used_edges, robot_index, Action::WAIT, time_step)) {
      return;
    }
    AstarHelper(parent_pos, tmp_pos, used_edges, robot_index + 1, time_step, past_cost);
  } else {
    for (const auto &action : kActionTypes) {
      if (!IsActionValid(tmp_pos, used_edges, robot_index, action, time_step)) {
        continue;
      }

      // Update to new values.
      tmp_pos[robot_index] = parent_pos[robot_index] + kActionToDelta[(int) action];

      if (action != Action::WAIT) {
        Edge edge(parent_pos[robot_index], tmp_pos[robot_index]);
        used_edges.insert(edge);
      }

      AstarHelper(parent_pos, tmp_pos, used_edges,
                  robot_index + 1, time_step, past_cost + 1);

      // Restore to old values.
      if (action != Action::WAIT) {
        Edge edge(parent_pos[robot_index], tmp_pos[robot_index]);
        assert(used_edges.erase(edge) == 1);
      }
      tmp_pos[robot_index] = parent_pos[robot_index];
    }
  }
}

optional<Solution> ICbsAstar::FindSolution(MapfMap *map_p,
                                           const std::set<Constrain> *constrains,
                                           const std::vector<Task> &tasks) {
  Cleanup();

  start_time_point_ms_ = GetCurrentTimeSinceEpochMS();

  map_p_ = map_p;
  constrains_ = constrains;
  robot_count_ = tasks.size();
  g_value_.clear();
  init_pos_.clear();
  target_pos_.clear();
  for (const auto &task : tasks) {
    init_pos_.push_back(task.src);
    target_pos_.push_back(task.dest);
  }
  pos_to_prev_.clear();

  open_.push(State({0, init_pos_}, 0, GetHeuristic(init_pos_)));

  while (!open_.empty()) {
    if (IsElapsedTimeLongerThan(start_time_point_ms_, time_limit_ms_)) {
      break;
    }

    State cur_state = open_.top();
    open_.pop();

    if (cur_state.stp.pos == target_pos_) {
      Solution rtn;
      SpatioTemporalPoint cur_stp = cur_state.stp;
      while (cur_stp.time_step != 0) {
        rtn.push_back(cur_stp.pos);
        cur_stp = pos_to_prev_[cur_stp];
      }
      rtn.push_back(init_pos_);
      reverse(rtn.begin(), rtn.end());
      return TransferPositionMatrix(rtn);
    }

    if (closed_.find(cur_state.stp) != closed_.end()) {
      continue;
    }

    closed_.insert(cur_state.stp);

    vector<Position> tmp_pos = cur_state.stp.pos;
    set<Edge> used_edges;
    int past_cost = cur_state.past_cost;
    AstarHelper(cur_state.stp.pos, tmp_pos, used_edges, 0,
                cur_state.stp.time_step + 1, past_cost);
  }

  // Timeout at astar level.
//  cout << "cannot find a solution for icbs: " << endl;
//  PrintConstrains(*constrains);
//  for (const auto &task : tasks) {
//    cout << "source: " << task.src.to_string() << " dest: " << task.dest.to_string() << endl;
//  }
  return nullopt;
}

}  // namespace icbs
}  // namespace mapf