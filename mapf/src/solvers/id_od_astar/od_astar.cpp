#include "od_astar.h"

#include <algorithm>
#include <iostream>

#include "utils.h"

using namespace std;

namespace mapf {
namespace id_od_astar {

namespace {
vector<Position> RevertActions(const vector<Position> &pos, const vector<Action> &actions) {
  assert(pos.size() == actions.size());
  vector<Position> rtn(pos);
  for (int i = 0; i < actions.size(); i++) {
    rtn[i] = rtn[i] - kActionToDelta[(int) actions[i]];
  }
  return rtn;
}
}  // namespace

int OdAstar::GetHeuristic(const vector<Position> &pos) {
  int rtn = 0;
  for (int i = 0; i < robot_count_; i++) {
    rtn += map_p_->GetDistance(pos[i], target_pos_[i]);
  }
  return rtn;
}

SearchResult OdAstar::Solve(MapfMap *map_p, const std::vector<Task> &tasks, uint64_t timelimit_ms) {
  while (!open_.empty()) {
    open_.pop();
  }
  closed_.clear();
  g_value_.clear();
  state_to_prev_.clear();
  init_pos_.clear();
  target_pos_.clear();

  map_p_ = map_p;
  robot_count_ = tasks.size();
  for (const Task &t : tasks) {
    init_pos_.push_back(t.src);
    target_pos_.push_back(t.dest);
  }

  start_time_point_ms_ = GetCurrentTimeSinceEpochMS();
  time_limit_ms_ = timelimit_ms;
  expanded_nodes_ = 0;

  State init_state(init_pos_,
                   vector<Action>(),
                   0,
                   GetHeuristic(init_pos_));

  open_.push(init_state);
  g_value_[init_pos_] = 0;

  State cur_state;
  while (!open_.empty()) {

    cur_state = open_.top();
    open_.pop();

    if (cur_state.IsStandardState() &&
        cur_state.pos == target_pos_) {
      break;
    }

    if (IsElapsedTimeLongerThan(start_time_point_ms_, time_limit_ms_)) {
      break;
    }

    if (cur_state.IsStandardState()) {
      expanded_nodes_++;
      if (closed_.find(cur_state.pos) != closed_.end()) {
        continue;
      }
      closed_.insert(cur_state.pos);
    }

    AstarHelper(cur_state);
  }

  if (cur_state.pos == target_pos_) {
    return SearchResult(GenActionSeq(target_pos_),
                        true,
                        GetCurrentTimeSinceEpochMS() - start_time_point_ms_,
                        expanded_nodes_, cur_state.past_cost);
  } else {
    return SearchResult({},
                        false,
                        GetCurrentTimeSinceEpochMS() - start_time_point_ms_,
                        expanded_nodes_, 0);
  }
}

vector<vector<Action>> OdAstar::GenActionSeq(const vector<Position> &pos) {
  vector<vector<Action>> rtn;
  while (target_pos_ != init_pos_) {
    vector<Position> from_pos = state_to_prev_[target_pos_];
    rtn.push_back(PositionSeqToActions(from_pos, target_pos_));
    target_pos_ = from_pos;
  }
  reverse(rtn.begin(), rtn.end());
  return rtn;
}

// TODO: this function basically duplicates with the one in basic_astar.cpp.
// Think of a way to reuse.
// Compare the newly moved robot with all previously moved ones.
bool OdAstar::IsActionValid(const std::vector<Position> &cur_pos,
                            const std::vector<Action> &actions,
                            int robot_index,
                            Action move) {
  Position new_pos = cur_pos[robot_index] + kActionToDelta[(int) move];
  if (!map_p_->IsPointValid(new_pos)) {
    return false;
  }

  // Check position overlap with moved robots.
  for (int i = 0; i < robot_index; i++) {
    if (cur_pos[i] == new_pos) {
      return false;
    }
  }

  // If a robot is already at target, it can only wait.
  if (cur_pos[robot_index] == target_pos_[robot_index]) {
    return move == Action::WAIT;
  }

  if (move == Action::WAIT) {
    // Do not check edge reuse for move type WAIT.
    return true;
  } else {
    // TODO: reuse the used_edges structure built here.
    set<Edge> used_edges;

    // Build used_edge for all previous actions.
    for (int i = 0; i < robot_index; i++) {
      Position prev = cur_pos[i] - kActionToDelta[(int) actions[i]];
      Position cur = cur_pos[i];
      if (prev != cur) {
        used_edges.insert({prev, cur});
      }
    }

    // No edge reuse.
    Edge new_edge(new_pos, cur_pos[robot_index]);
    return used_edges.find(new_edge) == used_edges.end();
  }
}

// Implements the expansion logic of Astar.
void OdAstar::AstarHelper(State cur_state) {
  int robot_index = cur_state.actions.size();
  for (const auto &action : kActionTypes) {
    if (!IsActionValid(cur_state.pos, cur_state.actions, robot_index, action)) {
      continue;
    }

    State new_state(cur_state);
    new_state.pos[robot_index] = new_state.pos[robot_index] + kActionToDelta[(int) action];
    new_state.actions.push_back(action);

    // Update cost.
    if (new_state.pos[robot_index] == target_pos_[robot_index]
        && action == Action::WAIT) {
      // Do nothing.
    } else {
      new_state.past_cost += 1;
    }

    new_state.heuristic = GetHeuristic(new_state.pos);
    if (new_state.actions.size() == robot_count_) {
      if (closed_.find(new_state.pos) != closed_.end()) {
        continue;
      }

      vector<Position> prev_pos = RevertActions(new_state.pos, new_state.actions);
      int tmp_g_value = new_state.past_cost;
      if (g_value_.find(new_state.pos) != g_value_.end()) {
        if (tmp_g_value >= g_value_[new_state.pos]) {
          continue;
        }
      }
      g_value_[new_state.pos] = tmp_g_value;
      state_to_prev_[new_state.pos] = prev_pos;
      new_state.actions.clear();
    }
    open_.push(new_state);
  }
}

}  // namespace
}  // namespace