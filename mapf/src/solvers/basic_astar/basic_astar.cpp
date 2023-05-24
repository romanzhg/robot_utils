#include "basic_astar.h"

#include <algorithm>
#include <cassert>
#include <string>

#include "constants.h"
#include "utils.h"
#include "flags.h"

using namespace std;
using namespace mapf;

namespace mapf {
namespace basic_astar {

int BasicAstar::GetHeuristicShortestPath(const vector<Position> &c) {
  int rtn = 0;
  for (int i = 0; i < robot_count_; i++) {
    rtn += map_p_->GetDistance(c[i], target_pos_[i]);
  }
  return rtn;
}

// This heuristic is much worse than the shortest distance. The code is kept here only for reference.
int BasicAstar::GetHeuristicManhattan(const vector<Position> &c) {
  int rtn = 0;
  for (int i = 0; i < robot_count_; i++) {
    rtn += GetManhattanDist(c[i].x, c[i].y, target_pos_[i].x, target_pos_[i].y);
  }
  return rtn;
}

// TODO: optimize this function.
// Move robot at the current index, check if it conflicts with any previous ones.
bool BasicAstar::IsActionValid(const vector<Position> &cur_positions,
                               const set<Edge> &used_edges,
                               int index,
                               Action move) {
  const auto &delta = kActionToDelta[(int) move];
  Position new_pos = cur_positions[index] + delta;
  if (!map_p_->IsPointValid(new_pos)) {
    return false;
  }

  // Check with moved robots.
  for (int i = 0; i < index; i++) {
    if (cur_positions[i] == new_pos) {
      return false;
    }
  }

  if (move == Action::WAIT) {
    // Do not check edge reuse for move type WAIT.
    return true;
  } else {
    // No edge reuse.
    Edge edge(new_pos, cur_positions[index]);
    return used_edges.find(edge) == used_edges.end();
  }
}

// Minimize sum of past_cost.
// The robot won't move once it reaches the target,
// and we stop counting the past_cost since then.
int BasicAstar::GetCostForCurStep(const vector<Position> &after_move_positions,
                                  const vector<Action> &moves) {
  int rtn = 0;
  for (int i = 0; i < robot_count_; i++) {
    if (after_move_positions[i] == target_pos_[i]
        && moves[i] == Action::WAIT) {
      // Already at target and waiting, does not count.
    } else {
      ++rtn;
    }
  }
  return rtn;
}

void BasicAstar::AstarHelper(const State &prev_state,
                             vector<Position> &cur_positions,
                             vector<Action> &moves,
                             set<Edge> &used_edges,
                             int index) {
  if (index == robot_count_) {
    if (closed_.find(cur_positions) != closed_.end()) {
      return;
    }

    int tmp_g_value = prev_state.past_cost + GetCostForCurStep(cur_positions, moves);
    if (g_value_.find(cur_positions) != g_value_.end()) {
      if (tmp_g_value >= g_value_.at(cur_positions)) {
        return;
      }
    }

    g_value_[cur_positions] = tmp_g_value;
    state_to_prev_[cur_positions] = prev_state.pos;

    open_.emplace(cur_positions,
                  prev_state.past_cost + GetCostForCurStep(cur_positions, moves),
                  GetHeuristic(cur_positions));
    return;
  }

  // Fill in moves.
  // Robots that are already at their targets will not move.
  const Position pos_old_value = cur_positions[index];
  if (pos_old_value == target_pos_[index]) {
    if (!IsActionValid(cur_positions, used_edges, index, Action::WAIT)) {
      return;
    }
    moves[index] = Action::WAIT;
    AstarHelper(prev_state, cur_positions, moves, used_edges, index + 1);
  } else {
    for (const auto &move : kActionTypes) {
      if (!IsActionValid(cur_positions, used_edges, index, move)) {
        continue;
      }
      moves[index] = move;

      // Update to new values.
      cur_positions[index] = pos_old_value + kActionToDelta[(int) moves[index]];

      if (move != Action::WAIT) {
        Edge edge(pos_old_value, cur_positions[index]);
        used_edges.insert(edge);
      }

      AstarHelper(prev_state, cur_positions, moves, used_edges, index + 1);

      // Restore to old values.
      if (move != Action::WAIT) {
        Edge edge(pos_old_value, cur_positions[index]);
        assert(used_edges.erase(edge) == 1);
      }
      cur_positions[index] = pos_old_value;
    }
  }
}

SearchResult BasicAstar::Astar() {
  uint64_t expanded_nodes = 0;

  open_.push(State(init_pos_, 0, GetHeuristic(init_pos_)));
  g_value_[init_pos_] = 0;

  while (!open_.empty()) {
    ++expanded_nodes;
    State cur_state = open_.top();
    open_.pop();

    if (cur_state.pos == target_pos_) {
      vector<vector<Action>> rtn;
      while (target_pos_ != init_pos_) {
        rtn.push_back(PositionSeqToActions(state_to_prev_[target_pos_], target_pos_));
        target_pos_ = state_to_prev_[target_pos_];
      }
      reverse(rtn.begin(), rtn.end());
      return SearchResult(rtn, true,
                          GetCurrentTimeSinceEpochMS() - start_time_point_ms_,
                          expanded_nodes, cur_state.past_cost);

    }

    if (IsElapsedTimeLongerThan(start_time_point_ms_, time_limit_ms_)) {
      break;
    }

    if (closed_.find(cur_state.pos) != closed_.end()) {
      continue;
    }
    closed_.insert(cur_state.pos);

    vector<Position> cur_pos(cur_state.pos);
    vector<Action> moves(robot_count_);
    set<Edge> used_edges;
    AstarHelper(cur_state, cur_pos, moves, used_edges, 0);
  }

  return SearchResult({}, false,
                      GetCurrentTimeSinceEpochMS() - start_time_point_ms_,
                      expanded_nodes, 0);
}

SearchResult BasicAstar::Solve(MapfMap *map_p, const std::vector<Task> &tasks, uint64_t timelimit_ms) {
  while (!open_.empty()) {
    open_.pop();
  }
  closed_.clear();
  g_value_.clear();
  state_to_prev_.clear();

  map_p_ = map_p;
  robot_count_ = tasks.size();

  start_time_point_ms_ = GetCurrentTimeSinceEpochMS();
  time_limit_ms_ = timelimit_ms;

  init_pos_.clear();
  target_pos_.clear();
  for (const Task &t : tasks) {
    init_pos_.push_back(t.src);
    target_pos_.push_back(t.dest);
  }

  return Astar();
}

int BasicAstar::GetHeuristic(const std::vector<Position> &c) {
  return GetHeuristicShortestPath(c);
}

}  // namespace basic_astar
}  // namespace mapf