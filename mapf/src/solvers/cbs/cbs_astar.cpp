#include "cbs_astar.h"

#include <algorithm>
#include <set>
#include <iostream>
#include <cassert>

using namespace std;

namespace mapf {
namespace cbs {

namespace {
// Debug only.
void PrintConstrains(const set<Constrain> &constrains) {
  cout << "Printing constrains:" << endl;
  for (const auto &c : constrains) {
    cout << c.to_string() << endl;
  }
}
}

std::optional<Path> CbsAstar::FindPath(MapfMap *map_p,
                                       const std::set<Constrain> &constrains,
                                       const Position &src,
                                       const Position &dest) {
//  PrintConstrains(constrains);

  // Clear data from previous runs.
  closed_.clear();
  while (!open_.empty()) {
    open_.pop();
  }
  prev_.clear();
  g_value_.clear();

  map_p_ = map_p;
  src_ = src;
  dest_ = dest;

  // Initialization.
  open_.emplace(SpatioTemporalPoint(0, src), 0, GetHeuristic(src));

  State cur_state;

  while (!open_.empty()) {
    cur_state = open_.top();
    open_.pop();

    if (cur_state.path_point.pos == dest_) {
      break;
    }

    if (closed_.find(cur_state.path_point) != closed_.end()) {
      continue;
    }
    closed_.insert(cur_state.path_point);

    for (const auto &move : kActionTypes) {
      int new_step = cur_state.path_point.time_step + 1;
      Position new_pos = cur_state.path_point.pos + kActionToDelta[(int) move];
      SpatioTemporalPoint new_path_point(new_step, new_pos);

      if (!IsMoveValid(new_path_point, constrains, cur_state.path_point)) {
        continue;
      }
      if (closed_.find(new_path_point) != closed_.end()) {
        continue;
      }

      int tmp_g_value = cur_state.past_cost + 1;
      if (g_value_.find(new_path_point) != g_value_.end()) {
        if (tmp_g_value >= g_value_.at(new_path_point)) {
          continue;
        }
      }
      g_value_[new_path_point] = tmp_g_value;

      prev_[new_path_point] = cur_state.path_point;
      open_.push(State(new_path_point, cur_state.past_cost + 1, GetHeuristic(new_pos)));
    }
  }

  Path rtn;
  if (cur_state.path_point.pos == dest_) {
    // Build the path and return.
    SpatioTemporalPoint tmp = cur_state.path_point;
    while (tmp.time_step != 0) {
      rtn.push_back(tmp.pos);
      assert(prev_[tmp].time_step == (tmp.time_step - 1));
      tmp = prev_[tmp];
    }
    rtn.push_back(src_);
    reverse(rtn.begin(), rtn.end());
    return rtn;
  } else {
    return nullopt;
  }
}

int CbsAstar::GetHeuristic(const Position &src) {
  return map_p_->GetDistance(src, dest_);
}

bool CbsAstar::IsMoveValid(const SpatioTemporalPoint &pp,
                           const std::set<Constrain> &constrains,
                           const SpatioTemporalPoint &prev_stp) {
  if (!map_p_->IsPointValid(pp.pos)) {
    return false;
  }

  // The robot won't move after it reach the dest, need to consider future
  // constrains there.
  if (pp.pos == dest_) {
    for (const Constrain &c : constrains) {
      if (c.type != ConstrainType::VERTEX) {
        continue;
      }
      if (pp.time_step <= c.time_step
          && pp.pos == c.pos) {
        return false;
      }
    }
  }

  // TODO: optimization needed, need to consider data structures/caches
  // to reduce the computation here.
  for (const Constrain &c : constrains) {
    if (c.type == ConstrainType::VERTEX) {
      if (pp.time_step == c.time_step
          && pp.pos == c.pos) {
        return false;
      }
    } else {
      // Check edge constrain.
      if (pp.time_step == c.time_step
          && prev_stp.pos == c.edge.first
          && pp.pos == c.edge.second) {
        return false;
      }
    }
  }
  return true;
}

}
}