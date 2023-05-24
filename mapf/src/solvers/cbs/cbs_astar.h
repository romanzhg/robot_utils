#ifndef MAPF_SRC_SOLVERS_CBS_ASTAR_H_
#define MAPF_SRC_SOLVERS_CBS_ASTAR_H_

#include <optional>
#include <set>
#include <queue>

#include "cbs_common.h"
#include "common_types.h"
#include "mapf_map.h"

namespace mapf {
namespace cbs {

struct SpatioTemporalPoint {
  int time_step;
  Position pos;

  SpatioTemporalPoint() = default;
  SpatioTemporalPoint(int time_step, Position pos)
      : time_step(time_step), pos(pos) {};

  bool operator<(const SpatioTemporalPoint &o) const {
    if (time_step == o.time_step) {
      return pos < o.pos;
    } else {
      return time_step < o.time_step;
    }
  }

  std::string to_string() {
    return std::to_string(time_step) + " " + pos.to_string();
  }

  bool operator==(const SpatioTemporalPoint &o) const {
    return time_step == o.time_step && pos == o.pos;
  }
};

// Data structures for the internal astar.
struct State {
  // Time step and position.
  SpatioTemporalPoint path_point;
  int past_cost;
  int heuristic;

  State() = default;
  State(SpatioTemporalPoint path_point, int past_cost, int heuristic)
      : path_point(std::move(path_point)), past_cost(past_cost), heuristic(heuristic) {};

  bool operator<(const State &o) const {
    if (past_cost + heuristic != o.past_cost + o.heuristic) {
      return past_cost + heuristic > o.past_cost + o.heuristic;
    }
    return heuristic > o.heuristic;
  }

  std::string to_string() {
    return path_point.to_string();
  }
};

class CbsAstar {
 public:
  CbsAstar() = default;

  // Returns a seq of positions, which forms path from @src to @dest.
  // Both @src to @dest are included.
  std::optional<Path> FindPath(MapfMap *map_p,
                               const std::set<Constrain> &constrains,
                               const Position &src,
                               const Position &dest);

 private:
  int GetHeuristic(const Position &src);
  bool IsMoveValid(const SpatioTemporalPoint &pp,
                   const std::set<Constrain> &constrains,
                   const SpatioTemporalPoint &prev_stp);

  MapfMap* map_p_;
  std::set<SpatioTemporalPoint> closed_;
  std::priority_queue<State> open_;
  std::map<SpatioTemporalPoint, int> g_value_;
  std::map<SpatioTemporalPoint, SpatioTemporalPoint> prev_;

  Position src_;
  Position dest_;
};

}  // namespace cbs
}  // namespace mapf

#endif //MAPF_SRC_SOLVERS_CBS_ASTAR_H_