#ifndef MAPF_SRC_SOLVERS_ICBS_ICBS_ASTAR_H_
#define MAPF_SRC_SOLVERS_ICBS_ICBS_ASTAR_H_

#include <optional>
#include <set>
#include <queue>
#include <utility>

#include "icbs_common.h"
#include "common_types.h"
#include "mapf_map.h"

namespace mapf {
namespace icbs {

struct SpatioTemporalPoint {
  int time_step;
  std::vector<Position> pos;

  SpatioTemporalPoint() = default;
  SpatioTemporalPoint(int time_step, std::vector<Position> pos)
      : time_step(time_step), pos(std::move(pos)) {};

  bool operator<(const SpatioTemporalPoint &o) const {
    if (time_step == o.time_step) {
      return pos < o.pos;
    } else {
      return time_step < o.time_step;
    }
  }

  std::string to_string() const {
    std::string rtn;
    rtn += "time step: " + std::to_string(time_step) + "\n";
    for (Position p : pos) {
      rtn += p.to_string() + "  ";
    }
    rtn += "\n";
    return rtn;
  }
};

struct State {
  // Time step and position for all robots, time step 0 corresponds to the initial position.
  SpatioTemporalPoint stp;
  int past_cost;
  int heuristic;

  State() = default;
  State(SpatioTemporalPoint stp, int past_cost, int heuristic)
      : stp(std::move(stp)), past_cost(past_cost), heuristic(heuristic) {};

  bool operator<(const State &o) const {
    if (past_cost + heuristic != o.past_cost + o.heuristic) {
      return past_cost + heuristic > o.past_cost + o.heuristic;
    }
    return heuristic > o.heuristic;
  }

  State &operator=(const State &o) = default;
};

class ICbsAstar {
 public:
  ICbsAstar() = default;

  // Returns a seq of positions, which forms path from @src to @dest.
  // Both @src to @dest are included.
  // robot_id in @constrains is not used.
  std::optional<Solution> FindSolution(MapfMap *map_p,
                        const std::set<Constrain> *constrains,
                        const std::vector<Task> &tasks);

 private:
  void Cleanup();
  void AstarHelper(const std::vector<Position> &parent_pos,
                   std::vector<Position> &tmp_pos,
                   std::set<Edge> &used_edges,
                   int robot_index,
                   int time_step,
                   int past_cost);

  bool IsActionValid(const std::vector<Position> &cur_positions,
                     const std::set<Edge> &used_edges,
                     int index,
                     Action action,
                     int time_step);

  std::priority_queue<State> open_;
  std::map<SpatioTemporalPoint, int> g_value_;
  std::set<SpatioTemporalPoint> closed_;

  MapfMap *map_p_;
  const std::set<Constrain> *constrains_;
  int robot_count_;
  std::vector<Position> init_pos_;
  std::vector<Position> target_pos_;
  std::map<SpatioTemporalPoint, SpatioTemporalPoint> pos_to_prev_;

  uint64_t start_time_point_ms_;
  uint64_t time_limit_ms_ = kPerRunTimeLimitMs;

  int GetHeuristic(const std::vector<Position> &cur_pos);
};

}
}

#endif