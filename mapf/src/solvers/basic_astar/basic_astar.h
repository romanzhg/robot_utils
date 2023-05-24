#ifndef MAPF_SRC_SOLVERS_BASIC_ASTAR_H_
#define MAPF_SRC_SOLVERS_BASIC_ASTAR_H_

#include <list>
#include <queue>
#include <set>
#include <string>
#include <vector>
#include <iostream>

#include "mapf_map.h"
#include "solver.h"

namespace mapf {
namespace basic_astar {

struct State {
  std::vector<Position> pos;
  int past_cost;
  int heuristic;

  State() = default;
  State(std::vector<Position> pos, int past_cost, int heuristic)
      : pos(std::move(pos)), past_cost(past_cost), heuristic(heuristic) {};

  bool operator<(const State &o) const {
    if (past_cost + heuristic != o.past_cost + o.heuristic) {
      return past_cost + heuristic > o.past_cost + o.heuristic;
    }
    return heuristic > o.heuristic;
  }
};

// TODO: the current astar assumes consistent heuristic, and the implementation is basically
// the same as Dijastra's algorithm.
// However there is no proof that the heuristic used here is consistent(intuitively it is).
// So maybe have another version of AStar which separate openset/close set and do repush(see the
// wiki page for such algorithm).
class BasicAstar : public MapfSolver {
 public:
  BasicAstar() = default;

  SearchResult Solve(MapfMap *map_p, const std::vector<Task> &tasks, uint64_t timelimit_ms) override;

 private:
  SearchResult Astar();

  void AstarHelper(const State &prev_state,
                   std::vector<Position> &cur_positions,
                   std::vector<Action> &moves,
                   std::set<Edge> &used_edges,
                   int index);

  int GetCostForCurStep(const std::vector<Position> &after_move_positions,
                        const std::vector<Action> &moves);

  int GetHeuristicShortestPath(const std::vector<Position> &c);
  int GetHeuristicManhattan(const std::vector<Position> &c);
  int GetHeuristic(const std::vector<Position> &c);

  bool IsActionValid(
      const std::vector<Position> &cur_positions,
      const std::set<Edge> &used_edges,
      int index,
      Action move);

  // Data structures used by the AstarHelper algorithm.
  std::set<std::vector<Position>> closed_;
  std::priority_queue<State> open_;
  std::map<std::vector<Position>, int> g_value_;
  std::map<std::vector<Position>, std::vector<Position>> state_to_prev_;

  // Input data.
  int robot_count_;
  MapfMap* map_p_;
  std::vector<Position> target_pos_;
  std::vector<Position> init_pos_;

  uint64_t start_time_point_ms_;
  uint64_t time_limit_ms_;
};

}  // namespace basic_astar
}  // namespace mapf

#endif //MAPF_SRC_SOLVERS_BASIC_ASTAR_H_