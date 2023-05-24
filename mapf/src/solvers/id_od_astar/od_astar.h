#ifndef MAPF_SRC_SOLVERS_ID_OD_ASTAR_OD_ASTAR_H_
#define MAPF_SRC_SOLVERS_ID_OD_ASTAR_OD_ASTAR_H_

#include <set>
#include <vector>
#include <queue>

#include "solver.h"

namespace mapf {
namespace id_od_astar {

struct State {
  // The state is defined by both pos and actions.
  // However in the implementation we mainly use pos.
  // A state with empty @actions is a standard state, otherwise it is a intermediate state.
  std::vector<Position> pos;
  std::vector<Action> actions;
  int past_cost;
  int heuristic;

  State() = default;
  State(std::vector<Position> pos,
        std::vector<Action> actions,
        int past_cost,
        int heuristic)
      : pos(std::move(pos)), actions(std::move(actions)),
        past_cost(past_cost), heuristic(heuristic) {};

  State &operator=(const State &o) = default;

  bool operator<(const State &o) const {
    if (past_cost + heuristic != o.past_cost + o.heuristic) {
      return past_cost + heuristic > o.past_cost + o.heuristic;
    }
    return heuristic > o.heuristic;
  }

  bool IsStandardState() {
    return actions.empty();
  }

};

// The class OdAstar can be used independently or in conjuncture with the independence detection framework.
class OdAstar : public MapfSolver {
 public:
  OdAstar() = default;
  SearchResult Solve(MapfMap *map_p, const std::vector<Task> &tasks, uint64_t timelimit_ms) override;

 private:
  void AstarHelper(State cur_state);
  int GetHeuristic(const std::vector<Position> &pos);
  std::vector<std::vector<Action>> GenActionSeq(const std::vector<Position> &pos);

  bool IsActionValid(
      const std::vector<Position> &cur_pos,
      const std::vector<Action>& actions,
      int robot_index,
      Action move);

  MapfMap *map_p_;

  std::priority_queue<State> open_;
  std::set<std::vector<Position>> closed_;
  std::map<std::vector<Position>, int> g_value_;
  std::map<std::vector<Position>, std::vector<Position>> state_to_prev_;

  int robot_count_;
  std::vector<Position> target_pos_;
  std::vector<Position> init_pos_;

  uint64_t start_time_point_ms_;
  uint64_t time_limit_ms_;
  uint64_t expanded_nodes_;
};

}
}

#endif //MAPF_SRC_SOLVERS_ID_ASTAR_OD_ASTAR_H_
