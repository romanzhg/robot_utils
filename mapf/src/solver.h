#ifndef MAPF_SRC_SOLVER_H_
#define MAPF_SRC_SOLVER_H_

#include <utility>
#include <vector>

#include "common_types.h"
#include "mapf_map.h"

namespace mapf {

// Index for the outer vector is timestamp.
// Index for the inner vector is robot id.
using ActionSequencePerTime = std::vector<std::vector<Action>>;

// Index for the outer vector is robot id.
// Index for the inner vector is timestamp.
using ActionSequencePerRobot = std::vector<std::vector<Action>>;

struct SearchResult {
  // C++ Notes: should look into memory sanitizer, wrong memory access may create unexpected effect at
  // unexpected places.
  // TODO: consider use ActionSequencePerRobot here.
  ActionSequencePerTime actions;

  // True if the solver got the solution in given time.
  // False otherwise.
  bool succeed;
  uint64_t time_spent_ms;
  uint64_t expanded_nodes;
  // Sum over all robots the action count(including wait) before the robot reaches its dest.
  int total_moves;

  SearchResult(ActionSequencePerTime actions,
               bool succeed,
               uint64_t time_spent_ms,
               uint64_t expanded_nodes,
               int total_moves)
      : actions(std::move(actions)),
        succeed(succeed),
        time_spent_ms(time_spent_ms),
        expanded_nodes(expanded_nodes),
        total_moves(total_moves) {};
};

class MapfSolver {
 public:
  // Solve the mapf problem. The caller should make sure the initial state
  // and target state is not the same. Empty result means no solution.
  virtual SearchResult Solve(MapfMap *map_p, const std::vector<Task> &tasks, uint64_t timelimit_ms) = 0;
};

class MoveSeq {
 public:
  MoveSeq() = default;

  void AddActionSeq(int robot_id, std::vector<Action> actions);
  int GetTotalCost();
  ActionSequencePerTime ExportToOutput();
  int GetActionLength();
  std::vector<Position> GetPositionAtTime(int time_step);
  void Validate();
 private:
  std::map<int, std::vector<Action>> actions_;
  std::map<int, Task> tasks_;
};

}

#endif //MAPF_SRC_SOLVER_H_