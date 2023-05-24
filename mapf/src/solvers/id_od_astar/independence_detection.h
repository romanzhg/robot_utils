#ifndef MAPF_SRC_SOLVERS_ID_OD_ASTAR_INDEPENDENCE_DETECTION_H_
#define MAPF_SRC_SOLVERS_ID_OD_ASTAR_INDEPENDENCE_DETECTION_H_

#include <vector>

#include "solver.h"
#include "utils.h"

namespace mapf {
namespace id_od_astar {

// The Independent Detection framework.
class IndependenceDetection : public MapfSolver {
 public:
  IndependenceDetection();
  SearchResult Solve(MapfMap *map_p, const std::vector<Task> &tasks, uint64_t timelimit_ms) override;
 private:
  // TODO: think of a way to unify all the different IsActionValid checks, in the variants of Astars.
  std::optional<std::pair<int, int>> CheckAndApplyActions(std::vector<Position> &cur_positions,
                                                          const std::vector<Action> &actions);
  std::optional<std::pair<int, int>> GetFirstConflict(
      const std::vector<Task> &tasks, const ActionSequencePerRobot& actions);
  int GetTotalCost(const ActionSequencePerRobot &actions);
  MapfSolver *solver_;
  int robot_count_;

  std::vector<Task> tasks_;

  UnionFind uf_;

  uint64_t start_time_point_ms_;
  uint64_t time_limit_ms_;
  std::vector<Task> GetTasks(const std::set<int> &robot_ids);
};
}

}

#endif //MAPF_SRC_SOLVERS_ID_OD_ASTAR_INDEPENDENCE_DETECTION_H_
