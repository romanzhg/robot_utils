#ifndef MAPF_SRC_SOLVERS_CBS_H_
#define MAPF_SRC_SOLVERS_CBS_H_

#include <optional>
#include <set>
#include <vector>
#include <queue>

#include "solver.h"
#include "cbs_common.h"
#include "cbs_astar.h"

namespace mapf {
namespace cbs {

using ConstrainPair = std::pair<Constrain, Constrain>;

// Solution is generated on the run, no need to store.
// Maybe consider to store it for optimization?
struct Node {
  Node *parent;
  Constrain constrain;
  // The cost here is a mix of past cost and heuristic.
  // Bacause the paths returned by low-level search may have conflicts, they are not totally valid.
  // The under-estimated part corresponds to the heuristic.
  int cost;
  // This field is owned by this object.
  Solution *sol;
  // The effectiveness of this field is to be examined.
  int conflict_count;

  Node() = default;
  Node(Node *parent, const Constrain &constrain, Solution *sol, int cost, int conflict_count)
      : parent(parent), constrain(constrain), sol(sol), cost(cost), conflict_count(conflict_count) {};

  ~Node() {
    delete sol;
  }

  Node(const Node &o) = default;
  Node &operator=(const Node &o) = delete;

};

struct NodeKey {
  Node *value;
  int cost;
  int conflict_count;

  NodeKey() = delete;
  NodeKey(Node *n) {
    value = n;
    cost = n->cost;
    conflict_count = n->conflict_count;
  }

  bool operator<(const NodeKey &o) const {
    if (cost == o.cost) {
      return conflict_count > o.conflict_count;
    } else {
      return cost > o.cost;
    }
  }
};

class ConflictBasedSearch : public MapfSolver {
 public:
  ConflictBasedSearch() = default;
  ~ConflictBasedSearch();
  SearchResult Solve(MapfMap *map_p, const std::vector<Task> &tasks, uint64_t timelimit_ms) override;

 private:
  std::optional<Path> GetLocalPath(int robot_id, const std::set<Constrain> &constrains);
  // Get accumulated constrains by walking up the tree.
  std::set<Constrain> GetInheritedConstrainForRobot(Node *n, int robot_id);

  // Returns empty if there is no conflict.
  // Returns only the first conflict.
  std::optional<ConstrainPair> ValidateSolution(const Solution &sol);
  int GetSolutionCost(const Solution &s);
  int GetConflictCount(const Solution &s);
  Node *UpdateRobotPath(Constrain constrain, Node *cur_node);

  // Manage nodes.
  Node *NewNode(Node *parent, const Constrain &constrain, Solution *sol, int conflict_count);
  void ClearNodes();

  // Data.
  MapfMap *map_p_;
  std::vector<Task> tasks_;

  std::priority_queue<NodeKey> pq_;
  int robot_count_;
  CbsAstar astar_;

  std::set<Node *> nodes_;

  uint64_t start_time_point_ms_;
  uint64_t time_limit_ms_;
};

}  // namespace cbs
}  // namespace mapf

#endif //MAPF_SRC_SOLVERS_CBS_H_
