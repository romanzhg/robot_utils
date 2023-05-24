#ifndef MAPF_SRC_SOLVERS_ICBS_ICBS_H_
#define MAPF_SRC_SOLVERS_ICBS_ICBS_H_

#include <optional>
#include <set>
#include <vector>
#include <queue>

#include "icbs_astar.h"
#include "icbs_common.h"
#include "solver.h"
#include "utils.h"

namespace mapf {
namespace icbs {

using ConstrainPair = std::pair<Constrain, Constrain>;

struct ConflictingRobots {
  int robot_a;
  int robot_b;

  ConflictingRobots() {
    robot_a = -1;
    robot_b = -1;
  }

  ConflictingRobots(int a, int b) {
    robot_a = a;
    robot_b = b;
    if (robot_a > robot_b) {
      std::swap(robot_a, robot_b);
    }
  }

  bool operator<(const ConflictingRobots &o) const {
    if (robot_a == o.robot_a) {
      return robot_b < o.robot_b;
    } else {
      return robot_a < o.robot_a;
    }
  }

  bool operator==(const ConflictingRobots &o) const {
    return robot_a == o.robot_a && robot_b == o.robot_b;
  }
};

struct Node {
  int node_id;
  Node *parent;
  Constrain constrain;

  // Solution and derived fields.
  Solution *sol;
  // The cost here is a mix of past cost and heuristic.
  // Bacause the paths returned by low-level search may have conflicts, they are not totally valid.
  // The under-estimated part corresponds to the heuristic.
  int cost;
  // Conflict count corresponds to the current solution.
  int conflict_count;
  // The conflict to split on.
  ConflictingRobots conflict_;

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

const std::set<Constrain> kEmptyConstrain = {};

class ICBS : public MapfSolver {
 public:
  ICBS() = default;
  ~ICBS();
  SearchResult Solve(MapfMap *map_p, const std::vector<Task> &tasks, uint64_t timelimit_ms) override;

 private:
  SearchResult SolveInternal();
  void CleanUp();

  // Get accumulated constrains by walking up the tree.
  std::set<Constrain> GetInheritedConstrainForRobot(Node *n, std::set<int> robot_ids);

  // Returns empty if there is no conflict.
  // Returns only the first conflict.
  std::optional<ConstrainPair> ValidateSolution(const Solution &sol);
  int GetSolutionCost(const Solution &s);
  int GetConflictCount(const Solution &s);
  Node *UpdateRobotPath(Constrain constrain, Node *cur_node);

  std::vector<Task> GetTasks(std::set<int> robot_ids);

  // Manage nodes.
  Node *NewNode(Node *parent, const Constrain &constrain, Solution *sol, int conflict_count);
  void ClearNodes();

  // Data.
  MapfMap *map_p_;
  std::vector<Task> tasks_;

  std::priority_queue<NodeKey> pq_;
  int robot_count_;
  ICbsAstar astar_;

  std::set<Node *> nodes_;
  UnionFind uf_;

  uint64_t start_time_point_ms_;
  uint64_t time_limit_ms_;
  bool ShouldMerge(Node *node_p);
};

}
}

#endif //MAPF_SRC_SOLVERS_ICBS_ICBS_H_
