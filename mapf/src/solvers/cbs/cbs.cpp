#include "cbs.h"

#include <algorithm>
#include <map>
#include <iostream>

#include "cbs_astar.h"
#include "utils.h"

using namespace std;

namespace mapf {
namespace cbs {

namespace {
// Get position for all robots, indexed by robot id.
// Step 0 refers to the initial position of a robot.
vector<Position> GetRobotPositionsAtStep(int step,
                                         const Solution &s,
                                         int robot_count) {
  vector<Position> rtn(robot_count);
  for (int robot_id = 0; robot_id < robot_count; robot_id++) {
    if (step < s[robot_id].size()) {
      rtn[robot_id] = s[robot_id].at(step);
    } else {
      rtn[robot_id] = s[robot_id].back();
    }
  }
  return rtn;
}

// Check for edge reuse. Return the first conflict found.
optional<ConstrainPair> IsTransitionValid(const vector<Position> &prev,
                                          const vector<Position> &cur,
                                          int time_step,
                                          int robot_count) {
  // Edge to robot id.
  map<Edge, int> used_edges;
  for (int i = 0; i < robot_count; i++) {
    if (prev[i] == cur[i]) {
      continue;
    }
    Edge edge(prev[i], cur[i]);
    if (used_edges.find(edge) != used_edges.end()) {
      Constrain c1(i, time_step, prev[i], cur[i]);

      int other_robot = used_edges[edge];
      Constrain c2(other_robot, time_step, prev[other_robot], cur[other_robot]);

      return make_pair(c1, c2);
    }
    used_edges.insert({edge, i});
  }
  return nullopt;
}

int GetInvalidTransitionCount(const vector<Position> &prev,
                              const vector<Position> &cur,
                              int robot_count) {
  int rtn = 0;
  // Edge to robot id.
  map<Edge, int> used_edges;
  for (int i = 0; i < robot_count; i++) {
    if (prev[i] == cur[i]) {
      continue;
    }
    Edge edge(prev[i], cur[i]);
    if (used_edges.find(edge) != used_edges.end()) {
      ++rtn;
    } else {
      used_edges.insert({edge, i});
    }
  }
  return rtn;
}

// Check for robot overlap. Return the first conflict found.
optional<ConstrainPair> IsCurrentPositionValid(const vector<Position> &cur, int time_step, int robot_count) {
  map<Position, int> pos_to_robot;
  for (int i = 0; i < robot_count; i++) {
    if (pos_to_robot.find(cur[i]) != pos_to_robot.end()) {
      Constrain c1(i, time_step, cur[i]);
      Constrain c2(pos_to_robot[cur[i]], time_step, cur[i]);
      return make_pair(c1, c2);
    }
    pos_to_robot.insert({cur[i], i});
  }
  return nullopt;
}

int GetInvalidCurrentPositionCount(const vector<Position> &cur, int robot_count) {
  int rtn = 0;
  map<Position, int> pos_to_robot;
  for (int i = 0; i < robot_count; i++) {
    if (pos_to_robot.find(cur[i]) != pos_to_robot.end()) {
      ++rtn;
    } else {
      pos_to_robot.insert({cur[i], i});
    }
  }
  return rtn;
}

void PrintSolution(Solution *sol, int robot_count) {
  cout << "print a whole solution" << endl;
  int longest_pos_len = 0;
  for (const auto &path : *sol) {
    longest_pos_len = max(longest_pos_len, (int) path.size());
  }

  for (int step = 0; step < longest_pos_len; step++) {
    vector<Position> cur_step = GetRobotPositionsAtStep(step, *sol, robot_count);
    cout << "step: " << to_string(step) << endl;
    for (int i = 0; i < robot_count; i++) {
      cout << "robot position: " << cur_step[i].to_string() << endl;
    }
  }
}

// From [robot_id][step][pos] to [step][robot_id][action].
vector<vector<Action>> ConvertRobotPositionsToTimedActions(
    const Solution &in) {
  int robot_count = in.size();
  int longest_pos_len = 0;
  for (const auto &path : in) {
    longest_pos_len = max(longest_pos_len, (int) path.size());
  }

  vector<vector<mapf::Action>> rtn(
      longest_pos_len - 1,
      vector<mapf::Action>(robot_count, mapf::Action::WAIT));
  for (int robot_id = 0; robot_id < robot_count; robot_id++) {
    for (int step = 1; step < in[robot_id].size(); step++) {
      rtn[step - 1][robot_id] = PositionToAction(in[robot_id][step - 1], in[robot_id][step]);
    }
  }
  return rtn;
}
}  // namespace

Node *ConflictBasedSearch::NewNode(
    Node *parent, const Constrain &constrain, Solution *sol, int conflict_count) {
  int cost = GetSolutionCost(*sol);
  Node *n = new Node(parent, constrain, sol, cost, conflict_count);
  nodes_.insert(n);
  return n;
}

void ConflictBasedSearch::ClearNodes() {
  for (Node *p : nodes_) {
    delete p;
  }
  nodes_.clear();
}

// Solution cost is the total moves before a robot reaches the destination.
int ConflictBasedSearch::GetSolutionCost(const Solution &s) {
  // Counts the total time span.
  int rtn = 0;
  for (const Path &p : s) {
    rtn += p.size() - 1;
  }
  return rtn;
}

optional<ConstrainPair> ConflictBasedSearch::ValidateSolution(const Solution &sol) {
  int longest_pos_len = 0;
  for (const auto &path : sol) {
    longest_pos_len = max(longest_pos_len, (int) path.size());
  }

  for (int step = 1; step < longest_pos_len; step++) {
    const vector<Position> &prev_step = GetRobotPositionsAtStep(step - 1, sol, robot_count_);;
    const vector<Position> &cur_step = GetRobotPositionsAtStep(step, sol, robot_count_);

    // 1. Check transition, no edge reuse.
    const auto &c0 = IsTransitionValid(prev_step, cur_step, step, robot_count_);
    if (c0.has_value()) {
      return c0;
    }

    // 2. Check new positions, no overlap.
    // Time step 0 refers to the first move.
    const auto &c1 = IsCurrentPositionValid(cur_step, step, robot_count_);
    if (c1.has_value()) {
      return c1;
    }
  }
  return std::nullopt;
}

set<Constrain> ConflictBasedSearch::GetInheritedConstrainForRobot(Node *n, int robot_id) {
  set<Constrain> rtn;
  while (n->parent != nullptr) {
    if (n->constrain.robot_id == robot_id) {
      rtn.insert(n->constrain);
    }
    n = n->parent;
  }
  return rtn;
}

// Find a shortest path(run an astar algorithm) with respect to the constrains.
// TODO: implement the optimization of "if two nodes(in a star) has the same weight the
// choose the one with less conflicts". This optimization requires all other pathes to present.
optional<Path> ConflictBasedSearch::GetLocalPath(int robot_id, const set<Constrain> &constrains) {
  const Position &src = tasks_[robot_id].src;
  const Position &dest = tasks_[robot_id].dest;

  // TODO: think about the semantic here.
  const auto &path = astar_.FindPath(map_p_, constrains, src, dest);
  return path;
}

// Create a new node according to the conflict.
Node *ConflictBasedSearch::UpdateRobotPath(Constrain constrain, Node *cur_node) {
  int robot_id = constrain.robot_id;
  set<Constrain> constrains = GetInheritedConstrainForRobot(cur_node, robot_id);
  constrains.insert(constrain);

  optional<Path> path = GetLocalPath(robot_id, constrains);
  if (!path.has_value()) {
    // No path exist, should just ignore this branch.
    // This could happen when all possible first step moves for a robot are banned.
    return nullptr;
  }
  auto *new_sol = new Solution(*cur_node->sol);
  (*new_sol)[robot_id] = path.value();

  return NewNode(cur_node, constrain, new_sol, GetConflictCount(*new_sol));
}

SearchResult ConflictBasedSearch::Solve(MapfMap *map_p, const std::vector<Task> &tasks, uint64_t timelimit_ms) {
  // Cleanup and initialize.
  while (!pq_.empty()) {
    pq_.pop();
  }
  ClearNodes();

  map_p_ = map_p;
  tasks_ = tasks;
  robot_count_ = tasks.size();

  start_time_point_ms_ = GetCurrentTimeSinceEpochMS();
  time_limit_ms_ = timelimit_ms;

  // Build the initial state.
  Solution *sol = new Solution;
  for (int robot_id = 0; robot_id < robot_count_; robot_id++) {
    (*sol).push_back(GetLocalPath(robot_id, {}).value());
  }

  Node *root = NewNode(nullptr, {}, sol, GetConflictCount(*sol));
  pq_.emplace(root);

  Node *cur_node_p;

  uint64_t expanded_nodes = 0;
  while (!pq_.empty()) {
    expanded_nodes++;
    if (IsElapsedTimeLongerThan(start_time_point_ms_, time_limit_ms_)) {
      break;
    }

    cur_node_p = pq_.top().value;
    pq_.pop();

    auto constrain_or_empty = ValidateSolution(*cur_node_p->sol);
    if (constrain_or_empty.has_value()) {
      ConstrainPair constrains = constrain_or_empty.value();
      Node *child_a = UpdateRobotPath(constrains.first, cur_node_p);
      Node *child_b = UpdateRobotPath(constrains.second, cur_node_p);
      if (child_a != nullptr) {
        pq_.emplace(child_a);
      }
      if (child_b != nullptr) {
        pq_.emplace(child_b);
      }
      delete cur_node_p->sol;
      cur_node_p->sol = nullptr;
    } else {
      // Current solution has no conflict.
      return SearchResult(ConvertRobotPositionsToTimedActions(*cur_node_p->sol),
                          true,
                          GetCurrentTimeSinceEpochMS() - start_time_point_ms_,
                          expanded_nodes, cur_node_p->cost);
    }
  }

  // Return result not found.
  return SearchResult({}, false, GetCurrentTimeSinceEpochMS() - start_time_point_ms_,
                      expanded_nodes, 0);
}

ConflictBasedSearch::~ConflictBasedSearch() {
  while (!pq_.empty()) {
    pq_.pop();
  }
  ClearNodes();
}
int ConflictBasedSearch::GetConflictCount(const Solution &sol) {
  int longest_pos_len = 0;
  for (const auto &path : sol) {
    longest_pos_len = max(longest_pos_len, (int) path.size());
  }

  int rtn = 0;
  for (int step = 1; step < longest_pos_len; step++) {
    const vector<Position> &prev_step = GetRobotPositionsAtStep(step - 1, sol, robot_count_);
    const vector<Position> &cur_step = GetRobotPositionsAtStep(step, sol, robot_count_);

    rtn += GetInvalidTransitionCount(prev_step, cur_step, robot_count_)
        + GetInvalidCurrentPositionCount(cur_step, robot_count_);
  }
  return rtn;
}

}  // namespace cbs
}  // namespace mapf