#include "icbs.h"

#include <algorithm>
#include <iostream>
#include <map>

#include "icbs_astar.h"
#include "utils.h"

using namespace std;

namespace mapf {
namespace icbs {

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
  cout << "Solution: " << endl;
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

Node *ICBS::NewNode(
    Node *parent, const Constrain &constrain, Solution *sol, int conflict_count) {
  int cost = GetSolutionCost(*sol);
  Node *n = new Node(parent, constrain, sol, cost, conflict_count);
  nodes_.insert(n);
  return n;
}

void ICBS::ClearNodes() {
  for (Node *p : nodes_) {
    delete p;
  }
  nodes_.clear();
}

// Solution cost is the total moves before a robot reaches the destination.
int ICBS::GetSolutionCost(const Solution &s) {
  int rtn = 0;
  for (int robot_id = 0; robot_id < robot_count_; robot_id++) {
    const Path &p = s[robot_id];
    const Position &dest = tasks_[robot_id].dest;
    const auto &end_it = find(p.begin(), p.end(), dest);
    if (end_it == p.end()) {
      cout << "invalid state, cannot find dest in solution" << endl;
      exit(0);
    }
    rtn += distance(p.begin(), end_it);
  }
  return rtn;
}

optional<ConstrainPair> ICBS::ValidateSolution(const Solution &sol) {
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
    const auto &c1 = IsCurrentPositionValid(cur_step, step, robot_count_);
    if (c1.has_value()) {
      return c1;
    }
  }
  return std::nullopt;
}

set<Constrain> ICBS::GetInheritedConstrainForRobot(Node *n, set<int> robot_ids) {
  set<Constrain> rtn;
  while (n->parent != nullptr) {
    if (robot_ids.find(n->constrain.robot_id) != robot_ids.end()) {
      rtn.insert(n->constrain);
    }
    n = n->parent;
  }
  return rtn;
}

// Create a new node according to the conflict.
Node *ICBS::UpdateRobotPath(Constrain constrain, Node *cur_node) {
  int robot_id = constrain.robot_id;
  // Get all the robots in the same group as robot_id.
  set<int> robot_ids = uf_.GetGroup(robot_id);
  // Get all the constrains between the robot group and other robots.
  set<Constrain> constrains = GetInheritedConstrainForRobot(cur_node, robot_ids);
  constrains.insert(constrain);

  // Run low level search.
  auto opt_partial_sol = astar_.FindSolution(map_p_, &constrains, GetTasks(robot_ids));

  Solution partial_sol;
  if (opt_partial_sol.has_value()) {
    partial_sol = opt_partial_sol.value();
  } else {
    return nullptr;
  }

  // Put the new partial_sol to solution.
  vector<int> robot_ids_seq;
  for (int robot_id : robot_ids) {
    robot_ids_seq.push_back(robot_id);
  }

  Solution *new_sol = new Solution(*cur_node->sol);
  for (int i = 0; i < robot_ids_seq.size(); i++) {
    (*new_sol)[robot_ids_seq[i]] = partial_sol[i];
  }

  return NewNode(cur_node, constrain, new_sol, GetConflictCount(*new_sol));
}

ICBS::~ICBS() {
  CleanUp();
}

int ICBS::GetConflictCount(const Solution &sol) {
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

SearchResult ICBS::Solve(MapfMap *map_p, const std::vector<Task> &tasks, uint64_t timelimit_ms) {
  map_p_ = map_p;
  tasks_ = tasks;
  robot_count_ = tasks.size();
  uf_.Init(robot_count_);

  start_time_point_ms_ = GetCurrentTimeSinceEpochMS();
  time_limit_ms_ = timelimit_ms;

  return SolveInternal();
}

SearchResult ICBS::SolveInternal() {
  CleanUp();

  // Build the initial state.
  Solution *sol = new Solution(robot_count_);

  map<int, set<int>> group_id_to_elem = uf_.GetGroupIdToElem();
  for (const auto &elem : group_id_to_elem) {
    vector<Task> tmp_tasks;
    vector<int> tmp_robot_id_seq;
    for (int robot_id : elem.second) {
      tmp_tasks.push_back(tasks_[robot_id]);
      tmp_robot_id_seq.push_back(robot_id);
    }

    auto opt_partial_sol = astar_.FindSolution(map_p_, &kEmptyConstrain, tmp_tasks);

    Solution partial_sol;
    if (opt_partial_sol.has_value()) {
      partial_sol = opt_partial_sol.value();
    } else {
      // The underlying search algorithm failed to find a solution.
      return SearchResult({}, false, GetCurrentTimeSinceEpochMS() - start_time_point_ms_,
                          0, 0);
    }

    for (int i = 0; i < tmp_robot_id_seq.size(); i++) {
      (*sol)[tmp_robot_id_seq[i]] = partial_sol[i];
    }
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

    auto constrain_or_empty = ValidateSolution(*(cur_node_p->sol));
    if (constrain_or_empty.has_value()) {
      ConstrainPair constrains = constrain_or_empty.value();
      ConflictingRobots cur_conflict{
          uf_.Find(constrains.first.robot_id),
          uf_.Find(constrains.second.robot_id)};

      cur_node_p->conflict_ = cur_conflict;
      if (ShouldMerge(cur_node_p)) {
        uf_.Union(constrains.first.robot_id, constrains.second.robot_id);
        return SolveInternal();
      }

      assert(uf_.Find(constrains.first.robot_id) != uf_.Find(constrains.second.robot_id));

      Node *child_a = UpdateRobotPath(constrains.first, cur_node_p);
      Node *child_b = UpdateRobotPath(constrains.second, cur_node_p);
      if (child_a == nullptr || child_b == nullptr) {
        return SearchResult({}, false, GetCurrentTimeSinceEpochMS() - start_time_point_ms_,
                            expanded_nodes, 0);
      }
      pq_.emplace(child_a);
      pq_.emplace(child_b);
      delete cur_node_p->sol;
      cur_node_p->sol = nullptr;
    } else {
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

void ICBS::CleanUp() {
  while (!pq_.empty()) {
    pq_.pop();
  }
  ClearNodes();
}

bool ICBS::ShouldMerge(Node *node_p) {
  ConflictingRobots cur_conflict = node_p->conflict_;

  int size_if_merged = uf_.GetSize(cur_conflict.robot_a) + uf_.GetSize(cur_conflict.robot_b);
  if (size_if_merged > kLargestGroupSize) {
    return false;
  }

  int cur_robot_a = uf_.Find(cur_conflict.robot_a);
  int cur_robot_b = uf_.Find(cur_conflict.robot_b);

  // TODO: verity that this field is exactly the same as cur_conflict, since we always restart after merge.
  ConflictingRobots tmp_cur_conflict(cur_robot_a, cur_robot_b);

  int conflict_count = 0;
  while (node_p != nullptr) {
    if (tmp_cur_conflict == node_p->conflict_) {
      conflict_count++;
    }
    node_p = node_p->parent;
  }

  return conflict_count > kMergeLimit;
}

vector<Task> ICBS::GetTasks(std::set<int> robot_ids) {
  vector<Task> rtn;
  for (int robot_id : robot_ids) {
    rtn.push_back(tasks_[robot_id]);
  }
  return rtn;
}

}
}
