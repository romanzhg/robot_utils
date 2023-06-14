#pragma once

#include "map.h"
#include "map_utils.h"
#include "math.h"
#include "types.h"
#include "tf_utils.h"


#include <map>
#include <queue>

// Note: use relative path just to make vscode happy.
#include "../rsmotion/rsmotion.h"

namespace basic_planning {

// Helper functions.

// Note the following two functions are problematic, need to debug.
// p_in_world = rotation_body_to_world * p_in_body + trans_in_world_world_to_body
// inline PlanarPose PlanarPoseWorldToBody(PlanarPose p_in_world, PlanarPose body_in_world) {
//   PlanarPose p_in_body;

//   p_in_body.x = p_in_world.x - body_in_world.x;
//   p_in_body.y = p_in_world.y - body_in_world.y;

//   p_in_body.x = p_in_body.x * cos(-body_in_world.psi) - p_in_body.y * sin(-body_in_world.psi);
//   p_in_body.y = p_in_body.x * sin(-body_in_world.psi) + p_in_body.y * cos(-body_in_world.psi);

//   p_in_body.psi = NormalizeAngle(p_in_world.psi - body_in_world.psi);

//   return p_in_body;
// }

// p_in_world = rotation_body_to_world * p_in_body + trans_in_world_world_to_body
// inline PlanarPose PlanarPoseBodyToWorld(PlanarPose p_in_body, PlanarPose body_in_world) {
//   PlanarPose p_in_world;

//   p_in_world.x = p_in_body.x * cos(body_in_world.psi) - p_in_body.y * sin(body_in_world.psi);
//   p_in_world.y = p_in_body.x * sin(body_in_world.psi) + p_in_body.y * cos(body_in_world.psi);

//   p_in_world.x += body_in_world.x;
//   p_in_world.y += body_in_world.y;

//   p_in_world.psi = NormalizeAngle(p_in_body.psi + body_in_world.psi);
//   return p_in_world;
// }


inline bool IsPoseValid(const PlanarPose& pp, const OccupancyGridMap& m) {
  double tmp_x = pp.x + Model::L / 2.0 * std::cos(pp.psi);
  double tmp_y = pp.y + Model::L / 2.0 * std::sin(pp.psi);

  return m.IsCollisionFree(tmp_x, tmp_y, pp.psi, Model::L, Model::W);
}

inline bool IsPathValid(const Path& path, const OccupancyGridMap& m) {
  for (const PlanarPose& pp : path.path) {
    if (not IsPoseValid(pp, m)) {
      return false;
    }
  }
  return true;
}

// inline Path GenRsPath(PlanarPose from, PlanarPose to) {
//   PlanarPose to_in_body = PlanarPoseWorldToBody(to, from);
//   double tr = Model::GetTurningRadius();

//   rsmotion::algorithm::State toState(to_in_body.x / tr,
//                                      to_in_body.y / tr,
//                                      to_in_body.psi,
//                                      false);
//   rsmotion::algorithm::Path rs_path = rsmotion::algorithm::SearchShortestPath(toState);
  
//   Path rtn;
//   double l = rs_path.Length();

//   rsmotion::algorithm::State fromState(from.x / tr, from.y / tr, from.psi, false);
//   for (double dist = 0; dist <= l; dist += (0.1 / tr)) {
//     PlanarPose pp;
//     const auto& tmp = rsmotion::algorithm::InterpolateDistance(fromState, rs_path, dist);

//     pp.x = tmp.X() * tr;
//     pp.y = tmp.Y() * tr;
//     pp.psi = tmp.Phi();

//     // rtn.path.push_back(PlanarPoseBodyToWorld(pp, from));
//     rtn.path.push_back(pp);
//   }
//   return rtn;
// }

inline Path GenRsPath(PlanarPose from, PlanarPose to) {
  auto ros_pose_from = XYYawToPose(from.x, from.y, from.psi);
  auto ros_pose_to = XYYawToPose(to.x, to.y, to.psi);
  auto ros_pose_to_in_body = PoseWorldToBody(ros_pose_to, ros_pose_from);

  PlanarPose to_in_body;
  PoseToXYYaw(ros_pose_to_in_body, to_in_body.x, to_in_body.y, to_in_body.psi);

  double tr = Model::GetTurningRadius();

  rsmotion::algorithm::State toState(to_in_body.x / tr,
                                     to_in_body.y / tr,
                                     to_in_body.psi,
                                     false);
  rsmotion::algorithm::Path rs_path = rsmotion::algorithm::SearchShortestPath(toState);
  
  Path rtn;
  double l = rs_path.Length();

  rsmotion::algorithm::State fromState(from.x / tr, from.y / tr, from.psi, false);
  for (double dist = 0; dist <= l; dist += (0.1 / tr)) {
    PlanarPose pp;
    const auto& tmp = rsmotion::algorithm::InterpolateDistance(fromState, rs_path, dist);

    pp.x = tmp.X() * tr;
    pp.y = tmp.Y() * tr;
    pp.psi = tmp.Phi();

    rtn.path.push_back(pp);
  }
  return rtn;
}

struct HeapNode {
  PlanarPose pose;
  double past_cost;
  double heuristic;
  uint64_t index;
  uint64_t parent_index;

  HeapNode() = default;
  HeapNode(PlanarPose pose, double past_cost, double heuristic)
      : pose(pose), past_cost(past_cost), heuristic(heuristic) {}

  bool operator<(const HeapNode& o) const {
    return past_cost + heuristic > o.past_cost + o.heuristic;
  }
};

// TODO: use astar to get heuristic.
double GetHeuristic(PlanarPose from, PlanarPose to, const OccupancyGridMap& m) {
  return hypot(to.x - from.x, to.y - from.y);
}

const double kStepLenM = 0.2;
const double kStepCost = 0.2;
const double kBinLen = 0.1;
const std::vector<double> kPsiSteps = {-0.4, -0.3, -0.2, -0.1, 0, 0.1, 0.2, 0.3, 0.4};

struct HybirdAstar {
  const uint64_t kHybirdAstarMaxElems = 128 * 1024 * 1024;

  HeapNode* global_array;
  uint64_t global_index;

  std::map<int, std::map<int, uint8_t[70]>> bins;

  HybirdAstar() {
    global_array = (HeapNode*)malloc(kHybirdAstarMaxElems * sizeof(HeapNode));
  }

  ~HybirdAstar() {
    free(global_array);
  }

  void BackTrack(HeapNode n, Path& front) {
    if (n.parent_index != UINT64_MAX) {
      BackTrack(global_array[n.parent_index], front);
    }
    front.path.push_back(n.pose);
  }

  Path ConstructPath(HeapNode cur, Path rs_path) {
    Path astar_path;
    BackTrack(cur, astar_path);

    std::cout << "astar end pose: " << cur.pose.ToString() << std::endl;

    astar_path.Combine(rs_path);
    return astar_path;
  }

  // Returns true for a successful insertion, false if the bin is occupied.
  bool InsertToGrid(int index_x, int index_y, int index_z) {
    if (bins.find(index_x) == bins.end()) {
      memset(bins[index_x][index_y], 0, sizeof(bins[index_x][index_y]));
      bins[index_x][index_y][index_z] = 1;
      return true;
    }
    if (bins[index_x].find(index_y) == bins[index_x].end()) {
      memset(bins[index_x][index_y], 0, sizeof(bins[index_x][index_y]));
      bins[index_x][index_y][index_z] = 1;
      return true;
    }
    if (bins[index_x][index_y][index_z] == 1) {
      return false;
    } else {
      bins[index_x][index_y][index_z] = 1;
      return true;
    }
  }

  Path SearchPath(PlanarPose from, PlanarPose to, const OccupancyGridMap& m) {
    global_index = 0;
    bins.clear();
    std::priority_queue<HeapNode> open;

    HeapNode start(from, 0, GetHeuristic(from, to, m));
    start.index = global_index;
    start.parent_index = UINT64_MAX;
    global_array[global_index] = start;
    global_index++;

    open.push(start);

    while (not open.empty()) {
      HeapNode cur_node = open.top();
      open.pop();

      Path tmp_path = GenRsPath(cur_node.pose, to);

      if (IsPathValid(tmp_path, m)) {
        return ConstructPath(cur_node, tmp_path);
      }

      std::vector<PlanarPose> next_steps = GetNext(cur_node.pose);

      for (const PlanarPose& pp : next_steps) {
        if (not IsPoseValid(pp, m)) {
          continue;
        }

        int grid_index_x = std::round(pp.x / kBinLen);
        int grid_index_y = std::round(pp.y / kBinLen);
        int grid_index_psi = std::round(NormalizeAnglePositive(pp.psi) * 10);
        bool insertion_succeed = InsertToGrid(grid_index_x, grid_index_y, grid_index_psi);
        if (!insertion_succeed) {
          continue;
        }

        HeapNode tmp_node(pp, cur_node.past_cost + kStepCost, GetHeuristic(pp, to, m));
        tmp_node.index = global_index;
        tmp_node.parent_index = cur_node.index;
        global_array[global_index] = tmp_node;
        global_index++;
        if (global_index >= kHybirdAstarMaxElems) {
          return {};
        }
        open.push(tmp_node);
      }
    }
    return {};
  }

  std::vector<PlanarPose> GetNext(PlanarPose from) {
    std::vector<PlanarPose> rtn;

    // Forward.
    for (const auto& psi_change : kPsiSteps) {
      PlanarPose tmp;
      tmp.psi = from.psi + psi_change;
      tmp.x = from.x + kStepLenM * std::cos(tmp.psi);
      tmp.y = from.y + kStepLenM * std::sin(tmp.psi);
      rtn.push_back(tmp);
    }

    // Backward.
    for (const auto& psi_change : kPsiSteps) {
      PlanarPose tmp;
      tmp.psi = from.psi + psi_change;
      tmp.x = from.x - kStepLenM * std::cos(tmp.psi);
      tmp.y = from.y - kStepLenM * std::sin(tmp.psi);
      rtn.push_back(tmp);
    }
    return rtn;
  }
};

}  // namespace basic_planning