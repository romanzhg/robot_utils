#pragma once

#include "constants.h"
#include "types.h"
#include "geometry.h"

#include <chrono>


namespace basic_control {

inline bool DoubleEquals(double a, double b) {
  return std::abs(a - b) < EPS;
}

inline bool DoubleEquals(double a, double b, double eps) {
  return std::abs(a - b) < eps;
}

// A replacement for ros::Time::now().toSec().
inline double GetTimeDouble() {
  return std::chrono::duration<double, std::chrono::seconds::period>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

inline void LimitByBounds(double& v, double lb, double ub) {
  if (v < lb) {
    v = lb;
  } else if (v > ub) {
    v = ub;
  } else {
    return;
  }
}


// p_in_world = rotation_body_to_world * p_in_body + trans_in_world_world_to_body
geometry::Point2 Point2WorldToBody(geometry::Point2 p_in_world, PlanarPose body_in_world) {
  geometry::Point2 p_in_body;

  p_in_body = p_in_world - geometry::Point2(body_in_world.x, body_in_world.y);
  p_in_body = RotateVector2(p_in_body, -body_in_world.psi);
  return p_in_body;
}

// p_in_world = rotation_body_to_world * p_in_body + trans_in_world_world_to_body
geometry::Point2 Point2PoseBodyToWorld(geometry::Point2 p_in_body, PlanarPose body_in_world) {
  geometry::Point2 p_in_world;

  // The rotation.
  p_in_world = RotateVector2(p_in_body, body_in_world.psi);

  // Add the translation.
  p_in_world = p_in_world + geometry::Point2(body_in_world.x, body_in_world.y);

  return p_in_world;
}
}