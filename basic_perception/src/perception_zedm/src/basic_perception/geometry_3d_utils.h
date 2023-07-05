#pragma once

#include <Eigen/Eigen>
#include <cmath>

#include "geometry_3d.h"

namespace basic_perception {

Quaternion GetRotationFromXoY(Point3 norm) {
  Eigen::Vector3d from{0, 0, 1};
  Eigen::Vector3d to{norm.x, norm.y, norm.z};
  Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(from, to);
  return Quaternion(q.x(), q.y(), q.z(), q.w());
}

}  // namespace basic_perception
