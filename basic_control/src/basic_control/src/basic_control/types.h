#pragma once

#include "constants.h"

namespace basic_control {

struct PlanarPose {
  PlanarPose() = default;
  PlanarPose(double x, double y, double psi) : x(x), y(y), psi(psi) {};

  double x, y, psi;
};



}