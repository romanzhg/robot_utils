#pragma once

#include "math.h"

#include <algorithm>
#include <cmath>

namespace basic_planning {

struct Model {
  constexpr static double L = 0.5;
  constexpr static double W = 0.3;
  constexpr static double Height = 0.25;
  // Steering angle no more than 35 degree.
  constexpr static double kSteeringLimit = 0.6;

  static double GetTurningRadius() {
    return L / std::tan(kSteeringLimit);
  }
};

struct PlanarPose {
  PlanarPose() = default;
  PlanarPose(double x, double y, double psi) : x(x), y(y), psi(psi) {};

  double x, y, psi;

  PlanarPose GetParkingForwardPose() {
    PlanarPose rtn;
    rtn.x = x - Model::L / 2.0 * std::cos(psi);
    rtn.y = y - Model::L / 2.0 * std::sin(psi);
    rtn.psi = psi;
    return rtn;
  }

  PlanarPose GetParkingBackwardPose() {
    PlanarPose rtn;
    rtn.x = x + Model::L / 2.0 * std::cos(psi);
    rtn.y = y + Model::L / 2.0 * std::sin(psi);
    rtn.psi = NormalizeAnglePositive(psi + M_PI);
    return rtn;
  }

  std::string ToString() {
    return " x: " + std::to_string(x) + " y: " + std::to_string(y) + " psi: " + std::to_string(psi);
  }
};

struct Path {
  Path() = default;

  std::vector<PlanarPose> path;

  void Combine(const Path& o) {
    path.insert(path.end(), o.path.begin(), o.path.end());
  }
};

}