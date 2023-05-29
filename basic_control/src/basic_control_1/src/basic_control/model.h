#pragma once

#include "utils.h"

#include <algorithm>
#include <cmath>

namespace basic_control {

// The bicycle model.
struct Model {
  constexpr static double L = 0.5;
  constexpr static double W = 0.3;
  constexpr static double Height = 0.25;
  constexpr static double vr_change_rate_limit_per_sec = 3.0;
  // Takes 3 seconds to rotate 90 degree.
  constexpr static double steering_change_rate_limit_per_sec = M_PI / 2.0 / 3.0;
  // Steering angle no more than 90 degree.
  constexpr static double steering_limit = M_PI / 2.0;

  static double GetDeltaFFromCurvature(double curvature) {
    if (DoubleEquals(curvature, 0)) {
      return 0;
    }
    double r = 1.0 / curvature;
    double df = std::atan2(L, r);
    return df;
  }

  Model() {
    x = 0;
    y = 0;
    psi = 0;

    vr = 1.0;
    steering = 0;

    target_vr = vr;
    target_steering = steering;
  }

  void Update(double delta_time) {
    // Update position base on current state.
    x = x + vr * std::cos(psi) * delta_time;
    y = y + vr * std::sin(psi) * delta_time;
    psi = psi + vr * std::tan(steering) / L * delta_time;

    // Update current state base on control.
    double max_steering_change = delta_time * steering_change_rate_limit_per_sec;
    double diff = std::abs(target_steering - steering);
    double sign = (target_steering - steering > 0) ? 1.0 : -1.0;

    steering += sign * (std::min(max_steering_change, diff));
  }

  void SetCommand(double target_vr, double target_steering) {
    LimitByBounds(target_steering, (-1.0) * steering_limit, steering_limit);
    this->target_vr = target_vr;
    this->target_steering = target_steering;
  }

  // Only for debug.
  void SetCommandWithoutLimit(double target_vr, double target_steering) {
    this->target_vr = target_vr;
    this->target_steering = target_steering;
    this->steering = target_steering;
  }

  // x, y, psi as in the world frame.
  double x, y, psi;
  double vr, steering;
  double target_vr, target_steering;
};

}