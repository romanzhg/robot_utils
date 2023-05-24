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
  // Takes 4 seconds to rotate 90 degree.
  constexpr static double delta_f_change_rate_limit_per_sec = M_PI / 2.0 / 4.0;
  // Steering angle no more than 60 degree.
  constexpr static double delta_f_limit = M_PI / 3.0;

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
    delta_f = 0;

    target_vr = vr;
    target_delta_f = delta_f;
  }

  void Update(double delta_time) {
    // Update position base on current state.
    x = x + vr * std::cos(psi) * delta_time;
    y = y + vr * std::sin(psi) * delta_time;
    psi = psi + vr * std::tan(delta_f) / L * delta_time;

    // Update current state base on control.
    double max_delta_f_change = delta_time * delta_f_change_rate_limit_per_sec;
    double diff = std::abs(target_delta_f - delta_f);
    double sign = (target_delta_f - delta_f > 0) ? 1.0 : -1.0;

    delta_f += sign * (std::min(max_delta_f_change, diff));
  }

  void SetCommand(double target_vr, double target_delta_f) {
    LimitByBounds(target_delta_f, (-1.0) * delta_f_limit, delta_f_limit);
    this->target_vr = target_vr;
    this->target_delta_f = target_delta_f;
  }

  // x, y, psi as in the world frame.
  double x, y, psi;
  double vr, delta_f;
  double target_vr, target_delta_f;
};

}