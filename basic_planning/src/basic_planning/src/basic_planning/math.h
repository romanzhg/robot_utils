#pragma once

#include <algorithm>
#include <cmath>

namespace basic_planning {

inline double NormalizeAnglePositive(double angle) {
  return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
}

inline double NormalizeAngle(double angle) {
  double a = NormalizeAnglePositive(angle);
  if (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  return a;
}


}