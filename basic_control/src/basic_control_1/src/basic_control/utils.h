#pragma once

#include "constants.h"

#include <chrono>

namespace basic_control {

inline bool DoubleEquals(double a, double b) {
  return std::abs(a - b) < EPS;
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

class PID {
 public:
  PID(double Kp, double Td, double Ti, double dt) {
    this->Kp = Kp;
    this->Td = Td;
    this->Ti = Ti;
    this->dt = dt;
    current_error = 0;
    previous_error = 0;
    sum_error = 0;
    current_deriv_error = 0;
    control = 0;

    alpha = 0.2;
  };
  
  void UpdateControl(double new_error) {
    current_error = alpha * previous_error + (1 - alpha) * new_error;
    current_deriv_error = (current_error - previous_error) / dt;
    sum_error += current_error * dt;

    previous_error = current_error;
  };

  double GetControl() {
    control = Kp * (current_error + Td * current_deriv_error + sum_error / Ti);
    
    return control;
  };

 private:
  double Kp;
  double Td;
  double Ti;

  double current_error;
  double previous_error;

  double sum_error;
  
  double current_deriv_error;
  
  // For now this is not used.
  double previous_deriv_error;
  
  double control;
  double dt;

  double alpha;
};
}