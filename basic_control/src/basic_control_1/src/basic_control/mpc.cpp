#include "mpc.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

namespace {
int kSteps = 30;
int x_start;
int y_start;
int psi_start;
int cte_start;
int steering_start;

using CppAD::AD;
class FG_eval {
  double step_len_meters = 0.02;
 public:
  FG_eval() = default;

  typedef std::vector<AD<double>> ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    fg[0] = 0;

    // Adding Cost section
    for (int t = 0; t < kSteps; t++) {
      fg[0] += 10 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 5 * CppAD::pow(vars[psi_start + t], 2);
    }

    // Minimize the use of actuators.
    for (int t = 0; t < kSteps - 1; t++) {
      fg[0] += 5 * CppAD::pow(vars[steering_start + t], 2);
    }

    // Minimize the value gap between sequential actuations. It enables vehicle move smoothly.
    for (int t = 0; t < kSteps - 2; t++) {
      fg[0] += 10 * CppAD::pow(vars[steering_start + t + 1] - vars[steering_start + t], 2);
    }

    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + cte_start] = vars[cte_start];

    // Create predicted trajectory. Its length depends on N.
    for (int t = 1; t < kSteps; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> cte1 = vars[cte_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> steering = vars[steering_start + t - 1];

      // Get next values
      fg[1 + x_start + t] = x1 - (x0 + step_len_meters * CppAD::cos(psi0));
      fg[1 + y_start + t] = y1 - (y0 + step_len_meters * CppAD::sin(psi0));
      fg[1 + psi_start + t] = psi1 - (psi0 + step_len_meters * CppAD::tan(steering) / 0.5);
      fg[1 + cte_start + t] = cte1 - y1;
    }
  }
};
}  // namespace

namespace basic_control {

typedef std::vector<double> Dvector;
ModelPredictiveController::ModelPredictiveController() {
  x_start = 0;
  y_start = x_start + kSteps;
  psi_start = y_start + kSteps;
  cte_start = psi_start + kSteps;
  steering_start = cte_start + kSteps;
}

void ModelPredictiveController::GetControl(
    double& target_vr, double& target_steering, const Model& m, const geometry::LineSegs& ref) {
  target_vr = 1.0;

  

  double x = m.x;
  double y = m.y;
  double psi = m.psi;
  double cte = m.y;

  const int kStateSize = 4;
  int n_vars = kStateSize * kSteps + (kSteps - 1);
  int n_constraints = kStateSize * kSteps;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[cte_start] = cte;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for (int i = 0; i < steering_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // For steering. [-60, 60] in radians
  for (int i = steering_start; i < n_vars; i++) {
    vars_lowerbound[i] = -0.6;
    vars_upperbound[i] = 0.6;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[cte_start] = cte;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[cte_start] = cte;

  FG_eval fg_eval;

  // options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  CppAD::ipopt::solve_result<Dvector> solution;

  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  if (solution.status == CppAD::ipopt::solve_result<Dvector>::success) {
    std::cout << "ipopt::solve succeed." << std::endl;
  } else {
    std::cout << "ipopt::solve failed." << std::endl;
  }

  target_steering = solution.x[steering_start];
  // for (int i = 0; i < kSteps - 1; i++) {
  //   std::cout << "returned target steering: " << solution.x[steering_start + i] << std::endl;
  // }
}

}  // namespace basic_control


// Note that in this version, the reference line is x axis.
//
//
// int kSteps = 30;
// int x_start;
// int y_start;
// int psi_start;
// int cte_start;
// int steering_start;
// class FG_eval_zero {
//   double step_len_meters = 0.02;
//  public:
//   FG_eval_zero() = default;

//   typedef std::vector<AD<double>> ADvector;
//   void operator()(ADvector& fg, const ADvector& vars) {
//     fg[0] = 0;

//     // Adding Cost section
//     for (int t = 0; t < kSteps; t++) {
//       fg[0] += 10 * CppAD::pow(vars[cte_start + t], 2);
//       fg[0] += 5 * CppAD::pow(vars[psi_start + t], 2);
//     }

//     // Minimize the use of actuators.
//     for (int t = 0; t < kSteps - 1; t++) {
//       fg[0] += 5 * CppAD::pow(vars[steering_start + t], 2);
//     }

//     // Minimize the value gap between sequential actuations. It enables vehicle move smoothly
//     for (int t = 0; t < kSteps - 2; t++) {
//       fg[0] += 10 * CppAD::pow(vars[steering_start + t + 1] - vars[steering_start + t], 2);
//     }

//     fg[1 + x_start] = vars[x_start];
//     fg[1 + y_start] = vars[y_start];
//     fg[1 + psi_start] = vars[psi_start];
//     fg[1 + cte_start] = vars[cte_start];

//     // Create predicted trajectory. Its length depends on N.
//     for (int t = 1; t < kSteps; t++) {
//       // The state at time t+1 .
//       AD<double> x1 = vars[x_start + t];
//       AD<double> y1 = vars[y_start + t];
//       AD<double> psi1 = vars[psi_start + t];
//       AD<double> cte1 = vars[cte_start + t];

//       // The state at time t.
//       AD<double> x0 = vars[x_start + t - 1];
//       AD<double> y0 = vars[y_start + t - 1];
//       AD<double> psi0 = vars[psi_start + t - 1];
//       AD<double> cte0 = vars[cte_start + t - 1];

//       // Only consider the actuation at time t.
//       AD<double> steering = vars[steering_start + t - 1];

//       // Get next values
//       fg[1 + x_start + t] = x1 - (x0 + step_len_meters * CppAD::cos(psi0));
//       fg[1 + y_start + t] = y1 - (y0 + step_len_meters * CppAD::sin(psi0));
//       fg[1 + psi_start + t] = psi1 - (psi0 + step_len_meters * CppAD::tan(steering) / 0.5);
//       fg[1 + cte_start + t] = cte1 - y1;
//     }
//   }
// };
//
// void ModelPredictiveController::GetControl(
//     double& target_vr, double& target_steering, const Model& m, const geometry::LineSegs& ref) {
//   target_vr = 1.0;

//   double x = m.x;
//   double y = m.y;
//   double psi = m.psi;
//   double cte = m.y;

//   const int kStateSize = 4;
//   int n_vars = kStateSize * kSteps + (kSteps - 1);
//   int n_constraints = kStateSize * kSteps;

//   // Initial value of the independent variables.
//   // SHOULD BE 0 besides initial state.
//   Dvector vars(n_vars);
//   for (int i = 0; i < n_vars; i++) {
//     vars[i] = 0;
//   }

//   vars[x_start] = x;
//   vars[y_start] = y;
//   vars[psi_start] = psi;
//   vars[cte_start] = cte;

//   Dvector vars_lowerbound(n_vars);
//   Dvector vars_upperbound(n_vars);
//   for (int i = 0; i < steering_start; i++) {
//     vars_lowerbound[i] = -1.0e19;
//     vars_upperbound[i] = 1.0e19;
//   }

//   // For steering. [-60, 60] in radians
//   for (int i = steering_start; i < n_vars; i++) {
//     vars_lowerbound[i] = -0.6;
//     vars_upperbound[i] = 0.6;
//   }

//   // Lower and upper limits for the constraints
//   // Should be 0 besides initial state.
//   Dvector constraints_lowerbound(n_constraints);
//   Dvector constraints_upperbound(n_constraints);
//   for (int i = 0; i < n_constraints; i++) {
//     constraints_lowerbound[i] = 0;
//     constraints_upperbound[i] = 0;
//   }
//   constraints_lowerbound[x_start] = x;
//   constraints_lowerbound[y_start] = y;
//   constraints_lowerbound[psi_start] = psi;
//   constraints_lowerbound[cte_start] = cte;

//   constraints_upperbound[x_start] = x;
//   constraints_upperbound[y_start] = y;
//   constraints_upperbound[psi_start] = psi;
//   constraints_upperbound[cte_start] = cte;

//   FG_eval fg_eval;

//   // options for IPOPT solver
//   std::string options;
//   options += "Integer print_level  0\n";
//   options += "Sparse  true        forward\n";
//   options += "Sparse  true        reverse\n";
//   options += "Numeric max_cpu_time          0.5\n";

//   CppAD::ipopt::solve_result<Dvector> solution;

//   CppAD::ipopt::solve<Dvector, FG_eval>(
//       options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
//       constraints_upperbound, fg_eval, solution);

//   // Check some of the solution values
//   if (solution.status == CppAD::ipopt::solve_result<Dvector>::success) {
//     std::cout << "ipopt::solve succeed." << std::endl;
//   } else {
//     std::cout << "ipopt::solve failed." << std::endl;
//   }

//   target_steering = solution.x[steering_start];
// }