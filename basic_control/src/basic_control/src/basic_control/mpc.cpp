#include "mpc.h"

#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

#include <Eigen/Eigen>

namespace {
// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd PolyFit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

double PolyEval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

const double kModelLen = 0.5;
int kSteps = 20;
int x_start;
int y_start;
int psi_start;
int cte_start;
int psie_start;
int steering_start;

using CppAD::AD;
class FG_eval {
  double step_len_meters = 0.02;
  Eigen::VectorXd coeffs;
 public:
  FG_eval(Eigen::VectorXd coeffients) {
    this->coeffs = coeffients;
  };

  typedef std::vector<AD<double>> ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    fg[0] = 0;

    // Adding Cost section
    for (int t = 0; t < kSteps; t++) {
      fg[0] += 10 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 5 * CppAD::pow(vars[psie_start + t], 2);
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
    fg[1 + psie_start] = vars[psie_start];

    // Create predicted trajectory. Its length depends on N.
    for (int t = 1; t < kSteps; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> psie1 = vars[psie_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> psie0 = vars[psie_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> steering = vars[steering_start + t - 1];

      AD<double> y_target = coeffs[3] * CppAD::pow(x0, 3) + coeffs[2] * CppAD::pow(x0, 2) + coeffs[1] * x0 + coeffs[0];
      AD<double> derivative = 3 * coeffs[3] * CppAD::pow(x0, 2) + 2 * coeffs[2] * x0 + coeffs[1];
      AD<double> psi_target = CppAD::atan(derivative);

      // Get next values.
      fg[1 + x_start + t] = x1 - (x0 + step_len_meters * CppAD::cos(psi0));
      fg[1 + y_start + t] = y1 - (y0 + step_len_meters * CppAD::sin(psi0));
      fg[1 + psi_start + t] = psi1 - (psi0 + step_len_meters * CppAD::tan(steering) / kModelLen);
      fg[1 + cte_start + t] = cte1 - (y_target - (y0 + step_len_meters * CppAD::sin(psie0)));
      fg[1 + psie_start + t] = psie1 - (psi_target - (psi0 + step_len_meters * CppAD::tan(steering) / kModelLen));
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
  psie_start = cte_start + kSteps;
  steering_start = psie_start + kSteps;
}

void ModelPredictiveController::GetControl(
    double& target_vr, double& target_steering, const Model& m, const geometry::LineSegs& ref) {
  target_vr = 1.0;

  geometry::LineSegs truncated_ref = ref.GetSegmentsOfLen(m.x, m.y, 5);
  geometry::LineSegs truncated_ref_in_body = RefLineWorldToBody(truncated_ref, {m.x, m.y, m.psi});

  int waypoint_count = truncated_ref_in_body.segs.size();
  Eigen::VectorXd x_values(waypoint_count), y_values(waypoint_count);
  for (int i = 0; i < waypoint_count; i++) {
    x_values[i] = truncated_ref_in_body.segs[i].x;
    y_values[i] = truncated_ref_in_body.segs[i].y;
  }
  Eigen::VectorXd coefficients = PolyFit(x_values, y_values, 3);
  double x = 0;
  double y = 0;
  double psi = 0;
  // error = target_value - actual_value
  double cte = PolyEval(coefficients, 0);
  double psie = std::atan(coefficients[1]);

  const int kStateSize = 5;
  const int n_vars = kStateSize * kSteps + (kSteps - 1);
  const int n_constraints = kStateSize * kSteps;

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
  vars[psie_start] = psie;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for (int i = 0; i < steering_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // For steering. [-45, 45] in radians
  for (int i = steering_start; i < n_vars; i++) {
    vars_lowerbound[i] = -DegreeToRadian(45);
    vars_upperbound[i] = DegreeToRadian(45);
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
  constraints_lowerbound[psie_start] = psie;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[psie_start] = psie;

  // TODO: make this static.
  FG_eval fg_eval(coefficients);

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
    target_steering = solution.x[steering_start];
  } else {
    target_steering = 0;
  }

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