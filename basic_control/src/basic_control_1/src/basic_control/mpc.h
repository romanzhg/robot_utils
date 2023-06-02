/**
 * Note if the program is linked with mpc.h, then needs to run
 *   export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
 * to add /usr/local/lib to the loader path.
 */
#pragma once

#include "utils.h"
#include "model.h"

#include <algorithm>
#include <vector>
#include <map>

namespace basic_control {

struct ModelPredictiveController {
  ModelPredictiveController();
  void GetControl(double& target_vr, double& target_steering, const Model& m, const geometry::LineSegs& ref);
};

}

/**

The code referenced:
https://github.com/tatsuyah/Model-Predictive-Control/blob/master/src/MPC.cpp
https://github.com/tatsuyah/Model-Predictive-Control/blob/master/src/main.cpp


#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using std::cout;
using std::endl;
using Eigen::VectorXd;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
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

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          cout << "ptsx[0]: " << ptsx[0] << " ptsy[0]: " << ptsy[0] << endl;
          cout << "ptsx[1]: " << ptsx[1] << " ptsy[1]: " << ptsy[1] << endl;
          cout << "ptsx[2]: " << ptsx[2] << " ptsy[2]: " << ptsy[2] << endl;
          cout << "ptsx[3]: " << ptsx[3] << " ptsy[3]: " << ptsy[3] << endl;
          cout << "ptsx[4]: " << ptsx[4] << " ptsy[4]: " << ptsy[4] << endl;
          cout << "ptsx[5]: " << ptsx[5] << " ptsy[5]: " << ptsy[5] << endl;
          cout << "" << endl;

          cout << "****Status****" << endl;
          cout << "px: " << px << endl;
          cout << "py: " << py << endl;
          cout << "psi: " << psi << endl;
          cout << "v: " << v << endl;

          // Affine transformation. Consider car's orientation
          int waypoints_number = ptsx.size();
          VectorXd vehicle_waypoints_x(waypoints_number);
          VectorXd vehicle_waypoints_y(waypoints_number);
          for (int i = 0; i < waypoints_number; i++) {
            double diff_x = ptsx[i] - px;
            double diff_y = ptsy[i] - py;
            vehicle_waypoints_x[i] = diff_x * cos(-psi) - diff_y * sin(-psi);
            vehicle_waypoints_y[i] = diff_y * cos(-psi) + diff_x * sin(-psi);
          }
          auto coeffs = polyfit(vehicle_waypoints_x, vehicle_waypoints_y, 3);
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          Eigen::VectorXd state(6);
          // Car's coordinate
          state << 0, 0, 0, v, cte, epsi;
          // Pass current state, reference trajectory's coefficients and get next actuator inputs
          // T is determined in MPC module
          auto vars = mpc.Solve(state, coeffs);
          steer_value = vars[0];
          throttle_value = vars[1];

          json msgJson;
          msgJson["steering_angle"] = steer_value / deg2rad(25);
          msgJson["throttle"] = throttle_value;

          cout << "steering_angle: " << steer_value / deg2rad(25) << endl;
          cout << "throttle: " << throttle_value << endl;

          //Display the MPC predicted trajectory 
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i = 2; i < vars.size(); i++) {
            if (i%2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            }
            else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (double i = 0; i < 100; i++){
            next_x_vals.push_back(i);
            next_y_vals.push_back(polyeval(coeffs, i));
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}


#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};




#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;


// The goal is to optimize the control inputs: [δ, a]

// T = N * dt should be as large as possible
size_t N = 10; 
double dt = 0.1; // This should be as small as possible

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;
double reference_v = 70;


class FG_eval {
 public:
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {

      // About fg
      // It's output argument. It's currently empty.

      // About vars
      // This contains current these variables(states, inputs) 
      // x, y, psi, v, cte, epsi, delta, a

    // fg[0] is cost value
    fg[0] = 0;

    // Adding Cost section
    for (int t = 0; t < N; t++) {
      fg[0] += 10 * CppAD::pow(vars[cte_start + t], 2);
      fg[0] += 1500 * CppAD::pow(vars[epsi_start + t], 2);
      fg[0] += CppAD::pow(vars[v_start + t] - reference_v, 2);  // Reference velocity cost
    }

    // Minimize the5use of actuators.
    for (int t = 0; t < N - 1; t++) {
      fg[0] += 2000 * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += 40 * CppAD::pow(vars[a_start + t], 2);
    }

    // Minimize the value gap between sequential actuations. It enables vehicle move smoothly
    for (int t = 0; t < N - 2; t++) {
      fg[0] += 7000 * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += 50 * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    // Initialization
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // Create predicted trajectory. Its length depends on N
    for (int t = 1; t < N; t++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start + t];
      AD<double> y1 = vars[y_start + t];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> v1 = vars[v_start + t];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];

      // The state at time t.
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi0 = vars[epsi_start + t - 1];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];

      // 3rd order polynomial
      AD<double> f0 = coeffs[3] * pow(x0, 3) + coeffs[2] * pow(x0, 2) + coeffs[1] * x0 + coeffs[0];
      AD<double> derivative = 3 * coeffs[3] * CppAD::pow(x0, 2) + 2 * coeffs[2] * x0 + coeffs[1];
      AD<double> psides0 = CppAD::atan(derivative);

      // Get next values
      double latency = 0.1;
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * (dt + latency));
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * (dt + latency));
      fg[1 + psi_start + t] = psi1 - (psi0 - v0 * delta0 / Lf * (dt + latency) );  // To turn to correct orientation
      fg[1 + v_start + t] = v1 - (v0 + a0 * (dt + latency));
      fg[1 + cte_start + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * (dt + latency)));
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * (dt + latency));
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  size_t n_vars = state.size() * N + 2 * (N-1);
  size_t n_constraints = N * state.size();

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for (int i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // For delta. [-25, 25] in radians
  for (int i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // For a
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
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
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  vector<double> result;
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);
  for (int i = 0; i < N-1; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  }
  return result;
}


*/