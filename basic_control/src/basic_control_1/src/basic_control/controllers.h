#pragma once

#include "utils.h"
#include "model.h"

#include <algorithm>
#include <vector>

#include <Eigen/Eigen>

namespace basic_control {

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
    // Note the multiply by -1.0 here.
    control = -1.0 * Kp * (current_error + Td * current_deriv_error + sum_error / Ti);
    
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

// TODO: try CrossTrackErrorController with a look ahead distance.
struct CrossTrackErrorController {
  CrossTrackErrorController(double freq) {
    double ku = 0.3;
    double tu = 2.0;
    // This controller needs kFreqHz = 50 for a reasonable performance.
    pid_p = new PID(ku * 0.6, tu / 8.0, tu * 8.0, 1.0 / freq);
  }

  ~CrossTrackErrorController() {
    delete pid_p;
  }

  void GetControl(double& target_vr, double& target_delta_f, const Model& m, const geometry::LineSegs& ref) {
    target_vr = 1.0;
    double cte = GetCrossTrackError(m.x, m.y, ref);
    pid_p->UpdateControl(cte);
    target_delta_f = pid_p->GetControl();
  }

  // Data.
  PID* pid_p;
};

struct PurePursuitController {
  PurePursuitController() = default;
  void GetControl(double& target_vr, double& target_delta_f, const Model& m, const geometry::LineSegs& ref) {
    target_vr = 1.0;
    double lookahead_dist = target_vr * 2.0;
    geometry::Point2 cur(m.x, m.y);
    geometry::Point2 target = GetTargetPoint(m, ref, lookahead_dist);
    double ld = geometry::LengthOfVector2(target - cur);
    double angle = geometry::ShortestAngularDistance(m.psi, atan2(target.y - cur.y, target.x - cur.x));
    target_delta_f = std::atan2(2.0 * Model::L * std::sin(angle), ld);
  }

  static geometry::Point2 GetTargetPoint(const Model& m, const geometry::LineSegs& ref, double lookahead_dist) {
    int min_index = ref.GetClosestPointIndex(m.x, m.y);
    return ref.ExtendFromIndex(min_index, lookahead_dist);
  }
};

struct ModelPredictiveController {
  ModelPredictiveController() = default;
  void GetControl(double& target_vr, double& target_delta_f, const Model& m, const geometry::LineSegs& ref) {
    target_vr = 1.0;
    target_delta_f = 0;
    return;
  }
};

// Note the LQR is implemented for a specific model.
struct LQR {
  static constexpr int kIterations = 100;
  Eigen::Matrix2d A;
  Eigen::Matrix2d B;
  Eigen::Matrix2d Q;
  Eigen::Matrix2d R;
  double step_len;

  LQR(double step_len) {
    this->step_len = step_len;

    // x_new = A * x + B * u;
    // cost = xT * Q * x + uT * R * u;
    // x = [cross_track_error, psi_error].transpose()
    // u = [<any>, tan(delta_f)].transpose()
    // psi_error is assumed to be small, so sin(a) = a, this makes the model linear.
    // cross_track_error = cross_track_error + sin(psi_error) * step
    // psi_error = psi_error + step_size / L * tan(delta_f) 
    A << 1, step_len, 0, 1;
    B << 0, 0, 0, step_len / Model::L;
    Q << 3, 0, 0, 3;
    R << 1, 0, 0, 3;
  }

  std::vector<Eigen::Vector2d> GetControl(Eigen::Vector2d x, std::vector<Eigen::Vector2d>& out_state_seq) {
    Eigen::Matrix2d prev_p = Q;
    std::vector<Eigen::Vector2d> control_seq;
    std::vector<Eigen::Vector2d> state_seq;
    std::vector<Eigen::Matrix2d> k_seq;
    
    for (int i = 0; i < kIterations; i++) {
      // Eigen::Matrix2d tmp_0 = R + B.transpose() * prev_p * B;
      Eigen::Matrix2d new_k = - (R + B.transpose() * prev_p * B).inverse() * B.transpose() * prev_p * A;
      k_seq.push_back(new_k);

      Eigen::Matrix2d tmp = A + B * new_k;
      Eigen::Matrix2d new_p = Q + new_k.transpose() * R * new_k + tmp.transpose() * prev_p * tmp;
      prev_p = new_p;
    }

    std::reverse(k_seq.begin(), k_seq.end());
    state_seq.push_back(x);
    for (const auto k : k_seq) {
      Eigen::Vector2d tmp_s = state_seq.back();
      control_seq.push_back(k * tmp_s);
      Eigen::Vector2d new_s = A * tmp_s + B * k * tmp_s;
      state_seq.push_back(new_s);
      // std::cout << "control: " << control_seq.back() << std::endl << std::endl;
      // std::cout << "new state: " << state_seq.back() << std::endl << std::endl;
    }

    out_state_seq = state_seq;
    return control_seq;
  }
};

struct LqrWithFeedForward {
  LqrWithFeedForward(double freq) : freq(freq) {
    double target_vr = 1.0;
    lqr_p = new LQR((1 / freq) * target_vr);
    lqr_param = 1.0;
  }

  void GetControl(double& target_vr, double& target_delta_f, const Model& m, const geometry::LineSegs& ref) {
    target_vr = 1.0;
    
    double c = GetCurvature(m, ref);
    double sign = c > 0 ? 1.0 : -1.0;
    double feed_forward_delta_f = sign * Model::GetDeltaFFromCurvature(std::abs(c));

    double cte, psi_e;
    GetError(m, ref, cte, psi_e);
    Eigen::Vector2d x;
    x << cte, psi_e;

    std::vector<Eigen::Vector2d> state_seq;
    std::vector<Eigen::Vector2d> ctl_seq = lqr_p->GetControl(x, state_seq);
    double lqr_delta_f = std::atan(ctl_seq.front()(1));

    target_delta_f = feed_forward_delta_f + lqr_param * lqr_delta_f;
  }

  // Error here is cur - target.
  static void GetError(const Model& m, const geometry::LineSegs& ref, double& cte, double& psi_e) {
    int min_index = ref.GetClosestPointIndex(m.x, m.y);
    int segs_size = ref.segs.size();
    if (min_index == 0 || min_index == (segs_size - 1)) {
      cte = 0;
      psi_e = 0;
      return;
    }

    geometry::Point2 p_prev = ref.segs[min_index - 1];
    geometry::Point2 p_ref = ref.segs[min_index];
    geometry::Point2 p_next = ref.segs[min_index + 1];

    cte = geometry::GetDist(geometry::Point2(m.x, m.y), p_ref);
    double sign = -1.0 * GetRotationSign(geometry::Point2(m.x, m.y), p_ref, p_next);
    cte = sign * cte;

    geometry::Vector2 tangent_vec = p_next - p_prev;
    double psi_ref = std::atan2(tangent_vec.y, tangent_vec.x);
    psi_e = -1.0 * geometry::GetRotationValue(m.psi, psi_ref);
  }

  static double GetCurvature(const Model& m, const geometry::LineSegs& ref) {
    int min_index = ref.GetClosestPointIndex(m.x, m.y);

    int segs_size = ref.segs.size();
    if (min_index == 0 || min_index == (segs_size - 1)) {
      return 0;
    }
    
    geometry::Point2 p_prev = ref.segs[min_index - 1];
    geometry::Point2 p_ref = ref.segs[min_index];
    geometry::Point2 p_next = ref.segs[min_index + 1];

    double curvature_value = geometry::GetCurvature(p_prev, p_ref, p_next);

    double curvature_sign = geometry::GetRotationSign(geometry::Point2(m.x, m.y), p_ref, p_next);
    return curvature_sign * curvature_value; 
  }

  LQR* lqr_p;
  double freq;
  // A value in [0, 1.0] to enable/disable lqr.
  double lqr_param;
};

}