#pragma once

#include "utils.h"
#include "model.h"

#include <algorithm>
#include <vector>

#include <Eigen/Eigen>

namespace basic_control {

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
    double xte = GetCrossTrackError(m, ref);
    pid_p->UpdateControl(xte);
    target_delta_f = pid_p->GetControl();
  }

  static double GetSign(const geometry::Point2& p, const geometry::Point2& a, const geometry::Point2& b) {
    geometry::Vector2 v_base = p - a, v_target = b - a;
    double v = CrossProduct(v_base, v_target);
    return v > 0 ? 1.0 : -1.0;
  }

  static double GetCrossTrackError(const Model& m, const geometry::LineSegs& ref) {
    geometry::Point2 p(m.x, m.y);
    int size = ref.segs.size();
    if (size < 2) {
      // TODO: print the line number here.
      printf("Invalid condition.\n");
      assert(false);
      exit(0);
    }
    
    double xte = DBL_MAX / 2;
    double sign = 1.0;
    for (int i = 0; i < size - 1; i++) {
      const geometry::Point2& a = ref.segs[i];
      const geometry::Point2& b = ref.segs[i + 1];
      double tmp_dist = geometry::DistanceToSegment(p, a, b);
      if (xte > tmp_dist) {
        xte = tmp_dist;
        sign = GetSign(p, a, b);
      }

    }
    return xte * sign;
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

// Here is a LQR implementation for a specific model.
struct LQR {
  static constexpr int kIterations = 20;
  LQR() = default;

  std::vector<Eigen::Vector2d> GetControl(Eigen::Matrix2d A,
                             Eigen::Matrix2d B,
                             Eigen::Matrix2d Q, 
                             Eigen::Matrix2d R,
                             Eigen::Vector2d x) {
    Eigen::Matrix2d prev_p = Q;
    std::vector<Eigen::Vector2d> control_seq;
    std::vector<Eigen::Vector2d> state_seq;
    std::vector<Eigen::Matrix2d> k_seq;
    
    for (int i = 0; i < kIterations; i++) {
      // Eigen::Matrix2d tmp_0 = R + B.transpose() * prev_p * B;
      // std::cout << "tmp_0: " << tmp_0 << std::endl << std::endl;
      Eigen::Matrix2d new_k = - (R + B.transpose() * prev_p * B).inverse() * B.transpose() * prev_p * A;
      // std::cout << "new_k: " << new_k << std::endl << std::endl;
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
      // std::cout << "control: " << control_seq.back() << std::endl << std::endl;
      Eigen::Vector2d new_s = A * tmp_s + B * k * tmp_s;
      state_seq.push_back(new_s);
    }

    return control_seq;
  }
};

struct LqrFeedForward {
  LqrFeedForward() = default;
  void GetControl(double& target_vr, double& target_delta_f, const Model& m, const geometry::LineSegs& ref) {
    target_vr = 1.0;
    
    double c = GetCurvature(m, ref);
    double feed_forward_delta_f = Model::GetDeltaFFromCurvature(c);

    // x_new = A * x + B * u;
    // cost = xT * Q * x + uT * R * u;
    // x = [cross_track_error, psi_error]T
    // u = [<any>, delta_f]T
    // delta_f is assumed to be small, so sin(a) = a, cos(a) = 1 - (a^2)/2, tan(a) = a,
    // this makes the model linear.
    double step_len = 0.1;
    Eigen::Matrix2d A;
    A << 1, step_len, 0, 1;
    Eigen::Matrix2d B;
    B << 0, 0, 0, 0.2;
    Eigen::Matrix2d Q;
    Q << 3, 0, 0, 3;
    Eigen::Matrix2d R;
    R << 1, 0, 0, 3;

    double cte, psi_e;

    Eigen::Vector2d x;
    x << m.y, m.psi;

    return;
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
    geometry::Vector2 tangent_vec = p_next - p_prev;
    double angle = std::atan2(tangent_vec.y, tangent_vec.x);
    // RotateVector2
    
    psi_e = m.psi - std::atan2(tangent_vec.y, tangent_vec.x);
  }

  static double GetCurvature(const Model& m, const geometry::LineSegs& ref) {
    int min_index = ref.GetClosestPointIndex(m.x, m.y);

    int segs_size = ref.segs.size();
    if (min_index == 0 || min_index == (segs_size - 1)) {
      return 0;
    }
    
    return geometry::GetCurvature(ref.segs[min_index - 1], ref.segs[min_index], ref.segs[min_index + 1]);
  }

};

}