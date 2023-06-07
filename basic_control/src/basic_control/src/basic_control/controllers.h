#pragma once

#include "utils.h"
#include "model.h"

#include <algorithm>
#include <vector>
#include <map>

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
  
  // Not used for now.
  double previous_deriv_error;
  
  double control;
  double dt;

  double alpha;
};

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

  void GetControl(double& target_vr, double& target_steering, const Model& m, const geometry::LineSegs& ref) {
    target_vr = 1.0;
    double cte = GetCrossTrackError(m.x, m.y, ref);
    pid_p->UpdateControl(cte);
    target_steering = pid_p->GetControl();
  }

  // Data.
  PID* pid_p;
};

struct PurePursuitController {
  PurePursuitController() = default;
  void GetControl(double& target_vr, double& target_steering, const Model& m, const geometry::LineSegs& ref) {
    target_vr = 1.0;
    double lookahead_dist = target_vr * 2.0;
    geometry::Point2 cur(m.x, m.y);
    geometry::Point2 target = GetTargetPoint(m, ref, lookahead_dist);
    double ld = geometry::LengthOfVector2(target - cur);
    double angle = geometry::ShortestAngularDistance(m.psi, atan2(target.y - cur.y, target.x - cur.x));
    target_steering = std::atan2(2.0 * Model::L * std::sin(angle), ld);
  }

  static geometry::Point2 GetTargetPoint(const Model& m, const geometry::LineSegs& ref, double lookahead_dist) {
    int min_index = ref.GetClosestPointIndex(m.x, m.y);
    return ref.ExtendFromIndex(min_index, lookahead_dist);
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

    // Note that here error is defined as (actual value) - (target value).
    // TODO: rewrite the code to so error is (target value) - (actual value).

    // x_new = A * x + B * u;
    // cost = xT * Q * x + uT * R * u;
    // x = [cross_track_error, psi_error].transpose()
    // u = [<any>, tan(steering)].transpose()
    // psi_error is assumed to be small, so sin(a) = a, this makes the model linear.
    // cross_track_error = cross_track_error + sin(psi_error) * step
    // psi_error = psi_error + step_size / L * tan(steering)
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

  void GetControl(double& target_vr, double& target_steering, const Model& m, const geometry::LineSegs& ref) {
    target_vr = 1.0;
    
    double c = GetCurvature(m, ref);
    double sign = c > 0 ? 1.0 : -1.0;
    double feed_forward_steering = sign * Model::GetSteeringFromCurvature(std::abs(c));

    double cte, psi_e;
    GetError(m, ref, cte, psi_e);
    Eigen::Vector2d x;
    x << cte, psi_e;

    std::vector<Eigen::Vector2d> state_seq;
    std::vector<Eigen::Vector2d> ctl_seq = lqr_p->GetControl(x, state_seq);
    double lqr_steering = std::atan(ctl_seq.front()(1));

    target_steering = feed_forward_steering + lqr_param * lqr_steering;
  }

  // Note that error here is actual_value - target_value.
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

// Predict agent state with a model, search the action sequence space to find a action sequence
// that minimize the cost.
struct ModelBasedSearchState {
  double x, y, psi, steering;
  int parent_index;
  double path_cost;

  bool operator < (const ModelBasedSearchState& o) const {
    return path_cost < o.path_cost;
  }
};

namespace {
const int kMbsSearchSteps = 35;
const int kMbsSearchStepSize = 4000;
const int kMbsSteeringSteps = 11;
const double kMbsGridBinSize = 0.05;
// Corresponds to 4 degree.
const double kMbsPsiBinSize = 0.07; 

ModelBasedSearchState g_state[kMbsSearchSteps][kMbsSearchStepSize];
}

struct ModelBasedSearch {
  double freq;
  double steering_change_limit; 
  double steering_step;
  double move_step;
  double search_lb;

  // ModelBasedSearchState* states_p;

  int step_elem_count[kMbsSearchSteps];

  // the array size corresponds to floor(2 * M_PI / kMbsPsiBinSize) + 1.
  std::map<int, std::map<int, uint8_t[91]>> bins;

  ModelBasedSearch(double freq) : freq(freq) {
    // steering_change_limit = Model::steering_change_rate_limit_per_sec * (1.0 / freq);
    steering_change_limit = 0.3;
    steering_step = steering_change_limit / 5.0;

    move_step = 0.2;
    
    search_lb = -1.0 * steering_change_limit;

    // states_p = (ModelBasedSearchState*)malloc(kMbsSearchSteps * kMbsSearchStepSize * sizeof(ModelBasedSearchState));
  }

  ~ModelBasedSearch() {
    // free(states_p);
  }

  ModelBasedSearchState& GetState(int index_0, int index_1) {
    // return states_p[index_0 * kMbsSearchStepSize + index_1];
    return g_state[index_0][index_1];
  }

  void GetControl(double& target_vr, double& target_steering,
      std::vector<geometry::Point2>& points, const Model& m, const geometry::LineSegs& ref) {
    auto start_ms = GetTimeStampMs();
    target_vr = 1.0;

    bins.clear();

    // Prepare to search.
    ModelBasedSearchState start_state;
    start_state.x = m.x;
    start_state.y = m.y;
    start_state.psi = m.psi;
    start_state.steering = m.steering;
    start_state.parent_index = -1;
    start_state.path_cost = 0;
    GetState(0, 0) = start_state;

    step_elem_count[0] = 1;

    // TODO: handle the case of reference line ends.
    for (int step_index = 0; step_index < kMbsSearchSteps - 1; step_index++) {
      Expend(step_index, ref);
    }

    target_steering = BackwardTrack(kMbsSearchSteps - 1, 0, points);
    auto end_ms = GetTimeStampMs();
    // std::cout << "points len: " << points.size() << std::endl;
    std::cout << "elapsed time ms: " << end_ms - start_ms << std::endl;
  }

  // Returns true for a successful insertion, false if the bin is occupied.
  bool InsertToGrid(int index_x, int index_y, int index_z) {
    if (bins.find(index_x) == bins.end()) {
      memset(bins[index_x][index_y], 0, sizeof(bins[index_x][index_y]));
      bins[index_x][index_y][index_z] = 1;
      return true;
    }
    if (bins[index_x].find(index_y) == bins[index_x].end()) {
      memset(bins[index_x][index_y], 0, sizeof(bins[index_x][index_y]));
      bins[index_x][index_y][index_z] = 1;
      return true;
    }
    if (bins[index_x][index_y][index_z] == 1) {
      return false;
    } else {
      bins[index_x][index_y][index_z] = 1;
      return true;
    }
  }

  void Expend(int step_index, const geometry::LineSegs& ref) {
    std::vector<ModelBasedSearchState> tmp_vec;
    int state_count = step_elem_count[step_index];
    for (int i = 0; i < state_count; i++) {
      const ModelBasedSearchState& src = GetState(step_index, i);
      std::vector<geometry::Point2> near_points = GetNearPoints(src.x, src.y, ref);
      geometry::Vector2 tangent_vec = near_points[1] - near_points[0];
      double psi_ref = std::atan2(tangent_vec.y, tangent_vec.x);
      for (int j = 0; j < kMbsSteeringSteps; j++) {
        ModelBasedSearchState tmp = src;
        tmp.steering = src.steering + double(j) * steering_step + search_lb;
        tmp.psi = tmp.psi + move_step * std::tan(tmp.steering) / Model::L;
        tmp.x = tmp.x + move_step * std::cos(tmp.psi);
        tmp.y = tmp.y + move_step * std::sin(tmp.psi);

        tmp.path_cost = src.path_cost + GetClosestPointDistSqr(near_points, tmp.x, tmp.y);
        tmp.parent_index = i;

        if (std::abs(geometry::GetRotationValue(tmp.psi, psi_ref)) > (M_PI / 3)) {
          continue;
        }

        int grid_index_x = std::floor(tmp.x / kMbsGridBinSize);
        int grid_index_y = std::floor(tmp.y / kMbsGridBinSize);
        int grid_index_psi = std::floor(geometry::NormalizeAnglePositive(tmp.psi) / kMbsPsiBinSize);
        bool insertion_succeed = InsertToGrid(grid_index_x, grid_index_y, grid_index_psi);
        if (!insertion_succeed) {
          continue;
        }
        tmp_vec.push_back(tmp);
      }
    }
    std::sort(tmp_vec.begin(), tmp_vec.end());

    for (int i = 0; i < tmp_vec.size() - 1; i++) {
      if (tmp_vec[i].path_cost > tmp_vec[i + 1].path_cost) {
        printf("error 1\n");
        exit(0);
      }
    }
    
    for (int i = 0; i < std::min(kMbsSearchStepSize, int(tmp_vec.size())); i++) {
      // states[step_index + 1][i] = tmp_vec[i];
      GetState(step_index + 1, i) = tmp_vec[i];
    }
    step_elem_count[step_index + 1] = std::min(kMbsSearchStepSize, int(tmp_vec.size()));
  }

  double GetClosestPointDistSqr(const std::vector<geometry::Point2>& near_points, double x, double y) {
    if (near_points.size() == 1) {
      double dx = near_points[0].x - x;
      double dy = near_points[0].y - y;
      return dx * dx + dy * dy;
    }

    double rtn = DBL_MAX / 2;
    for (int i = 0; i < near_points.size() - 1; i++) {
      geometry::Point2 p(x, y);
      geometry::Point2 a = near_points[i];
      geometry::Point2 b = near_points[i + 1];
      rtn = std::min(rtn, geometry::DistanceToSegment(p, a, b));
    }
    return rtn;
  }

  std::vector<geometry::Point2> GetNearPoints(double x, double y, const geometry::LineSegs& ref) {
    std::vector<geometry::Point2> rtn;
    int index = ref.GetClosestPointIndex(x, y);
    if (index >= 2) {
      rtn.push_back(ref.segs[index - 2]);
    }
    if (index >= 1) {
      rtn.push_back(ref.segs[index - 1]);
    }
    rtn.push_back(ref.segs[index]);
    if (index < ref.segs.size() - 1) {
      rtn.push_back(ref.segs[index + 1]);
    }
    if (index < ref.segs.size() - 2) {
      rtn.push_back(ref.segs[index + 2]);
    }
    return rtn;
  }

  double BackwardTrack(int step_index, int index, std::vector<geometry::Point2>& points) {
    if (step_index == 3) {
      // Note the choice of step_index.
      points.push_back(geometry::Point2(GetState(step_index, index).x, GetState(step_index, index).y));
      return GetState(step_index, index).steering;
    } else {
      auto& tmp_state = GetState(step_index, index);
      points.push_back(geometry::Point2(tmp_state.x, tmp_state.y));
      return BackwardTrack(step_index - 1, GetState(step_index, index).parent_index, points);
    }
  }
};

}