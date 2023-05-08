#include "basic_control/geometry.h"
#include "basic_control/utils.h"

#include <cmath>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

namespace basic_control {

ros::Publisher marker_pub;
static constexpr double kFreqHz = 50;

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

  Model() {
    x = 0;
    y = 0;
    psi = 0;

    vr = 1.0;
    delta_f = 0;

    target_vr = vr;
    target_delta_f = delta_f;

    last_update_ts = 0;
  }

  void Update(double ts) {
    double delta_time = ts - last_update_ts;
    last_update_ts = ts;
    if (delta_time > 10) {
      return;
    }

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

  double last_update_ts;
  // x, y, psi as in the world frame.
  double x, y, psi;
  double vr, delta_f;
  double target_vr, target_delta_f;
};

void GenRefLine(geometry::LineSegs& ls) {
  for (int i = 0; i < 500; i++) {
    double x = i * 0.1;
    double y = sin(x / 5.0) * x / 2.0;

    ls.segs.push_back(geometry::Point2(x, y));
  }
}

void DrawRefLine(const geometry::LineSegs& ls) {
  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.ns = "visualization_marker";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 1;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;

  line_strip.color.a = 0.8;
  line_strip.color.r = 0;
  line_strip.color.g = 0;
  line_strip.color.b = 1.0;

  line_strip.lifetime = ros::Duration(0, 0);

  for (const geometry::Point2& p : ls.segs) {
    geometry_msgs::Point tmp;
    tmp.x = p.x;
    tmp.y = p.y;
    tmp.z = 0;
    line_strip.points.push_back(tmp);
  }

  marker_pub.publish(line_strip);
}

void DrawTrajectory(const geometry::LineSegs& ls) {
  if (ls.segs.size() < 2) {
    return;
  }

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.ns = "visualization_marker";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 2;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;

  line_strip.color.a = 0.8;
  line_strip.color.r = 1.0;
  line_strip.color.g = 0;
  line_strip.color.b = 0;

  line_strip.lifetime = ros::Duration(0, 0);

  for (const geometry::Point2& p : ls.segs) {
    geometry_msgs::Point tmp;
    tmp.x = p.x;
    tmp.y = p.y;
    tmp.z = 0;
    line_strip.points.push_back(tmp);
  }

  marker_pub.publish(line_strip);
}

void DrawAgent(double x, double y, double yaw) {
  // Publish agent tf first.
  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "world";
  transformStamped.child_frame_id = "agent_frame";
  transformStamped.transform.translation.x = x;
  transformStamped.transform.translation.y = y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  br.sendTransform(transformStamped);

  // Base on agent frame, draw the agent.
  visualization_msgs::Marker marker;
  marker.header.frame_id = "agent_frame";

  marker.ns = "visualization_marker";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = Model::L / 2;
  marker.pose.position.y = 0;
  marker.pose.position.z = Model::Height / 2;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = Model::L;
  marker.scale.y = Model::W;
  marker.scale.z = Model::Height;

  marker.color.a = 0.8;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  marker.lifetime = ros::Duration(0, 0);

  marker_pub.publish(marker);
}

struct CrossTrackErrorController {
  CrossTrackErrorController() {
    double ku = 0.3;
    double tu = 2.0;
    // This controller needs kFreqHz = 50 for a reasonable performance.
    pid_p = new PID(ku * 0.6, tu / 8.0, tu * 8.0, 1.0 / kFreqHz);
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
    geometry::Point2 p(m.x, m.y);
    int size = ref.segs.size();
    
    double min_dist = DBL_MAX / 2;
    int min_index = 0;
    for (int i = 0; i < size; i++) {
      const geometry::Point2& a = ref.segs[i];
      double tmp_dist = geometry::LengthOfVector2(p - a);
      if (min_dist > tmp_dist) {
        min_dist = tmp_dist;
        min_index = i;
      }
    }
    
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

void Run() {
  geometry::LineSegs ls;
  GenRefLine(ls);
  // CrossTrackErrorController controller;
  PurePursuitController controller;
  

  geometry::LineSegs agent_trajectory;
  Model m;

  agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

  ros::Rate rate(kFreqHz);
  while (ros::ok()) {
    DrawRefLine(ls);

    DrawTrajectory(agent_trajectory);
    DrawAgent(m.x, m.y, m.psi);

    double target_vr, target_delta_f;
    controller.GetControl(target_vr, target_delta_f, m, ls);
    m.SetCommand(target_vr, target_delta_f);
    m.Update(GetTimeDouble());
    agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

    rate.sleep();
  }
}
}  // namespace basic_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "basic_control");
  ros::NodeHandle nh;
  basic_control::marker_pub = nh.advertise<visualization_msgs::Marker>("/basic_control", 1);

  basic_control::Run();
  return 0;
}