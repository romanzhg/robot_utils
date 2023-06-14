#include "basic_control/geometry.h"
#include "basic_control/utils.h"
#include "basic_control/tf_utils.h"
#include "basic_control/controllers.h"
#include "basic_control/mpc.h"

#include <cmath>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <Eigen/Eigen>

namespace basic_control {

ros::Publisher marker_pub;

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
  line_strip.header.stamp = ros::Time::now();

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
  line_strip.header.stamp = ros::Time::now();

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

void DrawCtrlTrajectory(const geometry::LineSegs& ls) {
  if (ls.segs.size() < 2) {
    return;
  }

  visualization_msgs::Marker line_strip;
  line_strip.header.frame_id = "world";
  line_strip.header.stamp = ros::Time::now();

  line_strip.ns = "visualization_marker";
  line_strip.action = visualization_msgs::Marker::ADD;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = 3;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_strip.scale.x = 0.1;

  line_strip.color.a = 0.8;
  line_strip.color.r = 0;
  line_strip.color.g = 1.0;
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

  geometry_msgs::Pose p_agent;

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

// Another way to draw agent, do transform by hand.
void DrawAgentAlt(double x, double y, double yaw) {
  geometry_msgs::Pose p_in_body;
  geometry_msgs::Pose body_in_world;

  p_in_body.position.x = Model::L / 2;
  p_in_body.position.y = 0;
  p_in_body.position.z = Model::Height / 2;
  p_in_body.orientation.x = 0.0;
  p_in_body.orientation.y = 0.0;
  p_in_body.orientation.z = 0.0;
  p_in_body.orientation.w = 1.0;

  body_in_world.position.x = x;
  body_in_world.position.y = y;
  body_in_world.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  body_in_world.orientation.x = q.x();
  body_in_world.orientation.y = q.y();
  body_in_world.orientation.z = q.z();
  body_in_world.orientation.w = q.w();

  geometry_msgs::Pose p_in_world = PoseBodyToWorld(p_in_body, body_in_world);

  // Base on agent frame, draw the agent.
  visualization_msgs::Marker marker;
  marker.header.stamp = ros::Time::now();
  marker.header.frame_id = "world";

  marker.ns = "visualization_marker";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Pose p_agent;

  marker.pose = p_in_world;

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

void RunCrossTrackErrorCtrl() {
  double kFreqHz = 50;
  CrossTrackErrorController controller(kFreqHz);
  
  geometry::LineSegs ls;
  GenRefLine(ls);
  geometry::LineSegs agent_trajectory;
  Model m;

  agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

  ros::Rate rate(kFreqHz);
  while (ros::ok()) {
    DrawRefLine(ls);

    DrawTrajectory(agent_trajectory);
    DrawAgent(m.x, m.y, m.psi);

    double target_vr, target_steering;
    controller.GetControl(target_vr, target_steering, m, ls);
    m.SetCommand(target_vr, target_steering);
    m.Update(1 / kFreqHz);
    agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

    rate.sleep();
  }
}

void RunPurePursuitCtrl() {
  double kFreqHz = 20;

  PurePursuitController controller;

  geometry::LineSegs ls;
  GenRefLine(ls);
  geometry::LineSegs agent_trajectory;
  Model m;

  agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

  ros::Rate rate(kFreqHz);
  while (ros::ok()) {
    DrawRefLine(ls);

    DrawTrajectory(agent_trajectory);
    DrawAgent(m.x, m.y, m.psi);

    double target_vr, target_steering;
    controller.GetControl(target_vr, target_steering, m, ls);
    m.SetCommand(target_vr, target_steering);
    m.Update(1 / kFreqHz);
    agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

    rate.sleep();
  }
}

// Draw the agent/control trajectory of a single LQR run.
void RunLqrDrawSeq() {
  double kFreqHz = 20;
  double velocity = 1.0;
  double step_len = velocity * (1 / kFreqHz);

  LQR lqr(step_len);

  double y_diff = 2;
  double psi_diff = 0;
  geometry::LineSegs agent_trajectory;
  geometry::LineSegs control_trajectory;
  Model m;
  m.y = y_diff;
  m.psi = psi_diff;

  agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

  double tmp_x = 0;
  control_trajectory.segs.push_back(geometry::Point2(0, m.y));

  
  Eigen::Vector2d x;
  x << y_diff, psi_diff;

  std::vector<Eigen::Vector2d> state_seq;
  std::vector<Eigen::Vector2d> ctl_seq = lqr.GetControl(x, state_seq);
  std::cout << ctl_seq.size() << " " << state_seq.size() << std::endl;
  for (int i = 0; i < ctl_seq.size(); i++) {
    Eigen::Vector2d ctl = ctl_seq[i];
    double target_vr, target_steering;
    target_vr = 1.0;
    target_steering = std::atan(ctl(1));
    // std::cout << "actual target_steering in rad: " << target_steering << std::endl;
    m.SetCommandWithoutLimit(target_vr, target_steering);
    m.Update(1 / kFreqHz);
    agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

    tmp_x += step_len * cos(state_seq[i + 1](1));
    control_trajectory.segs.push_back(geometry::Point2(tmp_x, state_seq[i + 1](0)));
  }
  DrawTrajectory(agent_trajectory);
  DrawCtrlTrajectory(control_trajectory);

  ros::Rate rate(kFreqHz);
  while (ros::ok()) {
    DrawTrajectory(agent_trajectory);
    DrawCtrlTrajectory(control_trajectory);
    rate.sleep();
  }
}

// LQR brings the cross track error and psi diff to 0.
// The reference line is x-axis.
void RunLqrAlone() {
  double kFreqHz = 20;
  double velocity = 1.0;
  double step_len = velocity * (1 / kFreqHz);

  LQR lqr(step_len);

  double y_diff = 0.4;
  double psi_diff = 0.2;
  geometry::LineSegs agent_trajectory;
  Model m;
  m.y = y_diff;
  m.psi = psi_diff;

  agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

  ros::Rate rate(kFreqHz);
  while (ros::ok()) {
    DrawTrajectory(agent_trajectory);
    DrawAgent(m.x, m.y, m.psi);

    double target_vr, target_steering;
    target_vr = velocity;
    
    Eigen::Vector2d x;
    x << m.y, m.psi;

    std::vector<Eigen::Vector2d> state_seq;
    std::vector<Eigen::Vector2d> ctl_seq = lqr.GetControl(x, state_seq);

    target_steering = std::atan(ctl_seq.front()(1));

    m.SetCommand(target_vr, target_steering);
    m.Update(1 / kFreqHz);
    agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

    rate.sleep();
  }
}

void RunLqrWithFeedForward() {
  double kFreqHz = 20;
  LqrWithFeedForward lqrff(kFreqHz);

  geometry::LineSegs ls;
  GenRefLine(ls);
  geometry::LineSegs agent_trajectory;
  Model m;
  m.y = -0.2;

  agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

  ros::Rate rate(kFreqHz);
  while (ros::ok()) {
    DrawRefLine(ls);

    DrawTrajectory(agent_trajectory);
    DrawAgent(m.x, m.y, m.psi);

    double target_vr, target_steering;
    lqrff.GetControl(target_vr, target_steering, m, ls);
    m.SetCommand(target_vr, target_steering);
    m.Update(1 / kFreqHz);
    agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

    rate.sleep();
  }
}

void RunModelBasedCtrl() {
  double kFreqHz = 20;
  ModelBasedSearch mbs(kFreqHz);

  geometry::LineSegs ls;
  GenRefLine(ls);
  geometry::LineSegs agent_trajectory;
  Model m;
  m.y = -0.2;

  agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

  ros::Rate rate(kFreqHz);
  while (ros::ok()) {
    rate.sleep();

    DrawRefLine(ls);

    DrawTrajectory(agent_trajectory);
    DrawAgent(m.x, m.y, m.psi);

    double target_vr, target_steering;
    std::vector<geometry::Point2> points;
    points.clear();
    geometry::LineSegs plan_trajectory;
    plan_trajectory.segs.clear();
    mbs.GetControl(target_vr, target_steering, points, m, ls);
    m.SetCommand(target_vr, target_steering);
    m.Update(1 / kFreqHz);
    agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));

    plan_trajectory.segs = points;
    DrawCtrlTrajectory(plan_trajectory);
  }
}

void RunMpc() {
  double kFreqHz = 20;

  ModelPredictiveController mpc;

  geometry::LineSegs ls;
  GenRefLine(ls);
  geometry::LineSegs agent_trajectory;
  Model m;
  m.y = -0.4;
  m.psi = -0.1;

  ros::Rate rate(kFreqHz);
  while (ros::ok()) {
    rate.sleep();

    DrawRefLine(ls);

    DrawTrajectory(agent_trajectory);
    DrawAgent(m.x, m.y, m.psi);

    double target_vr, target_steering;
    mpc.GetControl(target_vr, target_steering, m, ls);
    m.SetCommand(target_vr, target_steering);
    m.Update(1 / kFreqHz);
    agent_trajectory.segs.push_back(geometry::Point2(m.x, m.y));
  }

}
}  // namespace basic_control

int main(int argc, char** argv) {
  ros::init(argc, argv, "basic_control");
  ros::NodeHandle nh;
  basic_control::marker_pub = nh.advertise<visualization_msgs::Marker>("/basic_control", 1);

  // Uncomment to choose from different controllers.
  // basic_control::RunCrossTrackErrorCtrl();
  // basic_control::RunPurePursuitCtrl();
  // basic_control::RunLqrDrawSeq();
  // basic_control::RunLqrAlone();
  // basic_control::RunLqrWithFeedForward();
  // basic_control::RunModelBasedCtrl();
  // basic_control::RunMpc();
  return 0;
}