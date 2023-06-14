#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <iostream>

#include "basic_planning/hybird_astart.h"
#include "basic_planning/map.h"
#include "basic_planning/tf_utils.h"
#include "basic_planning/types.h"
#include "rsmotion/rsmotion.h"

namespace basic_planning {
ros::Publisher marker_pub;
ros::Publisher map_pub;

void DrawRefLine(const Path& p) {
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

  for (const PlanarPose& pp : p.path) {
    geometry_msgs::Point tmp;
    tmp.x = pp.x;
    tmp.y = pp.y;
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

void DrawParkingLot(double x, double y, double yaw, int id, double scale) {
  geometry_msgs::Pose parking_lot_in_world;
  parking_lot_in_world.position.x = x;
  parking_lot_in_world.position.y = y;
  parking_lot_in_world.position.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  parking_lot_in_world.orientation.x = q.x();
  parking_lot_in_world.orientation.y = q.y();
  parking_lot_in_world.orientation.z = q.z();
  parking_lot_in_world.orientation.w = q.w();

  // Base on agent frame, draw the agent.
  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time::now();

  marker.ns = "visualization_marker";
  marker.id = id;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Pose p_agent;

  marker.pose = parking_lot_in_world;

  marker.scale.x = Model::L * scale;
  marker.scale.y = Model::W * scale;
  marker.scale.z = 0;

  marker.color.a = 1;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;

  marker.lifetime = ros::Duration(0, 0);

  marker_pub.publish(marker);
}

void RunHybirdAstar() {
  PlanarPose agent;
  agent.x = 0;
  agent.y = 0;
  agent.psi = 0;

  PlanarPose parking_lot_1;
  double parking_log_scale = 1.4;
  parking_lot_1.x = 10;
  parking_lot_1.y = 3;
  parking_lot_1.psi = M_PI_2;
  DrawParkingLot(parking_lot_1.x, parking_lot_1.y, parking_lot_1.psi, 101, parking_log_scale);
  PlanarPose parking_pose = parking_lot_1.GetParkingForwardPose();

  double tr = Model::GetTurningRadius();
  // std::cout << "turning radius: " << tr << std::endl;

  OccupancyGridMap ogm;
  ogm.SetCircle(7, 2, 0.6, 80);
  // ogm.SetLine(0, 1, 0, 6, 0.2, 80);

  HybirdAstar ha;
  Path p = ha.SearchPath(agent, parking_lot_1, ogm);
  if (p.path.empty()) {
    std::cout << "Failed to find path." << std::endl;
  }

  double kFreqHz = 10;
  ros::Rate rate(kFreqHz);

  int index = 0;
  int path_size = p.path.size();
  while (ros::ok()) {
    rate.sleep();

    auto cur_pose = p.path[index++ % path_size];
    DrawAgent(cur_pose.x, cur_pose.y, cur_pose.psi);

    auto m = ogm.ToRosMap();
    map_pub.publish(m);
    DrawRefLine(p);
    DrawParkingLot(parking_lot_1.x, parking_lot_1.y, parking_lot_1.psi, 101, parking_log_scale);
  }
}

void RunMapDemo() {
  double kFreqHz = 10;

  PlanarPose agent;
  agent.x = 0;
  agent.y = 0;
  agent.psi = 0;

  OccupancyGridMap ogm;
  ogm.SetSquare(3, 2, 0.5, 80);
  ogm.SetRectangular(3.25, 2.25, 8, 8, 80);
  ogm.SetCircle(-3, -2, 1, 80);
  ogm.SetLine(4, 3, M_PI / 6.0, 3 * std::sqrt(2), 0.07, 80);

  ros::Rate rate(kFreqHz);
  while (ros::ok()) {
    rate.sleep();

    auto m = ogm.ToRosMap();
    map_pub.publish(m);
    DrawAgent(agent.x, agent.y, agent.psi);
  }
}

void RunDrawReedsSheppPath() {
  double kFreqHz = 10;

  PlanarPose parking_lot_1;
  double parking_log_scale = 1.4;
  parking_lot_1.x = 10;
  parking_lot_1.y = 3;
  parking_lot_1.psi = M_PI_2;
  DrawParkingLot(parking_lot_1.x, parking_lot_1.y, parking_lot_1.psi, 101, parking_log_scale);
  PlanarPose parking_pose = parking_lot_1.GetParkingForwardPose();

  double tr = Model::GetTurningRadius();

  // std::cout << "turning radius: " << tr << std::endl;

  PlanarPose from(5.406715, 1.319485, 0.3000000);
  // PlanarPose from(0, 0, 0);
  Path rs_path = GenRsPath(from, parking_pose);

  ros::Rate rate(kFreqHz);
  while (ros::ok()) {
    rate.sleep();

    DrawParkingLot(parking_lot_1.x, parking_lot_1.y, parking_lot_1.psi, 101, parking_log_scale);
    DrawRefLine(rs_path);
  }
}

void RunDrawReedsSheppProgress() {
  double kFreqHz = 10;

  PlanarPose parking_lot_1;
  double parking_log_scale = 1.4;
  parking_lot_1.x = 10;
  parking_lot_1.y = 3;
  parking_lot_1.psi = M_PI_2;
  DrawParkingLot(parking_lot_1.x, parking_lot_1.y, parking_lot_1.psi, 101, parking_log_scale);

  PlanarPose parking_pose = parking_lot_1.GetParkingForwardPose();
  // PlanarPose parking_pose = parking_lot_1.GetParkingBackwardPose();

  double tr = Model::GetTurningRadius();

  // std::cout << "turning radius: " << tr << std::endl;
  
  rsmotion::algorithm::State fromState(1, 1, 0.3, false);
  rsmotion::algorithm::State toState(parking_pose.x / tr, parking_pose.y / tr, parking_pose.psi, false);
  rsmotion::algorithm::Path path = rsmotion::algorithm::SearchShortestPath(toState);

  ros::Rate rate(kFreqHz);

  int i = 0;
  while (ros::ok()) {
    rate.sleep();
    i = (i + 1) % 101;
    // i = std::min(100, i);
    float progress = i * 0.01;
    rsmotion::algorithm::State cur = rsmotion::algorithm::InterpolateNormalized(fromState, path, progress);

    DrawAgent(cur.X() * tr, cur.Y() * tr, cur.Phi());
    DrawParkingLot(parking_lot_1.x, parking_lot_1.y, parking_lot_1.psi, 101, parking_log_scale);
  }
}
}  // namespace basic_planning

int main(int argc, char** argv) {
  ros::init(argc, argv, "basic_planning");
  ros::NodeHandle nh;
  basic_planning::marker_pub = nh.advertise<visualization_msgs::Marker>("/basic_planning", 1);
  basic_planning::map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/basic_planning_map", 1);

  // basic_planning::RunDrawReedsSheppPath();
  // basic_planning::RunDrawReedsSheppProgress();
  // basic_planning::RunMapDemo();
  basic_planning::RunHybirdAstar();
  return 0;
}