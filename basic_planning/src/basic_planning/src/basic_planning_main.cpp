#include "rsmotion/rsmotion.h"
#include "basic_planning/types.h"
#include "basic_planning/tf_utils.h"

#include <iostream>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

namespace basic_planning {
ros::Publisher marker_pub;

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

  marker.ns = "visualization_marker";
  marker.id = id;

  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  geometry_msgs::Pose p_agent;

  marker.pose = parking_lot_in_world;

  marker.scale.x = Model::L * scale;
  marker.scale.y = Model::W * scale;
  marker.scale.z = 0 ;

  marker.color.a = 1;
  marker.color.r = 0.5;
  marker.color.g = 0.5;
  marker.color.b = 0.5;

  marker.lifetime = ros::Duration(0, 0);

  marker_pub.publish(marker);
}

void RunDrawReedsSheppPath() {
  double kFreqHz = 10;

  PlanarPose agent;
  DrawAgent(agent.x, agent.y, agent.psi);

  PlanarPose parking_lot_1;
  double parking_log_scale = 1.4;
  parking_lot_1.x = -Model::L / 2;
  parking_lot_1.y = 1.4;
  parking_lot_1.psi = 0;
  DrawParkingLot(parking_lot_1.x, parking_lot_1.y, parking_lot_1.psi, 1, parking_log_scale);

  // PlanarPose parking_pose = parking_lot_1.GetParkingForwardPose();
  PlanarPose parking_pose = parking_lot_1.GetParkingBackwardPose();

  double tr = Model::GetTurningRadius();
  
  std::cout << "turning radius: " << tr << std::endl;

  rsmotion::algorithm::State fromState(0, 0, 0, false);
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
    DrawParkingLot(parking_lot_1.x, parking_lot_1.y, parking_lot_1.psi, 1, parking_log_scale);
  }
}
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "basic_planning");
  ros::NodeHandle nh;
  basic_planning::marker_pub = nh.advertise<visualization_msgs::Marker>("/basic_planning", 1);

  basic_planning::RunDrawReedsSheppPath();
  return 0;
}