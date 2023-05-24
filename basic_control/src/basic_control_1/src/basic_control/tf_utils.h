/**
 * 
 *
 * 
 * tf2 overall
 * http://docs.ros.org/en/melodic/api/tf2_geometry_msgs/html/c++/namespacetf2.html
 * tf2 transforms
 * http://docs.ros.org/en/indigo/api/tf2/html/classtf2_1_1Transform.html
 * 
 * 
 */

#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace basic_control {


inline double GetYawFromQuaternion(const tf2::Quaternion &q) {
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

inline double GetYawFromQuaternionMsg(const geometry_msgs::Quaternion &q) {
    tf2::Quaternion tmp;
    tf2::fromMsg(q, tmp);
    return GetYawFromQuaternion(tmp);
}

inline tf2::Quaternion CreateQuaternionFromYaw(double yaw) {
    tf2::Quaternion rtn;
    rtn.setRPY(0, 0, yaw);
    return rtn;
}

inline geometry_msgs::Quaternion CreateQuaternionMsgFromYaw(double yaw) {
    return tf2::toMsg(CreateQuaternionFromYaw(yaw));
}

geometry_msgs::Pose PoseWorldToBody(geometry_msgs::Pose p_in_world, geometry_msgs::Pose body_in_world) {
  tf2::Quaternion body_in_world_q;
  tf2::fromMsg(body_in_world.orientation, body_in_world_q);
  tf2::Vector3 origin(body_in_world.position.x, body_in_world.position.y, body_in_world.position.z);
  tf2::Transform trans(body_in_world_q, origin);
  tf2::Transform inv_trans = trans.inverse();
  geometry_msgs::TransformStamped inv_trans_stamped

  geometry_msgs::Pose p_in_body;
  tf2::doTransform(p_in_world, p_in_body, inv_trans);

  return p_in_body;
}

geometry_msgs::Pose PoseBodyToWorld(geometry_msgs::Pose p_in_body, geometry_msgs::Pose body_in_world) {
  tf2::Quaternion body_in_world_q;
  tf2::fromMsg(body_in_world.orientation, body_in_world_q);
  tf2::Vector3 origin(body_in_world.position.x, body_in_world.position.y, body_in_world.position.z);
  tf2::Transform trans(body_in_world_q, origin);

  geometry_msgs::Pose p_in_world;
  tf2::doTransform(p_in_body, p_in_world, trans);
  return p_in_world;
}

geometry_msgs::Pose XYYawToPose(double x, double y, double yaw) {
  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  p.position.z = 0;
  p.orientation = CreateQuaternionMsgFromYaw(yaw);
  return p;
}

void PoseToXYYaw(geometry_msgs::Pose p, double& x, double& y, double& yaw) {
  x = p.position.x;
  y = p.position.y;
  yaw = GetYawFromQuaternionMsg(p.orientation);
  return;
}

}