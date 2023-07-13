/**
 *
 * Convert svo file to ros messages.
 *
 */
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <iostream>
#include <random>
#include <sl/Camera.hpp>

using namespace std;

#define DEG2RAD 0.017453293
ros::Publisher cloud_pub;
ros::Publisher imu_pub;

inline void SleepMs(size_t ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

void PubRosMessage(const sl::Mat& point_cloud) {
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "base_link";
  msg.is_bigendian = false;
  msg.is_dense = false;

  msg.width = point_cloud.getWidth();
  msg.height = point_cloud.getHeight();

  sensor_msgs::PointCloud2Modifier modifier(msg);
  modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
                                "z", 1, sensor_msgs::PointField::FLOAT32, "rgb", 1, sensor_msgs::PointField::FLOAT32);

  sl::Vector4<float>* cpu_cloud = point_cloud.getPtr<sl::float4>();

  int pts_count = point_cloud.getWidth() * point_cloud.getHeight();
  uint data_len_bytes = 4 * pts_count * sizeof(float);

  msg.data.resize(data_len_bytes);
  void* pt_cloud_ptr = (void*)((msg.data.data()));
  memcpy(pt_cloud_ptr, (void*)cpu_cloud, 4 * pts_count * sizeof(float));

  cloud_pub.publish(msg);
}

void PubImu(sl::SensorsData sens_data) {
  sensor_msgs::Imu imu_msg;

  imu_msg.header.stamp = ros::Time::now();
  imu_msg.header.frame_id = "imu_link";

  imu_msg.orientation.x = sens_data.imu.pose.getOrientation()[0];
  imu_msg.orientation.y = sens_data.imu.pose.getOrientation()[1];
  imu_msg.orientation.z = sens_data.imu.pose.getOrientation()[2];
  imu_msg.orientation.w = sens_data.imu.pose.getOrientation()[3];

  imu_msg.angular_velocity.x = sens_data.imu.angular_velocity[0] * DEG2RAD;
  imu_msg.angular_velocity.y = sens_data.imu.angular_velocity[1] * DEG2RAD;
  imu_msg.angular_velocity.z = sens_data.imu.angular_velocity[2] * DEG2RAD;

  imu_msg.linear_acceleration.x = sens_data.imu.linear_acceleration[0];
  imu_msg.linear_acceleration.y = sens_data.imu.linear_acceleration[1];
  imu_msg.linear_acceleration.z = sens_data.imu.linear_acceleration[2];

  for (int i = 0; i < 3; ++i) {
    int r = 0;

    if (i == 0) {
      r = 0;
    } else if (i == 1) {
      r = 1;
    } else {
      r = 2;
    }

    imu_msg.orientation_covariance[i * 3 + 0] = sens_data.imu.pose_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
    imu_msg.orientation_covariance[i * 3 + 1] = sens_data.imu.pose_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
    imu_msg.orientation_covariance[i * 3 + 2] = sens_data.imu.pose_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;

    imu_msg.linear_acceleration_covariance[i * 3 + 0] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 0];
    imu_msg.linear_acceleration_covariance[i * 3 + 1] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 1];
    imu_msg.linear_acceleration_covariance[i * 3 + 2] = sens_data.imu.linear_acceleration_covariance.r[r * 3 + 2];

    imu_msg.angular_velocity_covariance[i * 3 + 0] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 0] * DEG2RAD * DEG2RAD;
    imu_msg.angular_velocity_covariance[i * 3 + 1] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 1] * DEG2RAD * DEG2RAD;
    imu_msg.angular_velocity_covariance[i * 3 + 2] = sens_data.imu.angular_velocity_covariance.r[r * 3 + 2] * DEG2RAD * DEG2RAD;
  }

  imu_pub.publish(imu_msg);
}

void SvoToRos(std::string record_name) {
  sl::Camera zed;
  sl::InitParameters init_parameters;
  init_parameters.input.setFromSVOFile(record_name.c_str());
  init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
  init_parameters.coordinate_units = sl::UNIT::METER;

  // Open the camera
  auto returned_state = zed.open(init_parameters);
  if (returned_state != sl::ERROR_CODE::SUCCESS) {
    std::cout << "Camera open failed, Exit program." << std::endl;
    return;
  }

  int svo_frame_rate = zed.getInitParameters().camera_fps;
  int nb_frames = zed.getSVONumberOfFrames();

  // std::cout << "svo frame rate: " << to_string(svo_frame_rate) << std::endl;
  // std::cout << "[Info] SVO contains " << to_string(nb_frames) << " frames" << std::endl;

  sl::RuntimeParameters runtime_parameters;
  runtime_parameters.sensing_mode = sl::SENSING_MODE::STANDARD;

  auto camera_config = zed.getCameraInformation().camera_configuration;
  sl::Mat point_cloud(camera_config.resolution, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);
  sl::SensorsData sens_data;

  zed.setSVOPosition(0);

  ros::Rate rate(15);
  while (ros::ok()) {
    std::cout << "one frame" << std::endl;
    rate.sleep();
    returned_state = zed.grab();

    if (returned_state == sl::ERROR_CODE::SUCCESS) {
      zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU, camera_config.resolution);
      zed.getSensorsData(sens_data, sl::TIME_REFERENCE::IMAGE);

      int svo_position = zed.getSVOPosition();
      zed.setSVOPosition(svo_position + 1);

      PubRosMessage(point_cloud);
      PubImu(sens_data);
    } else if (returned_state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
      break;
    } else if (returned_state == sl::ERROR_CODE::CAMERA_NOT_DETECTED) {
      std::cout << "Timeout." << std::endl;
      break;
    } else {
      std::cout << "Failed to grab image." << std::endl;
      break;
    }
  }
  zed.close();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "basic_perception");
  ros::NodeHandle nh;
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/points2", 1);
  imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1);

  SvoToRos(argv[1]);

  return 0;
}
