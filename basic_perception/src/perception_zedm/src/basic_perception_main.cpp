/**
 *
 * The cameral model: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
 *
 * Image size 1280×720.
 *
 *
 */
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Eigen>
#include <iostream>
#include <random>
#include <sl/Camera.hpp>

#include "basic_perception/geometry_3d.h"
#include "basic_perception/geometry_3d_utils.h"


using namespace std;
using namespace basic_perception;

ros::Publisher cloud_pub;
ros::Publisher plane_pub;

inline void SleepMs(size_t ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline int32_t GetRandomNumberInt32() {
  static std::random_device rd;
  static std::mt19937 gen(rd());
  static std::uniform_int_distribution<int32_t> distrib(0, INT32_MAX);

  return distrib(gen);
}

void DrawTriangleList(std::vector<Point3>& points, Point3 translation, Point3 norm) {
  visualization_msgs::Marker tl;
  tl.header.frame_id = "world";
  tl.header.stamp = ros::Time::now();

  tl.ns = "visualization_marker";
  tl.action = visualization_msgs::Marker::ADD;
  tl.id = 1;
  tl.type = visualization_msgs::Marker::TRIANGLE_LIST;

  tl.pose.position.x = translation.x;
  tl.pose.position.y = translation.y;
  tl.pose.position.z = translation.z;

  // Quaternion q = GetRotationFromXoY({-norm.x, -norm.y, -norm.z});
  Quaternion q = GetRotationFromXoY(norm);
  tl.pose.orientation.x = q.x;
  tl.pose.orientation.y = q.y;
  tl.pose.orientation.z = q.z;
  tl.pose.orientation.w = q.w;

  tl.scale.x = 5;
  tl.scale.y = 5;
  tl.scale.z = 1;

  tl.color.a = 0.8;
  tl.color.r = 0;
  tl.color.g = 1.0;
  tl.color.b = 0;

  tl.lifetime = ros::Duration(0, 0);

  for (const Point3& p : points) {
    geometry_msgs::Point tmp;
    tmp.x = p.x;
    tmp.y = p.y;
    tmp.z = p.z;
    tl.points.push_back(tmp);
  }

  plane_pub.publish(tl);
}

void DrawPlane(Point3 translation, Vector3 norm) {
  std::vector<Point3> plane_points;
  plane_points.push_back({-0.5, -0.5, 0});
  plane_points.push_back({0.5, -0.5, 0});
  plane_points.push_back({0.5, 0.5, 0});

  plane_points.push_back({-0.5, -0.5, 0});
  plane_points.push_back({0.5, 0.5, 0});
  plane_points.push_back({-0.5, 0.5, 0});

  DrawTriangleList(plane_points, translation, norm);
}


Point3 GetPoint(int index, const sl::Mat& point_cloud) {
  sl::float4 tmp_p;

  int row = index / 1280;
  int col = index % 1280;

  point_cloud.getValue(col, row, &tmp_p);
  return Point3(tmp_p.x, tmp_p.y, tmp_p.z);
  // Another way to get the values.
  //   float* start_ptr = (float*)(tmp_p + (row * row_l + col) * 16);
  //   float x = *start_ptr;
  //   float y = *(start_ptr + 1);
  //   float z = *(start_ptr + 2);
}

std::vector<int> GetInliners(PlaneModel plane, sl::float4* cpu_cloud, int pts_count) {
  std::vector<int> rtn;
  double dist_limit = 0.1;

  for (int i = 0; i < pts_count; i++) {
    sl::float4 tmp_p = cpu_cloud[i];
    if (not std::isfinite(tmp_p.z)) {
      continue;
    }
    Point3 p(tmp_p.x, tmp_p.y, tmp_p.z);
    double dist = DistanceToPlane(p, plane.p, plane.n);
    if (dist < dist_limit) {
      rtn.push_back(i);
    }
  }
  return rtn;
}

PlaneModel FitPlane(std::vector<int> points, sl::float4* cpu_cloud) {
  int point_count = points.size();
  Eigen::MatrixXf m(point_count, 3);
  double sum_x = 0, sum_y = 0, sum_z = 0;
  for (int i = 0; i < point_count; i++) {
    m(i, 0) = cpu_cloud[points[i]].x;
    sum_x += cpu_cloud[points[i]].x;

    m(i, 1) = cpu_cloud[points[i]].y;
    sum_y += cpu_cloud[points[i]].y;

    m(i, 2) = cpu_cloud[points[i]].z;
    sum_z += cpu_cloud[points[i]].z;
  }

  sum_x = sum_x / double(point_count);
  sum_y = sum_y / double(point_count);
  sum_z = sum_z / double(point_count);

  for (int i = 0; i < point_count; i++) {
    m(i, 0) = m(i, 0) - sum_x;
    m(i, 1) = m(i, 1) - sum_y;
    m(i, 2) = m(i, 2) - sum_z;
  }

  m.transposeInPlace();

  // For debug: https://www.cnblogs.com/wxl845235800/p/8892488.html
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(m, Eigen::ComputeThinU);
  auto U = svd.matrixU();

  Point3 center(sum_x, sum_y, sum_z);
  Vector3 normal(U(0, 2), U(1, 2), U(2, 2));
  return PlaneModel(center, normal);
}

std::vector<int> TakeSamples(const sl::Mat& point_cloud) {
  std::vector<int> rtn;
  int pts_count = point_cloud.getWidth() * point_cloud.getHeight();

  while (rtn.size() < 3) {
    int index = GetRandomNumberInt32() % pts_count;
    int row = index / 1280;
    int col = index % 1280;

    sl::float4 tmp_p;
    point_cloud.getValue(col, row, &tmp_p);
    if (std::isfinite(tmp_p.z)) {
      rtn.push_back(index);
    }
  }
  return rtn;
}

void FindPlanes(const sl::Mat& point_cloud) {
  // TODO:
  // 1. Filter on the point clould first.
  // 2. Use gpu to accelerate.
  int pts_count = point_cloud.getWidth() * point_cloud.getHeight();

  sl::float4* cpu_cloud = point_cloud.getPtr<sl::float4>();

  // cout << "step bytes: " << to_string(point_cloud.getStepBytes()) << endl;
  // cout << "getPixelBytes: " << to_string(point_cloud.getPixelBytes()) << endl;

  PlaneModel best_plane;
  int inliner_count = 0;
  for (int i = 0; i < 100; i++) {
    std::vector<int> samples = TakeSamples(point_cloud);

    PlaneModel plane(
        GetPoint(samples[0], point_cloud),
        GetPoint(samples[1], point_cloud),
        GetPoint(samples[2], point_cloud));
    std::vector<int> inliners = GetInliners(plane, cpu_cloud, pts_count);
    if (int(inliners.size()) > int(pts_count * 0.2)) {
      if (inliners.size() > inliner_count) {
        inliner_count = inliners.size();
        best_plane = FitPlane(inliners, cpu_cloud);
      }
    }
  }

  if (inliner_count > int(pts_count * 0.2)) {
    // Print best plane;
    // TODO: adjust the normal vector so it points to the origin.
    DrawPlane(best_plane.p, best_plane.n);
  }

  return;
}

void PubRosMessage(const sl::Mat& point_cloud) {
  sensor_msgs::PointCloud2 msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
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
  // float* pt_cloud_ptr = (float*)(&(msg.data[0]));
  void* pt_cloud_ptr = (void*)((msg.data.data()));
  memcpy(pt_cloud_ptr, (void*)cpu_cloud, 4 * pts_count * sizeof(float));

  cloud_pub.publish(msg);
}

void DetectPlanePlayback(std::string record_name) {
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

  zed.setSVOPosition(0);
  while (ros::ok()) {
    sleep(1);
    returned_state = zed.grab();

    if (returned_state == sl::ERROR_CODE::SUCCESS) {
      zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU, camera_config.resolution);
      int svo_position = zed.getSVOPosition();
      zed.setSVOPosition(svo_position + 15);

      PubRosMessage(point_cloud);
      FindPlanes(point_cloud);

    } else if (returned_state == sl::ERROR_CODE::END_OF_SVOFILE_REACHED) {
      zed.setSVOPosition(0);
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

void DetectPlaneRealtime() {
  sl::Camera zed;

  sl::InitParameters init_parameters;
  init_parameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
  init_parameters.coordinate_units = sl::UNIT::METER;
  init_parameters.camera_resolution = sl::RESOLUTION::HD720;
  init_parameters.camera_fps = 15;

  auto returned_state = zed.open(init_parameters);
  if (returned_state != sl::ERROR_CODE::SUCCESS) {
    std::cout << "Error " << returned_state << ", exit program." << std::endl;
    return;
  }

  sl::RuntimeParameters runtime_parameters;
  runtime_parameters.sensing_mode = sl::SENSING_MODE::STANDARD;

  auto camera_config = zed.getCameraInformation().camera_configuration;
  sl::Mat point_cloud(camera_config.resolution, sl::MAT_TYPE::F32_C4, sl::MEM::CPU);

  int i = 0;
  while (ros::ok()) {
    sleep(1);

    if (zed.grab(runtime_parameters) != sl::ERROR_CODE::SUCCESS) {
      std::cout << "Failed to grab image." << std::endl;
      break;
    }


    // zed.retrieveImage(image, sl::VIEW::LEFT);
    zed.retrieveMeasure(point_cloud, sl::MEASURE::XYZRGBA, sl::MEM::CPU, camera_config.resolution);

    // int x = point_cloud.getWidth() / 2;
    // int y = point_cloud.getHeight() / 2;
    // sl::float4 point_cloud_value;
    // point_cloud.getValue(x, y, &point_cloud_value);

    // if (std::isfinite(point_cloud_value.z)) {
    //   float distance = sqrt(point_cloud_value.x * point_cloud_value.x + point_cloud_value.y * point_cloud_value.y + point_cloud_value.z * point_cloud_value.z);
    //   cout << "Distance to Camera at {" << x << ";" << y << "}: " << distance << "mm" << endl;
    // } else {
    //   cout << "The Distance can not be computed at {" << x << ";" << y << "}" << endl;
    // }

      PubRosMessage(point_cloud);
      FindPlanes(point_cloud);
  }

  // Close the camera
  zed.close();
}

void DemoDrawPlane() {
  Point3 translation(0, 0, 0);
  Vector3 norm(1, 1, 1);

  while (ros::ok()) {
    sleep(1);

    DrawPlane(translation, norm);
  }
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "basic_perception");
  ros::NodeHandle nh;
  cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/basic_perception", 1);
  plane_pub = nh.advertise<visualization_msgs::Marker>("/basic_perception_plane", 1);

  // DemoDrawPlane();
  // DetectPlaneRealtime();
  DetectPlanePlayback(argv[1]);
  
  return 0;
}
