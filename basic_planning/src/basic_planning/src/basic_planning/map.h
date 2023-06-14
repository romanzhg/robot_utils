#pragma once

#include <vector>

#include <nav_msgs/OccupancyGrid.h>

namespace basic_planning {

struct OccupancyGridMap {
  int num_rows, num_cols, data_size;
  double meters_per_cell;
  nav_msgs::MapMetaData map_info;
  int8_t* data;
  int seq;

  OccupancyGridMap() {
    seq = 0;
    meters_per_cell = 0.05;
    num_rows = 1000;
    double width_m = num_rows * meters_per_cell;
    num_cols = 800;
    double height_m = num_cols * meters_per_cell;
    data_size = num_rows * num_cols;

    map_info.resolution = meters_per_cell;
    map_info.width = num_rows;
    map_info.height = num_cols;
    map_info.origin.position.x = -width_m / 2.0;
    map_info.origin.position.y = -height_m / 2.0;
    map_info.origin.position.z = 0;
    map_info.origin.orientation.x = 0;
    map_info.origin.orientation.y = 0;
    map_info.origin.orientation.z = 0;
    map_info.origin.orientation.w = 1;

    data = (int8_t*) malloc(num_rows * num_cols * sizeof(int8_t));

    memset(data, 0, num_rows * num_cols * sizeof(int8_t));
  }

  ~OccupancyGridMap() {
    free(data);
  }

  std::pair<int, int> CartesianToGridCoords(double x, double y) const {
    double gx = (x - map_info.origin.position.x) / meters_per_cell;
    double gy = (y - map_info.origin.position.y) / meters_per_cell;
    int row = std::min(std::max(int(gx), 0), num_rows - 1);
    int col = std::min(std::max(int(gy), 0), num_cols - 1);
    return {row, col};
  }

  bool IsInRange(int r, int c) const {
    return r >= 0 && r < num_rows && c >= 0 && c < num_cols;
  }

  // Note the way index is calculated.
  int RCToIndex(int r, int c) const {
    return c * num_rows + r;
  }

  // v in [-1, 100].
  void SetSquare(double x, double y, double l, uint8_t v) {
    std::pair<int, int> center = CartesianToGridCoords(x, y);
    int l_in_cells = int(l / 2.0 / meters_per_cell);
    for (int r = -l_in_cells; r <= l_in_cells; r++) {
      for (int c = -l_in_cells; c <= l_in_cells; c++) {
        int tmp_r = r + center.first;
        int tmp_c = c + center.second;
        if (IsInRange(tmp_r, tmp_c)) {
          data[RCToIndex(tmp_r, tmp_c)] = v;
        }
      }
    }
  }

  // x1 < x2, y1 < y2
  void SetRectangular(double x1, double y1, double x2, double y2, uint8_t v) {
    std::pair<int, int> lower_left = CartesianToGridCoords(x1, y1);
    std::pair<int, int> upper_right = CartesianToGridCoords(x2, y2);
    for (int r = lower_left.first; r <= upper_right.first; r++) {
      for (int c = lower_left.second; c <= upper_right.second; c++) {
        data[RCToIndex(r, c)] = v;
      }
    }
  }

  std::pair<double, double> GetCellCenter(int r, int c) {
    return {map_info.origin.position.x + r * meters_per_cell + meters_per_cell / 2.0,
        map_info.origin.position.y + c * meters_per_cell  + meters_per_cell / 2.0};
  }

  void SetCircle(double x, double y, double radius, uint8_t v) {
    std::pair<int, int> center = CartesianToGridCoords(x, y);
    int r_in_cells = int(radius/ meters_per_cell);
    for (int r = -r_in_cells; r <= r_in_cells; r++) {
      for (int c = -r_in_cells; c <= r_in_cells; c++) {
        int tmp_r = r + center.first;
        int tmp_c = c + center.second;
        std::pair<double, double> center_coords = GetCellCenter(tmp_r, tmp_c);
        double dist = std::hypot(x - center_coords.first, y - center_coords.second);
        if (IsInRange(tmp_r, tmp_c) && dist <= radius) {
          data[RCToIndex(tmp_r, tmp_c)] = v;
        }
      }
    }
  }

  void SetLine(double x, double y, double angle, double len, double width, uint8_t v) {
    std::vector<std::pair<double, double>> point_set;
    double finer_sample_dist = meters_per_cell / 2.1;
    int w_steps = width / 2.0 / finer_sample_dist;
    int l_steps = len / finer_sample_dist;

    for (int i = -w_steps; i <= w_steps; i++) {
      for (int j = 0; j <= l_steps; j++) {
        point_set.push_back({j * finer_sample_dist, i * finer_sample_dist});
      }
    }

    for (auto& p : point_set) {
      double tmp_x = p.first;
      double tmp_y = p.second;

      p.first = tmp_x * cos(angle) - tmp_y * sin(angle) + x;
      p.second = tmp_x * sin(angle) + tmp_y * cos(angle) + y;
    }

    for (const auto& p: point_set) {
      const auto& coords = CartesianToGridCoords(p.first, p.second);
      data[RCToIndex(coords.first, coords.second)] = v;
    }
  }

  bool IsCollisionFree(double x, double y, double angle, double L, double W) const {
    std::vector<std::pair<double, double>> point_set;
    double finer_sample_dist = meters_per_cell / 2.1;
    int w_steps = W / 2.0 / finer_sample_dist;
    int l_steps = L / 2.0 / finer_sample_dist;

    for (int i = -w_steps; i <= w_steps; i++) {
      for (int j = 0; j <= l_steps; j++) {
        point_set.push_back({j * finer_sample_dist, i * finer_sample_dist});
      }
    }

    for (auto& p : point_set) {
      double tmp_x = p.first;
      double tmp_y = p.second;

      p.first = tmp_x * cos(angle) - tmp_y * sin(angle) + x;
      p.second = tmp_x * sin(angle) + tmp_y * cos(angle) + y;
    }

    for (const auto& p: point_set) {
      const auto& coords = CartesianToGridCoords(p.first, p.second);
      if (data[RCToIndex(coords.first, coords.second)] >= 30) {
        return false;
      }
    }
    return true;
  }

  nav_msgs::OccupancyGrid ToRosMap() {
    nav_msgs::OccupancyGrid msg;
    msg.header.seq = seq++;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world"; 
    msg.info = map_info;
    msg.data.resize(data_size);
    memcpy(&msg.data[0], data, data_size);
    return msg;
  }
};
}