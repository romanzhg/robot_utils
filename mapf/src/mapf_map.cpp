#include "mapf_map.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
#include <queue>
#include <set>

#include "constants.h"
#include "common_types.h"
#include "utils.h"

using namespace std;

namespace mapf {

bool MapfMap::IsPointValid(int x, int y) const {
  if (x < 0 || x >= x_limit_) {
    return false;
  }
  if (y < 0 || y >= y_limit_) {
    return false;
  }
  return map_[x][y] == '.';
}

bool MapfMap::IsPointValid(const Position &p) const {
  return IsPointValid(p.x, p.y);
}

int MapfMap::GetDistance(const Position &src, const Position &dest) {
  if (dists_.find(dest) == dists_.end()) {
    BuildSingleDestDist(dest);
  }
  return dists_.at(dest).at(src);
}

bool MapfMap::PointsConnected(const Position &src, const Position &dest) {
  if (dists_.find(dest) == dists_.end()) {
    BuildSingleDestDist(dest);
  }
  return dists_.at(dest).find(src) != dists_.at(dest).end();
}

MapfMap::MapfMap(const string &file_name)
  : map_file_path_(file_name){
  ifstream input(map_file_path_);
  if (!input) {
    cout << "Cannot open map file" << endl;
    exit(0);
  }

  vector<string> buffer;
  string tmp_line;
  while (getline(input, tmp_line)) {
    if (tmp_line[0] == '#') {
      // Skip comments.
      continue;
    }
    buffer.push_back(tmp_line);
  }
  input.close();

  string height_str = buffer[1].substr(string("height ").length());
  x_limit_ = stoi(height_str);

  string width_str = buffer[2].substr(string("width ").length());
  y_limit_ = stoi(width_str);

  for (int x = 0; x < x_limit_; x++) {
    for (int y = 0; y < y_limit_; y++) {
      map_[x][y] = buffer[x + kMapFileHeaderOffset][y];
      if (map_[x][y] == '.') {
        positions_.emplace_back(x, y);
      }
    }
  }
}

void MapfMap::BuildSingleDestDist(const Position &dest) {
  queue<Position> to_expand;
  set<Position> visited;

  to_expand.push(dest);
  visited.insert(dest);

  int cur_dist = 0;
  int cur_level_size = 1;
  int next_level_size = 0;

  while (!to_expand.empty()) {
    Position cur_pos = to_expand.front();
    to_expand.pop();
    dists_[dest][cur_pos] = cur_dist;

    for (const auto &direction : kDeltas) {
      Position new_pos = cur_pos + direction;
      if (IsPointValid(new_pos) && visited.find(new_pos) == visited.end()) {
        next_level_size++;
        visited.insert(new_pos);
        to_expand.push(new_pos);
      }
    }

    cur_level_size--;
    if (cur_level_size == 0) {
      cur_dist++;
      cur_level_size = next_level_size;
      next_level_size = 0;
    }
  }
}

}