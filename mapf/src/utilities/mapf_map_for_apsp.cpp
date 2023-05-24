#include "utilities/mapf_map_for_apsp.h"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>
#include <queue>

#include "constants.h"
#include "common_types.h"
#include "utils.h"

using namespace std;

namespace util {

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

void MapfMap::LoadAllPairShortestDist() {
  // Need to build the distance map.
  ifstream input(map_file_path_ + kAllPairShortestPathSuffix);
  if (!input.good()) {
    cout << "Need to build the all pair shortest path file first." << endl;
    exit(0);
  }

  string tmp_line;
  while (getline(input, tmp_line)) {
    vector<string> splitted = StrSplit(tmp_line, ' ');
    int x1 = stoi(splitted[0]);
    int y1 = stoi(splitted[1]);
    int x2 = stoi(splitted[2]);
    int y2 = stoi(splitted[3]);
    int dist = stoi(splitted[4]);

    Position a(x1, y1);
    Position b(x2, y2);
    dists_[a][b] = dist;
    dists_[b][a] = dist;
  }
  input.close();

  for (const Position& p : positions_) {
    dists_[p][p] = 0;
  }
}

void MapfMap::BuildDistanceMap() {
  for (const Position &i : positions_) {
    for (const Position &j : positions_) {
      dists_[i][j] = kInf;
    }
  }

  for (const Position &p : positions_) {
    for (const auto &direction : kDeltas) {
      int new_x = p.x + direction.first;
      int new_y = p.y + direction.second;
      if (IsPointValid(new_x, new_y)) {
        Position t(new_x, new_y);
        dists_[p][t] = 1;
      }
    }
  }

  for (const Position &p : positions_) {
    dists_[p][p] = 0;
  }

  for (const Position &k : positions_) {
    cout << "Building map..." << endl;
    for (const Position &i : positions_) {
      for (const Position &j : positions_) {
        if (dists_[i][j] > dists_[i][k] + dists_[k][j]) {
          dists_[i][j] = dists_[i][k] + dists_[k][j];
        }
      }
    }
  }
}

void MapfMap::WriteAllPairShortestDist() {
  std::ofstream fout(map_file_path_ + kAllPairShortestPathSuffix);
  int vertex_count = positions_.size();
  for (int i = 0; i < vertex_count; i++) {
    for (int j = i + 1; j < vertex_count; j++) {
      fout << positions_[i].to_string() << " " << positions_[j].to_string()
           << " " << dists_[positions_[i]][positions_[j]] << endl;
    }
  }
  fout.close();
}

char MapfMap::GetCharAt(int x, int y) {
  return map_[x][y];
}

}