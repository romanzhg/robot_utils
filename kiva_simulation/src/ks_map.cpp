#include "ks_map.h"

#include <fstream>
#include <iostream>
#include <vector>

#include "constants.h"

namespace ks {
using namespace std;

KsMap::KsMap(std::string file_name) {
  ifstream input(file_name);
  if (!input) {
    cout << "Cannot open map file." << endl;
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

  x_limit_ = atoi(buffer[0].c_str());
  y_limit_ = atoi(buffer[1].c_str());
  robot_count_ = atoi(buffer[2].c_str());
  shelf_count_ = atoi(buffer[3].c_str());
  buffer.erase(buffer.begin(), buffer.begin() + 4);

  for (int x = 0; x < x_limit_; x++) {
    for (int y = 0; y < y_limit_; y++) {
      map_[x][y] = buffer[x][y];

      if (map_[x][y] == kShelfOperationPoint) {
        sops_.emplace_back(x, y);
      }
      if (map_[x][y] == kShelfStoragePoint) {
        ssps_.emplace_back(x, y);
      }
      if (map_[x][y] != 'B' && map_[x][y] != 'O' && map_[x][y] != 'T') {
        passable_.emplace_back(x, y);
      }
    }
  }
  operation_point_count_ = sops_.size();
  storage_point_count_ = ssps_.size();
}

const std::vector<Location> &KsMap::GetShelfOperationPoints() const {
  return sops_;
}

const std::vector<Location> &KsMap::GetShelfStoragePoints() const {
  return ssps_;
}

const std::vector<Location> &KsMap::GetPassableLocations() const {
  return passable_;
}

bool KsMap::IsLocationPassable(const Location &loc) const {
  int x = loc.x, y = loc.y;
  if (x < 0 || x >= x_limit_ || y < 0 || y >= y_limit_) {
    return false;
  }
  return map_[x][y] != 'B' && map_[x][y] != 'O' && map_[x][y] != 'T';
}

}