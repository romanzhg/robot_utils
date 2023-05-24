#ifndef MAPF_SRC_MAPF_MAP_H_
#define MAPF_SRC_MAPF_MAP_H_

#include <string>

#include "constants.h"
#include "common_types.h"

namespace mapf {

class MapfMap {
 public:
  MapfMap(const std::string &f);

  int GetDistance(const Position& src, const Position& dest);
  bool PointsConnected(const Position &src, const Position &dest);
  bool IsPointValid(int x, int y) const;
  bool IsPointValid(const Position& p) const;

 private:
  void BuildSingleDestDist(const Position& dest);
  // dists_[dest][src] -> dist.
  std::map<Position, std::map<Position, int>> dists_;

  // Data.
  const std::string map_file_path_;
  char map_[kMapSizeLimit][kMapSizeLimit];
  std::vector<Position> positions_;
  int x_limit_;
  int y_limit_;
};

}

#endif