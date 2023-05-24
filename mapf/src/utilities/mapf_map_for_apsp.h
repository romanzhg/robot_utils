#ifndef MAPF_SRC_MAPF_MAP_H_
#define MAPF_SRC_MAPF_MAP_H_

#include <string>

#include "constants.h"
#include "common_types.h"

using namespace mapf;

namespace util {

class MapfMap {
 public:
  MapfMap(const std::string &f);

  bool IsPointValid(int x, int y) const;
  bool IsPointValid(const Position& p) const;

  // Build/load all pairs. Not used for now, the shortest
  // dist map is generated on demand.
  void BuildDistanceMap();
  void WriteAllPairShortestDist();
  void LoadAllPairShortestDist();
  char GetCharAt(int x, int y);
  int GetDistance(const Position &src, const Position &dest);
  void BuildSingleDestDist(const Position &dest);

 private:
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