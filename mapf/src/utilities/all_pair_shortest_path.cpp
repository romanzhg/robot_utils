#include <iostream>

#include "constants.h"
#include "utilities/mapf_map_for_apsp.h"

using namespace std;

int main() {
  string map_file = "Berlin_1_256.map";
  string map_file_path = mapf::kMapDir + map_file;
  util::MapfMap map(map_file_path);

  // Debug.
  cout << "Printing a position:" << endl;
  cout << (map.GetCharAt(91, 227)) << endl;
  cout << (map.GetCharAt(172, 14)) << endl;
  cout << "Print the distance: " << endl;
  cout << to_string(map.GetDistance({91, 227}, {172, 14})) << endl;

  // Build the all pairs shortest path file.
//  map.BuildDistanceMap();
//  map.WriteAllPairShortestDist();
  return 0;
}