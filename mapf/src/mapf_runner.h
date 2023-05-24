#ifndef MAPF_SRC_MAPF_RUNNER_H_
#define MAPF_SRC_MAPF_RUNNER_H_

#include <set>
#include <string>
#include <vector>
#include <fstream>
#include <map>

#include "common_types.h"
#include "mapf_map.h"
#include "flags.h"
#include "solver.h"

namespace mapf {

struct RunConfig {
  std::string map_file_name;
  int scenes;
  int min_robots;
  int max_robots;
};

class Runner {
 public:
  Runner() = default;
  void Run();

  // Only for manual debugging.
  void VerifyWrittenResult();

 private:
  void RunPerConfig(const RunConfig &config);
  void RunPerSceneAndRobot(const RunConfig &config,
                           std::pair<std::string, std::string> scene_file_info,
                           MapfMap &map);

  std::vector<RunConfig> LoadConfig();
  std::ofstream summary_file_;
  std::map<SolverType, MapfSolver *> solvers;
  uint64_t cur_time_;
};

}

#endif