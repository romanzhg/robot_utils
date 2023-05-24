#include "mapf_runner.h"

#include <algorithm>
#include <iostream>
#include <string>
#include <vector>
#include <experimental/filesystem>

#include "constants.h"
#include "utils.h"
#include "solvers/basic_astar/basic_astar.h"
#include "solvers/cbs/cbs.h"
#include "solvers/id_od_astar/od_astar.h"
#include "solvers/id_od_astar/independence_detection.h"
#include "solvers/icbs/icbs.h"
#include "flags.h"

using namespace std;

namespace mapf {
namespace {
// Returns a pair of file name and file path(relative).
vector<pair<string, string>> GetFilesWithPrefix(string directory, string prefix) {
  vector<pair<string, string>> rtn;
  for (const auto &entry : experimental::filesystem::directory_iterator(directory)) {
    string file_name = entry.path().filename().string();
    if (mapf::StrStartsWith(file_name, prefix)) {
      rtn.push_back(make_pair(file_name, entry.path().string()));
    }
  }
  return rtn;
}

void WriteMovesToFile(
    const string &map_file_path,
    const vector<Task> tasks,
    const string &scene_name,
    const SearchResult &result,
    int robot_count,
    uint64_t timestamp,
    SolverType type) {
  string output_filepath(string(kOutputDir)
                             + scene_name + "." + to_string(robot_count) + "."
                             + kSolverTypeToName.at(type) + "."
                             + to_string(timestamp) + ".out");
  cout << output_filepath << endl;
  ofstream out_fp(output_filepath);
  out_fp << map_file_path << endl;
  out_fp << to_string(robot_count) << endl;
  out_fp << "# time spent(ms): " << to_string(result.time_spent_ms)
         << " expanded nodes: " << to_string(result.expanded_nodes)
         << endl;
  out_fp << "# total cost: " << to_string(result.total_moves) << endl;
  for (int i = 0; i < robot_count; i++) {
    out_fp << tasks[i].src.to_string() << " " << tasks[i].dest.to_string() << endl;
  }

  int step_counter = 0;
  for (const vector<Action> &step : result.actions) {
    out_fp << step_counter++ << " ";
    for (int i = 0; i < robot_count; i++) {
      out_fp << to_string(i) << " " << to_string((int) step[i]) << " ";
    }
    out_fp << endl;
  }
  out_fp.close();
}

vector<Task> LoadScene(const string &scene_file_path, MapfMap &map) {
  vector<string> buffer;

  ifstream input(scene_file_path);
  if (!input) {
    cout << "Cannot open scene file" << endl;
    exit(0);
  }

  string tmp_line;
  // Skip the first line.
  getline(input, tmp_line);
  while (getline(input, tmp_line)) {
    if (tmp_line[0] == '#') {
      // Skip comments.
      continue;
    }
    buffer.push_back(tmp_line);
  }
  input.close();

  vector<Task> rtn;
  for (string &line : buffer) {
    replace(line.begin(), line.end(), '\t', ' ');
    vector<string> splitted = StrSplit(line, ' ');
    Position src(stoi(splitted[4]),
                 stoi(splitted[5]));
    Position dest(stoi(splitted[6]),
                  stoi(splitted[7]));

    if (!map.IsPointValid(src) || !map.IsPointValid(dest)) {
      continue;
    }
    if (!map.PointsConnected(src, dest)) {
      continue;
    }
    rtn.emplace_back(src, dest);
  }
  return rtn;
}

// Exit if any task is not valid.
// 1. Check if two dest points are same.
// 2. Check if any src or dest point is invalid.
void CheckTasks(const vector<Task> &tasks, const MapfMap &map) {
  set<Position> dests;
  int invalid_point_count = 0;
  int total_points = 0;
  int dest_overlap = 0;
  for (const auto &t : tasks) {
    if (!map.IsPointValid(t.src)) {
      invalid_point_count++;
    }
    if (!map.IsPointValid(t.dest)) {
      invalid_point_count++;
    }
    total_points += 2;
    if (dests.find(t.dest) != dests.end()) {
      dest_overlap++;
    } else {
      dests.insert(t.dest);
    }
  }
  if (dest_overlap || invalid_point_count) {
    cout << "total points: " << to_string(total_points) << endl;
    cout << "invalid point count: " << to_string(invalid_point_count) << endl;
    cout << "dest overlap: " << to_string(dest_overlap) << endl;
    exit(0);
  }
}

MapfSolver *BuildSolver(SolverType type) {
  if (type == SolverType::CBS) {
    return new cbs::ConflictBasedSearch();
  } else if (type == SolverType::BasicAstar) {
    return new basic_astar::BasicAstar();
  } else if (type == SolverType::OdAstar) {
    return new id_od_astar::OdAstar();
  } else if (type == SolverType::ID) {
    return new id_od_astar::IndependenceDetection();
  } else if (type == SolverType::ICBS) {
    return new icbs::ICBS();
  } else {
    return nullptr;
  }
}

// Validate that search result is consistent.
// 1. Check the cost reported is same as the actual cost.
// 2. Validate there is no conflict.
void ValidateSolution(const SearchResult &result, const vector<Task> &tasks, MapfMap &mapf_map) {
  int robot_count = tasks.size();
  int total_time_steps = result.actions.size();

  // 1. Check total cost.
  int total_cost = 0;
  for (int robot_id = 0; robot_id < robot_count; robot_id++) {
    Position position = tasks[robot_id].src;
    int cost_for_robot = 0;
    for (int step = 0; step < total_time_steps; step++) {
      if (position != tasks[robot_id].dest) {
        cost_for_robot++;
        position = position + kActionToDelta[(int) result.actions[step][robot_id]];
      } else {
        break;
      }
    }
    total_cost += cost_for_robot;
  }

  if (total_cost != result.total_moves) {
    cout << "Invalid result. Actual cost: " << to_string(total_cost)
         << " reported cost: " << to_string(result.total_moves) << endl;
    exit(0);
  }

  // 2. Check conflicts.
  vector<Position> positions(robot_count);
  for (int robot_id = 0; robot_id < robot_count; robot_id++) {
    positions[robot_id] = tasks[robot_id].src;
  }
  for (int step = 0; step < total_time_steps; step++) {
    set<Edge> used_edges;

    // Apply action, check used edges, and check the new position is valid.
    for (int robot_id = 0; robot_id < robot_count; robot_id++) {
      Position new_pos = positions[robot_id] + kActionToDelta[(int) result.actions[step][robot_id]];
      if (positions[robot_id] != new_pos) {
        Edge edge(positions[robot_id], new_pos);
        if (used_edges.find(edge) != used_edges.end()) {
          cout << "Edge reuse." << endl;
          exit(0);
        }
        used_edges.insert(edge);
      }
      positions[robot_id] = new_pos;

      if (!mapf_map.IsPointValid(new_pos)) {
        cout << "Invalid position: " << new_pos.to_string() << endl;
        exit(0);
      }
    }

    // Check used positions.
    set<Position> occupied_positions;
    for (int robot_id = 0; robot_id < robot_count; robot_id++) {
      if (occupied_positions.find(positions[robot_id]) != occupied_positions.end()) {
        cout << "Position reuse." << endl;
        exit(0);
      }
      occupied_positions.insert(positions[robot_id]);
    }
  }

  // 3. Check all robots reached the destination.
  for (int robot_id = 0; robot_id < robot_count; robot_id++) {
    if (positions[robot_id] != tasks[robot_id].dest) {
      cout << "One robot has not reached the destination." << endl;
      exit(0);
    }
  }
}

}  // namespace

void Runner::Run() {
  cur_time_ = GetCurrentTimeSinceEpochS();
  summary_file_.open(string(kOutputDir) + "run."
                         + to_string(GetCurrentTimeSinceEpochS()) + ".out");

  for (SolverType type : fSolvers) {
    solvers[type] = BuildSolver(type);
  }

  for (const RunConfig &config : LoadConfig()) {
    RunPerConfig(config);
  }

  summary_file_.close();
}

vector<RunConfig> Runner::LoadConfig() {
  // Test config format:
  // '#' at the beginning of a line for commented out lines.
  // "# Map name, scenes to test, robots lower bound(inclusive), robots upper bound(inclusive)
  //  #maze_1 1 2 2
  //  maze-32-32-2 25 2 100"
  vector<RunConfig> rtn;
  vector<string> config = GetLinesOfFile(string(kDataDir) + "test_config");
  for (const auto &line : config) {
    const auto &fields = StrSplit(line, ' ');
    RunConfig config = {fields[0], stoi(fields[1]), stoi(fields[2]), stoi(fields[3])};
    rtn.push_back(config);
  }
  return rtn;
}
void Runner::RunPerConfig(const RunConfig &config) {
  string map_file_path = kMapDir + config.map_file_name + kMapFileSuffix;
  MapfMap map(map_file_path);

  vector<pair<string, string>> scene_files = GetFilesWithPrefix(kSceneDir, config.map_file_name);
  sort(scene_files.begin(), scene_files.end());

  for (int i = 0; i < min(config.scenes, (int) scene_files.size()); i++) {
    RunPerSceneAndRobot(config, scene_files[i], map);
  }
}

void Runner::RunPerSceneAndRobot(const RunConfig &config,
                                 pair<string, string> scene_file_info, MapfMap &map) {
  cout << "solving scene file: " << scene_file_info.second << endl;
  vector<Task> tasks = LoadScene(scene_file_info.second, map);
  CheckTasks(tasks, map);

  set<SolverType> failed_solvers;
  for (int robot_count = config.min_robots;
       robot_count <= min(config.max_robots, (int) tasks.size());
       robot_count++) {
    vector<Task> cur_tasks(tasks.begin(), tasks.begin() + robot_count);

    int result_to_compare = -1;
    for (SolverType type : fSolvers) {
      if (failed_solvers.find(type) != failed_solvers.end()) {
        continue;
      }

      SearchResult result = solvers[type]->Solve(&map, cur_tasks, kPerRunTimeLimitMs);

      if (!result.succeed) {
        cout << "Cannot find a solution within the time limit." << endl;
        cout << scene_file_info.first << " max robots: " << to_string(robot_count - 1) << endl;
        failed_solvers.insert(type);
        continue;
      } else {
        summary_file_ << "On scene:" << scene_file_info.first
                      << " with algorithm: " << kSolverTypeToName.at(type)
                      << " robots: " << to_string(robot_count)
                      << " total cost: " << to_string(result.total_moves)
                      << " nodes expanded: " << to_string(result.expanded_nodes)
                      << " time spent(ms): " << to_string(result.time_spent_ms)
                      << endl;
      }

      string map_file_path = kMapDir + config.map_file_name + kMapFileSuffix;
      WriteMovesToFile(map_file_path, cur_tasks, scene_file_info.first, result, robot_count, cur_time_, type);

      // Checks.
      ValidateSolution(result, cur_tasks, map);
      if (fEnableOptimalResultCheck) {
        // Check all solvers gives the same result.
        if (result_to_compare == -1) {
          result_to_compare = result.total_moves;
        } else {
          if (result_to_compare != result.total_moves) {
            cout << "One solver gives an invalid result" << endl;
          }
        }
      }
    }
  }
}

void Runner::VerifyWrittenResult() {
//  string file_name = "./output/room-32-32-4-even-1.scen.19.CBS.1571234672.out";
//  string map_file_name = "../data/mapf_map/room-32-32-4.map"
  string file_name = "";
  string map_file_name = "";
  MapfMap mapf_map(map_file_name);
  int total_cost = 0;
  int robot_count = 19;
  ActionSequencePerTime action_seq;

  vector<string> lines = GetLinesOfFile(file_name);
  vector<Task> tasks;
  int line_index = 2;
  for (int i = 0; i < robot_count; i++) {
    vector<string> tmp = StrSplit(lines[line_index + i], ' ');
    tasks.emplace_back(stoi(tmp[0]), stoi(tmp[1]), stoi(tmp[2]), stoi(tmp[3]));
  }
  line_index += robot_count;
  for (; line_index < lines.size(); line_index++) {
    cout << "processing line: " << lines[line_index] << endl;
    vector<string> tmp = StrSplit(lines[line_index], ' ');
    vector<Action> actions;
    for (int i = 0; i < robot_count; i++) {
      Action a = static_cast<Action >(stoi(tmp[1 + i * 2 + 1]));
      actions.push_back(a);
    }
    action_seq.push_back(actions);
  }

  SearchResult result(action_seq, true, 0, 0, total_cost);
  ValidateSolution(result, tasks, mapf_map);
}

}  // namespace mapf