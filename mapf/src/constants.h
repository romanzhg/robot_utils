#ifndef MAPF_SRC_CONSTANTS_H_
#define MAPF_SRC_CONSTANTS_H_

#include <climits>

#include "common_types.h"

namespace mapf {
const char * const kMapDir = "../data/mapf_map/";
const char * const kSceneDir = "../data/mapf_scen/";
const char * const kOutputDir = "./output/";
const char * const kDataDir = "../data/";

const int kMapSizeLimit = 300;
const uint64_t kPerRunTimeLimitMs = 30 * 1000;

const int kInf = INT_MAX / 2;

const char * const kAllPairShortestPathSuffix = ".apsp";
const char * const kMapFileSuffix = ".map";

const int kMapFileHeaderOffset = 4;

const std::vector<Action> kActionTypes = {
    Action::UP,
    Action::DOWN,
    Action::LEFT,
    Action::RIGHT,
    Action::WAIT,
};

const std::map<Action, std::string> kActionToString = {
    {Action::UP, "UP"},
    {Action::DOWN, "DOWN"},
    {Action::LEFT, "LEFT"},
    {Action::RIGHT, "RIGHT"},
    {Action::WAIT, "WAIT"},
};

const std::vector<std::pair<int, int>> kActionToDelta =
    {{-1, 0},
     {1, 0},
     {0, -1},
     {0, 1},
     {0, 0}};

const std::vector<std::pair<int, int>> kDeltas =
    {{-1, 0},
     {1, 0},
     {0, -1},
     {0, 1}};

const std::map<std::pair<int, int>, Action> kDeltaToAction =
    {{{-1, 0}, Action::UP},
     {{1, 0}, Action::DOWN},
     {{0, -1}, Action::LEFT},
     {{0, 1}, Action::RIGHT},
     {{0, 0}, Action::WAIT}};

const std::map<SolverType, std::string> kSolverTypeToName = {
    {SolverType::ID, "IndependenceDetection"},
    {SolverType::OdAstar, "OdAstar"},
    {SolverType::CBS, "CBS"},
    {SolverType::BasicAstar, "BasicAstar"},
    {SolverType::ICBS, "ICBS"},
};

}

#endif //MAPF_SRC_CONSTANTS_H_