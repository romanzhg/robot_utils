#ifndef MAPF_SRC_FLAGS_H_
#define MAPF_SRC_FLAGS_H_

#include <vector>

#include "common_types.h"

namespace mapf {

// C++ notes: these fields cannot be const.
extern std::vector<SolverType> fSolvers;

extern bool fEnableOptimalResultCheck;
}

#endif //MAPF_SRC_FLAGS_H_
