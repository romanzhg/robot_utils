#include <vector>

#include "common_types.h"

using namespace std;

namespace mapf {

vector<SolverType> fSolvers = {
    SolverType::BasicAstar,
    SolverType::CBS,
    SolverType::OdAstar,
    SolverType::ID,
    SolverType::ICBS,
};

bool fEnableOptimalResultCheck = true;
}
