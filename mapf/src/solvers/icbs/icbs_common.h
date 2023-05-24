#ifndef MAPF_SRC_SOLVERS_ICBS_ICBS_COMMON_H_
#define MAPF_SRC_SOLVERS_ICBS_ICBS_COMMON_H_

#include <vector>

#include "common_types.h"

namespace mapf {
namespace icbs {

// Semantic of this field: robot id -> time step -> robot position
// An object of type Solution may be empty if all robots are at the destination initially.
using Solution = std::vector<std::vector<Position>>;

// A list of positions for a robot(including the init position).
// Generate the moves by getting differentials on this.
// A path does not include the robot's initial position.
using Path = std::vector<Position>;

enum class ConstrainType : int {
  VERTEX = 0,
  EDGE = 1
};

// Robot @robot_id is prohibited at the position @pos at time @time_step.
struct Constrain {
  int robot_id;
  int time_step;
  ConstrainType type;
  // For vertex constrain.
  Position pos;
  // For edge constrain.
  std::pair<Position, Position> edge;

  Constrain() = default;
  Constrain(int robot_id, int time_step, const Position &pos)
      : robot_id(robot_id), time_step(time_step), pos(pos) {
    type = ConstrainType::VERTEX;
  };
  Constrain(int robot_id, int time_step, const Position &from, const Position &to)
      : robot_id(robot_id), time_step(time_step) {
    edge = std::make_pair(from, to);
    type = ConstrainType::EDGE;
  };

  Constrain &operator=(const Constrain &o) = default;

  // Needed for the constrain set.
  bool operator<(const Constrain &o) const {
    if (type == ConstrainType::VERTEX) {
      if (time_step != o.time_step) {
        return time_step < o.time_step;
      }
      return pos < o.pos;
    } else {
      if (time_step != o.time_step) {
        return time_step < o.time_step;
      }
      return edge < o.edge;
    }
  }

  std::string to_string() const {
    if (type == ConstrainType::VERTEX) {
      return std::string("constrain:")
          + " robot: " + std::to_string(robot_id)
          + " time step: " + std::to_string(time_step)
          + " position: " + pos.to_string();
    } else {
      return "";
    }
  }
};

// Constants.
// If two robots has more than @kMergeLimit conflicts, they will be merged.
// For a lower value(for example, 1, 5), icbs(mainly MA-CBS) will behave like ID.
// For a higher value(for example, 100), icbs will behave like CBS.
const int kMergeLimit = 1;
// If after merge, the new group has size larger than @kLargestGroupSize, don't allow the merge.
const int kLargestGroupSize = 4;


}
}

#endif //MAPF_SRC_SOLVERS_ICBS_ICBS_COMMON_H_
