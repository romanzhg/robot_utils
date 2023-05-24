#ifndef MAPF_SRC_SOLVERS_CBS_COMMON_H_
#define MAPF_SRC_SOLVERS_CBS_COMMON_H_

#include <vector>
#include <string>

#include "common_types.h"

namespace mapf {
namespace cbs {

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

}
}

#endif //MAPF_SRC_SOLVERS_CBS_COMMON_H_
