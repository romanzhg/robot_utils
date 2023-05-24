#ifndef MAPF_SRC_COMMON_TYPES_H_
#define MAPF_SRC_COMMON_TYPES_H_

#include <string>
#include <map>
#include <utility>
#include <vector>

namespace mapf {

enum class SolverType : int {
  BasicAstar,
  CBS,
  OdAstar,
  ID,
  ICBS,
};

enum class Action : int {
  UP = 0,     // x -= 1
  DOWN = 1,   // x += 1
  LEFT = 2,   // y -= 1
  RIGHT = 3,  // y += 1
  WAIT = 4,
};

struct Position {
  int x;
  int y;

  Position() = default;
  Position(int x, int y) : x(x), y(y) {};
  Position(const std::pair<int, int>& p) : x(p.first), y(p.second) {};

  bool operator<(const Position &o) const {
    if (x == o.x) {
      return y < o.y;
    } else {
      return x < o.x;
    }
  }

  Position &operator=(const Position &o) {
    x = o.x;
    y = o.y;
  }

  bool operator==(const Position &o) const {
    return x == o.x && y == o.y;
  }

  bool operator!=(const Position &o) const {
    return x != o.x || y != o.y;
  }

  std::string to_string() const {
    return std::to_string(x) + " " + std::to_string(y);
  }
};

inline Position operator+(const Position& p, const std::pair<int, int>& o) {
  return Position(p.x + o.first, p.y + o.second);
}

inline Position operator-(const Position& p, const std::pair<int, int>& o) {
  return Position(p.x - o.first, p.y - o.second);
}

struct Task {
  Position src;
  Position dest;

  Task() = delete;
  Task(int sx, int sy, int tx, int ty)
      : src(sx, sy), dest(tx, ty) {};
  Task(Position src, Position dest) : src(std::move(src)), dest(std::move(dest)) {};
  Task &operator=(const Task &o) = default;
};

struct Edge {
  Position from, to;

  Edge(Position a, Position b) {
    if (a < b) {
      from = a;
      to = b;
    } else if (b < a) {
      from = b;
      to = a;
    } else {
      exit(0);
    }
  }

  bool operator<(const Edge &o) const {
    if (from == o.from) {
      return to < o.to;
    } else {
      return from < o.from;
    }
  }
};

}

#endif //MAPF_SRC_COMMON_TYPES_H_
