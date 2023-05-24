#ifndef MAPF_SRC_UTILS_H_
#define MAPF_SRC_UTILS_H_

#include <chrono>
#include <string>
#include <thread>
#include <vector>
#include <cassert>
#include <set>
#include <iostream>

#include "common_types.h"
#include "constants.h"

namespace mapf {

std::vector<std::string> GetLinesOfFile(const std::string &file_name);

inline void SleepMS(size_t ms) {
  std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

inline uint64_t GetCurrentTimeSinceEpochS() {
  auto duration = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::seconds>(duration).count();
}

inline uint64_t GetCurrentTimeSinceEpochMS() {
  auto duration = std::chrono::system_clock::now().time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

inline bool IsElapsedTimeShorterThan(uint64_t time_point_ms, uint64_t duration_ms) {
  uint64_t current = GetCurrentTimeSinceEpochMS();
  return (current - time_point_ms) < duration_ms;
}

inline bool IsElapsedTimeLongerThan(uint64_t time_point_ms, uint64_t duration_ms) {
  return !IsElapsedTimeShorterThan(time_point_ms, duration_ms);
}

inline int GetManhattanDist(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

// String utilities.
void StrSplit(
    const std::string &str,
    char delim,
    std::vector<std::string> &result);

std::vector<std::string> StrSplit(const std::string &str, char delim);

bool StrStartsWith(const std::string &a, const std::string &b);

// C++ notes: The definition of an inline function must be present in the
// translation unit where it is accessed (not necessarily before the
// point of access).
inline Action PositionToAction(const Position from, const Position to) {
  int diff_x = to.x - from.x;
  int diff_y = to.y - from.y;
  return kDeltaToAction.at({diff_x, diff_y});
}

// Input is a vector of robot positions, indexed by robot id.
// Returns a vector of actions corresponds to this transition.
inline std::vector<Action> PositionSeqToActions(const std::vector<Position> &from,
                                                const std::vector<Position> &to) {
  assert(from.size() == to.size());
  int robot_count = from.size();
  std::vector<Action> rtn;
  for (int i = 0; i < robot_count; i++) {
    rtn.push_back(PositionToAction(from[i], to[i]));
  }
  return rtn;
}

// Transfer a m*n matrix to a n*m matrix.
inline std::vector<std::vector<Position>> TransferPositionMatrix(std::vector<std::vector<Position>> a) {
  int m = a.size();
  if (m == 0) {
    return {};
  }
  int n = a[0].size();
  if (n == 0) {
    return {};
  }

  std::vector<std::vector<Position>> rtn(n, std::vector<Position>(m));
  for (int i = 0; i < rtn.size(); i++) {
    for (int j = 0; j < rtn[0].size(); j++) {
      rtn[i][j] = a[j][i];
    }
  }
  return rtn;
}

struct UnionFind {
  UnionFind() = default;

  void Init(int size) {
    this->size = size;
    grouping.resize(size);
    for (int i = 0; i < size; i++) {
      grouping[i] = i;
    }
  }

  void Union(int a, int b) {
    int pa = Find(a);
    int pb = Find(b);
    grouping[pb] = pa;
  }

  int Find(int a) {
    if (grouping[a] == a) {
      return a;
    }
    grouping[a] = Find(grouping[a]);
    return grouping[a];
  }

  // TODO: Use a map to maintain the size.
  // Returns the size of the group associated with element @a.
  int GetSize(int a) {
    int rtn = 0;
    int pa = Find(a);
    for (int i = 0; i < size; i++) {
      if (Find(i) == pa) {
        rtn++;
      }
    }
    return rtn;
  }

  std::map<int, std::set<int>> GetGroupIdToElem() {
    std::map<int, std::set<int>> group_id_to_elem;
    for (int i = 0; i < size; i++) {
      group_id_to_elem[Find(i)].insert(i);
    }

    return group_id_to_elem;
  }

  std::set<int> GetGroup(int a) {
    int pa = Find(a);
    std::set<int> rtn;
    for (int i = 0; i < size; i++) {
      if (Find(i) == pa) {
        rtn.insert(i);
      }
    }
    return rtn;
  }

  void PrintGrouping() {
    for (int i = 0; i < size; i++) {
      std::cout << std::to_string(Find(i)) << " ";
    }
    std::cout << std::endl;
  }

  std::vector<int> grouping;
  int size;
};

}

#endif //MAPF_SRC_UTILS_H_