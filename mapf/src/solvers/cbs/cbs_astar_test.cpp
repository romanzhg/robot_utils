#include "cbs_astar.h"

#include "cbs_common.h"

#include "common_types.h"
#include "gtest/gtest.h"

using namespace mapf;
using namespace std;

namespace {
class Basic : public ::testing::Test {
 protected:
  Basic() {
  }

  ~Basic() override {
  }

  void SetUp() override {
  }

  void TearDown() override {
  }

  cbs::CbsAstar cbs_astar_;
};

TEST_F(Basic, Basic1) {
  MapfMap test_map("../data/mapf_map/maze_1.map");
  Task task{0, 0, 0, 4};
  set<cbs::Constrain> emptyConstrains{};
  auto p = cbs_astar_.FindPath(&test_map, emptyConstrains, task.src, task.dest);

  cout << "printing the solution for one robot:" << endl;
  for (const auto& pos : p.value()) {
    cout << pos.to_string() << endl;
  }
  cout << endl;
}

TEST_F(Basic, Basic2) {
  MapfMap test_map("../data/mapf_map/maze_1.map");
  Task task{0, 0, 0, 4};
  cbs::Constrain constrain0{0, 3, {0, 3}};
  cbs::Constrain constrain1{0, 4, {0, 3}};
  cbs::Constrain constrain2{0, 5, {0, 3}};

  set<cbs::Constrain> constrains;
  constrains.insert(constrain0);
  constrains.insert(constrain1);
  constrains.insert(constrain2);

  auto p = cbs_astar_.FindPath(&test_map, constrains, task.src, task.dest);

  cout << "printing the solution for one robot:" << endl;
  for (const auto& pos : p.value()) {
    cout << pos.to_string() << endl;
  }
  cout << endl;
}

TEST_F(Basic, Basic3) {
  MapfMap test_map("../data/mapf_map/maze-32-32-2.map");
  Task task{17, 21, 15, 16};
  cbs::Constrain constrain0{0, 39, {15, 16}};

  set<cbs::Constrain> constrains;
  constrains.insert(constrain0);

  auto p = cbs_astar_.FindPath(&test_map, constrains, task.src, task.dest);

  cout << "printing the solution for one robot:" << endl;
  for (const auto& pos : p.value()) {
    cout << pos.to_string() << endl;
  }
  cout << endl;
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
