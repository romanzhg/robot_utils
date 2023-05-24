#include "icbs_astar.h"

#include "icbs_common.h"

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

  icbs::ICbsAstar icbs_astar_;
};

TEST_F(Basic, Basic1) {
  MapfMap test_map("../data/mapf_map/maze_1.map");
  Task task{0, 0, 0, 4};
  set<icbs::Constrain> emptyConstrains{};
  icbs::Solution sol = icbs_astar_.FindSolution(&test_map, &emptyConstrains, {task}).value();

  cout << "printing the solution for one robot:" << endl;
  for (const auto& pos : sol[0]) {
    cout << pos.to_string() << endl;
  }
  cout << endl;
}

TEST_F(Basic, Basic2) {
  MapfMap test_map("../data/mapf_map/maze_1.map");
  Task task{0, 0, 0, 4};
  icbs::Constrain constrain0{0, 3, {0, 3}};
  icbs::Constrain constrain1{0, 4, {0, 3}};
  icbs::Constrain constrain2{0, 5, {0, 3}};

  set<icbs::Constrain> constrains;
  constrains.insert(constrain0);
  constrains.insert(constrain1);
  constrains.insert(constrain2);

  icbs::Solution sol = icbs_astar_.FindSolution(&test_map, &constrains, {task}).value();

  cout << "printing the solution for one robot:" << endl;
  for (const auto& pos : sol[0]) {
    cout << pos.to_string() << endl;
  }
  cout << endl;
}

TEST_F(Basic, Basic3) {
  MapfMap test_map("../data/mapf_map/maze-32-32-2.map");
  Task task{17, 21, 15, 16};
  icbs::Constrain constrain0{0, 21, {15, 16}};

  set<icbs::Constrain> constrains;
  constrains.insert(constrain0);

  icbs::Solution sol = icbs_astar_.FindSolution(&test_map, &constrains, {task}).value();

  cout << "printing the solution for one robot:" << endl;
  for (const auto& pos : sol[0]) {
    cout << pos.to_string() << endl;
  }
  cout << endl;
}

TEST_F(Basic, Basic4) {
  MapfMap test_map("../data/mapf_map/maze-32-32-2.map");
  vector<Task> tasks = {
      {2, 29, 31, 22},
      {13, 9, 14, 19},
      {14, 24, 19, 4},
      {17, 19, 25, 14},
      {17, 26, 13, 7}
  };
  icbs::Constrain constrain0{0, 21, {15, 16}};

  set<icbs::Constrain> constrains;
//  constrains.insert(constrain0);

  icbs::Solution sol = icbs_astar_.FindSolution(&test_map, &constrains, tasks).value();

  cout << "printing the solution for one robot:" << endl;
  for (const auto& pos : sol[0]) {
    cout << pos.to_string() << endl;
  }
  cout << endl;
}


}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
