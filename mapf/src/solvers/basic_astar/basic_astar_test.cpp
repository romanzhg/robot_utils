#include "basic_astar.h"

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

  basic_astar::BasicAstar basic_astar_;
};

TEST_F(Basic, Basic4) {
  MapfMap test_map("../data/mapf_map/maze-32-32-2.map");
  vector<Task> tasks = {
      {2, 29, 31, 22},
      {13, 9, 14, 19},
      {14, 24, 19, 4},
      {17, 19, 25, 14},
      {17, 26, 13, 7}
  };

  SearchResult result = basic_astar_.Solve(&test_map, tasks, 50000);

  assert(result.succeed);
}

}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
