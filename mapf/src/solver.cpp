#include "solver.h"

namespace mapf {

void mapf::MoveSeq::AddActionSeq(int robot_id, std::vector<Action> actions) {

}

int mapf::MoveSeq::GetTotalCost() {
  return 0;
}

mapf::ActionSequencePerTime mapf::MoveSeq::ExportToOutput() {
  return mapf::ActionSequencePerTime();
}

int mapf::MoveSeq::GetActionLength() {
  return 0;
}

std::vector<Position> mapf::MoveSeq::GetPositionAtTime(int time_step) {
  return std::vector<Position>();
}

// Validate current sequence. Exit for anything wrong.
void MoveSeq::Validate() {

}

}