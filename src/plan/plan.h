#ifndef DSIM_PLAN_PLAN_H_
#define DSIM_PLAN_PLAN_H_ 1

#include <vector>

#include "level/level.h"
#include "plan/stage.h"

namespace plan {

class Planner {
public:
  Planner(Level& l) : level(l) { Init(l); }

  void Init(Level& level);
  std::vector<Stage::Segment> Plan(int a, int b);

  std::vector<Stage> Plan(const ParkingLot& src_parking_lot, int parking_space,
                          const ParkingLot& dest_parking_lot,
                          int dest_parking_spot);

private:
  Level& level;
  std::vector<nacb::Vec2f> pos;
  absl::flat_hash_map<nacb::Vec2f, int, PointHash, PointEqual> ipos;
  std::vector<std::vector<int>> adj;
  std::vector<std::vector<std::pair<RoadSegment*, int>>> adj_info;
};

std::vector<Stage> PlanTravel(Level& level, const ParkingLot& src_parking_lot,
                              int parking_space,
                              const ParkingLot& dest_parking_lot,
                              int dest_parking_spot);
} // namespace plan

#endif // DSIM_PLAN_PLAN_H_
