#ifndef DSIM_PLAN_PLAN_H_
#define DSIM_PLAN_PLAN_H_ 1

#include <vector>

#include "level/level.h"
#include "plan/stage.h"

namespace plan {
std::vector<Stage> PlanTravel(Level& level,
                              const ParkingLot& src_parking_lot,
                              int parking_space,
                              const ParkingLot& dest_parking_lot,
                              int dest_parking_spot);
} // namespace plan

#endif  // DSIM_PLAN_PLAN_H_
