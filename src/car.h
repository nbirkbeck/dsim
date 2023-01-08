#ifndef DSIM_CAR_H_
#define DSIM_CAR_H 1

#include "level/level.h"
#include "plan/plan.h"
#include "plan/stage.h"
#include <nmath/vec2.h>
#include <set>
#include <vector>

class Car {
public:
  static constexpr double kAccelerationRate = 4.0;
  static constexpr double kDecelerationRate = 8.0;
  static constexpr double kParkingLotSpeedLimit = 2.0;
  static constexpr double kWheelSize = 0.39 / 4.0;

  struct RandomParameters {
    static double GenerateRandomSpeed() { return 12 + 4.0 * rand() / RAND_MAX; }
    // How long we will wait in a parking spot
    static double GenerateRandomWait() {
      return 2.0 * rand() / RAND_MAX + 0.00001;
    }
  };

  Car(Level& level, plan::Planner& planner, int id, int start, int end)
      : level_(level), planner_(planner), car_id_(id), start_(start), end_(end),
        travel_time_(0) {
    plan_ = planner.Plan(level.parking_lots[start],
                         id % level.parking_lots[start].parking_spots.size(),
                         level.parking_lots[end],
                         id % level.parking_lots[end].parking_spots.size());
    stage_index_ = 0;
    pos_ = (level.parking_lots[start].parking_spots[0].pos +
            level.parking_lots[start].pos);
    max_speed_ = RandomParameters::GenerateRandomSpeed();
  }

  RoadSegment* road_segment() const {
    if (stage_index_ == 1 && stage_index_ < (int)plan_.size()) {
      if (segment_index_ < (int)plan_[stage_index_].segments.size()) {
        return plan_[stage_index_].segments[segment_index_].road_segment;
      }
    }
    return nullptr;
  }

  void Step(const std::vector<Car*>::iterator& car_begin,
            const std::vector<Car*>::iterator& car_end, double t_abs,
            double dt);

  const nacb::Vec2f& pos() const { return pos_; }

private:
  void BuildNewPlan() {
    start_ = level_.PickRandomParkingLot();
    while (start_ == end_) {
      start_ = level_.PickRandomParkingLot();
    }
    plan_ = planner_.Plan(
        level_.parking_lots[end_],
        car_id_ % level_.parking_lots[end_].parking_spots.size(),
        level_.parking_lots[start_],
        car_id_ % level_.parking_lots[start_].parking_spots.size());
    std::swap(start_, end_);

    stage_index_ = 0;
    segment_index_ = 0;
    sub_index_ = 0;
    distance_travelled_so_far_ = 0;
    distance_to_travel = 0;
    wait_ = 0;
    upcoming_intersection_ = nullptr;
  }

  double GetAccelerationForRoad();
  void UpdateAccelerationForIntersection(double& accel);

  void UpdateAccelerationForCars(const std::vector<Car*>::iterator& car_begin,
                                 const std::vector<Car*>::iterator& car_end,
                                 const nacb::Vec2f& dpos, double dpos_len,
                                 double& accel);

  void HandleStepExitParkingLot();
  void HandleStepFindParkingSpot();
  void HandleStepDriveRoad(double t_abs, double dt);

public:
  Level& level_;
  plan::Planner& planner_;
  int car_id_ = 0;

  // The current plan
  std::vector<plan::Stage> plan_;

  // Indices into the current state of the plan.
  int stage_index_ = 0;
  int segment_index_ = 0;
  int sub_index_ = 0;

  // State
  nacb::Vec2f pos_;
  double speed_ = 0;
  double max_speed_ = 0;

  // Fields for navigation
  double wait_ = 0;
  double distance_to_travel = 0;
  double distance_travelled_so_far_ = 0;
  IntersectionControl* upcoming_intersection_ = nullptr;
  std::set<IntersectionControl*> ignore_intersections_;

  // Fields used for accounting of timing stats
  int start_, end_;
  double travel_time_ = 0;

  // Implied fields, used for rendering only
  nacb::Vec2f last_pos_;
  float angle_ = 0;
  float wheel_rot_ = 0;
  float wheel_anim_ = 0;
  int breaking_ = 0;
};

#endif // DSIM_CAR_H_
