#ifndef DSIM_CAR_H_
#define DSIM_CAR_H 1

#include <set>
#include <vector>
#include <nmath/vec2.h>
#include "level/level.h"
#include "plan/stage.h"
#include "plan/plan.h"


class Car {
public:
  static constexpr double kAccelerationRate = 4.0;
  static constexpr double kDecelerationRate = 8.0;
  static constexpr double kParkingLotSpeedLimit = 2.0;
  static constexpr double kWheelSize = 0.39 / 4.0;

  struct RandomParameters  {
    static double GenerateRandomSpeed() {
      return 12 + 4.0 * rand() / RAND_MAX;
    }
    // How long we will wait in a parking spot
    static double GenerateRandomWait() {
      return 2.0 * rand() / RAND_MAX + 0.00001;
    }
  };
  
  Car(Level& level, plan::Planner& planner, int id, int start, int end) :
    level_(level), planner_(planner), car_id_(id), start_(start), end_(end), travel_time_(0) {
    plan_ = planner.Plan(level.parking_lots[start], id % level.parking_lots[start].parking_spots.size(),
                         level.parking_lots[end], id % level.parking_lots[end].parking_spots.size());
    stage_index_ = 0;
    pos_ = (level.parking_lots[start].parking_spots[0].pos + level.parking_lots[start].pos);
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

  void BuildNewPlan() {
    start_ = level_.PickRandomParkingLot();
    while (start_ == end_) {
      start_ = level_.PickRandomParkingLot();
    }
    plan_ = planner_.Plan(level_.parking_lots[end_], car_id_ % level_.parking_lots[end_].parking_spots.size(),
                          level_.parking_lots[start_], car_id_ % level_.parking_lots[start_].parking_spots.size());
    std::swap(start_, end_);

    stage_index_ = 0;
    segment_index_ = 0;
    sub_index_ = 0;
    distance_travelled_so_far_ = 0;
    distance_to_travel = 0;
    wait_ = 0;
    upcoming_intersection_ = nullptr;
  }

  double GetAccelerationForRoad() {
    double accel = 1;
    RoadSegment* rs = road_segment();
    if (rs) {
      plan::Stage& stage = plan_[stage_index_];      
      auto* segment = &stage.segments[segment_index_];
      nacb::Vec2d p1, p2, p3;
      segment->GetPoints(sub_index_, &p1, &p2);
      
      plan::Stage::Segment* next_segment = nullptr;
      double max_upcoming_speed = rs->speed_limit;
      if (stage.GetUpcomingPoint(segment_index_, sub_index_, &next_segment, &p3)) {
        if (next_segment) {
          max_upcoming_speed = next_segment->road_segment->speed_limit;
        }
        nacb::Vec2d d1 = (p2 - p1);
        nacb::Vec2d d2 = (p3 - p2);
        d1.normalize();
        d2.normalize();

        if (d1.dot(d2) <= 1e-4) {
          const double kMaxCornerSpeed = 2.5;
          max_upcoming_speed = std::min(max_upcoming_speed, kMaxCornerSpeed);
        }
        const double speed_to_shed = speed_ - max_upcoming_speed;
        if (speed_to_shed > 0) {
          const double distance_to_corner = (p2 - pos_).len();
          const double time_to_corner = distance_to_corner / std::max(0.1, speed_);
          const double time_to_shed_speed = speed_to_shed / kDecelerationRate;
          if (time_to_corner <= time_to_shed_speed) {
            accel = -1;
            if (time_to_corner < 0.5 * time_to_shed_speed) {
              accel = -4;
            }
            if (time_to_corner < 0.75 * time_to_shed_speed) {
              accel = -2;
            }
          }
        }
      }
    }
    return accel;
  }
  void UpdateAccelerationForIntersection(double& accel) {
    if (upcoming_intersection_) {
      const auto d_pos = (upcoming_intersection_->pos - pos_);
      const double d = d_pos.len();
      const bool should_slow_down =
        upcoming_intersection_->IsStopSign() ||
        (upcoming_intersection_->IsLight() &&
         !upcoming_intersection_->IsGreen(road_segment()));

      if (should_slow_down) {
        if (d < 0.5) accel = -8;
        else if (d < 1) accel = -4;
        else if (d < 2) accel = -2;
        if (speed_ <= 1 && d >= 1.0) {
          accel = 1.0;
        }
      }
      if (upcoming_intersection_->IsStopSign()) {
        // If we've stopped and we're close enough. If there are no cars, then go.
        if (speed_ <= 0.01 && d <= 1.5) {
          auto* segment = &plan_[stage_index_].segments[segment_index_];
          const RoadSegment* my_road_segment = segment->road_segment;
          bool has_car = false;
          for (const auto& incoming_segs : upcoming_intersection_->incoming_segments) {
            if (incoming_segs.first == my_road_segment) continue;
            if (incoming_segs.first->cars[incoming_segs.second].size()) has_car = true;
          }
          if (!has_car) {
            ignore_intersections_.insert(upcoming_intersection_);
          }
        }
      }
    }
  }

  void UpdateAccelerationForCars(std::vector<Car>& cars,
                                 const nacb::Vec2d& dpos,
                                 double dpos_len,
                                 double& accel) {
    for (const auto& car: cars) {
      if (&car == this) continue;
      // Skip cars that are waiting on an intersection that is different than ours
      if (road_segment() && car.upcoming_intersection_ &&
          road_segment() != car.road_segment()) continue;
      const auto d_car = (car.pos_ - pos_);
      const double d = d_car.len();

      if (d / std::min(std::max(speed_, 1.0), 4.0) > 1.0) continue;
      if (d_car.dot(dpos) > 0.70 * d * dpos_len) {
        if (d < 1) {
          accel += -16;  // If we are 1 unit away, collision is imminent. Try real hard to stop
        } else if (d < 2) {
          accel += -8; // If we are 2 unit away, try "hard" to stop
        } else {
          accel += -2; // Else, try moderatly hard to stop.
        }
      }
    }
  }
  
  
  void Step(std::vector<Car>& cars,
            double t_abs,
            double dt) {
    breaking_ = std::max(0, (int)breaking_ - 1);
    if (wait_ > 0) {
      wait_ -= dt;
      if (wait_ < 0) {
        BuildNewPlan();
      } else {
        return;
      }
    }
    // Compute direction of travel
    const auto dpos = (pos_ -  last_pos_);
    const double dpos_len = dpos.len();
    
    // Acceleration / deceleration
    // Need to decelerate when approaching a stop (or when getting too close to another)
    {
      double accel = GetAccelerationForRoad();
      
      UpdateAccelerationForIntersection(accel);
      UpdateAccelerationForCars(cars, dpos, dpos_len, accel);
      
      // TODO(birkbeck): move this into GetAccelerationForRoad()
      RoadSegment* rs = road_segment();
      const double speed_limit = rs ? rs->speed_limit : kParkingLotSpeedLimit;
      if (speed_ > 1.05 * speed_limit) {
        accel = -2; // Try moderately hard to decrease speed.
      } else if (speed_ > speed_limit) {
        speed_ = speed_limit;
      }
      
      
      // Apply accleration
      if (accel > 0) {
        speed_ += accel * kAccelerationRate * dt;
        if (speed_ >= max_speed_) {
          speed_ = max_speed_;
        }
      } else {
        speed_ += accel * kDecelerationRate * dt;
        if (accel == -1) {
          if (speed_ <= 1.0) {
            speed_ = 1.0;
          }
        } else {
          if (speed_ < 0) speed_ = 0;
        }
        breaking_ = std::min(breaking_ + 1, 4);
      }
    }
    
    // Update animation variables
    if (dpos_len > 0) {
      const double angle_before = angle_;
      // Fudge angle interpolation so we don't just snap orientation.
      angle_ = atan2(sin(angle_) * 0.4 + 0.6 * dpos.y / dpos_len,
                     cos(angle_) * 0.4 + 0.6 * dpos.x / dpos_len);

      // This is a total hack to get the wheel "turn" angle to look "right"
      wheel_rot_ = 4 * atan2(sin(angle_ - angle_before),
                             cos(angle_ - angle_before));
    }
    travel_time_ += dt;
    distance_travelled_so_far_ += speed_ * dt;
    distance_to_travel += speed_ * dt;
    last_pos_ = pos_;


    if (plan_[stage_index_].type == plan::Stage::EXIT_PARKING_LOT) {
      const auto* segment = &plan_[stage_index_].segments[0];
      const auto& p1 = segment->road_segment->points[segment->start_index];
      nacb::Vec2d dir = p1 - plan_[stage_index_].point;
      const double d = dir.normalize();

      nacb::Vec2d e1, e2;
      plan_[(stage_index_ + 1)].segments[0].GetPoints(0, &e1, &e2);
      nacb::Vec2d offset = e2 - e1;
      offset.normalize();
      offset *= std::max(0.3, dir.dot(offset) / 8.0);

      const double t = std::min(d, distance_to_travel) / d;
      pos_ = CubicBezier(plan_[stage_index_].point,
                         plan_[stage_index_].point + offset,
                         p1 - offset, p1, t);

      if (distance_to_travel >= d) {
        distance_travelled_so_far_ = 0;
        distance_to_travel = 0;
        stage_index_++;
      }
    } else if (plan_[stage_index_].type == plan::Stage::FIND_PARKING_SPOT) {
      const auto* segment = &plan_[stage_index_].segments[0];
      const auto& p1 = segment->road_segment->points[segment->start_index];
      nacb::Vec2d dir = plan_[stage_index_].point - p1;
      const double d = dir.normalize();

      nacb::Vec2d e1, e2;
      const auto& prev_segments = plan_[stage_index_ - 1].segments;
      prev_segments.back().GetPoints(prev_segments.back().end_index - prev_segments.back().start_index - 1 , &e1, &e2);
      nacb::Vec2d offset = e2 - e1;
      offset.normalize();
      offset *= std::max(0.3, dir.dot(offset) / 8.0);

      pos_ = CubicBezier(p1, p1 + offset,
                         plan_[stage_index_].point - offset,
                         plan_[stage_index_].point,
                         std::min(d, distance_to_travel) / d);
      if (distance_to_travel >= d) {
        // Now we've completed the journey, record the time
        level_.stats.AddObservation(plan_[0].parking_lot->name,
                                    plan_[2].parking_lot->name,
                                    travel_time_);
        travel_time_ = 0;
        distance_travelled_so_far_ = 0;
        distance_to_travel = 0;
        stage_index_ = 0;
        speed_ = 0;
        wait_ = RandomParameters::GenerateRandomWait();
      }
    } else {
      plan::Stage& stage = plan_[stage_index_];
      // TODO(birkbeck): make a segment iterator.
      if (segment_index_ < (int)stage.segments.size()) {
        auto* segment = &stage.segments[segment_index_];

        upcoming_intersection_ = nullptr;
        segment->road_segment->cars[(segment->start_index + sub_index_) % segment->road_segment->points.size()].erase(this);
        
        while ((sub_index_ + segment->start_index) % (int)segment->road_segment->points.size() != segment->end_index &&
               segment_index_ < (int)stage.segments.size()) {
          int i1, i2;
          segment->GetPoint(sub_index_, &i1);
          nacb::Vec2d p2 = segment->GetPoint(sub_index_ + 1, &i2);
          if (segment->road_segment->intersections[i2]) {
            if (!ignore_intersections_.count(segment->road_segment->intersections[i2])) {
              upcoming_intersection_ = segment->road_segment->intersections[i2];
            }
          }
          if (segment->GetInterpolatedPoint(sub_index_, &distance_to_travel, &pos_)) {
            segment->road_segment->cars[i1].insert(this);
            segment->road_segment->AddSpeedEstimate(t_abs, (pos_ - last_pos_).len() / dt);
            break;
          }
          const bool zero_length =  (segment->info[sub_index_].segment_length == 0);
          pos_ = p2;
          ignore_intersections_.clear();
          sub_index_++;

          // This is a special hack for a looped segment
          if (zero_length) {
            continue;
          }

          // Advance iterator
          if ((sub_index_ + segment->start_index) % (int)segment->road_segment->points.size() == segment->end_index) {
            sub_index_ = 0;
            segment_index_++;
            
            if (segment_index_ < (int)stage.segments.size()) {
              segment = &stage.segments[segment_index_];
            } else {
              // Advance stage
              stage_index_++;
              segment = nullptr;
              break;
            }
          }
        }
      }
    }
    wheel_anim_ += speed_ / (2 * M_PI * kWheelSize) * dt;
  }

  const nacb::Vec2d& pos() const {
    return pos_;
  }
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
  nacb::Vec2d pos_;
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
  nacb::Vec2d last_pos_;
  float angle_ = 0;
  float wheel_rot_ = 0;
  float wheel_anim_ = 0;  
  int breaking_ = 0;
};


#endif  // DSIM_CAR_H_
