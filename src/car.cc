#include "car.h"

// Old implementations (slightly slower, but more readable)
//#define CAR_ACCELERATION_FOR_CARS_READABLE
//#define CAR_ATAN_WHEEL_INTERP

double Car::GetAccelerationForRoad() {
  double accel = 1;
  RoadSegment* rs = road_segment();
  if (rs) {
    plan::Stage& stage = plan_[stage_index_];
    auto* segment = &stage.segments[segment_index_];
    nacb::Vec2f p1, p2, p3;
    segment->GetPoints(sub_index_, &p1, &p2);

    plan::Stage::Segment* next_segment = nullptr;
    double max_upcoming_speed = rs->speed_limit;
    if (stage.GetUpcomingPoint(segment_index_, sub_index_, &next_segment,
                               &p3)) {
      if (next_segment) {
        max_upcoming_speed = next_segment->road_segment->speed_limit;
      }
      double d1_dot_d2 = 0;
      double rhs = 0;
      {
        nacb::Vec2f d1 = (p2 - p1);
        nacb::Vec2f d2 = (p3 - p2);
        d1_dot_d2 = d1.dot(d2);
        rhs = 1e-4 * (FastLength(d1) * FastLength(d2));
      }
      
      if (d1_dot_d2 <= rhs) {
        const double kMaxCornerSpeed = 2.5;
        max_upcoming_speed = std::min(max_upcoming_speed, kMaxCornerSpeed);
      }
      const double speed_to_shed = speed_ - max_upcoming_speed;
      if (speed_to_shed > 0) {
        const double distance_to_corner = FastLength(p2 - pos_);
        const double time_to_corner =
            distance_to_corner / std::max(0.1, speed_);
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

void Car::UpdateAccelerationForIntersection(double& accel) {
  if (upcoming_intersection_) {
    const auto d_pos = (upcoming_intersection_->pos - pos_);
    const double d = d_pos.len();
    const bool should_slow_down =
        upcoming_intersection_->IsStopSign() ||
        (upcoming_intersection_->IsLight() &&
         !upcoming_intersection_->IsGreen(road_segment()));

    if (should_slow_down) {
      if (d < 0.5)
        accel = 1; // At this point, just run it...
      else if (d < 1)
        accel = -8;
      else if (d < 2)
        accel = -2;
      else if (d < 3)
        accel = -1;
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
        for (const auto& incoming_segs :
             upcoming_intersection_->incoming_segments) {
          if (incoming_segs.first == my_road_segment)
            continue;
          if (incoming_segs.first->cars[incoming_segs.second].size())
            has_car = true;
        }
        if (!has_car) {
          ignore_intersections_.insert(upcoming_intersection_);
        }
      }
    }
  }
}

void Car::UpdateAccelerationForCars(
    const std::vector<Car*>::iterator& car_begin,
    const std::vector<Car*>::iterator& car_end, const nacb::Vec2f& dpos,
    double dpos_len, double& accel) {
  for (auto car_p = car_begin; car_p != car_end; car_p++) {
    const auto& car = **car_p;
    if (&car == this)
      continue;
    // Skip cars that are waiting on an intersection that is different than ours
    if (road_segment() && car.upcoming_intersection_ &&
        road_segment() != car.road_segment())
      continue;
    const auto d_car = (car.pos_ - pos_);

#ifdef CAR_ACCELERATION_FOR_CARS_READABLE
    const double d = FastLength(d_car);
    if (d / std::min(std::max(speed_, 1.0), 4.0) > 1.0)
      continue;
    const double dot = d_car.dot(dpos);
    const double rhs = 0.70 * d * dpos_len;
    if (dot > rhs) {
      if (d < 1) {
        accel += -16; // If we are 1 unit away, collision is imminent. Try real
                      // hard to stop
      } else if (d < 2) {
        accel += -8; // If we are 2 unit away, try "hard" to stop
      } else {
        accel += -2; // Else, try moderatly hard to stop.
      }
    }
#else
    const double d2 = d_car.dot(d_car);
    const double denom = std::min(std::max(speed_, 1.0), 4.0);
    if (d2 > denom * denom) continue;
    const double dot = d_car.dot(dpos);
    if (dot <= 0) continue;
    const double rhs2 = 0.70 * dpos_len;
    if (dot * dot > rhs2 * rhs2 * d2) {
      if (d2 < 1) {
        accel += -16; // If we are 1 unit away, collision is imminent. Try real
                      // hard to stop
      } else if (d2 < 4) {
        accel += -8; // If we are 2 unit away, try "hard" to stop
      } else {
        accel += -2; // Else, try moderatly hard to stop.
      }
    }
#endif
  }
}

void Car::HandleStepExitParkingLot() {
   const auto* segment = &plan_[stage_index_].segments[0];
   const auto& p1 = segment->road_segment->points[segment->start_index];
   nacb::Vec2f dir = p1 - plan_[stage_index_].point;
   const double d = dir.normalize();

   nacb::Vec2f e1, e2;
   plan_[(stage_index_ + 1)].segments[0].GetPoints(0, &e1, &e2);
   nacb::Vec2f offset = e2 - e1;
   offset.normalize();
   offset *= std::max(0.3, dir.dot(offset) / 8.0);

   const double t = std::min(d, distance_to_travel) / d;
   pos_ = CubicBezier(plan_[stage_index_].point,
                      plan_[stage_index_].point + offset, p1 - offset, p1, t);

   if (distance_to_travel >= d) {
     distance_travelled_so_far_ = 0;
     distance_to_travel = 0;
     stage_index_++;
   }
}

void Car::HandleStepFindParkingSpot() {
  const auto* segment = &plan_[stage_index_].segments[0];
  const auto& p1 = segment->road_segment->points[segment->start_index];
  nacb::Vec2f dir = plan_[stage_index_].point - p1;
  const double d = dir.normalize();

  nacb::Vec2f e1, e2;
  const auto& prev_segments = plan_[stage_index_ - 1].segments;
  prev_segments.back().GetPoints(prev_segments.back().end_index -
                                 prev_segments.back().start_index - 1,
                                 &e1, &e2);
  nacb::Vec2f offset = e2 - e1;
  offset.normalize();
  offset *= std::max(0.3, dir.dot(offset) / 8.0);

  pos_ = CubicBezier(p1, p1 + offset, plan_[stage_index_].point - offset,
                     plan_[stage_index_].point,
                     std::min(d, distance_to_travel) / d);
  if (distance_to_travel >= d) {
    // Now we've completed the journey, record the time
    level_.stats.AddObservation(plan_[0].parking_lot->name,
                                plan_[2].parking_lot->name, travel_time_);
    travel_time_ = 0;
    distance_travelled_so_far_ = 0;
    distance_to_travel = 0;
    stage_index_ = 0;
    speed_ = 0;
    wait_ = RandomParameters::GenerateRandomWait();
  }
}

void Car::HandleStepDriveRoad(double t_abs, double dt) {
  plan::Stage& stage = plan_[stage_index_];
  // TODO(birkbeck): make a segment iterator.
  if (segment_index_ < (int)stage.segments.size()) {
    auto* segment = &stage.segments[segment_index_];

    upcoming_intersection_ = nullptr;
    segment->road_segment
      ->cars[(segment->start_index + sub_index_) %
             segment->road_segment->points.size()]
      .erase(this);

    while ((sub_index_ + segment->start_index) %
           (int)segment->road_segment->points.size() !=
           segment->end_index &&
           segment_index_ < (int)stage.segments.size()) {
      int i1, i2;
      segment->GetPoint(sub_index_, &i1);
      nacb::Vec2f p2 = segment->GetPoint(sub_index_ + 1, &i2);
      if (segment->road_segment->intersections[i2]) {
        if (!ignore_intersections_.count(
                                         segment->road_segment->intersections[i2])) {
          upcoming_intersection_ = segment->road_segment->intersections[i2];
        }
      }
      if (segment->GetInterpolatedPoint(sub_index_, &distance_to_travel,
                                        &pos_)) {
        segment->road_segment->cars[i1].insert(this);
        segment->road_segment->AddSpeedEstimate(
                                                t_abs, FastLength(pos_ - last_pos_) / dt);
        break;
      }
      const bool zero_length =
        (segment->info[sub_index_].segment_length == 0);
      pos_ = p2;
      ignore_intersections_.clear();
      sub_index_++;

      // This is a special hack for a looped segment
      if (zero_length) {
        continue;
      }

      // Advance iterator
      if ((sub_index_ + segment->start_index) %
          (int)segment->road_segment->points.size() ==
          segment->end_index) {
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

void Car::Step(const std::vector<Car*>::iterator& car_begin,
               const std::vector<Car*>::iterator& car_end, double t_abs,
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
  const auto dpos = (pos_ - last_pos_);
  const double dpos_len = FastLength(dpos);

  // Update Acceleration / deceleration
  // E.g., need to decelerate when approaching a stop (or when getting too close to
  // another)
  {
    double accel = GetAccelerationForRoad();

    UpdateAccelerationForIntersection(accel);
    UpdateAccelerationForCars(car_begin, car_end, dpos, dpos_len, accel);

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
        if (speed_ < 0)
          speed_ = 0;
      }
      breaking_ = std::min(breaking_ + 1, 4);
    }
  }

  // Update animation variables
  if (dpos_len > 0) {
    const double angle_before = angle_;

#ifdef CAR_ATAN_WHEEL_INTERP
    // Fudge angle interpolation so we don't just snap orientation.
    angle_ = atan2f(sinf(angle_) * 0.4f + 0.6f * dpos.y / dpos_len,
                    cosf(angle_) * 0.4f + 0.6f * dpos.x / dpos_len);
    wheel_rot_ = 4 * atan2f(sinf(angle_ - angle_before), cosf(angle_ - angle_before));
#else
    float new_angle = atan2f(dpos.y, dpos.x);
    float angle_diff = new_angle - angle_;
    if (angle_diff > M_PI) new_angle -= 2 * M_PI;
    if (angle_diff <- M_PI) new_angle += 2 * M_PI;
    angle_ = angle_ * 0.4  + new_angle * 0.6;
    if (angle_ < 0) angle_ += M_PI;
    if (angle_ > 2 * M_PI) angle_ -= 2 * M_PI;
    
    // This is a total hack to get the wheel "turn" angle to look "right"
    float diff = angle_ - angle_before;
    if (diff > M_PI) {
      diff -= 2 * M_PI;
    }
    if (diff < M_PI) {
      diff += 2 * M_PI;
    }
    wheel_rot_ = 4 * diff;
#endif
  }
  travel_time_ += dt;
  distance_travelled_so_far_ += speed_ * dt;
  distance_to_travel += speed_ * dt;
  last_pos_ = pos_;

  if (plan_[stage_index_].type == plan::Stage::EXIT_PARKING_LOT) {
    HandleStepExitParkingLot();
  } else if (plan_[stage_index_].type == plan::Stage::FIND_PARKING_SPOT) {
    HandleStepFindParkingSpot();
  } else {
    HandleStepDriveRoad(t_abs, dt);
  }
  wheel_anim_ += speed_ / (2 * M_PI * kWheelSize) * dt;
}
