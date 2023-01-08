#ifndef DSIM_ROAD_SEGMENT_H_
#define DSIM_ROAD_SEGMENT_H_ 1

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include <deque>
#include <nmath/vec2.h>
#include <string>
#include <unordered_map>
#include <vector>

#include "level/intersection_control.h"

class Car;

class RoadSegment {
public:
  static constexpr int kMaxDeckSize = 10;
  struct SpeedEstimateEntry {
    int t = 0;
    double avg_speed = 0;
    double num_avg_speed = 0;
    SpeedEstimateEntry(int time = 0) : t(time) {}

    bool operator==(const SpeedEstimateEntry& other) const {
      return t == other.t && avg_speed == other.avg_speed;
    }
    bool operator!=(const SpeedEstimateEntry& other) const {
      return !((*this) == (other));
    }
  };
  RoadSegment(const dsim::RoadSegment& seg)
      : name(seg.name()), speed_limit(seg.speed_limit()) {
    for (const auto& p : seg.points()) {
      points.push_back(nacb::Vec2f(p.x(), p.y()));
    }
    cars.resize(seg.points().size());

    if (speed_limit <= 0)
      speed_limit = 8;

    UpdateSpeedEstimate();
  }

  void AddSpeedEstimate(double t, double sp) {
    if (deck.size() > kMaxDeckSize) {
      deck.pop_front();
    }
    if (deck.empty() || int(t) != deck.back().t) {
      deck.push_back(SpeedEstimateEntry(int(t)));
    }
    deck.back().avg_speed += sp;
    deck.back().num_avg_speed++;
  }

  void UpdateSpeedEstimate() {
    double num_avg_speed = 1.0 / 8.0;
    avg_speed = num_avg_speed * speed_limit;
    for (const auto& entry : deck) {
      avg_speed += entry.avg_speed;
      num_avg_speed += entry.num_avg_speed;
    }
    avg_speed /= num_avg_speed;
  }

  void SetIntersections(std::vector<IntersectionControl>& isects) {
    for (int j = 0; j < (int)isects.size(); ++j) {
      for (int i = 1; i < (int)points.size(); ++i) {
        if ((points[i] - isects[j].pos).len() < 1e-6) {
          const auto& dir = points[i] - points[i - 1];
          if (!isects[j].IsDirectional() ||
              isects[j].IsEnforcedDirection(dir)) {
            intersections[i] = &isects[j];
          }
          isects[j].incoming_segments.push_back(
              std::pair<RoadSegment*, int>(this, i - 1));
        }
      }
    }
  }

  double GetAverageSpeed() {
    if (deck.empty() || deck.back() != last_updated) {
      UpdateSpeedEstimate();
      if (deck.size())
        last_updated = deck.back();
    }
    return avg_speed;
  }

  std::string name;
  double speed_limit;
  std::vector<nacb::Vec2f> points;
  absl::flat_hash_map<int, IntersectionControl*> intersections;
  std::vector<absl::flat_hash_set<const Car*>> cars;

  SpeedEstimateEntry last_updated;
  std::deque<SpeedEstimateEntry> deck;
  double avg_speed = 0;
};

#endif
