#ifndef DSIM_LEVEL_H_
#define DSIM_LEVEL_H_ 1

#include <vector>
#include "level/trip_stats.h"
#include "level/parking_lot.h"
#include "level/road_segment.h"
#include "level.pb.h"

class Level {
public:
  Level(const dsim::Level& level) {
    for (const auto& lot : level.parking_lot()) {
      parking_lots.push_back(ParkingLot(lot));
    }
    for (const auto& seg : level.road_segment()) {
      road_segments.push_back(RoadSegment(seg));
    }
    for (const auto& isect : level.intersection_control()) {
      intersections.push_back(IntersectionControl(isect));
    }
    for (auto& seg : road_segments) {
      seg.SetIntersections(intersections);
    }
  }

  void GetBounds(nacb::Vec2f* min_point,
                 nacb::Vec2f* max_point) {
    for (const auto& seg : road_segments) {
      for (const auto& p : seg.points) {
        *min_point = min_point->min(p);
        *max_point = max_point->max(p);
      }
    }
    for (const auto& lot: parking_lots) {
      for (const auto& spot: lot.parking_spots) {
        *min_point = min_point->min(spot.pos + lot.pos);
        *max_point = max_point->max(spot.pos + lot.pos);
      }
    }
  }

  int PickRandomParkingLot() {
    double r = (double)rand() / RAND_MAX;
    int num_spots = 0;
    for (int i = 0; i < (int)parking_lots.size(); ++i) {
      num_spots += parking_lots[i].parking_spots.size();
    }
    double val = 0;
    for (int i = 0; i < (int)parking_lots.size(); ++i) {
      val += (double)parking_lots[i].parking_spots.size() / num_spots;
      if (val > r) return i;
    }
    return (int)parking_lots.size() - 1;
  }

  TripStats stats;
  std::vector<ParkingLot> parking_lots;
  std::vector<RoadSegment> road_segments;
  std::vector<IntersectionControl> intersections;
};


#endif  // DSIM_LEVEL_H_
