#ifndef DSIM_PARKING_LOT_H_
#define  DSIM_PARKING_LOT_H_ 1

#include <vector>
#include <string>
#include <nmath/vec2.h>
#include "level.pb.h"

class ParkingSpot {
public:
  ParkingSpot(double x, double y): pos(x, y) {}
  nacb::Vec2f pos;
};

class ParkingLot {
public:
  ParkingLot(const dsim::ParkingLot& lot):
    name(lot.name()),
    pos(lot.position().x(), lot.position().y()),
    width_(lot.width()),
    height_(lot.height()) {
    for (const auto& spot : lot.parking_spots()) {
      parking_spots.push_back(ParkingSpot(spot.position().x(), spot.position().y()));
    }
  }

  const double width() const { return width_; }
  const double height() const { return height_; }
  
  std::string name;
  nacb::Vec2f pos;
  double width_, height_;
  std::vector<ParkingSpot> parking_spots;
};

#endif  // DSIM_PARKING_LOT_H_
