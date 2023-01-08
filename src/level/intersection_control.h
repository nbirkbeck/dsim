#ifndef DSIM_INTERSECTION_CONTROL_H_
#define DSIM_INTERSECTION_CONTROL_H_ 1

#include "level.pb.h"
#include <vector>

class RoadSegment;

class IntersectionControl {
public:
  IntersectionControl(const dsim::IntersectionControl& control)
      : type(control.type()), name(control.name()),
        pos(control.position().x(), control.position().y()),
        dir(control.dir().x(), control.dir().y()), time(control.time()) {}
  bool IsDirectional() const {
    if (type == dsim::IntersectionControl::STOP)
      return true;
    if (type == dsim::IntersectionControl::YIELD)
      return true;
    return false;
  }
  bool IsEnforcedDirection(const nacb::Vec2f& test) const {
    return dir.dot(test) >= 0.999 * test.len();
  }
  bool IsStopSign() const { return type == dsim::IntersectionControl::STOP; }
  bool IsLight() const { return type == dsim::IntersectionControl::LIGHTS; }
  bool IsGreen(RoadSegment* segment) const {
    for (int j = 0; j < (int)incoming_segments.size(); ++j) {
      if (segment == incoming_segments[j].first) {
        return parity == j;
      }
    }
    return false;
  }
  void Step(double dt) {
    elapsed += dt;
    if (elapsed >= time) {
      parity = (parity + 1) % incoming_segments.size();
      elapsed = 0;
    }
  }

  std::vector<std::pair<RoadSegment*, int>> incoming_segments;

  dsim::IntersectionControl::Type type;
  std::string name;
  nacb::Vec2f pos;
  nacb::Vec2f dir;
  double time;
  int parity = false;
  double elapsed = 0;
};

#endif
