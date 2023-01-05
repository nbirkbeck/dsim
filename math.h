#ifndef DSIM_MATH_H_
#define DSIM_MATH_H_ 1

#include <nmath/vec2.h>
#include "level.pb.h"

inline dsim::Point Add(const dsim::Point& p1, const dsim::Point p2) {
  dsim::Point p;
  p.set_x(p1.x() + p2.x());
  p.set_y(p1.y() + p2.y());
  return p;
}

inline dsim::Point Sub(const dsim::Point& p1, const dsim::Point p2) {
  dsim::Point p;
  p.set_x(p1.x() - p2.x());
  p.set_y(p1.y() - p2.y());
  return p;
}

inline double Distance(const dsim::Point& p1, const dsim::Point p2) {
  const double dx = p1.x() - p2.x();
  const double dy = p1.y() - p2.y();
  return sqrt(dx * dx + dy * dy);
}

inline dsim::Point Interpolate(const dsim::Point& p1, const dsim::Point p2, double t) {
  dsim::Point p;
  const double dx = p2.x() - p1.x();
  const double dy = p2.y() - p1.y();
  p.set_x(p1.x() + dx * t);
  p.set_y(p1.y() + dy * t);
  return p;
}

inline bool IsNear(const double x, const double y) {
  return abs(x - y) < 1e-5;
}

inline nacb::Vec2d Interpolate(const nacb::Vec2d& p1, const nacb::Vec2d& p2, double t) {
  return p1 * (1.0 - t) + p2 * t;
}

inline bool IsOnRectangle(const nacb::Vec2d& rect_center,
                          double width, double height,
                          const nacb::Vec2d& pos) {
  const float x0 = rect_center.x - width / 2;
  const float x1 = rect_center.x + width / 2;
  const float y0 = rect_center.y - height / 2;
  const float y1 = rect_center.y + height / 2;
 
  return
    ((IsNear(pos.x, x0) || IsNear(pos.x, x1)) &&
     (y0 <= pos.y && pos.y <= y1)) ||
    ((IsNear(pos.y, y0) || IsNear(pos.y, y1)) &&
     (x0 <= pos.x && pos.x <= x1));
}

inline double EstimateCurveLength(const nacb::Vec2d& a,
                                  const nacb::Vec2d& b,
                                  const nacb::Vec2d& c) {
  nacb::Vec2d prev;
  double len = 0;
  for (int i = 0; i <= 10; ++i) {
    double t = double(i) / 10;
    nacb::Vec2d point = Interpolate(Interpolate(a, b, t), Interpolate(b, c, t), t);
    if (i) {
      len += (point - prev).len();
    }
    prev = point;
  }
  return len;
}

inline uint64_t Hash(const dsim::Point& pos) {
  int32_t x = (int32_t)pos.x();
  int32_t y = (int32_t)pos.y();
  return (((uint64_t)x) << 32) | ((uint64_t)y);
}

inline uint64_t Hash(const nacb::Vec2d& pos) {
  int32_t x = (int32_t)pos.x;
  int32_t y = (int32_t)pos.y;
  return (((uint64_t)x) << 32) | ((uint64_t)y);
}

struct PointEqual {
  bool operator()(const dsim::Point& p1, const dsim::Point& p2) const {
    return p1.x() == p2.x() && p1.y() == p2.y();
  }
  bool operator()(const nacb::Vec2d& p1, const nacb::Vec2d& p2) const {
    return p1.x == p2.x && p1.y == p2.y;
  }
};

struct PointHash {
  size_t operator()(const dsim::Point& pos) const {
    return Hash(pos);
  }
  size_t operator()(const nacb::Vec2d& pos) const {
    return Hash(pos);
  }
};



#endif  // DSIM_MATH_H_
