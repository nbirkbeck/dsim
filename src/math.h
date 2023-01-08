#ifndef DSIM_MATH_H_
#define DSIM_MATH_H_ 1

#include "level.pb.h"
#include <nmath/vec2.h>

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

inline dsim::Point Interpolate(const dsim::Point& p1, const dsim::Point p2,
                               double t) {
  dsim::Point p;
  const double dx = p2.x() - p1.x();
  const double dy = p2.y() - p1.y();
  p.set_x(p1.x() + dx * t);
  p.set_y(p1.y() + dy * t);
  return p;
}

inline float FastLength(const nacb::Vec2f& v) {
  // Old call-sites did return v.len(), which uses sqrt
  return sqrtf(v.x * v.x + v.y * v.y);
}

inline bool IsNear(const double x, const double y) { return abs(x - y) < 1e-5; }

inline nacb::Vec2f CubicBezier(const nacb::Vec2f& p1, const nacb::Vec2f& p2,
                               const nacb::Vec2f& p3, const nacb::Vec2f& p4,
                               double t) {
  const float t1 = (1.0f - t);
  return p1 * (t1 * t1 * t1) + p2 * (3 * (t1 * t1 * t)) +
         p3 * (3 * t1 * t * t) + p4 * (t * t * t);
}

inline nacb::Vec2f QuadraticBezier(const nacb::Vec2f& p1, const nacb::Vec2f& p2,
                                   const nacb::Vec2f& p3, double t) {
  const float t1 = (1.0 - t);
  return p1 * (t1 * t1) + p2 * (2.f * (t1 * t)) + p3 * (t * t);
}

inline nacb::Vec2f Interpolate(const nacb::Vec2f& p1, const nacb::Vec2f& p2,
                               double t) {
  return nacb::Vec2f(p1.x * (1.0 - t) + p2.x * t, p1.y * (1.0 - t) + p2.y * t);
}

inline bool IsOnRectangle(const nacb::Vec2f& rect_center, double width,
                          double height, const nacb::Vec2f& pos) {
  const float x0 = rect_center.x - width / 2;
  const float x1 = rect_center.x + width / 2;
  const float y0 = rect_center.y - height / 2;
  const float y1 = rect_center.y + height / 2;

  return ((IsNear(pos.x, x0) || IsNear(pos.x, x1)) &&
          (y0 <= pos.y && pos.y <= y1)) ||
         ((IsNear(pos.y, y0) || IsNear(pos.y, y1)) &&
          (x0 <= pos.x && pos.x <= x1));
}

inline double EstimateCurveLength(const nacb::Vec2f& a, const nacb::Vec2f& b,
                                  const nacb::Vec2f& c) {
  nacb::Vec2f prev;
  double len = 0;
  for (int i = 0; i <= 6; ++i) {
    double t = double(i) / 6;
    nacb::Vec2f point = QuadraticBezier(a, b, c, t);
    if (i) {
      len += FastLength(point - prev);
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

inline uint64_t Hash(const nacb::Vec2f& pos) {
  int32_t x = (int32_t)pos.x;
  int32_t y = (int32_t)pos.y;
  return (((uint64_t)x) << 32) | ((uint64_t)y);
}

struct PointEqual {
  bool operator()(const dsim::Point& p1, const dsim::Point& p2) const {
    return p1.x() == p2.x() && p1.y() == p2.y();
  }
  bool operator()(const nacb::Vec2f& p1, const nacb::Vec2f& p2) const {
    return p1.x == p2.x && p1.y == p2.y;
  }
};

struct PointHash {
  size_t operator()(const dsim::Point& pos) const { return Hash(pos); }
  size_t operator()(const nacb::Vec2f& pos) const { return Hash(pos); }
};

#endif // DSIM_MATH_H_
