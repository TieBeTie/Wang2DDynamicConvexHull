#pragma once

#include "geometry/angles.hpp"
#include "geometry/point.hpp"

namespace geometry {

inline bool IsRightTurn(const Point& a, const Point& b, const Point& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) < 0;
}

inline bool IsLeftTurn(const Point& a, const Point& b, const Point& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0;
}

inline double GetAngle(const Point& a, const Point& b) {
  return std::atan2(b.y - a.y, b.x - a.x);
}

}  // namespace geometry