#pragma once

#include "geometry/angles.hpp"
#include "geometry/point.hpp"

constexpr double kEpsilon = 1e-10;

namespace geometry {

inline bool IsRightTurn(const Point& a, const Point& b, const Point& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) < kEpsilon;
}

inline bool IsLeftTurn(const Point& a, const Point& b, const Point& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > kEpsilon;
}

inline double GetAngle(const Point& a, const Point& b) {
  return std::atan2(b.y - a.y, b.x - a.x);
}

}  // namespace geometry

inline bool operator==(const Point& a, const Point& b) {
  return std::abs(a.x - b.x) < kEpsilon && std::abs(a.y - b.y) < kEpsilon;
}