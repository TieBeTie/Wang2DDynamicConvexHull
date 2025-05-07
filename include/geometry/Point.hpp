#pragma once
#include <cmath>

struct Point {
  int x;
  double y;
  Point(int x, double y) : x(x), y(y) {}
  bool operator==(const Point& other) const = default;
};

inline double cross(const Point& o, const Point& a, const Point& b);

inline double slope(const Point& a, const Point& b);

inline bool IsRightTurn(const Point& a, const Point& b, const Point& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) < 0;
}

inline bool IsLeftTurn(const Point& a, const Point& b, const Point& c) {
  return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x) > 0;
}
