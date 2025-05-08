#pragma once
#include <cmath>

struct Point {
  int x;
  double y;
  Point(int x, double y) : x(x), y(y) {}
  bool operator==(const Point& other) const = default;
};
