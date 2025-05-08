#pragma once

#include <memory>

#include "geometry/point.hpp"

struct Node {
  Node(const Point& p) : p(p) {}
  Point p;
};