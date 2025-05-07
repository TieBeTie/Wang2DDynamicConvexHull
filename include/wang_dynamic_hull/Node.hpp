#pragma once

#include <memory>

#include "geometry/Point.hpp"

struct Node {
  Node(const Point& p) : p(p) {}
  Point p;
};