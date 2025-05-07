#pragma once
#include "geometry/geometry_utils.hpp"
#include "wang_dynamic_hull/WangConvexHull.hpp"

class DynamicSlidingConvex2DHull {
 public:
  DynamicSlidingConvex2DHull() = default;
  void Add(double y);
  void Remove();
  double upperTangentFromRightmostPoint() const;
  double lowerTangentFromRightmostPoint() const;

 private:
  int nextX_, leftX_;
  WangConvexHull convex_hull_;
};

void DynamicSlidingConvex2DHull::Add(double y) {
  convex_hull_.pushRight({nextX_, y});
  nextX_++;
}

void DynamicSlidingConvex2DHull::Remove() {
  convex_hull_.popLeft();
  leftX_++;
}

double DynamicSlidingConvex2DHull::upperTangentFromRightmostPoint() const {
  return convex_hull_.upperTangentFromRightmostPoint();
}

double DynamicSlidingConvex2DHull::lowerTangentFromRightmostPoint() const {
  return convex_hull_.lowerTangentFromRightmostPoint();
}
