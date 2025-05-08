#pragma once
#include "geometry/geometry_utils.hpp"
#include "wang_dynamic_hull/wang_upper_convex_hull.hpp"

class DynamicSlidingConvex2DHull {
 public:
  DynamicSlidingConvex2DHull() = default;
  void Add(double y);
  void Remove();
  double upperTangentFromRightmostPoint() const;
  double lowerTangentFromRightmostPoint() const;

 private:
  int nextX_ = 0, leftX_ = 0;
  WangConvexHull convex_hull_upper_;
  WangConvexHull convex_hull_lower_;
};

void DynamicSlidingConvex2DHull::Add(double y) {
  convex_hull_upper_.pushRight({nextX_, y});
  convex_hull_lower_.pushRight({nextX_, -y});
  nextX_++;
}

void DynamicSlidingConvex2DHull::Remove() {
  convex_hull_upper_.popLeft();
  convex_hull_lower_.popLeft();
  leftX_++;
}

double DynamicSlidingConvex2DHull::upperTangentFromRightmostPoint() const {
  return convex_hull_upper_.upperTangentFromRightmostPoint();
}

double DynamicSlidingConvex2DHull::lowerTangentFromRightmostPoint() const {
  return -convex_hull_lower_.upperTangentFromRightmostPoint();
}
