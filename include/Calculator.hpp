#pragma once

#include <vector>

#include "DynamicSlidingConvex2DHull.hpp"
#include "geometry/geometry_utils.hpp"

/**
 * Calculates angles using a sliding convex hull
 * @param inputs Input values to process
 * @param result Vector to store the calculated angles
 * @param window Window size for the sliding convex hull
 */
void calculate(const std::vector<double> &inputs, std::vector<Angles> &result,
               int window) {
  DynamicSlidingConvex2DHull convex_hull;
  for (int i = 0; i < window; ++i) {
    convex_hull.Add(inputs[i]);
    result[i] = {convex_hull.upperTangentFromRightmostPoint(),
                 convex_hull.lowerTangentFromRightmostPoint()};
  }
  for (int i = window; i < inputs.size(); ++i) {
    convex_hull.Remove();
    convex_hull.Add(inputs[i]);
    result[i] = {convex_hull.upperTangentFromRightmostPoint(),
                 convex_hull.lowerTangentFromRightmostPoint()};
  }
}