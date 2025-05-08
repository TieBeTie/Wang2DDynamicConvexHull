#pragma once

#include <iostream>
#include <list>
#include <memory>

// Forward declaration of the class to befriend
// class WangConvexHull; // Removed this line

#include "wang_upper_convex_hull.hpp"

// Declaration of the friend function
void PrintWangConvexHullDebugInfo(const WangConvexHull& hull);

// Definition of the friend function
void PrintWangConvexHullDebugInfo(const WangConvexHull& hull) {
  std::cout << "Left Hull: ";
  for (const auto& node : hull.convex_left_upper_hull_) {
    if (auto shared_node = node.lock()) {
      std::cout << shared_node->p.x << " ";
    }
  }
  std::cout << std::endl;
  std::cout << "Right Hull: ";
  for (const auto& node : hull.convex_right_upper_hull_) {
    if (auto shared_node = node.lock()) {
      std::cout << shared_node->p.x << " ";
    }
  }
  std::cout << std::endl;

  if (hull.t1_ != hull.convex_left_upper_hull_.end()) {
    if (auto shared_node = hull.t1_->lock()) {
      std::cout << "t1: " << shared_node->p.x << std::endl;
    } else {
      std::cout << "t1: Expired" << std::endl;
    }
  } else {
    std::cout << "t1: None" << std::endl;
  }
  if (hull.t2_ != hull.convex_right_upper_hull_.end()) {
    if (auto shared_node = hull.t2_->lock()) {
      std::cout << "t2: " << shared_node->p.x << std::endl;
    } else {
      std::cout << "t2: Expired" << std::endl;
    }
  } else {
    std::cout << "t2: None" << std::endl;
  }
}