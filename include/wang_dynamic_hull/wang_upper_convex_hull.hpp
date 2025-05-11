#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <list>
#include <stdexcept>
#include <vector>

#include "../geometry/geometry_utils.hpp"
#include "node_left_set.hpp"
#include "node_right_set.hpp"

class WangConvexHull {
 public:
  WangConvexHull() = default;
  // O(1) amortized time complexity!
  void pushRight(const Point& p);
  // O(1) amortized time complexity!
  void popLeft();
  const Point& back() const;
  bool empty() const;
  double upperTangentFromRightmostPoint() const;
  double lowerTangentFromRightmostPoint() const;

  friend void PrintWangConvexHullDebugInfo(const WangConvexHull& hull);

 private:
  std::list<std::weak_ptr<RNode>> convex_left_upper_hull_;
  std::list<std::weak_ptr<LNode>> convex_right_upper_hull_;

  std::list<std::shared_ptr<RNode>> active_left_points_;
  std::list<std::shared_ptr<LNode>> active_right_points_;

  std::list<std::weak_ptr<RNode>>::iterator t1_ = convex_left_upper_hull_.end();
  std::list<std::weak_ptr<LNode>>::iterator t2_ =
      convex_right_upper_hull_.end();

  std::list<std::weak_ptr<RNode>>::iterator GetLeftmostNodeIt() {
    return convex_left_upper_hull_.begin();
  }
  std::list<std::weak_ptr<RNode>>::const_iterator GetLeftmostNodeIt() const {
    return convex_left_upper_hull_.begin();
  }

  std::list<std::weak_ptr<LNode>>::iterator GetRightmostNodeIt() {
    return std::prev(convex_right_upper_hull_.end());
  }

  std::list<std::weak_ptr<LNode>>::const_iterator GetRightmostNodeIt() const {
    return std::prev(convex_right_upper_hull_.end());
  }

  Point getRightmostPoint() const {
    if (convex_left_upper_hull_.empty() && convex_right_upper_hull_.empty()) {
      throw std::logic_error(
          "left convex hull and right convex hull are empty during "
          "getRightmostPoint");
    }
    return GetRightmostNodeIt() != convex_right_upper_hull_.end()
               ? GetRightmostNodeIt()->lock()->p
               : convex_left_upper_hull_.rbegin()->lock()->p;
  }

 private:
  // insertion-type tangent searching procedure
  void UpdateTangentAfterInsertion();
  // deletion-type tangent searching procedure
  void UpdateTangentAfterDeletion();
  // left-hull construction procedure
  void RebuildLeftHull();
  void RightHullGrahamRestore();
  void LeftHullRestoreByHistoryStack();
  bool IsRightTurn(const std::weak_ptr<Node>& a, const std::weak_ptr<Node>& b,
                   const std::weak_ptr<Node>& c) const;
  bool IsLeftTurn(const std::weak_ptr<Node>& a, const std::weak_ptr<Node>& b,
                  const std::weak_ptr<Node>& c) const;
  bool TryShiftT1Leftwards();
  bool TryShiftT2Leftwards();
  Point GetUpperPointAdjacentToRightmostPoint() const;
};

// O(1) amortized time complexity!
void WangConvexHull::pushRight(const Point& p) {
  // 3.2 Insertions
  // Update hull
  active_right_points_.emplace_back(std::make_shared<LNode>(p));
  convex_right_upper_hull_.emplace_back(active_right_points_.back());

  RightHullGrahamRestore();
  // actual t2 point can be changed by GrahamScanright convex hull

  UpdateTangentAfterInsertion();
}

// O(1) amortized time complexity!
void WangConvexHull::popLeft() {
  if (convex_left_upper_hull_.empty()) {
    RebuildLeftHull();
  }
  // 4.1 Deletions
  // If p_i ̸= t1, then pi
  // is strictly to the left of t1 and t1t2 is still the common tangent
  // of the new left convex hull and right convex hull, and thus we are done
  // with the deletion
  bool is_tangent_changed =
      GetLeftmostNodeIt() == t1_ && t1_ != convex_left_upper_hull_.end();
  active_left_points_.pop_front();
  convex_left_upper_hull_.pop_front();
  if (is_tangent_changed) {
    t1_ = convex_left_upper_hull_.begin();  // new t1 after deletion
  }
  // 3.3 Deletions

  LeftHullRestoreByHistoryStack();

  if (is_tangent_changed) {
    UpdateTangentAfterDeletion();
  }
}

inline const Point& WangConvexHull::back() const {
  return convex_right_upper_hull_.back().lock()->p;
}

inline bool WangConvexHull::empty() const { return false; }

void WangConvexHull::RebuildLeftHull() {
  // rebuild left hull with GrahamHistoryStack based on active_right_points_
  if (convex_right_upper_hull_.empty()) {
    throw std::logic_error("right convex hull is empty during RebuildLeftHull");
  }
  convex_right_upper_hull_.clear();
  while (!active_right_points_.empty()) {
    Point p = (*active_right_points_.rbegin())->p;
    active_right_points_.pop_back();
    active_left_points_.emplace_front(std::make_shared<RNode>(p));
    while (convex_left_upper_hull_.size() >= 2 &&
           IsLeftTurn(active_left_points_.front(),
                      *(convex_left_upper_hull_.begin()),
                      *(std::next(convex_left_upper_hull_.begin())))) {
      convex_left_upper_hull_.pop_front();
    }
    if (!convex_left_upper_hull_.empty()) {
      convex_left_upper_hull_.begin()->lock()->graham_history_stack.push(
          active_left_points_.front());
    }
    convex_left_upper_hull_.emplace_front(active_left_points_.front());
  }
}

void WangConvexHull::RightHullGrahamRestore() {
  while (convex_right_upper_hull_.size() >= 3) {
    auto it_last = std::prev(convex_right_upper_hull_.end());
    auto it_mid = std::prev(it_last);
    auto it_prev = std::prev(it_mid);

    if (IsLeftTurn(*it_prev, *it_mid, *it_last)) {
      convex_right_upper_hull_.erase(it_mid);
    } else {
      break;
    }
  }
}

void WangConvexHull::LeftHullRestoreByHistoryStack() {
  if (convex_left_upper_hull_.empty()) {
    return;
  }
  convex_left_upper_hull_.begin()->lock()->graham_history_stack.pop();
  auto node = convex_left_upper_hull_.front().lock();
  while (!node->graham_history_stack.empty() &&
         !node->graham_history_stack.top().expired()) {
    node = node->graham_history_stack.top().lock();
    convex_left_upper_hull_.emplace_front(node);
  }
}

bool WangConvexHull::IsRightTurn(const std::weak_ptr<Node>& a,
                                 const std::weak_ptr<Node>& b,
                                 const std::weak_ptr<Node>& c) const {
  return geometry::IsRightTurn(a.lock()->p, b.lock()->p, c.lock()->p);
}

bool WangConvexHull::IsLeftTurn(const std::weak_ptr<Node>& a,
                                const std::weak_ptr<Node>& b,
                                const std::weak_ptr<Node>& c) const {
  return geometry::IsLeftTurn(a.lock()->p, b.lock()->p, c.lock()->p);
}

void WangConvexHull::UpdateTangentAfterInsertion() {
  if (convex_left_upper_hull_.empty() || convex_right_upper_hull_.empty()) {
    t1_ = convex_left_upper_hull_.end();
    t2_ = convex_right_upper_hull_.end();
    return;
  }
  // Let q be the vertex such that qqj is the edge of the
  // new hull U(right convex hull) (e.g., see Fig. 1).
  bool condition1 = false;
  bool condition2 = false;
  if (convex_right_upper_hull_.size() == 1) {
    t1_ = std::prev(convex_left_upper_hull_.end());
  } else {
    Point q_point = std::prev(GetRightmostNodeIt())->lock()->p;
    // means right edge of t2 does not change ->
    // points located on the one side of tangent
    condition1 = (t2_->lock()->p.x < q_point.x);
    // another case is when q is the same as t2 and we need check only last edge
    condition2 = (q_point == t2_->lock()->p) &&
                 IsRightTurn(*t1_, *t2_, *GetRightmostNodeIt());
  }
  if (!condition1 && !condition2) {
    t2_ = GetRightmostNodeIt();
    while (t1_ != convex_left_upper_hull_.begin() &&
           IsLeftTurn(*std::prev(t1_), *t1_, *t2_)) {
      t1_ = std::prev(t1_);
    }
  }
}

bool WangConvexHull::TryShiftT1Leftwards() {
  if (convex_left_upper_hull_.empty()) {
    throw std::logic_error(
        "left convex hull is empty during TryShiftT1Leftwards");
  }
  bool is_tangent_changed = false;
  while (convex_left_upper_hull_.begin() != t1_ &&
         IsLeftTurn(*std::prev(t1_), *t1_, *t2_)) {
    t1_ = std::prev(t1_);
    is_tangent_changed = true;
  }
  return is_tangent_changed;
}

bool WangConvexHull::TryShiftT2Leftwards() {
  if (convex_right_upper_hull_.empty()) {
    throw std::logic_error(
        "right convex hull is empty during TryShiftT2Leftwards");
  }
  bool is_tangent_changed = false;
  while (convex_right_upper_hull_.begin() != t2_ &&
         IsLeftTurn(*t1_, *t2_, *std::prev(t2_))) {
    t2_ = std::prev(t2_);
    is_tangent_changed = true;
  }
  return is_tangent_changed;
}

// Starting with U(S₁), we move p leftwards until p t₂ is tangent to U(S₁).
// Then we move t₂ leftwards on U(S₂) until p t₂ is also tangent to U(S₂).
// If the tangency with U(S₁) is lost, we repeat the process.

void WangConvexHull::UpdateTangentAfterDeletion() {
  if (convex_left_upper_hull_.empty() || convex_right_upper_hull_.empty()) {
    t1_ = convex_left_upper_hull_.end();
    t2_ = convex_right_upper_hull_.end();
    return;
  }
  t1_ = t1_ == convex_left_upper_hull_.end() || t1_->expired()
            ? std::prev(convex_left_upper_hull_.end())
            : t1_;
  while (TryShiftT1Leftwards() || TryShiftT2Leftwards());
}

double WangConvexHull::upperTangentFromRightmostPoint() const {
  if (convex_right_upper_hull_.empty()) {
    throw std::logic_error(
        "right convex hull is empty during upperTangentFromRightmostPoint");
  }
  if (convex_left_upper_hull_.size() + convex_right_upper_hull_.size() <= 1) {
    return 0;
  }
  Point rightmost_point = getRightmostPoint();
  Point adjacent_vertex = GetUpperPointAdjacentToRightmostPoint();

  return -geometry::GetAngle(adjacent_vertex, rightmost_point);
}

double WangConvexHull::lowerTangentFromRightmostPoint() const { return 0; }

Point WangConvexHull::GetUpperPointAdjacentToRightmostPoint() const {
  if (convex_right_upper_hull_.empty()) {
    return std::next(convex_left_upper_hull_.rbegin())->lock()->p;
  }
  if (t2_ == GetRightmostNodeIt()) {
    return t1_->lock()->p;
  }
  return std::next(convex_right_upper_hull_.rbegin())->lock()->p;
}