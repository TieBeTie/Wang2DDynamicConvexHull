#pragma once
#include <algorithm>
#include <cmath>
#include <deque>
#include <iterator>
#include <list>
#include <stdexcept>
#include <vector>

#include "../geometry/geometry_utils.hpp"
#include "NodeS1.hpp"
#include "NodeS2.hpp"

class WangConvexHull {
 public:
  WangConvexHull() = default;
  void pushRight(const Point& p);
  void popLeft();
  const Point& back() const;
  bool empty() const;
  double upperTangentFromRightmostPoint() const;
  double lowerTangentFromRightmostPoint() const;

 private:
  std::list<std::weak_ptr<NodeS1>> convex_left_upper_hull_;
  std::list<std::weak_ptr<NodeS2>> convex_right_upper_hull_;

  std::list<std::shared_ptr<NodeS1>> active_left_points_;
  std::list<std::shared_ptr<NodeS2>> active_right_points_;

  std::list<std::weak_ptr<NodeS1>>::iterator t1_ =
      convex_left_upper_hull_.end();
  std::list<std::weak_ptr<NodeS2>>::iterator t2_ =
      convex_right_upper_hull_.end();

  std::list<std::weak_ptr<NodeS1>>::iterator GetLeftmostNodeIt() {
    return convex_left_upper_hull_.begin();
  }
  std::list<std::weak_ptr<NodeS1>>::const_iterator GetLeftmostNodeIt() const {
    return convex_left_upper_hull_.begin();
  }

  std::list<std::weak_ptr<NodeS2>>::iterator GetRightmostNodeIt() {
    return std::prev(convex_right_upper_hull_.end());
  }

  std::list<std::weak_ptr<NodeS2>>::const_iterator GetRightmostNodeIt() const {
    return std::prev(convex_right_upper_hull_.end());
  }

 private:
  // insertion-type tangent searching procedure
  void UpdateTangentAfterInsertion();
  // deletion-type tangent searching procedure
  void DeletionTangentSearch();
  // left-hull construction procedure
  void RebuildLeftHull();
  void GrahamScanS2();
  void RestoreS1LeftSide();
  void ValidateInvariants() const;
  bool IsRightTurn(const std::weak_ptr<Node>& a, const std::weak_ptr<Node>& b,
                   const std::weak_ptr<Node>& c) const;
  bool IsLeftTurn(const std::weak_ptr<Node>& a, const std::weak_ptr<Node>& b,
                  const std::weak_ptr<Node>& c) const;
  bool TryShiftT1Leftwards();
  bool TryShiftT2Leftwards();
};

void WangConvexHull::pushRight(const Point& p) {
  // 3.2 Insertions
  // Update hull
  active_right_points_.emplace_back(std::make_shared<NodeS2>(p));
  convex_right_upper_hull_.emplace_back(active_right_points_.back());

  GrahamScanS2();
  // actual t2 point can be changed by GrahamScanS2

  UpdateTangentAfterInsertion();
}

void WangConvexHull::popLeft() {
  if (convex_left_upper_hull_.empty()) {
    throw std::logic_error("S1 is empty during popLeft");
  }

  bool is_tangent_changed = GetLeftmostNodeIt() == t1_;
  active_left_points_.pop_front();
  convex_left_upper_hull_.pop_front();
  // 4.1 Deletions
  if (convex_left_upper_hull_.size() == 1) {
    RebuildLeftHull();
    return;
  }
  // If p_i ̸= t1, then pi
  // is strictly to the left of t1 and t1t2 is still the common tangent
  // of the new S1 and S2, and thus we are done with the deletion
  t1_ = convex_left_upper_hull_.begin();  // new t1 after deletion
  // 3.3 Deletions
  RestoreS1LeftSide();

  if (is_tangent_changed) {
    DeletionTangentSearch();
  }
}

inline const Point& WangConvexHull::back() const {
  return convex_right_upper_hull_.back().lock()->p;
}

inline bool WangConvexHull::empty() const { return false; }

void WangConvexHull::RebuildLeftHull() {
  // rebuild left hull with GrahamHistoryStack based on active_right_points_
  for (Point p = (*active_right_points_.rbegin())->p;
       !active_right_points_.empty(); p = (*active_right_points_.rbegin())->p) {
    active_right_points_.pop_back();
    active_left_points_.emplace_front(std::make_shared<NodeS1>(p));
    while (convex_left_upper_hull_.size() >= 2) {
      if (IsRightTurn(active_left_points_.front(),
                      *(convex_left_upper_hull_.begin()),
                      *(std::next(convex_left_upper_hull_.begin())))) {
        convex_left_upper_hull_.pop_back();
      } else {
        convex_left_upper_hull_.begin()->lock()->graham_history_stack.push(
            active_left_points_.front());
        convex_left_upper_hull_.emplace_back(active_left_points_.front());
        break;
      }
    }
    if (active_left_points_.size() <= 2) {
      convex_left_upper_hull_.emplace_front(active_left_points_.front());
    }
  }
}

void WangConvexHull::GrahamScanS2() {
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

void WangConvexHull::RestoreS1LeftSide() {
  convex_left_upper_hull_.begin()->lock()->graham_history_stack.pop();
  for (auto first_node_graham_history_stack =
           convex_left_upper_hull_.begin()->lock()->graham_history_stack;
       !first_node_graham_history_stack.empty() &&
       !first_node_graham_history_stack.top().expired();
       convex_left_upper_hull_.push_front(
           first_node_graham_history_stack.top()));
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
  // Let q be the vertex such that qqj is the edge of the
  // new hull U(S2) (e.g., see Fig. 1).
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
    while (std::prev(t1_) != convex_left_upper_hull_.begin() &&
           IsLeftTurn(*std::prev(t1_), *t1_, *t2_)) {
      t1_ = std::prev(t1_);
    }
  }
}

bool WangConvexHull::TryShiftT1Leftwards() {
  bool is_tangent_changed = false;
  while (convex_left_upper_hull_.begin() != t1_ &&
         IsRightTurn(*std::prev(t1_), *t1_, *t2_)) {
    t1_ = std::prev(t1_);
    is_tangent_changed = true;
  }
  return is_tangent_changed;
}

bool WangConvexHull::TryShiftT2Leftwards() {
  bool is_tangent_changed = false;
  while (convex_right_upper_hull_.begin() != t2_ &&
         IsRightTurn(*t1_, *t2_, *std::prev(t2_))) {
    t2_ = std::prev(t2_);
    is_tangent_changed = true;
  }
  return is_tangent_changed;
}

// Starting with U(S₁), we move p leftwards until p t₂ is tangent to U(S₁).
// Then we move t₂ leftwards on U(S₂) until p t₂ is also tangent to U(S₂).
// If the tangency with U(S₁) is lost, we repeat the process.

void WangConvexHull::DeletionTangentSearch() {
  while (TryShiftT1Leftwards() || TryShiftT2Leftwards());
}

void WangConvexHull::ValidateInvariants() const {}

double WangConvexHull::upperTangentFromRightmostPoint() const {
  Point rightmost_point = GetRightmostNodeIt()->lock()->p;
  Point adjacent_vertex = std::prev(GetRightmostNodeIt())->lock()->p;
  return geometry::GetAngle(rightmost_point, adjacent_vertex);
}

double WangConvexHull::lowerTangentFromRightmostPoint() const { return 0; }