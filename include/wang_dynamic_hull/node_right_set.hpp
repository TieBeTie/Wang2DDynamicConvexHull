#pragma once

#include <stack>

#include "node.hpp"

struct RNode : public Node {
  RNode(const Point& p) : Node(p) {}
  std::stack<std::weak_ptr<RNode>> graham_history_stack;
};