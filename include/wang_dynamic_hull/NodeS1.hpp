#pragma once

#include <stack>

#include "Node.hpp"

struct NodeS1 : public Node {
  NodeS1(const Point& p) : Node(p) {}
  std::stack<std::weak_ptr<NodeS1>> graham_history_stack;
};