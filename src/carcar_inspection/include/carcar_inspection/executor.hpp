// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#pragma once
#include <functional>
#include <memory>
#include <behaviortree_cpp_v3/bt_factory.h>
#include "carcar_inspection/domain.hpp"
namespace carcar_inspection {
class Executor {
public:
  using Step = std::function<BT::NodeStatus(const std::string &, const json &, bool)>;
  using Halt = std::function<void()>;
  Executor(Step step, Halt halt);
  ~Executor();
  void start(json snapshot);
  BT::NodeStatus tick();
  void pause();
  void resume();
  void cancel();
  std::string phase = "IDLE";
  size_t point_index = 0;
  size_t approach_index = 0;
  bool parking = false;
  json missed = json::array();
  std::string last_kind;
private:
  void make_tree();
  json context() const;
  BT::BehaviorTreeFactory factory_;
  std::unique_ptr<BT::Tree> tree_;
  Step step_;
  Halt halt_;
  json snapshot_;
};
}  // namespace carcar_inspection
