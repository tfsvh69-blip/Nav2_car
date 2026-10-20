// Copyright 2026 carcar maintainers. SPDX-License-Identifier: Apache-2.0
#include "carcar_inspection/executor.hpp"
namespace carcar_inspection {
namespace {
class MissionStep : public BT::StatefulActionNode {
public:
  MissionStep(const std::string & name, const BT::NodeConfiguration & config,
              Executor::Step callback, Executor::Halt halt, std::function<json()> context)
  : BT::StatefulActionNode(name, config), callback_(std::move(callback)), halt_(std::move(halt)), context_(std::move(context)) {}
  static BT::PortsList providedPorts() {return {BT::InputPort<std::string>("kind")};}
  BT::NodeStatus onStart() override {kind_ = getInput<std::string>("kind").value();return callback_(kind_,context_(),true);}
  BT::NodeStatus onRunning() override {return callback_(kind_,context_(),false);}
  void onHalted() override {halt_();}
private:
  Executor::Step callback_; Executor::Halt halt_; std::function<json()> context_; std::string kind_;
};
}
Executor::Executor(Step step, Halt halt) : step_(std::move(step)), halt_(std::move(halt)) {
  factory_.registerBuilder<MissionStep>("MissionStep", [this](const std::string & name,const BT::NodeConfiguration & config) {
    return std::make_unique<MissionStep>(name,config,[this](const std::string & kind,const json & value,bool start) {
      last_kind=kind;return step_(kind,value,start);
    },halt_,[this]() -> json {return context();});
  });
}
json Executor::context() const {
  if(parking) return {{"pose",snapshot_.at("final_parking_pose")},{"parking",true}};
  json point=snapshot_.at("points").at(point_index);
  const auto approaches=approach_list(point);
  const auto & approach=approaches.at(approach_index);
  point["pose"]=approach.at("pose");
  point["target_degrees"]=approach.at("target_degrees");
  point["board_command"]=approach.at("board_command");
  point["approach_index"]=approach_index;
  point["approach_count"]=approaches.size();
  if(approach.contains("reference"))point["reference"]=approach["reference"];
  return point;
}
Executor::~Executor() {if(tree_)tree_->haltTree();}
void Executor::make_tree() {
  const std::string body = parking ?
    "<MissionStep kind=\"restore\"/><MissionStep kind=\"navigate\"/><MissionStep kind=\"stop\"/>" :
    "<MissionStep kind=\"restore\"/><MissionStep kind=\"navigate\"/><MissionStep kind=\"stop\"/>"
    "<MissionStep kind=\"adjust\"/><MissionStep kind=\"capture\"/><MissionStep kind=\"restore\"/>";
  tree_ = std::make_unique<BT::Tree>(factory_.createTreeFromText(
    "<root main_tree_to_execute=\"Inspection\"><BehaviorTree ID=\"Inspection\"><Sequence>"+body+"</Sequence></BehaviorTree></root>"));
}
void Executor::start(json snapshot) {
  require(phase != "RUNNING" && phase != "PAUSED", "已有巡检任务");
  require(snapshot.at("points").is_array() && !snapshot.at("points").empty(), "任务没有巡检点");
  validate_pose(snapshot.at("final_parking_pose"));
  for(const auto & point:snapshot.at("points")) require(approach_list(point).size()<=3, "每个拍摄目标最多3个停车位");
  snapshot_ = std::move(snapshot); point_index = 0; approach_index = 0; parking = false; missed = json::array();
  last_kind.clear(); phase = "RUNNING"; make_tree();
}
BT::NodeStatus Executor::tick() {
  if(phase != "RUNNING")return BT::NodeStatus::IDLE;
  const auto status = tree_->tickRoot();
  if(status == BT::NodeStatus::FAILURE) {
    if(!parking && last_kind=="navigate") {
      const auto point=snapshot_.at("points").at(point_index);
      const auto count=approach_list(point).size();
      if(approach_index+1<count){++approach_index;make_tree();return BT::NodeStatus::RUNNING;}
      missed.push_back({{"point_id",point.at("id")},{"reason","approaches_unreachable"}});
      approach_index=0;++point_index;parking=point_index==snapshot_.at("points").size();make_tree();
      return BT::NodeStatus::RUNNING;
    }
    phase="FAILED";tree_->haltTree();return status;
  }
  if(status == BT::NodeStatus::SUCCESS) {
    if(parking) {phase="SUCCEEDED";return status;}
    ++point_index; approach_index=0; parking = point_index == snapshot_.at("points").size(); make_tree();
  }
  return BT::NodeStatus::RUNNING;
}
void Executor::pause() {require(phase=="RUNNING","当前任务不能暂停");phase="PAUSED";tree_->haltTree();}
void Executor::resume() {require(phase=="PAUSED","当前任务不能继续");phase="RUNNING";make_tree();}
void Executor::cancel() {if(tree_)tree_->haltTree();phase="CANCELED";}
}  // namespace carcar_inspection
