// 本地Nav2 1.1.20兼容入口：官方LifecycleManager与Bond保持，单一默认调用改为有界等待。
// 此同签名定义仅链接进mapping_lifecycle_manager，不替换系统libnav2_util或其他入口。
#include <nav2_util/lifecycle_service_client.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <cmath>
namespace nav2_util {
bool LifecycleServiceClient::change_state(std::uint8_t transition){
  using State=lifecycle_msgs::msg::State;using Transition=lifecycle_msgs::msg::Transition;
  if(!node_->has_parameter("transition_response_timeout"))node_->declare_parameter("transition_response_timeout",5.0);
  const double seconds=node_->get_parameter("transition_response_timeout").as_double();
  if(!std::isfinite(seconds)||seconds<.2||seconds>10)throw std::runtime_error("生命周期应答等待须为0.2至10秒");
  const auto timeout=std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(seconds));
  if(!change_state_.wait_for_service(timeout))throw std::runtime_error("生命周期接口尚未就绪，未发送过渡");
  auto request=std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();request->transition.id=transition;
  try{
    const auto response=change_state_.invoke(request,timeout);
    if(response&&response->success)return true;
    RCLCPP_ERROR(node_->get_logger(),"生命周期接口 %s 拒绝过渡 %u，保持未就绪",change_state_.getServiceName().c_str(),transition);
    return false;
  }catch(const std::runtime_error&error){
    RCLCPP_WARN(node_->get_logger(),"生命周期接口 %s 应答超时：%s；核对真实状态，不重复下发过渡",change_state_.getServiceName().c_str(),error.what());
  }
  uint8_t expected=0;
  switch(transition){
    case Transition::TRANSITION_CONFIGURE:expected=State::PRIMARY_STATE_INACTIVE;break;
    case Transition::TRANSITION_ACTIVATE:expected=State::PRIMARY_STATE_ACTIVE;break;
    case Transition::TRANSITION_DEACTIVATE:expected=State::PRIMARY_STATE_INACTIVE;break;
    case Transition::TRANSITION_CLEANUP:expected=State::PRIMARY_STATE_UNCONFIGURED;break;
    case Transition::TRANSITION_UNCONFIGURED_SHUTDOWN:
    case Transition::TRANSITION_INACTIVE_SHUTDOWN:
    case Transition::TRANSITION_ACTIVE_SHUTDOWN:expected=State::PRIMARY_STATE_FINALIZED;break;
    default:return false;
  }
  try{
    const auto actual=get_state(std::chrono::seconds(2));
    const bool confirmed=actual==expected;
    RCLCPP_INFO(node_->get_logger(),"生命周期 %s transition=%u expected=%u actual=%u result=%s；按实际状态确认",
      change_state_.getServiceName().c_str(),transition,expected,actual,confirmed?"confirmed":"refused");
    return confirmed;
  }catch(const std::runtime_error&error){
    RCLCPP_ERROR(node_->get_logger(),"生命周期状态核对失败：%s；保持未就绪",error.what());return false;
  }
}
}  // namespace nav2_util
