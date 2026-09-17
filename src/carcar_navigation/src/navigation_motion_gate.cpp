#include "carcar_navigation/navigation_motion_gate.hpp"
int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<NavigationMotionGate>();
  std::weak_ptr<NavigationMotionGate> weak=node;
  node->get_node_base_interface()->get_context()->add_pre_shutdown_callback([weak] {
    if (auto node=weak.lock()) {node->stop();}
  });
  rclcpp::spin(node);
  if (rclcpp::ok()) {node->stop();}
  rclcpp::shutdown();return 0;
}
