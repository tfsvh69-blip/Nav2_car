#include <nav2_lifecycle_manager/lifecycle_manager.hpp>
int main(int argc,char**argv){
 rclcpp::init(argc,argv);
 rclcpp::spin(std::make_shared<nav2_lifecycle_manager::LifecycleManager>());
 rclcpp::shutdown();return 0;
}
