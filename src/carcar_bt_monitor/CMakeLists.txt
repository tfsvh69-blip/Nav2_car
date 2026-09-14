cmake_minimum_required(VERSION 3.8)
project(carcar_bt_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# BehaviorTree.CPP v3（ros-humble-behaviortree-cpp-v3 3.8.7）
# ZMQ 已通过 behaviortree_cpp_v3 的 CMake 目标传递
find_package(behaviortree_cpp_v3 REQUIRED)

# NAV-006: BT 状态日志桥接节点
add_executable(bt_monitor_node src/bt_monitor_node.cpp)
ament_target_dependencies(bt_monitor_node
  rclcpp
  nav2_msgs
  action_msgs
  diagnostic_msgs
)
target_link_libraries(bt_monitor_node
  BT::behaviortree_cpp_v3
)

install(TARGETS
  bt_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()

