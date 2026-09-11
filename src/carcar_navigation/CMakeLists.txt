cmake_minimum_required(VERSION 3.8)
project(carcar_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# C++ 地图重载与格式验证工具
add_executable(check_map_reload src/check_map_reload.cpp)
ament_target_dependencies(check_map_reload
  rclcpp
  nav_msgs
)

# C++ 导航链路状态检查工具
add_executable(check_nav_link src/check_nav_link.cpp)
ament_target_dependencies(check_nav_link
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2_ros
)

# 安装可执行文件
install(TARGETS
  check_map_reload
  check_nav_link
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置、Launch、地图、RViz与行为树资源
install(DIRECTORY config launch maps rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
