cmake_minimum_required(VERSION 3.13)
project(carcar_tf2_compat)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(tf2_ros 0.25.23 EXACT REQUIRED)
foreach(dep rclcpp rclcpp_action tf2 tf2_msgs geometry_msgs builtin_interfaces message_filters)
  find_package(${dep} REQUIRED)
endforeach()
add_library(carcar_tf2_ros_compat SHARED
  vendor/tf2_ros/src/buffer.cpp vendor/tf2_ros/src/create_timer_ros.cpp
  vendor/tf2_ros/src/transform_listener.cpp vendor/tf2_ros/src/buffer_client.cpp
  vendor/tf2_ros/src/buffer_server.cpp vendor/tf2_ros/src/transform_broadcaster.cpp
  vendor/tf2_ros/src/static_transform_broadcaster.cpp)
target_include_directories(carcar_tf2_ros_compat PRIVATE ${tf2_ros_INCLUDE_DIRS})
target_compile_definitions(carcar_tf2_ros_compat PRIVATE TF2_ROS_BUILDING_DLL)
ament_target_dependencies(carcar_tf2_ros_compat rclcpp rclcpp_action tf2 tf2_msgs geometry_msgs builtin_interfaces message_filters)
set_target_properties(carcar_tf2_ros_compat PROPERTIES OUTPUT_NAME tf2_ros)
# 不导出库、不放入全局lib搜索目录；只有显式XML组合才加载。系统ROS库原样保留。
install(TARGETS carcar_tf2_ros_compat LIBRARY DESTINATION lib/${PROJECT_NAME})
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tf2_registration test/test_registration.cpp TIMEOUT 20)
  target_include_directories(test_tf2_registration PRIVATE ${tf2_ros_INCLUDE_DIRS})
  ament_target_dependencies(test_tf2_registration rclcpp tf2 tf2_msgs geometry_msgs)
  target_link_libraries(test_tf2_registration carcar_tf2_ros_compat)
  set_tests_properties(test_tf2_registration PROPERTIES ENVIRONMENT "ROS_DOMAIN_ID=209;ROS_LOCALHOST_ONLY=1;LD_LIBRARY_PATH=$<TARGET_FILE_DIR:carcar_tf2_ros_compat>:$ENV{LD_LIBRARY_PATH}")
endif()
ament_package()
