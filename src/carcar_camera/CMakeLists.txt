cmake_minimum_required(VERSION 3.8)
project(carcar_camera)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS aruco calib3d core imgproc imgcodecs)
find_package(PCL 1.12 REQUIRED COMPONENTS filters)

add_executable(depth_cloud_filter src/depth_cloud_filter.cpp)
target_include_directories(depth_cloud_filter PRIVATE include)
ament_target_dependencies(depth_cloud_filter
  diagnostic_msgs rclcpp sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(depth_ror_shadow src/depth_ror_shadow.cpp)
target_include_directories(depth_ror_shadow PRIVATE include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(depth_ror_shadow diagnostic_msgs rclcpp sensor_msgs)
target_link_libraries(depth_ror_shadow ${PCL_LIBRARIES})

add_executable(depth_layer_supervisor src/depth_layer_supervisor.cpp)
target_include_directories(depth_layer_supervisor PRIVATE include)
ament_target_dependencies(depth_layer_supervisor
  diagnostic_msgs nav2_msgs nav_msgs rcl_interfaces rclcpp std_msgs visualization_msgs)

add_executable(charuco_extrinsic_calibrator src/charuco_extrinsic_calibrator.cpp)
target_include_directories(charuco_extrinsic_calibrator PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(charuco_extrinsic_calibrator
  diagnostic_msgs geometry_msgs rclcpp sensor_msgs std_srvs tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(charuco_extrinsic_calibrator ${OpenCV_LIBS})

add_executable(depth_snapshot src/depth_snapshot.cpp)
target_compile_features(depth_snapshot PRIVATE cxx_std_17)
target_include_directories(depth_snapshot PRIVATE ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(depth_snapshot rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)
target_link_libraries(depth_snapshot ${OpenCV_LIBS})

add_executable(depth_costmap_audit src/depth_costmap_audit.cpp)
target_compile_features(depth_costmap_audit PRIVATE cxx_std_17)
ament_target_dependencies(depth_costmap_audit
  diagnostic_msgs nav2_msgs nav_msgs rclcpp rosbag2_cpp rosbag2_compression
  sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_msgs tf2_ros)

add_executable(depth_map_audit src/depth_map_audit.cpp)
target_compile_features(depth_map_audit PRIVATE cxx_std_17)
ament_target_dependencies(depth_map_audit nav_msgs rclcpp rosbag2_cpp rosbag2_compression)

install(TARGETS depth_cloud_filter depth_ror_shadow depth_layer_supervisor charuco_extrinsic_calibrator depth_snapshot
  depth_costmap_audit depth_map_audit
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_depth_logic test/test_depth_logic.cpp)
  target_include_directories(test_depth_logic PRIVATE include)
  ament_add_gtest(test_radius_outlier_shadow test/test_radius_outlier_shadow.cpp)
  target_include_directories(test_radius_outlier_shadow PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_radius_outlier_shadow ${PCL_LIBRARIES})
  ament_add_gtest(test_extrinsic_math test/test_extrinsic_math.cpp)
  target_include_directories(test_extrinsic_math PRIVATE include)
  ament_target_dependencies(test_extrinsic_math tf2)
endif()

ament_package()
