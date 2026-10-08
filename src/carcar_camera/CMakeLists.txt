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
find_package(realsense2 REQUIRED)

add_executable(depth_device_settings src/depth_device_settings.cpp)
target_compile_features(depth_device_settings PRIVATE cxx_std_17)
target_link_libraries(depth_device_settings realsense2::realsense2)

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
  diagnostic_msgs nav2_msgs nav_msgs rcl_interfaces rclcpp sensor_msgs std_msgs visualization_msgs)

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

add_executable(depth_cloud_preview src/depth_cloud_preview.cpp)
target_compile_features(depth_cloud_preview PRIVATE cxx_std_17)
ament_target_dependencies(depth_cloud_preview rclcpp sensor_msgs)

add_executable(depth_floor_profile src/depth_floor_profile.cpp)
target_compile_features(depth_floor_profile PRIVATE cxx_std_17)
target_include_directories(depth_floor_profile PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_floor_profile ${OpenCV_LIBS})

add_executable(depth_health_window src/depth_health_window.cpp)
target_compile_features(depth_health_window PRIVATE cxx_std_17)
ament_target_dependencies(depth_health_window diagnostic_msgs rclcpp sensor_msgs)

add_executable(depth_target_window src/depth_target_window.cpp)
target_compile_features(depth_target_window PRIVATE cxx_std_17)
ament_target_dependencies(depth_target_window rclcpp sensor_msgs tf2 tf2_geometry_msgs tf2_ros)

add_executable(depth_costmap_audit src/depth_costmap_audit.cpp)
target_compile_features(depth_costmap_audit PRIVATE cxx_std_17)
ament_target_dependencies(depth_costmap_audit
  diagnostic_msgs nav2_msgs nav_msgs rclcpp rosbag2_cpp rosbag2_compression
  sensor_msgs std_msgs tf2 tf2_geometry_msgs tf2_msgs tf2_ros)

add_executable(depth_flicker_audit src/depth_flicker_audit.cpp)
target_compile_features(depth_flicker_audit PRIVATE cxx_std_17)
target_include_directories(depth_flicker_audit PRIVATE include)
ament_target_dependencies(depth_flicker_audit
  nav_msgs rclcpp rosbag2_cpp rosbag2_compression sensor_msgs tf2 tf2_geometry_msgs tf2_msgs tf2_ros)

add_executable(depth_ror_sweep src/depth_ror_sweep.cpp)
target_compile_features(depth_ror_sweep PRIVATE cxx_std_17)
target_include_directories(depth_ror_sweep PRIVATE include ${PCL_INCLUDE_DIRS})
ament_target_dependencies(depth_ror_sweep
  rclcpp rosbag2_cpp rosbag2_compression sensor_msgs tf2 tf2_geometry_msgs tf2_msgs tf2_ros)
target_link_libraries(depth_ror_sweep ${PCL_LIBRARIES})

add_executable(depth_clearing_coverage src/depth_clearing_coverage.cpp)
target_compile_features(depth_clearing_coverage PRIVATE cxx_std_17)
target_include_directories(depth_clearing_coverage PRIVATE include)
ament_target_dependencies(depth_clearing_coverage
  rclcpp rosbag2_cpp rosbag2_compression sensor_msgs tf2 tf2_geometry_msgs tf2_msgs tf2_ros)

add_executable(depth_map_audit src/depth_map_audit.cpp)
target_compile_features(depth_map_audit PRIVATE cxx_std_17)
ament_target_dependencies(depth_map_audit nav_msgs rclcpp rosbag2_cpp rosbag2_compression)

add_executable(depth_stvl_audit src/depth_stvl_audit.cpp)
target_compile_features(depth_stvl_audit PRIVATE cxx_std_17)
target_include_directories(depth_stvl_audit PRIVATE include)
ament_target_dependencies(depth_stvl_audit rclcpp rosbag2_cpp sensor_msgs)

add_executable(depth_temporal_audit src/depth_temporal_audit.cpp)
target_compile_features(depth_temporal_audit PRIVATE cxx_std_17)
target_include_directories(depth_temporal_audit PRIVATE include)
ament_target_dependencies(depth_temporal_audit
  rclcpp rosbag2_cpp sensor_msgs tf2 tf2_geometry_msgs tf2_msgs tf2_ros)

add_executable(image_reference_preview src/image_reference_preview.cpp)
target_compile_features(image_reference_preview PRIVATE cxx_std_17)
target_include_directories(image_reference_preview PRIVATE include ${OpenCV_INCLUDE_DIRS})
ament_target_dependencies(image_reference_preview rclcpp sensor_msgs)
target_link_libraries(image_reference_preview ${OpenCV_LIBS})

add_executable(reference_camera_info src/reference_camera_info.cpp)
target_include_directories(reference_camera_info PRIVATE include)
ament_target_dependencies(reference_camera_info rclcpp sensor_msgs)

install(TARGETS reference_camera_info depth_device_settings image_reference_preview depth_cloud_filter depth_cloud_preview depth_floor_profile depth_health_window depth_target_window depth_ror_shadow depth_layer_supervisor charuco_extrinsic_calibrator depth_snapshot
  depth_costmap_audit depth_map_audit depth_stvl_audit depth_temporal_audit depth_flicker_audit depth_ror_sweep depth_clearing_coverage
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_program(REGISTER_EXECUTABLE register_node PATHS /opt/ros/humble/lib/depth_image_proc REQUIRED)
  ament_add_gtest(test_reference_camera_info test/test_reference_camera_info.cpp)
  target_include_directories(test_reference_camera_info PRIVATE include)
  ament_target_dependencies(test_reference_camera_info rclcpp sensor_msgs tf2_msgs)
  target_compile_definitions(test_reference_camera_info PRIVATE REGISTER_EXECUTABLE="${REGISTER_EXECUTABLE}")
  set_tests_properties(test_reference_camera_info PROPERTIES TIMEOUT 15
    ENVIRONMENT "ROS_DOMAIN_ID=91;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_image_reference test/test_image_reference.cpp)
  target_include_directories(test_image_reference PRIVATE include ${OpenCV_INCLUDE_DIRS})
  ament_target_dependencies(test_image_reference sensor_msgs)
  target_link_libraries(test_image_reference ${OpenCV_LIBS})
  ament_add_gtest(test_required_stvl_health test/test_required_stvl_health.cpp)
  target_include_directories(test_required_stvl_health PRIVATE include)
  ament_add_gtest(test_depth_health_window test/test_depth_health_window.cpp TIMEOUT 110)
  target_compile_features(test_depth_health_window PRIVATE cxx_std_17)
  target_compile_definitions(test_depth_health_window PRIVATE
    HEALTH_WINDOW_EXECUTABLE="$<TARGET_FILE:depth_health_window>")
  add_dependencies(test_depth_health_window depth_health_window)
  ament_target_dependencies(test_depth_health_window diagnostic_msgs rclcpp sensor_msgs)
  set_tests_properties(test_depth_health_window PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=88;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_depth_target_window test/test_depth_target_window.cpp TIMEOUT 80)
  target_compile_features(test_depth_target_window PRIVATE cxx_std_17)
  target_compile_definitions(test_depth_target_window PRIVATE
    TARGET_WINDOW_EXECUTABLE="$<TARGET_FILE:depth_target_window>")
  add_dependencies(test_depth_target_window depth_target_window)
  ament_target_dependencies(test_depth_target_window geometry_msgs rclcpp sensor_msgs tf2_ros)
  set_tests_properties(test_depth_target_window PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=89;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_depth_cloud_preview test/test_depth_cloud_preview.cpp TIMEOUT 20)
  target_compile_features(test_depth_cloud_preview PRIVATE cxx_std_17)
  target_compile_definitions(test_depth_cloud_preview PRIVATE
    CLOUD_PREVIEW_EXECUTABLE="$<TARGET_FILE:depth_cloud_preview>")
  add_dependencies(test_depth_cloud_preview depth_cloud_preview)
  ament_target_dependencies(test_depth_cloud_preview rclcpp sensor_msgs)
  set_tests_properties(test_depth_cloud_preview PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=90;ROS_LOCALHOST_ONLY=1")
  ament_add_gtest(test_stvl_roi test/test_stvl_roi.cpp)
  target_include_directories(test_stvl_roi PRIVATE include)
  ament_target_dependencies(test_stvl_roi sensor_msgs)
  find_package(nav2_voxel_grid REQUIRED)
  ament_add_gtest(test_low_obstacle_pipeline test/test_low_obstacle_pipeline.cpp)
  target_include_directories(test_low_obstacle_pipeline PRIVATE include)
  ament_target_dependencies(test_low_obstacle_pipeline nav2_voxel_grid)
  ament_add_gtest(test_depth_logic test/test_depth_logic.cpp)
  target_include_directories(test_depth_logic PRIVATE include)
  ament_add_gtest(test_radius_outlier_shadow test/test_radius_outlier_shadow.cpp)
  target_include_directories(test_radius_outlier_shadow PRIVATE include ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_radius_outlier_shadow ${PCL_LIBRARIES})
  ament_add_gtest(test_temporal_confirmation test/test_temporal_confirmation.cpp)
  target_include_directories(test_temporal_confirmation PRIVATE include)

  ament_add_gtest(test_flicker_logic test/test_flicker_logic.cpp)
  target_include_directories(test_flicker_logic PRIVATE include)
  ament_add_gtest(test_extrinsic_math test/test_extrinsic_math.cpp)
  target_include_directories(test_extrinsic_math PRIVATE include)
  ament_target_dependencies(test_extrinsic_math tf2)
endif()

ament_package()
