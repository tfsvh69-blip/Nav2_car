cmake_minimum_required(VERSION 3.8)
project(carcar_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(carcar_interfaces REQUIRED)
find_package(rmw REQUIRED)
find_package(dwb_core REQUIRED)
find_package(dwb_plugins REQUIRED)
find_package(dwb_msgs REQUIRED)
find_package(nav_2d_msgs REQUIRED)

# NAV-003/NAV-004: C++ 地图重载与格式验证工具（支持 expected_map_yaml 参数）
add_executable(check_map_reload src/check_map_reload.cpp)
ament_target_dependencies(check_map_reload
  rclcpp
  nav_msgs
)

# NAV-003/NAV-005/NAV-009: C++ 导航链路状态检查工具（扩展生命周期、定位健康与桥接状态）
add_executable(check_nav_link src/check_nav_link.cpp)
ament_target_dependencies(check_nav_link
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  diagnostic_msgs
  lifecycle_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# NAV-007/NAV-008: 只读获取正在运行的 Nav2 参数，供每次试验保存“实际加载值”快照。
add_executable(nav_runtime_snapshot src/nav_runtime_snapshot.cpp)
ament_target_dependencies(nav_runtime_snapshot rclcpp)

# NAV-007: 只读定位健康监控。它只发布诊断和门控话题，不发布速度命令。
add_executable(localization_monitor src/localization_monitor.cpp)
ament_target_dependencies(localization_monitor
  rclcpp
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

add_executable(imu_gyro_bias_corrector src/imu_gyro_bias_corrector.cpp)
target_include_directories(imu_gyro_bias_corrector PRIVATE include)
ament_target_dependencies(imu_gyro_bias_corrector rclcpp sensor_msgs diagnostic_msgs)

# NAV-007/NAV-008: 供 bt_navigator 动态加载的本项目行为树节点。
# RecoverLocalization 只调用 AMCL 服务并等待健康门控；RearClear 只读取
# 本地代价地图、扫描新鲜度和 TF，所有实际运动仍由 Nav2 behavior_server 执行。
add_library(carcar_recovery_runtime SHARED src/recovery_runtime.cpp)
target_include_directories(carcar_recovery_runtime PUBLIC include)
ament_target_dependencies(carcar_recovery_runtime rclcpp nav2_util nav2_msgs nav_msgs sensor_msgs
  geometry_msgs std_msgs diagnostic_msgs tf2 tf2_ros tf2_geometry_msgs nav2_costmap_2d)
target_link_libraries(carcar_recovery_runtime BT::behaviortree_cpp_v3)

add_library(carcar_nav_bt_nodes SHARED src/carcar_nav_bt_nodes.cpp src/nav012_nodes.cpp)
target_include_directories(carcar_nav_bt_nodes PUBLIC include)
ament_target_dependencies(carcar_nav_bt_nodes
  rclcpp
  nav2_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  diagnostic_msgs
  rclcpp_action
  nav2_behavior_tree
  nav2_util
  nav_msgs
)
target_link_libraries(carcar_nav_bt_nodes BT::behaviortree_cpp_v3 carcar_recovery_runtime)
target_compile_definitions(carcar_nav_bt_nodes PRIVATE BT_PLUGIN_EXPORT)

# NAV-012: 倒车周期安全防护插件
add_library(carcar_protected_backup SHARED src/protected_backup.cpp)
target_include_directories(carcar_protected_backup PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(carcar_protected_backup
  rclcpp
  rclcpp_lifecycle
  nav2_behaviors
  nav2_core
  nav2_msgs
  nav2_util
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pluginlib
)
pluginlib_export_plugin_description_file(nav2_core behavior_plugins.xml)
target_link_libraries(carcar_protected_backup carcar_recovery_runtime)

add_library(carcar_dwb_plugins SHARED src/split_horizon_traj_generator.cpp)
target_include_directories(carcar_dwb_plugins PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(carcar_dwb_plugins
  rclcpp nav2_util dwb_core dwb_plugins dwb_msgs nav_2d_msgs pluginlib)

# 旧 escape_controller / 自写 EKF 草稿保留但停用；恢复统一走受保护 Action。
pluginlib_export_plugin_description_file(dwb_core controller_plugins.xml)

# NAV-008/NAV-011: 导航事件监控、终端状态摘要与多维度会话录包节点
add_executable(nav_event_logger src/nav_event_logger.cpp)
ament_target_dependencies(nav_event_logger
  rclcpp
  rcl_interfaces
  action_msgs
  diagnostic_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rosbag2_cpp
  rosbag2_storage
  rosbag2_transport
  rmw
)
target_link_libraries(nav_event_logger
  nlohmann_json::nlohmann_json
)
target_include_directories(nav_event_logger PRIVATE include)

add_executable(navigation_motion_gate src/navigation_motion_gate.cpp)
target_include_directories(navigation_motion_gate PRIVATE include)
ament_target_dependencies(navigation_motion_gate rclcpp geometry_msgs std_msgs diagnostic_msgs)

# NAV-008/NAV-011: 导航问题点诊断、会话分析与离线回看工具
add_executable(nav_log_tool src/nav_log_tool.cpp)
ament_target_dependencies(nav_log_tool
  rclcpp
  action_msgs
  diagnostic_msgs
  geometry_msgs
  nav2_msgs
  nav_msgs
  sensor_msgs
  rosbag2_cpp
  rosbag2_storage
)
target_link_libraries(nav_log_tool
  nlohmann_json::nlohmann_json
)

# NAV-014: 导航声光提示与停车诊断转发节点
add_executable(navigation_indicator src/navigation_indicator.cpp)
target_include_directories(navigation_indicator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(navigation_indicator
  rclcpp
  diagnostic_msgs
  carcar_interfaces
)

# 安装头文件
install(DIRECTORY include/
  DESTINATION include
)

# 安装可执行文件
install(TARGETS
  check_map_reload
  check_nav_link
  nav_runtime_snapshot
  localization_monitor
  imu_gyro_bias_corrector
  nav_event_logger
  nav_log_tool
  navigation_indicator
  navigation_motion_gate
  DESTINATION lib/${PROJECT_NAME}
)

# bt_navigator 按 ament 前缀的 lib/ 目录查找动态 BT 插件；不能安装在包专用
# lib/carcar_navigation/ 目录，否则实验树启动时会找不到 carcar_nav_bt_nodes。
install(TARGETS carcar_nav_bt_nodes carcar_protected_backup carcar_recovery_runtime
  carcar_dwb_plugins
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安装配置、Launch、地图、RViz 与行为树资源
install(DIRECTORY config launch maps rviz behavior_trees
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_protected_backup_execution test/test_protected_backup_execution.cpp)
  target_include_directories(test_protected_backup_execution PRIVATE include)
  ament_target_dependencies(test_protected_backup_execution rclcpp rclcpp_lifecycle
    nav2_behaviors nav2_core nav2_util nav2_msgs nav2_costmap_2d tf2 tf2_ros
    tf2_geometry_msgs sensor_msgs nav_msgs geometry_msgs)
  target_link_libraries(test_protected_backup_execution carcar_protected_backup)
  set_tests_properties(test_protected_backup_execution PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=211;ROS_LOCALHOST_ONLY=1" RUN_SERIAL TRUE TIMEOUT 20)
  ament_add_gtest(test_nav_escape_logic test/test_nav_escape_logic.cpp)
  target_include_directories(test_nav_escape_logic PRIVATE include)
  ament_target_dependencies(test_nav_escape_logic rclcpp nav2_util dwb_core dwb_plugins dwb_msgs
    nav_2d_msgs pluginlib)
  target_link_libraries(test_nav_escape_logic carcar_dwb_plugins)
  set_tests_properties(test_nav_escape_logic PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=211;ROS_LOCALHOST_ONLY=1" RUN_SERIAL TRUE TIMEOUT 30)
  ament_add_gtest(test_nav012_regressions test/test_nav012_regressions.cpp)
  ament_target_dependencies(test_nav012_regressions rclcpp rclcpp_action nav2_msgs nav2_behavior_tree
    nav2_util nav_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs action_msgs)
  target_link_libraries(test_nav012_regressions carcar_nav_bt_nodes BT::behaviortree_cpp_v3)
  target_compile_definitions(test_nav012_regressions PRIVATE
    CARCAR_NAV_BT_PLUGIN_PATH="$<TARGET_FILE:carcar_nav_bt_nodes>"
    NAV_LOGGER_EXECUTABLE="$<TARGET_FILE:nav_event_logger>"
    BEHAVIOR_TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees")
  set_tests_properties(test_nav012_regressions PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=211;ROS_LOCALHOST_ONLY=1" RUN_SERIAL TRUE TIMEOUT 90)
  ament_add_gtest(test_nav_bt_nodes test/test_nav_bt_nodes.cpp)
  ament_target_dependencies(test_nav_bt_nodes
    rclcpp
    nav2_msgs
    sensor_msgs
    std_msgs
    std_srvs
    diagnostic_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
  )
  target_link_libraries(test_nav_bt_nodes
    carcar_nav_bt_nodes
    BT::behaviortree_cpp_v3
  )
  target_compile_definitions(test_nav_bt_nodes PRIVATE
    BEHAVIOR_TREES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/behavior_trees"
    CARCAR_NAV_BT_PLUGIN_PATH="$<TARGET_FILE:carcar_nav_bt_nodes>"
  )

  ament_add_gtest(test_obstacle_bypass_offline test/test_obstacle_bypass_offline.cpp)
  ament_target_dependencies(test_obstacle_bypass_offline
    rclcpp
    nav2_msgs
    geometry_msgs
    nav_msgs
  )
  target_link_libraries(test_obstacle_bypass_offline
    nlohmann_json::nlohmann_json
  )

  ament_add_gtest(test_nav_logger_logic test/test_nav_logger_logic.cpp)
  target_include_directories(test_nav_logger_logic PRIVATE include)
  ament_target_dependencies(test_nav_logger_logic
    rclcpp
    action_msgs
  )
  target_link_libraries(test_nav_logger_logic
    nlohmann_json::nlohmann_json
  )

  ament_add_gtest(test_navigation_indicator_logic test/test_navigation_indicator_logic.cpp)
  target_include_directories(test_navigation_indicator_logic PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(test_navigation_indicator_logic
    rclcpp
    carcar_interfaces
  )
  ament_add_gtest(test_imu_gyro_bias test/test_imu_gyro_bias.cpp)
  target_include_directories(test_imu_gyro_bias PRIVATE include)
  set_tests_properties(test_nav_bt_nodes test_obstacle_bypass_offline test_nav_logger_logic
    test_navigation_indicator_logic test_imu_gyro_bias PROPERTIES
    ENVIRONMENT "ROS_DOMAIN_ID=211;ROS_LOCALHOST_ONLY=1" RUN_SERIAL TRUE)
endif()

ament_package()
