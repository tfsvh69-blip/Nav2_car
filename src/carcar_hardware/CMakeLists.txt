cmake_minimum_required(VERSION 3.8)
project(carcar_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/differential_drive.cpp
  src/rosmaster_protocol.cpp
  src/rosmaster_system.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(
  hardware_interface carcar_hardware_plugins.xml
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch urdf DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/encoder_state_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_rosmaster_protocol test/test_rosmaster_protocol.cpp)
  target_link_libraries(test_rosmaster_protocol ${PROJECT_NAME})
  target_include_directories(test_rosmaster_protocol PRIVATE include)

  ament_add_gtest(test_plugin_loading test/test_plugin_loading.cpp)
  ament_target_dependencies(test_plugin_loading hardware_interface pluginlib)

  ament_add_gtest(test_differential_drive test/test_differential_drive.cpp)
  target_link_libraries(test_differential_drive ${PROJECT_NAME})
  target_include_directories(test_differential_drive PRIVATE include)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)
ament_package()
