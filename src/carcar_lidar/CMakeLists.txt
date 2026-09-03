cmake_minimum_required(VERSION 3.8)
project(carcar_lidar)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
