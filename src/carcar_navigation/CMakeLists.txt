cmake_minimum_required(VERSION 3.8)
project(carcar_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

ament_package()
