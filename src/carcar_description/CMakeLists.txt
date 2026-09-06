cmake_minimum_required(VERSION 3.8)
project(carcar_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch meshes rviz third_party urdf
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
