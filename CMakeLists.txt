cmake_minimum_required(VERSION 3.8)
project(campus_nav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  urdf
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
