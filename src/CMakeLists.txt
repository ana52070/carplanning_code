cmake_minimum_required(VERSION 3.8)
project(carplanning_code)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  scripts
  DESTINATION share/${PROJECT_NAME}
)


# livox CustomMsg → PointCloud2 转换节点
find_package(livox_ros_driver2 REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(livox_to_pc2 livox_to_pc2.cpp)
ament_target_dependencies(livox_to_pc2
  rclcpp sensor_msgs livox_ros_driver2)
install(TARGETS livox_to_pc2
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  bringup.rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
