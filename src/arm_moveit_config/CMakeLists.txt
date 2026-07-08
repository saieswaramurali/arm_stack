cmake_minimum_required(VERSION 3.8)
project(arm_moveit_config)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz srdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
