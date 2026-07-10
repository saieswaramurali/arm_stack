cmake_minimum_required(VERSION 3.8)
project(arm_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(OpenCV REQUIRED)
find_package(arm_interfaces REQUIRED)

add_executable(object_detector src/object_detector.cpp)
target_compile_features(object_detector PUBLIC cxx_std_17)
target_include_directories(object_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(object_detector ${OpenCV_LIBRARIES})
ament_target_dependencies(object_detector
  rclcpp
  sensor_msgs
  geometry_msgs
  moveit_msgs
  shape_msgs
  cv_bridge
  message_filters
  tf2_ros
  tf2_geometry_msgs
  moveit_ros_planning_interface
  arm_interfaces
)

install(TARGETS
  object_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
