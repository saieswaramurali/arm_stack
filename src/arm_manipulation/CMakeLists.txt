cmake_minimum_required(VERSION 3.8)
project(arm_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(goto_pose src/goto_pose.cpp)
add_executable(goto_named src/goto_named.cpp)
add_executable(scene_demo src/scene_demo.cpp)


foreach(target goto_pose goto_named scene_demo)
  target_compile_features(${target} PUBLIC cxx_std_17)
  ament_target_dependencies(${target}
    rclcpp
    geometry_msgs
    moveit_ros_planning_interface
  )
endforeach()

install(TARGETS
  goto_pose
  goto_named
  scene_demo
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
