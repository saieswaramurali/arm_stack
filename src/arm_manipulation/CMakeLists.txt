cmake_minimum_required(VERSION 3.8)
project(arm_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(arm_interfaces REQUIRED)

add_executable(goto_pose src/goto_pose.cpp)
add_executable(goto_named src/goto_named.cpp)
add_executable(scene_demo src/scene_demo.cpp)
add_executable(trace_cartesian_path src/trace_cartesian_path.cpp)
add_executable(pick_place_static src/pick_place_static.cpp)
add_executable(pick_place_dynamic src/pick_place_dynamic.cpp)
add_executable(tune_pick_place src/tune_pick_place.cpp)


foreach(target goto_pose goto_named scene_demo trace_cartesian_path pick_place_static pick_place_dynamic tune_pick_place)
  target_compile_features(${target} PUBLIC cxx_std_17)
  ament_target_dependencies(${target}
    rclcpp
    rclcpp_action
    control_msgs
    geometry_msgs
    moveit_msgs
    shape_msgs
    visualization_msgs
    moveit_ros_planning_interface
    arm_interfaces
  )
endforeach()

install(TARGETS
  goto_pose
  goto_named
  scene_demo
  trace_cartesian_path
  pick_place_static
  pick_place_dynamic
  tune_pick_place
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
