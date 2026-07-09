# arm-stack: UR5e Manipulation Workspace (ROS 2 Humble)

UR5e simulation in Gazebo Classic driven by MoveIt2, built as clean local packages
(no Universal Robots vendor repos). Work in progress toward a full
perception-driven pick-and-place stack.

## Architecture

```text
src/
  arm_description/      # UR5e URDF/Xacro, meshes, limits, kinematics (data only)
  arm_moveit_config/    # SRDF, OMPL/kinematics/controller config for move_group
  arm_bringup/          # Gazebo, ros2_control controllers, top-level launch
  arm_manipulation/     # C++ MoveIt client nodes: goto_named, goto_pose, scene_demo
  robotiq_description/  # Vendored Robotiq 2F-85 description package only
```

Runtime flow: `arm_bringup` loads the URDF into Gazebo + `controller_manager` and
starts `move_group` (configured by `arm_moveit_config`). The `arm_manipulation`
nodes send goals to `move_group`, which plans (OMPL, group `ur_manipulator`) and
executes through `joint_trajectory_controller` back into Gazebo.

## Gripper

The end effector is a Robotiq 2F-85 attached at `tool0`. It has one actuated
joint (`robotiq_85_left_knuckle_joint`, mimic joints handled by
`gazebo_ros2_control`), its own `gripper` planning group with `open`/`closed`
named states, a `tcp` grasp frame at the fingertips, and a
`GripperCommand` action interface via `gripper_controller`.

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install --ignore-src --from-paths src -y
colcon build --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch arm_bringup bringup.launch.py
```

Starts Gazebo, `robot_state_publisher`, `gazebo_ros2_control`,
`joint_state_broadcaster`, `joint_trajectory_controller`, `gripper_controller`,
`move_group`, and RViz.

## Run the nodes

```bash
ros2 run arm_manipulation goto_named          # go to SRDF "home" (or: goto_named up)
ros2 run arm_manipulation goto_pose           # plan+execute to a Cartesian pose
ros2 run arm_manipulation scene_demo          # collision object + attach/detach demo
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.8, max_effort: 50.0}}"  # close
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 50.0}}"  # open
```

## Checks

```bash
xacro src/arm_description/urdf/ur.urdf.xacro sim_gazebo:=true | check_urdf -
ros2 control list_controllers   # expect joint_state_broadcaster, joint_trajectory_controller, gripper_controller active
ros2 action list | grep gripper_cmd
```

## Credits

The UR5e description (URDF/Xacro, meshes, kinematics) and the MoveIt/Gazebo
configs are derived from Universal Robots' open-source packages:
[Universal_Robots_ROS2_Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description),
[Universal_Robots_ROS2_Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver), and
[Universal_Robots_ROS2_Gazebo_Simulation](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation),
under the BSD-3-Clause license. Thanks to Universal Robots and the ROS
community for maintaining them.

The Robotiq 2F-85 gripper description (URDF/Xacro and meshes) is vendored from
[PickNikRobotics/ros2_robotiq_gripper](https://github.com/PickNikRobotics/ros2_robotiq_gripper),
under the BSD-3-Clause license; only the description assets are included, not
the hardware driver packages. Thanks to PickNik Robotics and Robotiq for the
open-source gripper support.
