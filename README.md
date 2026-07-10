# arm-stack: UR5e Manipulation Workspace (ROS 2 Humble)

UR5e simulation in Gazebo Fortress driven by MoveIt2, built as clean local packages
(no Universal Robots vendor repos). A perception-driven pick-and-place stack:
wrist-camera RGB-D detection, MoveIt Task Constructor picking, and drop-to-bin.

![Dynamic pick and place demo](docs/images/pick_and_place_dynamic.gif)

## Setup

First-time setup on ROS 2 Humble: clone and run the setup script. It installs
dependencies via rosdep and apt, then builds the workspace.

```bash
git clone https://github.com/saieswaramurali/arm_stack.git
cd arm_stack
./setup.sh
source install/setup.bash
```

For manual step-by-step installation and the full launch walkthrough, see
[SETUP.md](SETUP.md).

## Dependencies

ROS 2 Humble, MoveIt2, ros2_control/ros2_controllers, Gazebo Fortress
(`ros_gz`), `gz_ros2_control`, RViz2, and xacro.

## Architecture

```text
src/
  arm_description/      # UR5e URDF/Xacro, meshes, limits, kinematics (data only)
  robotiq_description/  # Vendored Robotiq 2F-85 description package only
  arm_moveit_config/    # SRDF, OMPL/kinematics/controller config for move_group
  arm_bringup/          # Gazebo worlds, ros2_control controllers, object spawner, launch
  arm_manipulation/     # C++ MoveIt clients: goto_*, scene_demo, trace_cartesian_path,
                        # pick_place_static, pick_place_dynamic, tune_pick_place
  arm_perception/       # Wrist-camera RGB-D object detector + /detect_objects service
  arm_interfaces/       # Shared ROS interfaces for perception/manipulation
  arm_mtc/              # MoveIt Task Constructor dynamic pick/place pipeline
```

Runtime flow: `arm_bringup` loads the URDF into Gazebo Fortress and
`controller_manager`, and starts `move_group` (configured by
`arm_moveit_config`). `arm_perception` reads the wrist camera, deprojects
detected boxes to world poses, and serves them via `/detect_objects`.
`arm_mtc` moves to the scan pose, requests detections, and plans pick and
place tasks through `move_group`, which executes through
`joint_trajectory_controller` and `gripper_controller` back into Gazebo.

## Gripper

The end effector is a Robotiq 2F-85 attached at `tool0`. It has one actuated
joint (`robotiq_85_left_knuckle_joint`, mimic joints handled by
`gz_ros2_control`), its own `gripper` planning group with `open`/`closed`
named states, a `tcp` grasp frame at the fingertips, and a
`GripperCommand` action interface via `gripper_controller`.

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install --ignore-src --from-paths src -y
colcon build --symlink-install
source install/setup.bash
```

## Normal Robot Flow

Use this when you only need the robot, MoveIt, controllers, and RViz.

```bash
ros2 launch arm_bringup bringup.launch.py
```

Starts Gazebo Fortress, `robot_state_publisher`, `gz_ros2_control`,
`joint_state_broadcaster`, `joint_trajectory_controller`, `gripper_controller`,
`move_group`, and RViz with the plain robot setup.

Useful nodes:

```bash
ros2 run arm_manipulation goto_named        # go to SRDF "home" (or: goto_named up)
ros2 run arm_manipulation goto_pose         # plan+execute to a Cartesian pose
ros2 run arm_manipulation scene_demo        # collision object + attach/detach demo
```

## Dynamic Pick/Place Pipeline

The main demo: random colored boxes, wrist-camera perception, and the
drop-to-bin flow. By default this launch deletes the fixed `pick_box`, spawns
two colored boxes at calibrated camera-visible spots, and adds the drop bin.

Terminal 1:

```bash
ros2 launch arm_bringup pick_and_place_bringup.launch.py
```

Useful launch parameters:

```bash
ros2 launch arm_bringup pick_and_place_bringup.launch.py spawn_randomly:=true random_box_count:=2
ros2 launch arm_bringup pick_and_place_bringup.launch.py enable_camera:=true
```

Terminal 2, run perception:

```bash
ros2 launch arm_perception perception.launch.py
```

Debug the detector:

```bash
ros2 launch arm_perception perception.launch.py show_debug_window:=true
ros2 topic hz /detected_objects
ros2 topic echo /detected_objects --once
```

Terminal 3, run the dynamic MTC-assisted pick/place:

```bash
ros2 run arm_mtc pick_place_mtc
```

The MTC flow moves to the elbow-up detection pose, calls `/detect_objects`,
picks each detected box, drops it directly above the bin, then returns to the
startup joint pose from `pick_place_initial_positions.yaml`.

Useful MTC parameters:

```bash
ros2 run arm_mtc pick_place_mtc --ros-args \
  -p max_picks:=2 -p grasp_close_position:=0.62 -p grasp_max_effort:=32.0
```

In RViz, add an `Image` display for:

```text
/object_detector/debug_image
```

Random scene utility:

```bash
ros2 run arm_bringup spawn_objects.py --reset
ros2 run arm_bringup spawn_objects.py --reset --count 2
ros2 run arm_bringup spawn_objects.py --reset --count 1 --x 0.0 --y 0.75 --yaw 0.0 --color r
```

Gripper actions:

```bash
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.8, max_effort: 50.0}}"  # close
ros2 action send_goal /gripper_controller/gripper_cmd control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 50.0}}"  # open
```

## Static Pick/Place Flow (debugging)

A debugging flow that skips perception: one known blue box at a fixed pose,
picked with hardcoded coordinates by `pick_place_static`. Useful for isolating
grasp tuning, gripper behavior, and motion issues from the perception pipeline.

```bash
ros2 launch arm_bringup pick_and_place_bringup.launch.py spawn_randomly:=false
ros2 run arm_manipulation pick_place_static
```

Useful static parameters:

```bash
ros2 run arm_manipulation pick_place_static --ros-args \
  -p box_x:=0.0 -p box_y:=0.75 -p box_z:=0.33 \
  -p place_x:=0.25 -p place_y:=0.75 -p place_z:=0.33 \
  -p grasp_close_position:=0.62 -p grasp_max_effort:=32.0
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
