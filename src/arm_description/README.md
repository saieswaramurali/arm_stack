# arm_description

UR5e robot description with the Robotiq 2F-85 gripper and wrist camera. Pure
data package, no nodes.

## Working
- Provides the top-level `ur.urdf.xacro` combining the UR5e arm, the Robotiq
  2F-85 (from `robotiq_description`), the `tcp` grasp frame, and the wrist
  RGB-D camera link and sensor.
- Provides `ur.ros2_control.xacro` with the `gz_ros2_control` (Ignition/Gazebo
  Fortress) hardware branch for all arm and gripper joints, including the
  gripper mimic joints.
- Provides UR5e meshes, joint limits, kinematics, and physical/visual
  parameter yaml files.

## Frames
| Frame | What It Is |
|---|---|
| base_link | Robot base, planning chain root |
| tool0 | UR flange, gripper and camera mount parent |
| tcp | Grasp frame between the gripper fingertips |
| wrist_camera_link | Camera body, fixed to tool0 |
| wrist_camera_optical_frame | REP-103 optical frame used by perception |

## Usage
```bash
xacro urdf/ur.urdf.xacro sim_ignition:=true | check_urdf -
```

Consumed by `arm_moveit_config` (SRDF wraps these frames) and `arm_bringup`
(loads the URDF into Gazebo and controller_manager).
