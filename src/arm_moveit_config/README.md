# arm_moveit_config

MoveIt2 configuration for the UR5e + Robotiq 2F-85. Config package, starts the
stock `move_group` node.

## Working
- SRDF defines two planning groups: `ur_manipulator` (chain `base_link` to
  `tcp`, 6 DOF) and `gripper` (`robotiq_85_left_knuckle_joint`).
- Named states: `home`, `up`, `test_configuration` for the arm; `open`,
  `closed` for the gripper.
- Disabled-collision matrix covers the gripper mount chain and finger links.
- `kinematics.yaml`: KDL solver for `ur_manipulator`, timeout 0.05.
- `ompl_planning.yaml`: OMPL pipeline, RRTConnect default.
- `controllers.yaml`: routes arm trajectories to `joint_trajectory_controller`
  (FollowJointTrajectory) and gripper commands to `gripper_controller`
  (GripperCommand).

## Launch
```bash
ros2 launch arm_moveit_config moveit.launch.py
```

Starts `move_group` and RViz with the MoveIt plugin. Normally included by
`arm_bringup` launches rather than run directly.
