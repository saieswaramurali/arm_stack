# arm_mtc

MoveIt Task Constructor pick/place pipeline driven by wrist-camera perception.

## Working
- Moves the arm to the elbow-up detection pose.
- Calls the `/detect_objects` service for fresh box poses.
- For each detected box (up to `max_picks`): approach, descend, staged gripper
  close with contact detection, attach, lift, transport, and drop above the
  bin.
- Returns to the startup joint pose when done.

## Input Expectation
- Running `pick_and_place_bringup.launch.py` (table world, camera bridged).
- Running `arm_perception` serving `/detect_objects`.
- Boxes inside the wrist camera view at the detection pose
  (`spawn_objects.py` spots).

## Service Clients

| Service | Type | What It Does |
|---|---|---|
| /detect_objects | arm_interfaces/srv/DetectObjects | Fetches detected box ids + world/tcp poses. |
| /gripper_controller/gripper_cmd | control_msgs/action/GripperCommand | Staged gripper close/open (action). |

## Parameters
- `max_picks`
- `place_x`, `place_y`, `place_z`, `place_tcp_z_offset`
- `approach_clearance`, `lift_height`
- `grasp_tcp_z_offset`, `grasp_close_position`, `grasp_max_effort`,
  `grasp_step`, `grasp_pause_ms`, `grasp_settle_ms`,
  `grasp_contact_timeout_ms`

## Build and Run
```bash
colcon build --packages-select arm_mtc --symlink-install
source install/setup.bash
ros2 run arm_mtc pick_place_mtc
ros2 run arm_mtc pick_place_mtc --ros-args -p max_picks:=2 -p grasp_close_position:=0.62 -p grasp_max_effort:=32.0
```
