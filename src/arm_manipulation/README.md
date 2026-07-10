# arm_manipulation

C++ MoveIt2 client nodes for the UR5e. One package, one executable per node.
All nodes force `use_sim_time` and talk to `move_group` (group
`ur_manipulator`, end-effector link `tcp`).

## Nodes

| Node | What It Does |
|---|---|
| goto_named | Moves to an SRDF named state. `ros2 run arm_manipulation goto_named up` (default home). |
| goto_pose | Plans and executes to a hardcoded Cartesian pose (IK demo). |
| scene_demo | Adds a box CollisionObject, plans around it, attaches/detaches it. |
| trace_cartesian_path | Traces a heart in a vertical plane with Cartesian path + TOTG retiming, publishes RViz markers. |
| pick_place_static | Full hardcoded pick and place of the fixed box (no perception). Debugging flow. |
| pick_place_dynamic | Perception-driven pick and place loop using detected object poses. |
| tune_pick_place | Prints camera world pose and detected poses vs ground truth for calibration. |

## Published Topics (trace_cartesian_path)

| Topic | Type | What It Does |
|---|---|---|
| /heart_cartesian_path | visualization_msgs/msg/Marker | Heart outline + waypoint dots (transient local). |

## Parameters

pick_place_static (also shared by pick_place_dynamic and arm_mtc):
- `box_x`, `box_y`, `box_z` (static only), `place_x`, `place_y`, `place_z`
- `approach_clearance`, `lift_height`
- `grasp_tcp_z_offset`, `place_tcp_z_offset`
- `grasp_close_position`, `grasp_max_effort`, `grasp_step`,
  `grasp_pause_ms`, `grasp_settle_ms`, `grasp_contact_timeout_ms`

pick_place_dynamic additionally:
- `detect_x`, `detect_y`, `detect_camera_z` (scan pose)

trace_cartesian_path:
- `center_x`, `center_y`, `center_z`, `scale`, `samples`, `plane_yaw_deg`,
  `execute_path`, `use_current_orientation`

## Build and Run
```bash
colcon build --packages-select arm_manipulation --symlink-install
source install/setup.bash
ros2 run arm_manipulation pick_place_static
ros2 run arm_manipulation pick_place_static --ros-args -p grasp_close_position:=0.62 -p grasp_max_effort:=32.0
```

Requires a running bringup (`arm_bringup`); the pick/place nodes additionally
expect the table world and, for the dynamic node, `arm_perception`.
