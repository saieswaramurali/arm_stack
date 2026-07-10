# arm_bringup

Top-level launch, Gazebo Fortress worlds, ros2_control controller
configuration, and the object spawner. No custom nodes except the spawner
script.

## Working
- `bringup.launch.py`: plain robot flow. Gazebo Fortress,
  `robot_state_publisher`, `gz_ros2_control`, controller spawners, clock
  bridge, `move_group`, RViz.
- `pick_and_place_bringup.launch.py`: pick/place flow. Table world, robot on
  the platform, wrist camera bridge, optional random box spawning and drop
  bin.
- `sim_control.launch.py` / `sim_moveit.launch.py`: building blocks included
  by the two above.
- `config/controllers.yaml`: `joint_state_broadcaster`,
  `joint_trajectory_controller`, `gripper_controller`
  (position_controllers/GripperActionController on
  `robotiq_85_left_knuckle_joint`).
- `worlds/`: table + pick box world for Fortress.
- `scripts/spawn_objects.py`: deletes/spawns colored boxes at camera-visible
  calibrated spots and spawns the drop bin.

## Launch
```bash
ros2 launch arm_bringup bringup.launch.py
ros2 launch arm_bringup pick_and_place_bringup.launch.py
```

Useful launch arguments for `pick_and_place_bringup.launch.py`:
- `spawn_randomly` (default true)
- `random_box_count`
- `enable_camera`
- `robot_base_z`

## Object Spawner
```bash
ros2 run arm_bringup spawn_objects.py --reset
ros2 run arm_bringup spawn_objects.py --reset --count 2
ros2 run arm_bringup spawn_objects.py --reset --count 1 --x 0.0 --y 0.75 --yaw 0.0 --color r
```
- Spawns boxes at 4 calibrated spots inside the wrist camera view at the scan
  pose (camera measured at world (-0.162, 0.808, 0.668) looking down).
- `--x/--y/--yaw/--color` override spots for calibration tests.
- Recalibrate the spots in the script if the scan pose changes.
