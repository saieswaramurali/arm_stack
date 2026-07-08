# UR5e Gazebo + MoveIt2 Workspace

This workspace is a UR5e-only simulation setup for Gazebo Classic and MoveIt2.
The upstream Universal Robots source repos have been replaced with three local
project packages:

```text
src/
  arm_description/      # UR5e URDF/Xacro, meshes, limits, kinematics
  arm_moveit_config/    # UR5e MoveIt2 SRDF, OMPL, kinematics, controllers
  arm_bringup/          # Gazebo, ros2_control, and top-level launch files
```

## Build

```bash
source /opt/ros/humble/setup.bash
rosdep install --ignore-src --from-paths src -y
colcon build --symlink-install
source install/setup.bash
```

If you deleted `install/` in a terminal that had already sourced this workspace,
reset the environment before rebuilding:

```bash
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## Launch

```bash
ros2 launch arm_bringup bringup.launch.py
```

This starts:

- Gazebo Classic
- `robot_state_publisher`
- `gazebo_ros2_control`
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- MoveIt `move_group`
- RViz with the MoveIt plugin

## Useful Checks

```bash
xacro src/arm_description/urdf/ur.urdf.xacro sim_gazebo:=true > /tmp/ur5e.urdf
check_urdf /tmp/ur5e.urdf
colcon list
rosdep check --ignore-src --from-paths src
```

After launching:

```bash
ros2 control list_controllers
```

Expected active controllers:

```text
joint_state_broadcaster
joint_trajectory_controller
```
