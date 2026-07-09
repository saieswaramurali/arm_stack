# Setup

## Clone

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
git clone https://github.com/saieswaramurali/arm_stack.git
cd arm_stack
```

## Install Dependencies

```bash
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --ignore-src --from-paths src -y
```

Core packages are ROS 2 Humble, MoveIt2, `ros2_control`,
`ros2_controllers`, Gazebo Fortress via `ros_gz`, `ign_ros2_control`, RViz2,
and xacro.

If needed:

```bash
sudo apt install \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-ros-gz \
  ros-humble-ign-ros2-control \
  ros-humble-xacro \
  liburdfdom-tools
```

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Normal Robot

Terminal 1:

```bash
source install/setup.bash
ros2 launch arm_bringup bringup.launch.py
```

Terminal 2:

```bash
source install/setup.bash
ros2 run arm_manipulation goto_named
ros2 run arm_manipulation goto_pose
ros2 run arm_manipulation scene_demo
```

## Pick/Place Demo

Terminal 1:

```bash
source install/setup.bash
ros2 launch arm_bringup pick_and_place_bringup.launch.py
```

Terminal 2:

```bash
source install/setup.bash
ros2 run arm_manipulation pick_place_static
```

## Checks

```bash
ros2 control list_controllers
ros2 action list | grep gripper_cmd
xacro src/arm_description/urdf/ur.urdf.xacro sim_ignition:=true | check_urdf -
```
