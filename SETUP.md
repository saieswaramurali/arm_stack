# Setup

## Install Dependencies

Recommended:

```bash
rosdep update
rosdep install --ignore-src --from-paths src -y
```

Equivalent core apt packages:

```bash
sudo apt install \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-xacro \
  liburdfdom-tools
```

## Build

```bash
cd /home/sai/Desktop/ros_ws/Universal_Robots_ROS2_Gazebo_Simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
ros2 launch arm_bringup bringup.launch.py
```

## Notes

This workspace no longer needs these UR source packages:

```text
ur_description
ur_moveit_config
ur_simulation_gazebo
ur_robot_driver
ur_controllers
ur_dashboard_msgs
```

It also no longer needs the old source-build dependencies:

```text
ros-humble-ur-msgs
ros-humble-ur-client-library
ros-humble-ros2-controllers-test-nodes
ros-humble-hardware-interface-testing
ros-humble-warehouse-ros-sqlite
socat
```
