from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_bringup"), "launch", "sim_moveit.launch.py"]
            )
        )
    )

    return LaunchDescription([sim_moveit_launch])
