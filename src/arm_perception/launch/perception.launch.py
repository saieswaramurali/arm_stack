from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params_file = PathJoinSubstitution(
        [FindPackageShare("arm_perception"), "config", "detector_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("show_debug_window", default_value="false"),
            DeclareLaunchArgument("publish_debug_image", default_value="true"),
            Node(
                package="arm_perception",
                executable="object_detector",
                name="object_detector",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": True,
                        "show_debug_window": LaunchConfiguration("show_debug_window"),
                        "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                    },
                ],
            )
        ]
    )
