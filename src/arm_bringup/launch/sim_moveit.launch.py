from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("runtime_config_package", default_value="arm_bringup"),
        DeclareLaunchArgument("controllers_file", default_value="controllers.yaml"),
        DeclareLaunchArgument("description_package", default_value="arm_description"),
        DeclareLaunchArgument("description_file", default_value="ur.urdf.xacro"),
        DeclareLaunchArgument("moveit_config_package", default_value="arm_moveit_config"),
        DeclareLaunchArgument("moveit_config_file", default_value="arm.srdf.xacro"),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_description"), "config", "initial_positions.yaml"]
            ),
        ),
        DeclareLaunchArgument("robot_base_z", default_value="0.0"),
        DeclareLaunchArgument("world", default_value="empty.sdf"),
        DeclareLaunchArgument("enable_camera", default_value="false"),
    ]

    sim_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_bringup"), "launch", "sim_control.launch.py"]
            )
        ),
        launch_arguments={
            "safety_limits": LaunchConfiguration("safety_limits"),
            "runtime_config_package": LaunchConfiguration("runtime_config_package"),
            "controllers_file": LaunchConfiguration("controllers_file"),
            "description_package": LaunchConfiguration("description_package"),
            "description_file": LaunchConfiguration("description_file"),
            "prefix": LaunchConfiguration("prefix"),
            "initial_positions_file": LaunchConfiguration("initial_positions_file"),
            "robot_base_z": LaunchConfiguration("robot_base_z"),
            "world": LaunchConfiguration("world"),
            "enable_camera": LaunchConfiguration("enable_camera"),
        }.items(),
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("arm_moveit_config"), "launch", "moveit.launch.py"]
            )
        ),
        launch_arguments={
            "safety_limits": LaunchConfiguration("safety_limits"),
            "description_package": LaunchConfiguration("description_package"),
            "description_file": LaunchConfiguration("description_file"),
            "moveit_config_package": LaunchConfiguration("moveit_config_package"),
            "moveit_config_file": LaunchConfiguration("moveit_config_file"),
            "prefix": LaunchConfiguration("prefix"),
            "robot_base_z": LaunchConfiguration("robot_base_z"),
            "use_sim_time": "true",
            "launch_rviz": "true",
        }.items(),
    )

    return LaunchDescription(declared_arguments + [sim_control_launch, moveit_launch])
