from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    world = LaunchConfiguration("world")
    robot_base_z = LaunchConfiguration("robot_base_z")
    enable_camera = LaunchConfiguration("enable_camera")

    controllers_file_abs = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            "name:=ur",
            " ",
            "prefix:=",
            prefix,
            " ",
            "robot_base_z:=",
            robot_base_z,
            " ",
            "sim_ignition:=true",
            " ",
            "simulation_controllers:=",
            controllers_file_abs,
            " ",
            "initial_positions_file:=",
            initial_positions_file,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": ["-r ", world]}.items(),
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_ur",
        arguments=["-name", "ur", "-topic", "robot_description", "-z", "0"],
        output="screen",
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    camera_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/wrist_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/wrist_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image",
            "/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
            "/wrist_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
        ],
        output="screen",
        condition=IfCondition(enable_camera),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            initial_joint_controller,
            "-c",
            "/controller_manager",
            "--stopped",
            "--controller-manager-timeout",
            "60",
        ],
        condition=UnlessCondition(start_joint_controller),
    )
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-c",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
    )

    return [
        gz_sim,
        robot_state_publisher_node,
        spawn_robot,
        clock_bridge,
        camera_bridge,
        joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        gripper_controller_spawner,
    ]


def generate_launch_description():
    resource_paths = [
        AppendEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            PathJoinSubstitution([FindPackageShare("arm_description"), ".."]),
            separator=":",
        ),
        AppendEnvironmentVariable(
            "IGN_GAZEBO_RESOURCE_PATH",
            PathJoinSubstitution([FindPackageShare("robotiq_description"), ".."]),
            separator=":",
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            PathJoinSubstitution([FindPackageShare("arm_description"), ".."]),
            separator=":",
        ),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH",
            PathJoinSubstitution([FindPackageShare("robotiq_description"), ".."]),
            separator=":",
        ),
    ]

    declared_arguments = [
        DeclareLaunchArgument("safety_limits", default_value="true"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("runtime_config_package", default_value="arm_bringup"),
        DeclareLaunchArgument("controllers_file", default_value="controllers.yaml"),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("arm_description"), "config", "initial_positions.yaml"]
            ),
        ),
        DeclareLaunchArgument("description_package", default_value="arm_description"),
        DeclareLaunchArgument("description_file", default_value="ur.urdf.xacro"),
        DeclareLaunchArgument("prefix", default_value='""'),
        DeclareLaunchArgument("start_joint_controller", default_value="true"),
        DeclareLaunchArgument("initial_joint_controller", default_value="joint_trajectory_controller"),
        DeclareLaunchArgument("robot_base_z", default_value="0.0"),
        DeclareLaunchArgument("world", default_value="empty.sdf"),
        DeclareLaunchArgument("enable_camera", default_value="false"),
    ]

    return LaunchDescription(
        resource_paths + declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
