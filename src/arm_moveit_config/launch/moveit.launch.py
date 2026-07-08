import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    absolute_path = os.path.join(package_path, relative_path)
    with open(absolute_path, "r", encoding="utf-8") as yaml_file:
        return yaml.safe_load(yaml_file)


def launch_setup(context, *args, **kwargs):
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

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
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(moveit_config_package), "srdf", moveit_config_file]),
            " ",
            "name:=ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }
    publish_robot_description_semantic = {"publish_robot_description_semantic": True}

    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "config", "kinematics.yaml"]
    )
    robot_description_planning = {
        "robot_description_planning": load_yaml(
            str(moveit_config_package.perform(context)), "config/joint_limits.yaml"
        )
    }

    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml(
        str(moveit_config_package.perform(context)), "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    controllers_yaml = load_yaml(
        str(moveit_config_package.perform(context)), "config/controllers.yaml"
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": use_sim_time},
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": use_sim_time},
        ],
    )

    return [move_group_node, rviz_node]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enable safety limits in the robot description.",
        ),
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="Margin to lower and upper joint limits.",
        ),
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="Safety controller k-position factor.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="arm_description",
            description="Package containing the robot URDF/Xacro.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/Xacro file to load.",
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="arm_moveit_config",
            description="Package containing MoveIt configuration.",
        ),
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="arm.srdf.xacro",
            description="SRDF/Xacro file to load.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Joint/link name prefix.",
        ),
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz."),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
