"""Standalone MoveIt2 move_group launcher (expects ros2_control already running).

Prefer digital_twin.launch.py for a complete one-command bringup.
Use this only when you need to restart move_group independently.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument("ur_type", default_value="ur10"),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("use_fake_hardware", default_value="true"),
    ]

    ur_type           = LaunchConfiguration("ur_type")
    prefix            = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro",
            ]),
            " ur_type:=",          ur_type,
            " use_fake_hardware:=", use_fake_hardware,
            " name:=ur",
            " prefix:=",           prefix,
        ]),
        value_type=str,
    )

    robot_description_semantic_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro",
            ]),
            " name:=ur",
            " prefix:=", prefix,
        ]),
        value_type=str,
    )

    kinematics_yaml = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml",
    ])

    ompl_yaml = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"), "config", "ompl_planning.yaml",
    ])

    moveit_controllers = PathJoinSubstitution([
        FindPackageShare("ur_moveit_config"), "config", "moveit_controllers.yaml",
    ])

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description":          robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
            kinematics_yaml,
            ompl_yaml,
            moveit_controllers,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription(declared_args + [move_group])
