"""MoveIt2 move_group for the real UR10 CB3.1.

Used by digital_twin.launch.py when use_mock_hardware=false.
Loads moveit_controllers_real.yaml so MoveIt2 drives
joint_trajectory_controller/follow_joint_trajectory (our Python
URScript executor) instead of the ros2_control scaled controller.
"""
import os

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

_CALIBRATION_FILE = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "config", "ur10_calibration.yaml")
)
_HAS_CALIBRATION = os.path.isfile(_CALIBRATION_FILE)


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument("ur_type", default_value="ur10"),
        DeclareLaunchArgument("prefix", default_value=""),
    ]

    ur_type = LaunchConfiguration("ur_type")
    prefix  = LaunchConfiguration("prefix")

    urdf_args = [
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro",
        ]),
        " ur_type:=", ur_type,
        " use_fake_hardware:=false",
        " name:=ur",
        " prefix:=", prefix,
    ]
    if _HAS_CALIBRATION:
        urdf_args += [f" kinematics_params_file:={_CALIBRATION_FILE}"]

    robot_description_content = ParameterValue(
        Command(urdf_args),
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

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            {"robot_description":          robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"), "config", "kinematics.yaml",
            ]),
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"), "config", "ompl_planning.yaml",
            ]),
            PathJoinSubstitution([
                FindPackageShare("telamoto_bringup"), "config", "moveit_controllers_real.yaml",
            ]),
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription(declared_args + [move_group])
