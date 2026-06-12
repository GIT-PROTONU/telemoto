"""Launch RViz2 with the MoveIt2 motion-planning plugin.

Passes robot_description, robot_description_semantic, and planning_pipelines
directly as node parameters so the MoveIt plugin never has to wait on topic
subscriptions and always has the planning library available on startup.
"""
import os
from pathlib import Path

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
from launch_param_builder import ParameterBuilder
from moveit_configs_utils import MoveItConfigsBuilder

_CALIBRATION_FILE = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "config", "ur10_calibration.yaml")
)
_HAS_CALIBRATION = os.path.isfile(_CALIBRATION_FILE)

# Reuse the same builder instance so RViz gets identical planning pipeline
# params to move_group — avoids "no planning library loaded" in the plugin.
_moveit_cfg = (
    MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
    .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur", "prefix": ""})
    .to_moveit_configs()
)


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument("ur_type", default_value="ur10"),
        DeclareLaunchArgument("prefix",  default_value=""),
        DeclareLaunchArgument(
            "use_fake_hardware", default_value="true",
            description="Match the value used when the URDF was loaded",
        ),
    ]

    ur_type           = LaunchConfiguration("ur_type")
    prefix            = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    urdf_args = [
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro",
        ]),
        " ur_type:=",          ur_type,
        " use_fake_hardware:=", use_fake_hardware,
        " name:=ur",
        " prefix:=",           prefix,
    ]
    if _HAS_CALIBRATION:
        urdf_args += [f" kinematics_params_file:={_CALIBRATION_FILE}"]

    robot_description_content = ParameterValue(
        Command(urdf_args), value_type=str
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

    rviz_config = PathJoinSubstitution([
        FindPackageShare("telamoto_bringup"), "rviz", "moveit.rviz",
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            # Planning pipeline config must match move_group exactly so RViz
            # knows which planners are available without querying the service.
            _moveit_cfg.planning_pipelines,
            _moveit_cfg.joint_limits,
            # TRAC-IK for the interactive-marker IK (same file move_group loads).
            {"robot_description_kinematics": ParameterBuilder("telamoto_bringup")
                .yaml("config/kinematics.yaml").to_dict()},
            {"robot_description":          robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
        ],
    )

    return LaunchDescription(declared_args + [rviz_node])
