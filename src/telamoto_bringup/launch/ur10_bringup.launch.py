"""Top-level bringup for the UR10 CB3.1 — full motion control via MoveIt2 + RViz2.

Real robot:    pixi run bringup          (robot_ip=192.168.10.2 by default)
Fake robot:    pixi run bringup-fake

Before running on the real robot:
  1. pixi run patch-rtde   (already done automatically by `pixi run bringup`)
  2. Robot must be powered on, in Remote Control mode, not running a program
  3. ur_robot_driver sends the URScript automatically — do NOT load ext.urp manually
"""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_CALIBRATION_FILE = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "config", "ur10_calibration.yaml")
)
_HAS_CALIBRATION = os.path.isfile(_CALIBRATION_FILE)


def launch_setup(context, *args, **kwargs):
    robot_ip          = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    launch_rviz       = LaunchConfiguration("launch_rviz")

    ur_control_args = {
        "ur_type":                    "ur10",
        "robot_ip":                   robot_ip,
        "use_mock_hardware":          use_mock_hardware,
        "launch_rviz":                "false",
        "controller_spawner_timeout": "60",
        "reverse_ip":                 "192.168.10.1",
    }
    if _HAS_CALIBRATION:
        ur_control_args["kinematics_parameters_file"] = _CALIBRATION_FILE

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch", "ur_control.launch.py",
            ])
        ),
        launch_arguments=ur_control_args.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                # Vendored copy of ur_moveit_config's launcher: identical except
                # move_group gets TRAC-IK from telamoto_bringup/config/kinematics.yaml.
                FindPackageShare("telamoto_bringup"),
                "launch", "ur_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type":     "ur10",
            "launch_rviz": "false",
            # ur_moveit.launch.py references this LaunchConfiguration from an
            # OpaqueFunction; its DeclareLaunchArgument default is not applied
            # when included, so pass it explicitly or the include throws
            # "launch configuration 'warehouse_sqlite_path' does not exist".
            "warehouse_sqlite_path": os.path.expanduser("~/.ros/warehouse_ros.sqlite"),
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("telamoto_bringup"),
                "launch", "moveit_rviz.launch.py",
            ])
        ),
        launch_arguments={"use_fake_hardware": use_mock_hardware}.items(),
        condition=IfCondition(launch_rviz),
    )

    # Movable test wall in the planning scene — drag it in RViz to test
    # collision detection (planning routes around it, Servo gates the jog).
    scene_wall = Node(
        package="telamoto_bringup",
        executable="scene_wall.py",
        name="scene_wall",
        output="screen",
    )

    # MoveIt must NOT be wrapped in a TimerAction: ur_moveit.launch.py starts
    # move_group from an OnProcessExit handler (wait_for_robot_description),
    # which fires after a TimerAction's launch-configuration scope is popped —
    # the deferred nodes then crash with "launch configuration
    # 'warehouse_sqlite_path' does not exist". The wait node already provides
    # the start ordering a timer would have.
    rviz_delayed = TimerAction(period=15.0, actions=[rviz])

    return [ur_driver, scene_wall, moveit, rviz_delayed]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.10.2",
            description="IP address of the UR10 controller",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock hardware instead of the physical robot",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 with MoveIt2 plugin",
        ),
        OpaqueFunction(function=launch_setup),
    ])
