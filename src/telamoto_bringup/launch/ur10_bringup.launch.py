"""Top-level bringup for the UR10 CB3.1.

Starts:
  - ur_robot_driver (real or fake hardware)
  - ros2_control joint-state broadcaster + joint-trajectory controller
  - MoveIt2 move_group
  - robot_state_publisher
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    robot_ip         = LaunchConfiguration("robot_ip")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    launch_rviz      = LaunchConfiguration("launch_rviz")

    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch", "ur_control.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type":            "ur10",
            "robot_ip":           robot_ip,
            "use_fake_hardware":  use_fake_hardware,
            "launch_rviz":        "false",
            # CB3 kinematics calibration — replace with your robot's file
            # "kinematics_params_file": "/path/to/calibration.yaml",
        }.items(),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("telamoto_bringup"),
                "launch", "moveit.launch.py",
            ])
        ),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("telamoto_bringup"),
                "launch", "moveit_rviz.launch.py",
            ])
        ),
        condition=IfCondition(launch_rviz),
    )

    # Give the driver ~3 s to come up before starting MoveIt2
    moveit_delayed = TimerAction(period=3.0, actions=[moveit])

    return [ur_driver, moveit_delayed, rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.10.2",
            description="IP address of the UR10 controller",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
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
