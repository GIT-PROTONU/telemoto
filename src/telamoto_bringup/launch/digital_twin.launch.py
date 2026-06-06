"""Digital twin + motion control: UR10 CB3.1 with MoveIt2 + RViz2.

Fake hardware (default):  pixi run twin
Real robot:               pixi run twin-real

Fake mode — full ros2_control stack (no robot needed):
  ur_control.launch.py    ur_robot_driver mock + controllers
  ur_moveit.launch.py     MoveIt2 → scaled_joint_trajectory_controller
  rviz2

Real mode — no ur_robot_driver (CB3 3.15 PolyScopeX holds all RTDE inputs,
            causing SIGABRT in ros2_control_node regardless of recipe content):
  ur_rsp.launch.py          robot_state_publisher only
  ur_rtde_joint_pub.py      /joint_states via RTDE outputs (read-only, no conflict)
  ur_servo_controller.py    TCP server port 50001, 125 Hz servoj via reverse-
                            interface protocol; auto-play waits for it to open
  ur_dashboard_autoplay.py  plays ext.urp once port 50001 is open
  moveit_real.launch.py     MoveIt2 → joint_trajectory_controller action server
  rviz2
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

_CALIBRATION_FILE = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "config", "ur10_calibration.yaml")
)
_HAS_CALIBRATION = os.path.isfile(_CALIBRATION_FILE)

_AUTOPLAY_SCRIPT = PathJoinSubstitution([
    FindPackageShare("telamoto_bringup"),
    "..", "..", "lib", "telamoto_bringup", "ur_dashboard_autoplay.py",
])


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument(
            "robot_ip", default_value="192.168.10.2",
            description="IP of the UR10 CB3.1 controller",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="true",
            description="true = fake hardware (no robot), false = real robot",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
        ),
        DeclareLaunchArgument(
            "ext_program", default_value="ext.urp",
            description="URCap program to auto-play (real mode only)",
        ),
    ]

    robot_ip           = LaunchConfiguration("robot_ip")
    use_mock_hardware  = LaunchConfiguration("use_mock_hardware")
    initial_joint_ctrl = LaunchConfiguration("initial_joint_controller")
    ext_program        = LaunchConfiguration("ext_program")

    # ═══════════════════════════════════════════════════════════════════════
    # FAKE MODE
    # ═══════════════════════════════════════════════════════════════════════
    ur_control_args = {
        "ur_type":                    "ur10",
        "robot_ip":                   robot_ip,
        "use_mock_hardware":          "true",
        "initial_joint_controller":   initial_joint_ctrl,
        "launch_rviz":                "false",
        "controller_spawner_timeout": "60",
    }
    if _HAS_CALIBRATION:
        ur_control_args["kinematics_parameters_file"] = _CALIBRATION_FILE

    ur_control_fake = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py",
        ])),
        launch_arguments=ur_control_args.items(),
        condition=IfCondition(use_mock_hardware),
    )

    ur_moveit_fake = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py",
        ])),
        launch_arguments={"ur_type": "ur10", "launch_rviz": "false"}.items(),
        condition=IfCondition(use_mock_hardware),
    )

    # ═══════════════════════════════════════════════════════════════════════
    # REAL MODE
    # ═══════════════════════════════════════════════════════════════════════
    rsp_args = {
        "ur_type":           "ur10",
        "robot_ip":          robot_ip,
        "use_mock_hardware": "false",
    }
    if _HAS_CALIBRATION:
        rsp_args["kinematics_params_file"] = _CALIBRATION_FILE

    ur_rsp_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py",
        ])),
        launch_arguments=rsp_args.items(),
        condition=UnlessCondition(use_mock_hardware),
    )

    rtde_joint_pub = Node(
        package="telamoto_bringup",
        executable="ur_rtde_joint_pub.py",
        name="ur_rtde_joint_pub",
        output="screen",
        parameters=[{"robot_ip": robot_ip}],
        condition=UnlessCondition(use_mock_hardware),
    )

    # Reverse-interface server on port 50001 — robot connects after ext.urp plays
    servo_controller = Node(
        package="telamoto_bringup",
        executable="ur_servo_controller.py",
        name="ur_servo_controller",
        output="screen",
        parameters=[{"robot_ip": robot_ip}],
        condition=UnlessCondition(use_mock_hardware),
    )

    # Auto-play ext.urp via Dashboard Server once port 50001 is open
    auto_play = Node(
        package="telamoto_bringup",
        executable="ur_dashboard_autoplay.py",
        name="ur_dashboard_autoplay",
        output="screen",
        arguments=[robot_ip, ext_program],
        condition=UnlessCondition(use_mock_hardware),
    )

    ur_moveit_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("telamoto_bringup"), "launch", "moveit_real.launch.py",
        ])),
        condition=UnlessCondition(use_mock_hardware),
    )

    # ═══════════════════════════════════════════════════════════════════════
    # RVIZ2 — separate instances per condition (reusing one object breaks launch)
    # ═══════════════════════════════════════════════════════════════════════
    def _rviz():
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("telamoto_bringup"), "launch", "moveit_rviz.launch.py",
            ])),
        )

    return LaunchDescription(
        declared_args + [
            # Fake mode
            ur_control_fake,
            TimerAction(period=5.0, actions=[ur_moveit_fake], condition=IfCondition(use_mock_hardware)),
            TimerAction(period=8.0, actions=[_rviz()],        condition=IfCondition(use_mock_hardware)),
            # Real mode
            ur_rsp_real,
            rtde_joint_pub,
            servo_controller,
            auto_play,          # waits internally for port 50001, then plays ext.urp
            TimerAction(period=3.0, actions=[ur_moveit_real], condition=UnlessCondition(use_mock_hardware)),
            TimerAction(period=6.0, actions=[_rviz()],        condition=UnlessCondition(use_mock_hardware)),
        ]
    )
