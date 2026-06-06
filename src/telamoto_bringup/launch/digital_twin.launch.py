"""Digital twin + motion control: UR10 CB3.1 with MoveIt2 + RViz2.

Fake hardware (default):  pixi run twin
Real robot:               pixi run twin-real

Both modes run the full ur_robot_driver + ros2_control + MoveIt2 stack.
In real mode ext.urp is auto-played on the robot via the Dashboard Server
~5 s after launch so the External Control URCap can call back to the driver.

CB3 note: PolyScopeX 3.15 holds all RTDE input variables permanently.
The empty RTDE input recipe (applied by `pixi run twin-real` via patch-rtde)
lets on_configure() receive a valid recipe_id and succeed. Actual motion and
joint-state reading happen through the reverse connection on port 50001 and
RTDE outputs respectively — neither needs RTDE inputs.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_CALIBRATION_FILE = os.path.normpath(
    os.path.join(os.path.dirname(__file__), "..", "config", "ur10_calibration.yaml")
)
_HAS_CALIBRATION = os.path.isfile(_CALIBRATION_FILE)

# FindPackageShare resolves to install/telamoto_bringup/share/telamoto_bringup;
# ../../lib/telamoto_bringup/ reaches the sibling lib directory at install time.
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
            description="URCap program to auto-play on the real robot",
        ),
    ]

    robot_ip                = LaunchConfiguration("robot_ip")
    use_mock_hardware       = LaunchConfiguration("use_mock_hardware")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    ext_program             = LaunchConfiguration("ext_program")

    # ── UR DRIVER (both modes) ────────────────────────────────────────────────
    ur_control_args = {
        "ur_type":                    "ur10",
        "robot_ip":                   robot_ip,
        "use_mock_hardware":          use_mock_hardware,
        "initial_joint_controller":   initial_joint_controller,
        "launch_rviz":                "false",
        "controller_spawner_timeout": "60",
        "reverse_ip":                 "192.168.10.1",
    }
    if _HAS_CALIBRATION:
        ur_control_args["kinematics_parameters_file"] = _CALIBRATION_FILE

    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"), "launch", "ur_control.launch.py",
            ])
        ),
        launch_arguments=ur_control_args.items(),
    )

    # ── AUTO-PLAY ext.urp ON REAL ROBOT ──────────────────────────────────────
    # Waits 5 s for the driver to start listening on port 50001, then tells the
    # robot Dashboard Server to load and play the External Control program.
    auto_play_ext = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=["python3", _AUTOPLAY_SCRIPT, robot_ip, ext_program],
            output="screen",
        )],
        condition=UnlessCondition(use_mock_hardware),
    )

    # ── MOVEIT2 ───────────────────────────────────────────────────────────────
    ur_moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"), "launch", "ur_moveit.launch.py",
            ])
        ),
        launch_arguments={
            "ur_type":     "ur10",
            "launch_rviz": "false",
        }.items(),
    )

    # Fake: controllers ready in ~3 s; Real: driver connects after ext.urp plays (~8 s)
    moveit_delayed_fake = TimerAction(
        period=5.0,
        actions=[ur_moveit],
        condition=IfCondition(use_mock_hardware),
    )
    moveit_delayed_real = TimerAction(
        period=12.0,
        actions=[ur_moveit],
        condition=UnlessCondition(use_mock_hardware),
    )

    # ── RVIZ2 ─────────────────────────────────────────────────────────────────
    # Each TimerAction needs its own IncludeLaunchDescription instance —
    # reusing one object across multiple actions breaks ROS2 launch.
    def _rviz():
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare("telamoto_bringup"), "launch", "moveit_rviz.launch.py",
                ])
            ),
        )

    rviz_delayed_fake = TimerAction(
        period=8.0,
        actions=[_rviz()],
        condition=IfCondition(use_mock_hardware),
    )
    rviz_delayed_real = TimerAction(
        period=16.0,
        actions=[_rviz()],
        condition=UnlessCondition(use_mock_hardware),
    )

    return LaunchDescription(
        declared_args + [
            ur_control,
            auto_play_ext,
            moveit_delayed_fake,
            moveit_delayed_real,
            rviz_delayed_fake,
            rviz_delayed_real,
        ]
    )
