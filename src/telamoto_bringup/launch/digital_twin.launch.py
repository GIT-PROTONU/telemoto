"""Digital twin: UR10 CB3.1 with MoveIt2 + RViz2.

Fake hardware (default):  pixi run twin
Real robot:               pixi run twin-real
Custom IP:                pixi run twin-real robot_ip:=<ip>

Fake mode (use_mock_hardware=true)
  ur_control.launch.py  → RSP + ros2_control + controllers (mock)
  ur_moveit.launch.py   → MoveIt2 planning server
  rviz2

Real mode (use_mock_hardware=false)
  ur_rsp.launch.py      → robot_state_publisher (real URDF, no ros2_control)
  ur_rtde_joint_pub.py  → RTDE outputs reader → /joint_states
  ur_moveit.launch.py   → MoveIt2 (monitoring mode, no execution)
  rviz2

CB3 note: PolyScopeX 3.15 holds all RTDE INPUT variables internally.
ur_robot_driver's on_configure() fails fatally trying to subscribe them.
We bypass it entirely: direct RTDE output reading gives joint states for RViz.
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
        DeclareLaunchArgument(
            "robot_ip", default_value="192.168.10.2",
            description="IP of the UR10 CB3.1 controller",
        ),
        DeclareLaunchArgument(
            "use_mock_hardware", default_value="true",
            description="Use simulated hardware (twin) or real robot (twin-real)",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
        ),
    ]

    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")

    # ── FAKE MODE ─────────────────────────────────────────────────────────────
    # Full ros2_control stack with mock hardware (no robot required).
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
        condition=IfCondition(use_mock_hardware),
    )

    # ── REAL MODE ─────────────────────────────────────────────────────────────
    # robot_state_publisher only (no ros2_control), joints from RTDE reader.
    rsp_args = {
        "ur_type":           "ur10",
        "robot_ip":          robot_ip,
        "use_mock_hardware": "false",
    }
    if _HAS_CALIBRATION:
        rsp_args["kinematics_params_file"] = _CALIBRATION_FILE

    ur_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"), "launch", "ur_rsp.launch.py",
            ])
        ),
        launch_arguments=rsp_args.items(),
        condition=UnlessCondition(use_mock_hardware),
    )

    rtde_joint_pub = Node(
        package="telamoto_bringup",
        executable="ur_rtde_joint_pub.py",
        name="ur_rtde_joint_pub",
        output="screen",
        parameters=[{"robot_ip": robot_ip, "frequency": 50.0}],
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

    # ── RVIZ2 ─────────────────────────────────────────────────────────────────
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro",
            ]),
            " ur_type:=ur10",
            " use_fake_hardware:=", use_mock_hardware,
            " name:=ur10",
        ]),
        value_type=str,
    )

    robot_description_semantic_content = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"), "srdf", "ur.srdf.xacro",
            ]),
            " name:=ur10",
        ]),
        value_type=str,
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("telamoto_bringup"), "rviz", "moveit.rviz",
    ])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            {"robot_description":          robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
        ],
    )

    return LaunchDescription(
        declared_args + [
            ur_control,     # fake mode only
            ur_rsp,         # real mode only
            rtde_joint_pub, # real mode only
            ur_moveit,
            TimerAction(period=8.0, actions=[rviz]),
        ]
    )
