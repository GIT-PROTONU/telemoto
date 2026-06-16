"""MoveIt Servo for the EMULATED UR10 (pixi run twin, mock hardware).

On the real robot, servo_node is only a SAFETY SENTINEL: it evaluates the jog
twist and ur_servo_controller streams the actual motion to the arm via URScript
speedl (see moveit_real.launch.py). The fake arm has no URScript endpoint, so
here servo_node does double duty — it ALSO drives the mock arm: its
JointTrajectory output is pointed at the active ros2_control trajectory
controller's streaming topic, so a web/keyboard jog moves the simulated robot in
RViz and the 3D twin.

Started alongside the mock move_group (vendored ur_moveit.launch.py): Servo runs
as a non-primary planning-scene monitor, so it needs move_group up to receive the
robot state / monitored scene. ur_servo_controller already publishes the jog
twist on /servo_node/delta_twist_cmds in both modes — no controller change.

⚠ This streams to the SAME controller MoveIt uses for planned moves
(scaled_joint_trajectory_controller's joint_trajectory topic). Servo only emits
output while a non-zero twist is being commanded, so an idle jog leaves the
controller free for RViz Plan & Execute; while you are actively jogging, the jog
owns the controller (expected).
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

# SRDF / kinematics / joint limits come from the upstream builder (identical to
# moveit_real.launch.py — Servo's kinematics must not diverge between modes).
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
            "jog_command_topic",
            default_value="/scaled_joint_trajectory_controller/joint_trajectory",
            description="JTC streaming topic Servo drives in mock mode",
        ),
    ]

    ur_type = LaunchConfiguration("ur_type")
    prefix  = LaunchConfiguration("prefix")
    jog_command_topic = LaunchConfiguration("jog_command_topic")

    # Mock-hardware URDF — kinematics identical to real, just the ros2_control
    # hardware plugin differs (irrelevant to Servo, which only reads kinematics).
    urdf_args = [
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([
            FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro",
        ]),
        " ur_type:=", ur_type,
        " use_fake_hardware:=true",
        " name:=ur",
        " prefix:=", prefix,
    ]
    if _HAS_CALIBRATION:
        urdf_args += [f" kinematics_params_file:={_CALIBRATION_FILE}"]

    robot_description_content = ParameterValue(Command(urdf_args), value_type=str)

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

    # Reuse the real sentinel config (same singularity/collision tuning) but point
    # the JointTrajectory output at the live controller so it ACTUATES the fake arm
    # (on real, command_out_topic is the unused /telamoto/servo_command).
    servo_yaml = ParameterBuilder("telamoto_bringup").yaml("config/ur_servo.yaml").to_dict()
    servo_params = {"moveit_servo": servo_yaml}

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            # command_out_topic override (after servo_params so it wins): drive the
            # mock JTC's streaming interface instead of the unused real-mode topic.
            {"moveit_servo.command_out_topic": jog_command_topic},
            # Smoother plugin reads these un-namespaced (see moveit_real.launch.py).
            {"planning_group_name": "ur_manipulator"},
            {"update_period": 0.004},
            {"robot_description":          robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
            _moveit_cfg.robot_description_kinematics,
            _moveit_cfg.joint_limits,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription(declared_args + [servo_node])
