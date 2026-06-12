"""MoveIt2 move_group for the real UR10 CB3.1.

Uses MoveItConfigsBuilder (same as the upstream ur_moveit.launch.py) so that
planning pipelines, trajectory execution, planning scene monitor, joint limits,
and kinematics are guaranteed-correct.  Only the robot_description (real URDF,
no ros2_control hardware interface) and the controller config (our custom
FollowJointTrajectory action server) are overridden.
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

# Build the full MoveIt config using the upstream builder.
# - robot_description:          falls back to /robot_description topic (fine —
#                               ur_rsp.launch.py publishes it); overridden below.
# - robot_description_semantic: loaded from the SRDF xacro; overridden below
#                               with our explicit call so prefix works.
# - planning_pipelines, trajectory_execution, planning_scene_monitor,
#   joint_limits, kinematics:   auto-configured correctly by the builder.
_moveit_cfg = (
    MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
    .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur", "prefix": ""})
    .to_moveit_configs()
)


def generate_launch_description():
    declared_args = [
        DeclareLaunchArgument("ur_type", default_value="ur10"),
        DeclareLaunchArgument("prefix",  default_value=""),
    ]

    ur_type = LaunchConfiguration("ur_type")
    prefix  = LaunchConfiguration("prefix")

    # Real-hardware URDF (no ros2_control hardware plugin section needed).
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

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        # Robot-off cold start: with no RTDE there are no /joint_states, the
        # planning scene monitor times out (wait_for_initial_state_timeout)
        # and move_group exits FATAL — permanently for the session, taking
        # planned moves AND scene_wall's ACM fetch (floor + base column) with
        # it. Respawn until the robot comes up; once joint states flow it
        # configures and stays.
        respawn=True,
        respawn_delay=5.0,
        parameters=[
            # Builder provides: planning pipeline (OMPL), trajectory execution,
            # planning scene monitor options, joint limits, kinematics.
            _moveit_cfg.to_dict(),
            # TRAC-IK for planned moves (overrides the builder's KDL per-key).
            # servo_node below deliberately keeps the builder kinematics: Servo
            # is the jog safety sentinel — don't change what it was tuned with.
            {"robot_description_kinematics": ParameterBuilder("telamoto_bringup")
                .yaml("config/kinematics.yaml").to_dict()},
            # Override robot_description with the real-hardware URDF.
            {"robot_description":          robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
            # Use our action server (ur_servo_controller) instead of ros2_control.
            PathJoinSubstitution([
                FindPackageShare("telamoto_bringup"), "config", "moveit_controllers_real.yaml",
            ]),
            {
                # Explicit joint-state topic so the planning scene monitor is
                # seeded from the RTDE publisher, not ros2_control.
                "planning_scene_monitor_options": {
                    "joint_state_topic":              "/joint_states",
                    "wait_for_initial_state_timeout": 10.0,
                },
                "publish_robot_description_semantic": True,
                "use_sim_time": False,
                # The web tuning UI time-scales executed trajectories (Speed
                # slider). Don't abort when a slowed-down move takes longer than
                # MoveIt planned.
                "trajectory_execution.execution_duration_monitoring": False,
            },
        ],
    )

    # MoveIt Servo for WASD jogging. Receives TwistStamped on
    # /servo_node/delta_twist_cmds and publishes a JointTrajectory on
    # /telamoto/servo_command, which ur_servo_controller streams to the robot.
    servo_params = {
        "moveit_servo": ParameterBuilder("telamoto_bringup")
        .yaml("config/ur_servo.yaml").to_dict()
    }
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params,
            # Top-level (un-namespaced) params the AccelerationLimitedPlugin
            # smoother reads directly off the node: planning_group_name and
            # update_period (= publish_period). update_period has NO default and
            # the plugin THROWS on init if it's missing → Servo never servos and
            # WASD jog goes dead silent. Keep update_period == ur_servo.yaml's
            # publish_period.
            {"planning_group_name": "ur_manipulator"},
            {"update_period": 0.004},
            {"robot_description":          robot_description_content},
            {"robot_description_semantic": robot_description_semantic_content},
            _moveit_cfg.robot_description_kinematics,
            _moveit_cfg.joint_limits,
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription(declared_args + [move_group, servo_node])
