"""Digital twin + motion control: UR10 CB3.1 with MoveIt2 + RViz2.

Fake hardware (default):  pixi run twin
Real robot:               pixi run twin-real
                          (append launch_rviz:=false for headless jog-tuning)

Fake mode — full ros2_control stack (no robot needed):
  ur_control.launch.py    ur_robot_driver mock + controllers
  ur_moveit.launch.py     MoveIt2 → scaled_joint_trajectory_controller
  rviz2

Real mode — no ur_robot_driver (CB3 3.15 PolyScopeX holds all RTDE inputs,
            causing SIGABRT in ros2_control_node regardless of recipe content):
  ur_rsp.launch.py          robot_state_publisher only
  ur_rtde_joint_pub.py      /joint_states via RTDE outputs (read-only, no conflict)
  ur_servo_controller.py    TCP server port 50001, 125 Hz servoj via reverse-
                            interface; uploads URScript with auto-detected IP
  moveit_real.launch.py     MoveIt2 → joint_trajectory_controller action server
  rviz2
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    AndSubstitution,
    LaunchConfiguration,
    NotSubstitution,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
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
            description="true = fake hardware (no robot), false = real robot",
        ),
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="scaled_joint_trajectory_controller",
        ),
        DeclareLaunchArgument(
            "launch_rviz", default_value="true",
            description="false = headless (jog-tuning via the web UI only)",
        ),
        DeclareLaunchArgument(
            "use_lerobot", default_value="true",
            description="launch the LeRobot teach node (lerobot_record.py, "
                        "runs in the isolated `lerobot` pixi env)",
        ),
        DeclareLaunchArgument(
            "lerobot_root", default_value="~/.lerobot/teleocorpus",
            description="abs path to the LeRobotDataset corpus dir",
        ),
        DeclareLaunchArgument(
            "lerobot_task", default_value="teleop",
            description="per-frame `task` string stamped into recorded episodes",
        ),
    ]

    robot_ip           = LaunchConfiguration("robot_ip")
    use_mock_hardware  = LaunchConfiguration("use_mock_hardware")
    initial_joint_ctrl = LaunchConfiguration("initial_joint_controller")
    launch_rviz        = LaunchConfiguration("launch_rviz")
    use_lerobot        = LaunchConfiguration("use_lerobot")
    lerobot_root       = LaunchConfiguration("lerobot_root")
    lerobot_task       = LaunchConfiguration("lerobot_task")

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
            # Vendored launcher: move_group gets TRAC-IK (config/kinematics.yaml).
            FindPackageShare("telamoto_bringup"), "launch", "ur_moveit.launch.py",
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

    # External Control URCap driver: serves the servoj script on request_program
    # and streams packets, both on the reverse socket (50001). Start it by
    # pressing Play on the pendant's External Control program.
    # Runs in BOTH modes: it also serves the :8080 web UI (mobile page + 3D twin
    # + webcam), so the emulated robot has the same UI. In mock mode there's no
    # robot on 50001, so jog frames are accepted but don't actuate the fake arm
    # (planned moves from RViz/MoveIt still move it via scaled_joint_trajectory_
    # controller); MoveIt-mock drives scaled_jtc, so the controller's unused
    # joint_trajectory_controller action server doesn't conflict.
    servo_controller = Node(
        package="telamoto_bringup",
        executable="ur_servo_controller.py",
        name="ur_servo_controller",
        output="screen",
        parameters=[{"robot_ip": robot_ip}],
    )

    ur_moveit_real = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("telamoto_bringup"), "launch", "moveit_real.launch.py",
        ])),
        condition=UnlessCondition(use_mock_hardware),
    )

    # Movable test wall in the planning scene (both modes): drag it in RViz to
    # test collision detection — move_group plans around it, Servo's collision
    # monitor gates the jog near/at it.
    scene_wall = Node(
        package="telamoto_bringup",
        executable="scene_wall.py",
        name="scene_wall",
        output="screen",
    )

    # LeRobot teach node: records teleop demos (see CLAUDE.md §LeRobot teaching).
    # It imports lerobot lazily at the first `start`, so it needs the isolated
    # `lerobot` pixi env python — reach it via a pixi prefix instead of a bare
    # ros2 executable (the default env has no lerobot). Optional: default on
    # in both modes so the web UI teaches the mock or the real robot.
    lerobot_record = Node(
        package="telamoto_bringup",
        executable="lerobot_record.py",
        name="lerobot_record",
        output="screen",
        prefix="pixi run --environment lerobot python",
        parameters=[{"dataset_root": lerobot_root, "task": lerobot_task}],
        condition=IfCondition(use_lerobot),
    )

    # ═══════════════════════════════════════════════════════════════════════
    # RVIZ2 — separate instances per condition (reusing one object breaks launch)
    # ═══════════════════════════════════════════════════════════════════════
    def _rviz(fake: bool, condition=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("telamoto_bringup"), "launch", "moveit_rviz.launch.py",
            ])),
            launch_arguments={"use_fake_hardware": "true" if fake else "false"}.items(),
            condition=condition,
        )

    return LaunchDescription(
        declared_args + [
            # Snapshot the user's RViz choice into a config the sub-includes never
            # touch: ur_control_fake/ur_moveit_fake pass launch_rviz:=false to
            # their child launches, and that value LEAKS back into the parent
            # scope — so by the time the RViz conditions below evaluate
            # `launch_rviz` it has become "false" and RViz silently never starts.
            SetLaunchConfiguration("rviz_enabled", launch_rviz),
            scene_wall,
            # Fake mode
            ur_control_fake,
            # NOT wrapped in a TimerAction: ur_moveit.launch.py self-orders via its
            # wait_for_robot_description node, whose OnProcessExit handler starts
            # move_group AFTER any TimerAction scope would have been popped —
            # TimerAction push/pops launch configurations, so a deferred include
            # dies with "launch configuration 'warehouse_sqlite_path' does not
            # exist" when the handler finally fires.
            ur_moveit_fake,
            # RViz (mock): included DIRECTLY, not via a TimerAction — the timer's
            # deferred include never fired in this launch (no error, just no
            # RViz), while the standalone moveit_rviz path works, so mirror it.
            # RViz starts immediately and waits on move_group/topics on its own.
            _rviz(fake=True,
                  condition=IfCondition(AndSubstitution(
                      LaunchConfiguration("rviz_enabled"), use_mock_hardware))),
            # Real mode
            ur_rsp_real,
            rtde_joint_pub,
            servo_controller,   # opens port 50001, plays ext.urp, replays on disconnect
            lerobot_record,
            TimerAction(period=3.0,  actions=[ur_moveit_real],     condition=UnlessCondition(use_mock_hardware)),
            # 15 s gives move_group time to receive /joint_states and publish
            # a populated planning scene so the goal marker starts at the real pose.
            TimerAction(period=15.0, actions=[_rviz(fake=False)],
                        condition=IfCondition(AndSubstitution(
                            LaunchConfiguration("rviz_enabled"), NotSubstitution(use_mock_hardware)))),
        ]
    )
