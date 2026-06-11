#!/usr/bin/env python3
"""Self-collision sentinel E2E: a REAL moveit_servo servo_node (no robot, no
move_group) + fake /joint_states + a live twist feed, watching ~/status.

Verifies the chain the web UI's collision warnings depend on:
  A. With self_collision_proximity_threshold = 0.30 (the old shipped default,
     pushed via /servo_node/set_parameters exactly like the web slider does)
     the UR10 is ALWAYS inside the threshold (forearm↔wrist_2 geometry) →
     DECELERATE_FOR_COLLISION at a perfectly safe pose → jog permanently
     gated ×0.25. This is why the default is now 0.01.
  B. Threshold back to 0.01 (shipped default) → no collision codes at the
     safe pose — proving pushed params are APPLIED by the running collision
     monitor (not just stored), in both directions. The boundary is real and
     structural: the forearm↔wrist_2 surfaces stand 1.5–2 cm apart at EVERY
     pose (0.02 → permanent DECEL, 0.015 → clean; measured by sweeping).
  C. A genuinely self-colliding pose (elbow folded, wrist into upper arm)
     → HALT_FOR_COLLISION.
  D. Back to the safe pose → recovers, no collision codes.

Status codes are aggregated over a time window, never point-sampled: with a
jittery (non-SCHED_FIFO) test feed servo's verdict stream can interleave
NO_WARNING between collision codes, which a last-value sample mistakes for
"clear" (the real controller feeds at a steady 50 Hz where this doesn't occur).

Isolated: ROS_DOMAIN_ID=79 — safe to run beside a live bringup.
Run:  pixi run python src/telamoto_bringup/test/test_selfcoll_e2e.py
"""
import os
import subprocess
import sys
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = "79"
os.environ.pop("ROS_LOCALHOST_ONLY", None)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter as RclpyParameter  # noqa: E402
from rcl_interfaces.srv import SetParameters  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from moveit_msgs.msg import ServoStatus  # noqa: E402
from moveit_msgs.srv import ServoCommandType  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402

UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
SAFE_POSE = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]   # classic "ready", wrist clear
FOLD_POSE = [0.0, -1.57, 3.05, 0.0, 0.0, 0.0]       # elbow folded → wrist into upper arm

DECEL_COLLISION, HALT_COLLISION = 4, 5

results = []


def check(name, ok, info=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name:42s} {info}")


def xacro(path, **args):
    cmd = ["xacro", path] + [f"{k}:={v}" for k, v in args.items()]
    return subprocess.run(cmd, capture_output=True, text=True, check=True).stdout


def main():
    ur_desc = get_package_share_directory("ur_description")
    ur_mcfg = get_package_share_directory("ur_moveit_config")
    # Own config from the source tree (the pixi env has no workspace overlay).
    bringup = os.path.normpath(os.path.join(os.path.dirname(__file__), ".."))

    urdf = xacro(os.path.join(ur_desc, "urdf", "ur.urdf.xacro"),
                 ur_type="ur10", name="ur", prefix="", use_fake_hardware="false")
    srdf = xacro(os.path.join(ur_mcfg, "srdf", "ur.srdf.xacro"), name="ur", prefix="")

    import yaml
    with open(os.path.join(bringup, "config", "ur_servo.yaml")) as f:
        servo_cfg = yaml.safe_load(f)
    servo_cfg["is_primary_planning_scene_monitor"] = True  # no move_group here
    with open(os.path.join(ur_mcfg, "config", "kinematics.yaml")) as f:
        kinematics = yaml.safe_load(f)
    with open(os.path.join(ur_mcfg, "config", "joint_limits.yaml")) as f:
        joint_limits = yaml.safe_load(f)

    params = {"servo_node": {"ros__parameters": {
        "moveit_servo": servo_cfg,
        "planning_group_name": "ur_manipulator",
        "update_period": 0.004,
        "robot_description": urdf,
        "robot_description_semantic": srdf,
        "robot_description_kinematics": kinematics,
        "robot_description_planning": joint_limits,
        "use_sim_time": False,
    }}}
    pfile = tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False)
    yaml.safe_dump(params, pfile)
    pfile.flush()

    rclpy.init()
    node = Node("selfcoll_probe")
    js_pub = node.create_publisher(JointState, "/joint_states", 10)
    tw_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    state = {"pose": SAFE_POSE, "hist": []}
    node.create_subscription(
        ServoStatus, "/servo_node/status",
        lambda m: state["hist"].append(m.code), 10)

    def tick():       # 50 Hz: joint states + twist feed (status only flows
        m = JointState()                       # while servo processes commands)
        m.header.stamp = node.get_clock().now().to_msg()
        m.name, m.position = UR_JOINTS, list(state["pose"])
        m.velocity = [0.0] * 6
        js_pub.publish(m)
        t = TwistStamped()
        t.header.stamp = node.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.twist.linear.x = 0.05
        tw_pub.publish(t)
    node.create_timer(0.02, tick)

    servo = subprocess.Popen(
        ["ros2", "run", "moveit_servo", "servo_node",
         "--ros-args", "--params-file", pfile.name],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def window(sec=3.0, settle=2.0):
        """Codes seen in a `sec` window after `settle` seconds of settling."""
        spin(settle)
        state["hist"] = []
        spin(sec)
        return set(state["hist"]), len(state["hist"])

    def push_selfd(value):
        req = SetParameters.Request(parameters=[RclpyParameter(
            "moveit_servo.self_collision_proximity_threshold",
            RclpyParameter.Type.DOUBLE, value).to_parameter_msg()])
        fut = pcli.call_async(req)
        spin(1.0)
        return fut.done() and all(r.successful for r in fut.result().results)

    try:
        # Arm servo in TWIST mode (same as ur_servo_controller._arm_servo).
        cli = node.create_client(ServoCommandType, "/servo_node/switch_command_type")
        deadline = time.monotonic() + 30.0
        while not cli.service_is_ready() and time.monotonic() < deadline:
            spin(0.2)
        check("servo_node up (command-type srv)", cli.service_is_ready())
        fut = cli.call_async(ServoCommandType.Request(
            command_type=ServoCommandType.Request.TWIST))
        spin(1.0)
        check("TWIST mode armed", fut.done() and fut.result().success)
        pcli = node.create_client(SetParameters, "/servo_node/set_parameters")
        spin(0.5)

        # A: the OLD default 0.30 → collision decel at a perfectly safe pose.
        check("A push selfd=0.30 accepted", push_selfd(0.30))
        codes, n = window()
        check("A selfd=0.30: DECEL_COLLISION at safe pose",
              DECEL_COLLISION in codes and HALT_COLLISION not in codes,
              f"codes={sorted(codes)} n={n} (why the default is now 0.01)")

        # B: the shipped default 0.01 → clean. Proves the slider's param push
        # is APPLIED by the collision monitor, in the relaxing direction too.
        check("B push selfd=0.01 accepted", push_selfd(0.01))
        codes, n = window()
        check("B selfd=0.01: clean at safe pose",
              DECEL_COLLISION not in codes and HALT_COLLISION not in codes
              and n > 0,
              f"codes={sorted(codes)} n={n}")

        # C: fold the elbow → wrist cluster into the upper arm → halt.
        state["pose"] = FOLD_POSE
        codes, n = window()
        check("C folded pose: HALT_COLLISION", HALT_COLLISION in codes,
              f"codes={sorted(codes)} n={n}")

        # D: back out → recovers clean.
        state["pose"] = SAFE_POSE
        codes, n = window()
        check("D safe pose again: clean",
              DECEL_COLLISION not in codes and HALT_COLLISION not in codes
              and n > 0,
              f"codes={sorted(codes)} n={n}")
    finally:
        servo.terminate()
        try:
            servo.wait(5)
        except subprocess.TimeoutExpired:
            servo.kill()
        node.destroy_node()
        rclpy.shutdown()
        os.unlink(pfile.name)

    print("RESULT:", "ALL PASS" if all(results) else "FAILURES")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
