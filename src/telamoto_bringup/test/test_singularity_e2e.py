#!/usr/bin/env python3
"""Singularity sentinel E2E: a REAL moveit_servo servo_node (no robot, no
move_group) + fake /joint_states + a live twist feed, watching ~/status.

Verifies the chain the WASD/web jog's singularity protection depends on
(ur_servo_controller gates speedl off these codes: 0→×1, 1/3/4/6→×0.25,
2/5→×0; the web banner names the reason via servoCode):
  A. Ready pose (well-conditioned): linear jogs never HALT, and NO_WARNING
     is the overall majority verdict. Proves the thresholds (20/40 in
     ur_servo.yaml) don't block normal jogging. (Some directions DO show
     intermittent DECELERATE codes here — MoveIt's condition-number metric
     sits near the lower threshold even at this pose; the gate map turns
     that into a brief ×0.25, never a stop.)
  B. Approach band (wrist_2 = -0.08, condition number in Servo's decel
     band): jogging +X (drives wrist_2 toward 0) yields
     DECELERATE_FOR_APPROACHING_SINGULARITY — the jog crawls ×0.25 instead
     of letting J⁻¹ amplify (the original incident).
  C. ESCAPE at the same pose: the REVERSED jog (-X) re-evaluates as
     DECELERATE_FOR_LEAVING_SINGULARITY (never a halt) — Servo is
     direction-aware, so the operator can always back out of a halt the jog
     itself caused (the halt fires at the boundary of this band).
  D. Deep singularity (wrist_2 = -0.01, condition number ≈600 ≫ hard stop):
     HALT_FOR_SINGULARITY in EVERY linear direction — the jog is fully dead.
     This zone is unreachable by jogging (B/C halt you at the boundary
     first); only a planned move can park the arm here, and only a planned
     move gets it out (the web banner says so). Blind crawl is deliberately
     NOT offered here.
  E. Back to ready → recovers (no halt latch).

Status codes are aggregated over a time window, never point-sampled: with a
jittery (non-SCHED_FIFO) test feed servo's verdict stream can interleave
NO_WARNING between codes (the real controller feeds at a steady 50 Hz).

Startup is gated on OBSERVED tracking, not on the command-type service:
DDS delivery of /joint_states into a fresh servo_node on this host can
black out for tens of seconds after launch (seen with BOTH rmw_cyclonedds
and rmw_fastrtps; services and the status stream are unaffected; servo
meanwhile evaluates the default all-zero pose — itself singular — and
reports HALT). The gate feeds the ready pose and waits patiently for
NO_WARNING, which a blacked-out node can never produce. Do NOT "fix" a slow
gate by restarting servo — a restart resets the blackout clock. In
production this failure mode is SAFE: ur_servo_controller hard-blocks the
jog whenever ~/status goes silent for 0.5 s. servo_node is launched as the
binary directly (its own process group, killed with killpg): `ros2 run`
does not reliably forward SIGTERM, and a leaked servo_node poisons every
later run on this domain with a second status publisher.

Isolated: ROS_DOMAIN_ID=75 — safe to run beside a live bringup.
Run:  pixi run python src/telamoto_bringup/test/test_singularity_e2e.py
"""
import os
import signal
import subprocess
import sys
import tempfile
import time

os.environ["ROS_DOMAIN_ID"] = "75"
os.environ.pop("ROS_LOCALHOST_ONLY", None)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402
from geometry_msgs.msg import TwistStamped  # noqa: E402
from moveit_msgs.msg import ServoStatus  # noqa: E402
from moveit_msgs.srv import ServoCommandType  # noqa: E402
from ament_index_python.packages import get_package_share_directory  # noqa: E402

UR_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
             "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
READY_POSE = [0.0, -1.57, 1.57, -1.57, -1.57, 0.0]  # condition number ≈6
BAND_POSE  = [0.0, -1.57, 1.57, -1.57, -0.08, 0.0]  # wrist_2 near 0: decel band
DEEP_POSE  = [0.0, -1.57, 1.57, -1.57, -0.01, 0.0]  # condition number ≈600

APPROACH = (0.2, 0.0, 0.0)    # +X at BAND_POSE drives wrist_2 toward 0
REVERSE  = (-0.2, 0.0, 0.0)
LIN_DIRS = [(0.2, 0, 0), (-0.2, 0, 0), (0, 0.2, 0),
            (0, -0.2, 0), (0, 0, 0.2), (0, 0, -0.2)]

NO_WARNING, DECEL_APPROACHING, HALT_SINGULARITY, DECEL_LEAVING = 0, 1, 2, 3

results = []


def check(name, ok, info=""):
    results.append(ok)
    print(f"{'PASS' if ok else 'FAIL'}  {name:46s} {info}")


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
        kinematics = yaml.safe_load(f)  # servo keeps upstream KDL (sentinel role)
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
    node = Node("singularity_probe")
    js_pub = node.create_publisher(JointState, "/joint_states", 10)
    tw_pub = node.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
    state = {"pose": READY_POSE, "twist": APPROACH, "hist": []}
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
        t.twist.linear.x, t.twist.linear.y, t.twist.linear.z = state["twist"]
        tw_pub.publish(t)
    node.create_timer(0.02, tick)

    servo_bin = os.path.join(
        subprocess.run(["ros2", "pkg", "prefix", "moveit_servo"],
                       capture_output=True, text=True, check=True).stdout.strip(),
        "lib", "moveit_servo", "servo_node")

    def start_servo():
        return subprocess.Popen(
            [servo_bin, "--ros-args", "--params-file", pfile.name],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            start_new_session=True)

    def kill_servo(proc):
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait(5)

    servo = start_servo()

    def spin(sec):
        end = time.monotonic() + sec
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.02)

    def window(pose, twist, sec=2.0, settle=1.5):
        """Code counts in a `sec` window after `settle` seconds of settling."""
        state["pose"], state["twist"] = pose, tuple(float(v) for v in twist)
        spin(settle)
        state["hist"] = []
        spin(sec)
        from collections import Counter
        return Counter(state["hist"])

    def dominant(counts):
        return max(counts, key=counts.get) if counts else None

    def arm(timeout=30.0):
        cli = node.create_client(ServoCommandType, "/servo_node/switch_command_type")
        deadline = time.monotonic() + timeout
        while not cli.service_is_ready() and time.monotonic() < deadline:
            spin(0.2)
        if not cli.service_is_ready():
            return False
        fut = cli.call_async(ServoCommandType.Request(
            command_type=ServoCommandType.Request.TWIST))
        spin(1.0)
        return fut.done() and fut.result().success

    def tracking_live(timeout=120.0):
        """Feed ready pose + twist until NO_WARNING flows: proves servo is
        armed, evaluating, and tracking OUR joint feed (a node with wedged
        /joint_states delivery evaluates the all-zero default pose, which is
        singular, and can never emit NO_WARNING)."""
        state["pose"], state["twist"] = READY_POSE, APPROACH
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            state["hist"] = []
            spin(0.5)
            if NO_WARNING in state["hist"]:
                return True
        return False

    def resync(phase):
        """The delivery blackout can also strike MID-RUN (the feed dies and
        servo silently falls back to a stale state → every later verdict is
        garbage). Re-prove tracking before each phase so a transient blackout
        stretches the runtime instead of corrupting assertions; a permanent
        one aborts with an explicit environmental message."""
        if not tracking_live(timeout=90.0):
            raise RuntimeError(
                f"joint-state delivery to servo died before phase {phase} "
                "(host UDP/DDS issue — check `grep Udp: /proc/net/snmp` "
                "RcvbufErrors; not a servo-logic failure)")

    try:
        t0 = time.monotonic()
        live = arm() and tracking_live()
        check("servo armed and tracking the joint feed", live,
              f"took {time.monotonic() - t0:.0f}s (startup delivery blackout"
              " — see docstring — can take tens of seconds)")
        if not live:
            raise RuntimeError("servo_node never tracked the joint feed")

        # A: ready pose, all 6 linear jog directions: never a halt anywhere,
        # NO_WARNING the overall majority verdict.
        total = {NO_WARNING: 0, DECEL_APPROACHING: 0, DECEL_LEAVING: 0}
        for d in LIN_DIRS:
            resync(f"A {d}")
            c = window(READY_POSE, d)
            for k in total:
                total[k] += c.get(k, 0)
            check(f"A ready pose: no halt for {d}",
                  HALT_SINGULARITY not in c and sum(c.values()) > 0,
                  f"codes={dict(sorted(c.items()))}")
        check("A ready pose: NO_WARNING majority overall",
              total[NO_WARNING] > total[DECEL_APPROACHING] + total[DECEL_LEAVING],
              f"totals={total}")

        # B: decel band, jog toward the wrist singularity → approaching-decel
        # (the controller maps this to a ×0.25 crawl), still no halt.
        resync("B")
        c = window(BAND_POSE, APPROACH)
        check("B band pose +X: DECEL_APPROACHING dominates",
              dominant(c) == DECEL_APPROACHING,
              f"codes={dict(sorted(c.items()))}")

        # C: same pose, REVERSED jog → leaving-decel and NEVER a halt: the
        # escape direction out of a jog-caused singularity halt always passes.
        resync("C")
        c = window(BAND_POSE, REVERSE)
        check("C band pose -X: DECEL_LEAVING dominates",
              dominant(c) == DECEL_LEAVING,
              f"codes={dict(sorted(c.items()))}")
        check("C band pose -X: no halt", HALT_SINGULARITY not in c,
              f"codes={dict(sorted(c.items()))}")

        # D: deep singularity → halt in EVERY linear direction (jog dead;
        # planned-move territory — unreachable by jogging, B/C halt first).
        resync("D")
        for d in LIN_DIRS:
            c = window(DEEP_POSE, d)
            check(f"D deep singularity halts {d}",
                  dominant(c) == HALT_SINGULARITY,
                  f"codes={dict(sorted(c.items()))}")

        # E: back to ready → recovers, no halt latch. (Doubles as the proof
        # that the feed was still alive through D — a dead feed would leave
        # the stale deep-pose verdict pinned at HALT here.)
        c = window(READY_POSE, APPROACH)
        check("E ready again: recovered (no halt latch)",
              HALT_SINGULARITY not in c and NO_WARNING in c,
              f"codes={dict(sorted(c.items()))}")
    except RuntimeError as e:
        print("ABORT:", e)
    finally:
        kill_servo(servo)
        node.destroy_node()
        rclpy.shutdown()
        os.unlink(pfile.name)

    print("RESULT:", "ALL PASS" if all(results) else "FAILURES")
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
