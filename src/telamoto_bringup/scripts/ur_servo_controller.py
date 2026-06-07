#!/usr/bin/env python3
"""
FollowJointTrajectory action server driving a UR CB3 through the
**External Control URCap** at 125 Hz.

Why this design
===============
ur_robot_driver cannot run on this CB3: PolyScope owns every RTDE *input*
register (speed_slider, digital/analog outputs, ...), so the driver's
hardware interface aborts at configure with "Variable '...' is currently
controlled by another RTDE client", and an empty recipe is rejected too.
This controller never touches RTDE inputs — it talks to the robot only over
the External Control URCap's script channel and a reverse TCP socket.

How the External Control URCap works (FZI externalcontrol-1.0.5)
===============================================================
When the External Control program node runs, the URCap:
  1. connects to the configured Host IP : Custom Port (default 50002),
  2. sends the literal request  "request_program",
  3. reads a URScript back until the socket closes,
  4. splits it on the "# HEADER_BEGIN" / "# HEADER_END" anchors and runs it.

So this node:
  1. SCRIPT_SENDER server on port 50002 — answers request_program with a
     servoj URScript that connects back to us on REVERSE_PORT.
  2. REVERSE server on port 50001 — the running URScript connects here; we
     stream 8×int32 packets at 125 Hz and the robot calls servoj().
  3. Dashboard (29999) auto-loads + plays ext.urp so the URCap fires without
     anyone touching the pendant (you can also just press Play yourself).

Packet format — 8 × int32 big-endian
  params[0]  count          always 8 (prepended by the URScript runtime)
  params[1]  timeout_ms     robot sets read_timeout = this / 1000.0
  params[2..7]  q[0..5] × MULT
  params[8]  control_mode   1 == MODE_SERVOJ
"""
import socket
import struct
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState

UR_JOINT_ORDER = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

REVERSE_PORT       = 50001   # robot's running URScript connects back here
SCRIPT_SENDER_PORT = 50002   # External Control URCap requests the script here
DASHBOARD_PORT     = 29999
STEP_TIME          = 0.010   # 100 Hz send — deliberately a touch SLOWER than
                             # the robot's servoj consume rate so packets never
                             # back up in the TCP buffer (which caused growing
                             # lag and the robot "stopping" until re-played).
MULT               = 1_000_000
MODE_SERVOJ        = 1
TIMEOUT_MS         = 200      # robot drops to a missed-read if no packet for this long

# Control URScript served to the URCap on request_program.  The "# HEADER_*"
# anchors are mandatory: the URCap splits global definitions (header) from the
# body it injects at the program node.  We keep the header empty and make the
# body fully self-contained.  A single missed read is NOT treated as a
# disconnect (the Python sender can stall briefly under GIL contention) — the
# body only exits after a long, real silence (~10 s at 0.2 s read timeout).
_URSCRIPT = """\
# HEADER_BEGIN
# telamoto external control — no global definitions required
# HEADER_END
steptime = get_steptime()
textmsg("telamoto: external control active, steptime=", steptime)
socket_open("{host}", {port}, "reverse_socket")
read_timeout = 0
ctrl = -1
misses = 0
keep_going = True
while keep_going:
  p = socket_read_binary_integer(8, "reverse_socket", read_timeout)
  if p[0] > 0:
    misses = 0
    read_timeout = p[1] / 1000.0
    if p[8] == 1:
      q = [p[2]/1000000.0, p[3]/1000000.0, p[4]/1000000.0, p[5]/1000000.0, p[6]/1000000.0, p[7]/1000000.0]
      servoj(q, t={servoj_time}, lookahead_time={lookahead}, gain={gain})
    end
  else:
    misses = misses + 1
    if misses > 50:
      keep_going = False
    end
  end
end
socket_close("reverse_socket")
textmsg("telamoto: external control stopped")
"""


def _pack(q: list[float]) -> bytes:
    return struct.pack(">8i",
        TIMEOUT_MS,
        int(q[0] * MULT), int(q[1] * MULT), int(q[2] * MULT),
        int(q[3] * MULT), int(q[4] * MULT), int(q[5] * MULT),
        MODE_SERVOJ,
    )


class URServoController(Node):

    def __init__(self) -> None:
        super().__init__("ur_servo_controller")
        self.declare_parameter("robot_ip",     "192.168.10.2")
        self.declare_parameter("reverse_port", REVERSE_PORT)
        self.declare_parameter("script_sender_port", SCRIPT_SENDER_PORT)
        self.declare_parameter("ext_program",  "ext.urp")
        # Dashboard auto-play runs the *last compiled* program, which uses the
        # URCap's cached script. To pick up a freshly-served script you must
        # recompile on the pendant (press Play). Set auto_play:=false to drive
        # the standard URCap workflow (press Play yourself).
        self.declare_parameter("auto_play", True)
        # servoj tuning (baked into the served script). t > steptime smooths over
        # sender jitter; higher lookahead = smoother/laggier; gain = stiffness.
        # servoj blocks for `t`, so the robot consumes one packet per `t`. It
        # MUST equal the 125 Hz send interval (8 ms) — larger backs up the TCP
        # buffer and the robot trails ever further behind (lag); smaller makes
        # it wait between packets. Keep it at the step time.
        self.declare_parameter("servoj_time",      0.008)
        self.declare_parameter("servoj_lookahead", 0.1)
        self.declare_parameter("servoj_gain",      300)

        self._robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self._port     = self.get_parameter("reverse_port").get_parameter_value().integer_value
        self._sender_port = self.get_parameter("script_sender_port").get_parameter_value().integer_value
        self._ext_prog = self.get_parameter("ext_program").get_parameter_value().string_value
        self._auto_play = self.get_parameter("auto_play").get_parameter_value().bool_value
        self._servoj_time      = self.get_parameter("servoj_time").get_parameter_value().double_value
        self._servoj_lookahead = self.get_parameter("servoj_lookahead").get_parameter_value().double_value
        self._servoj_gain      = self.get_parameter("servoj_gain").get_parameter_value().integer_value

        # Current robot joint positions (from RTDE outputs, read-only).
        self._q_current: list[float] = [0.0] * 6
        self._q_lock    = threading.Lock()

        # Gate: set when first JointState arrives; _control_loop waits on this.
        self._js_ready  = threading.Event()

        # 125 Hz command target — held position when no trajectory is active.
        self._q_target: list[float] = [0.0] * 6
        self._tgt_lock  = threading.Lock()

        # Active trajectory: (waypoints, total_time, t_start) or None. The single
        # control loop owns timing — it interpolates AND sends in one place, so
        # there are no two competing 125 Hz Python loops to jitter against.
        self._traj = None
        self._traj_lock = threading.Lock()
        self._traj_done = threading.Event()

        # Active robot connection on the reverse socket — None when disconnected.
        self._conn: socket.socket | None = None
        self._conn_lock = threading.Lock()

        # Single-flight guard so overlapping disconnects don't stack dashboard
        # replays on top of each other.
        self._recovery_lock = threading.Lock()

        cb = ReentrantCallbackGroup()
        self._js_sub = self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
            execute_callback = self._execute_cb,
            goal_callback    = lambda _: GoalResponse.ACCEPT,
            cancel_callback  = lambda _: CancelResponse.ACCEPT,
            callback_group   = cb,
        )

        threading.Thread(target=self._script_sender_loop, daemon=True, name="ri-script").start()
        threading.Thread(target=self._reverse_server_loop, daemon=True, name="ri-server").start()
        threading.Thread(target=self._control_loop,        daemon=True, name="ri-ctrl").start()

        self.get_logger().info(
            f"URServoController: script sender :{self._sender_port}, reverse :{self._port}, "
            f"robot at {self._robot_ip}"
        )

    # ── Joint states ──────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        # We only need the robot's start pose to seed the hold target. Control
        # is open-loop servoj streaming, so after the first sample we drop this
        # subscription — running it at 125 Hz starves the GIL and makes the
        # control loop stutter. (MoveIt keeps its own /joint_states sub.)
        ntp = dict(zip(msg.name, msg.position))
        try:
            q = [ntp[j] for j in UR_JOINT_ORDER]
        except KeyError:
            return
        with self._tgt_lock:
            self._q_target = list(q)
        with self._q_lock:
            self._q_current = q
        if not self._js_ready.is_set():
            self._js_ready.set()
            self.get_logger().info("[ctrl] first RTDE joint states received — control loop active")
            self.destroy_subscription(self._js_sub)

    # ── Helpers ────────────────────────────────────────────────────────────────

    @staticmethod
    def _recv_line(s: socket.socket) -> str:
        buf = bytearray()
        while True:
            ch = s.recv(1)
            if not ch:
                break
            buf += ch
            if ch == b"\n":
                break
        return buf.decode("utf-8", errors="replace").strip()

    def _dashboard_cmd(self, s: socket.socket, cmd: str) -> str:
        s.sendall((cmd + "\n").encode("utf-8"))
        try:
            return self._recv_line(s)
        except socket.timeout:
            return ""

    def _pc_ip(self) -> str:
        """Return the local IP that routes to the robot."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect((self._robot_ip, DASHBOARD_PORT))
            return s.getsockname()[0]

    # ── Script sender (External Control URCap protocol) ────────────────────────

    def _script_sender_loop(self) -> None:
        """Answer the URCap's request_program with the servoj control script."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self._sender_port))
        srv.listen(1)
        self.get_logger().info(f"[script] listening on :{self._sender_port}")

        # Now that the sender is up, kick off auto-play of ext.urp (unless the
        # operator wants to press Play on the pendant — the URCap-standard flow,
        # which also guarantees a fresh script request).
        if self._auto_play:
            threading.Thread(target=self._play_ext_program, daemon=True).start()
        else:
            self.get_logger().info("[script] auto_play disabled — press Play on the pendant")

        while rclpy.ok():
            try:
                conn, addr = srv.accept()
                self.get_logger().info(f"[script] CONNECTION on :{self._sender_port} from {addr[0]}")
                with conn:
                    conn.settimeout(5)
                    try:
                        req = self._recv_line(conn)   # expected: "request_program"
                    except Exception:
                        req = ""
                    self._serve_script(conn, addr, req)
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().warn(f"[script] error: {exc}")

    def _serve_script(self, conn: socket.socket, addr, req: str = "") -> None:
        host   = self._pc_ip()
        script = _URSCRIPT.format(
            host=host, port=self._port,
            servoj_time=self._servoj_time,
            lookahead=self._servoj_lookahead,
            gain=self._servoj_gain,
        )
        conn.sendall(script.encode("utf-8"))
        self.get_logger().info(
            f"[script] served control script to {addr[0]} "
            f"(request={req!r}, connect-back {host}:{self._port})"
        )

    # ── Dashboard auto-play ────────────────────────────────────────────────────

    def _play_ext_program(self) -> None:
        """Load + play ext.urp via the Dashboard Server so the External Control
        URCap fires and requests our script.  Retries on transient failures.
        Single-flighted so concurrent disconnects don't stack replays."""
        if not self._recovery_lock.acquire(blocking=False):
            return  # a play/replay is already in progress

        try:
            ip, prog = self._robot_ip, self._ext_prog

            self.get_logger().info("[auto] waiting for RTDE joint states ...")
            if not self._js_ready.wait(timeout=30.0):
                self.get_logger().error(
                    "[auto] no joint states after 30 s — is ur_rtde_joint_pub running?"
                )
                return

            for attempt in range(20):
                if not rclpy.ok():
                    return
                with self._conn_lock:
                    if self._conn is not None:
                        return  # robot already controlling — nothing to do
                dash = None
                try:
                    dash = socket.create_connection((ip, DASHBOARD_PORT), timeout=10)
                    dash.settimeout(5)
                    self._recv_line(dash)  # consume welcome banner

                    mode = self._dashboard_cmd(dash, "robotmode").upper()
                    self.get_logger().info(f"[auto] attempt {attempt+1}: robotmode={mode}")
                    if "RUNNING" not in mode:
                        dash.close(); dash = None
                        self.get_logger().warn("[auto] robot not RUNNING, waiting 3 s ...")
                        time.sleep(3.0)
                        continue

                    self._dashboard_cmd(dash, "stop")
                    time.sleep(1.0)
                    load_r = self._dashboard_cmd(dash, f"load {prog}")
                    self.get_logger().info(f"[auto] load -> {load_r}")
                    if "error" in load_r.lower() or "not found" in load_r.lower():
                        self.get_logger().error(
                            f"[auto] could not load {prog}: {load_r} — check it exists on the robot"
                        )
                        dash.close(); return
                    play_r = self._dashboard_cmd(dash, "play")
                    self.get_logger().info(f"[auto] play -> {play_r}")
                    dash.close(); dash = None

                    if "fail" in play_r.lower() or "error" in play_r.lower():
                        self.get_logger().warn(f"[auto] play not ready ({play_r}), retry in 2 s")
                        time.sleep(2.0)
                        continue

                    # Wait for the robot's URScript to connect back.
                    deadline = time.monotonic() + 20.0
                    while time.monotonic() < deadline and rclpy.ok():
                        with self._conn_lock:
                            if self._conn is not None:
                                self.get_logger().info("[auto] robot connected — 125 Hz servoj active")
                                return
                        time.sleep(0.1)

                    self.get_logger().warn("[auto] robot did not connect within 20 s — retrying ...")

                except Exception as exc:
                    self.get_logger().warn(f"[auto] attempt {attempt+1}/20: {exc} — retry in 2 s")
                    if dash is not None:
                        try: dash.close()
                        except Exception: pass
                    time.sleep(2.0)

            self.get_logger().error("[auto] gave up after 20 attempts")
        finally:
            self._recovery_lock.release()

    # ── Reverse server (robot's URScript connects here) ────────────────────────

    def _reverse_server_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self._port))
        srv.listen(1)
        self.get_logger().info(f"[server] listening on :{self._port}")

        while rclpy.ok():
            try:
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

                # The External Control URCap may issue request_program on THIS
                # port (when its Custom Port == the reverse port).  Peek the
                # first bytes: a script request is text starting with
                # "request_program"; the robot's reverse connection sends
                # nothing first (it waits to read), so the peek times out.
                is_request = False
                try:
                    conn.settimeout(2.0)
                    first = conn.recv(32, socket.MSG_PEEK)
                    is_request = first.startswith(b"request_program")
                except Exception:
                    pass

                if is_request:
                    self.get_logger().info(
                        f"[script] request_program on reverse port :{self._port} from {addr[0]}"
                    )
                    try:
                        with conn:
                            conn.settimeout(5)
                            req = self._recv_line(conn)
                            self._serve_script(conn, addr, req)
                    except Exception as exc:
                        self.get_logger().warn(f"[script] serve on reverse port failed: {exc}")
                    continue

                conn.settimeout(None)
                self.get_logger().info(f"[server] robot connected from {addr[0]}")
                with self._conn_lock:
                    old, self._conn = self._conn, conn
                if old:
                    try: old.close()
                    except Exception: pass
                threading.Thread(
                    target=self._drain_then_replay, args=(conn,), daemon=True,
                ).start()
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().warn(f"[server] accept error: {exc}")

    def _drain_then_replay(self, conn: socket.socket) -> None:
        """Block until the robot closes the reverse socket, then re-play ext.urp.

        Unlike the URCap-less variant, the robot does NOT auto-reconnect (the
        program ends when the URScript exits), so we replay via the dashboard.
        _play_ext_program is single-flighted, so a flurry of disconnects can't
        stack replays."""
        try:
            conn.settimeout(None)
            while conn.recv(256):
                pass
        except Exception:
            pass
        with self._conn_lock:
            if self._conn is conn:
                self._conn = None
        if self._auto_play:
            self.get_logger().warn("[server] robot disconnected — replaying ext.urp")
            time.sleep(1.0)
            if rclpy.ok():
                self._play_ext_program()
        else:
            self.get_logger().warn("[server] robot disconnected — press Play on the pendant to resume")

    # ── 125 Hz control loop (single timing authority) ──────────────────────────

    @staticmethod
    def _interp(waypoints, t_now: float) -> list[float]:
        """Cubic Hermite interpolation using each waypoint's position AND
        velocity → continuous velocity (C1), no per-waypoint jerk.  Falls back
        to linear if MoveIt didn't supply velocities."""
        seg = len(waypoints) - 2
        for i in range(len(waypoints) - 1):
            if t_now < waypoints[i + 1][0]:
                seg = i
                break
        t0, q0, v0 = waypoints[seg]
        t1, q1, v1 = waypoints[seg + 1]
        h = t1 - t0
        if h <= 0:
            return list(q1)
        s = max(0.0, min(1.0, (t_now - t0) / h))
        if v0 is None or v1 is None:
            return [q0[j] + s * (q1[j] - q0[j]) for j in range(6)]
        s2, s3 = s * s, s * s * s
        h00 =  2 * s3 - 3 * s2 + 1
        h10 =      s3 - 2 * s2 + s
        h01 = -2 * s3 + 3 * s2
        h11 =      s3 -     s2
        return [h00 * q0[j] + h10 * h * v0[j] + h01 * q1[j] + h11 * h * v1[j]
                for j in range(6)]

    def _control_loop(self) -> None:
        self._js_ready.wait()
        sent_count = 0
        next_t = time.monotonic()
        # Loop-health diagnostics: how steady is our 125 Hz?
        last_tick = time.monotonic()
        ticks = 0
        overruns = 0
        max_gap = 0.0
        health_t = time.monotonic()
        while rclpy.ok():
            _now = time.monotonic()
            _gap = _now - last_tick
            last_tick = _now
            ticks += 1
            if _gap > max_gap:
                max_gap = _gap
            if _gap > STEP_TIME * 1.5:   # tick arrived >50% late
                overruns += 1
            if _now - health_t >= 5.0:
                self.get_logger().info(
                    f"[ctrl] rate health: {ticks} ticks/5s, {overruns} late "
                    f"(>{STEP_TIME*1.5*1000:.0f}ms), max gap {max_gap*1000:.1f}ms"
                )
                ticks = 0; overruns = 0; max_gap = 0.0; health_t = _now
            # Compute the command for this tick: interpolate the active
            # trajectory here (not in a second loop) so timing is uniform.
            with self._traj_lock:
                traj = self._traj
            if traj is not None:
                waypoints, total_time, t_start = traj
                t_now = time.monotonic() - t_start
                if t_now >= total_time:
                    q = list(waypoints[-1][1])
                    with self._tgt_lock:
                        self._q_target = q
                    with self._traj_lock:
                        if self._traj is traj:
                            self._traj = None
                    self._traj_done.set()
                else:
                    q = self._interp(waypoints, t_now)
                    with self._tgt_lock:
                        self._q_target = q
            else:
                with self._tgt_lock:
                    q = list(self._q_target)

            with self._conn_lock:
                conn = self._conn
            if conn is not None:
                try:
                    conn.sendall(_pack(q))
                    sent_count += 1
                    if sent_count == 1:
                        self.get_logger().info("[ctrl] first packet sent to robot")
                except Exception:
                    with self._conn_lock:
                        if self._conn is conn:
                            self._conn = None
                    self.get_logger().warn("[ctrl] send failed — connection lost")
                    sent_count = 0

            next_t += STEP_TIME
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()

    # ── FollowJointTrajectory action ──────────────────────────────────────────

    @staticmethod
    def _reorder(names: list[str], pos: list[float]) -> list[float]:
        try:
            return [pos[names.index(j)] for j in UR_JOINT_ORDER]
        except (ValueError, IndexError) as e:
            raise ValueError(
                f"Trajectory joint names {names} do not include all UR joints: {e}"
            ) from e

    def _execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory
        if not traj.points:
            goal_handle.abort()
            r = FollowJointTrajectory.Result()
            r.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return r

        try:
            jnames    = list(traj.joint_names)
            waypoints = [
                (
                    pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9,
                    self._reorder(jnames, list(pt.positions)),
                    self._reorder(jnames, list(pt.velocities)) if pt.velocities else None,
                )
                for pt in traj.points
            ]
        except ValueError as e:
            self.get_logger().error(f"[exec] invalid trajectory: {e}")
            goal_handle.abort()
            r = FollowJointTrajectory.Result()
            r.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            return r

        total_time = waypoints[-1][0]

        with self._conn_lock:
            connected = self._conn is not None
        self.get_logger().info(
            f"[exec] {len(waypoints)}-pt traj, {total_time:.2f} s, robot connected={connected}"
        )
        if not connected:
            self.get_logger().warn(
                "[exec] robot not connected — target queued; play ext.urp / External Control"
            )

        # Hand the trajectory to the control loop, which interpolates + streams
        # it at a uniform 125 Hz, and wait for it to finish (or cancel).
        self._traj_done.clear()
        with self._traj_lock:
            self._traj = (waypoints, total_time, time.monotonic())

        while not self._traj_done.wait(timeout=0.05):
            if goal_handle.is_cancel_requested:
                with self._traj_lock:
                    self._traj = None
                goal_handle.canceled()
                self.get_logger().info("[exec] trajectory canceled")
                return FollowJointTrajectory.Result()
            if not rclpy.ok():
                return FollowJointTrajectory.Result()

        goal_handle.succeed()
        r = FollowJointTrajectory.Result()
        r.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info("[exec] trajectory complete")
        return r


def main() -> None:
    rclpy.init()
    node = URServoController()
    # 2 threads: one to run the (blocking) action execute_cb, one to service
    # goal cancellation concurrently. Fewer threads = less GIL contention with
    # the 125 Hz control-loop thread. The default spawns one per CPU, which
    # starves the sender and makes motion stutter.
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
