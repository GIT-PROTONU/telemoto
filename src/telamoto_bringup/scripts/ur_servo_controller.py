#!/usr/bin/env python3
"""
FollowJointTrajectory action server using the UR CB3 reverse-interface
protocol at 125 Hz.

Architecture
============
1. A TCP server listens on port 50001 (REVERSE_PORT).
2. _play_ext_program() auto-detects the PC's IP, then uploads a compact
   URScript to the robot via port 30001 (UR primary interface).  The
   URScript connects back to port 50001 and calls servoj() at 125 Hz.
3. _control_loop() sends 8-int32 packets at 125 Hz on the live connection.
4. _execute_cb() is the FollowJointTrajectory action handler; it updates
   _q_target at 125 Hz while _control_loop sends those positions to the
   robot.  The robot physically follows the trajectory via servoj.
5. On disconnect _drain_then_replay() re-uploads the URScript after 3 s.

Packet format — 8 × int32 big-endian
  (verified against external_control.urscript from URCap 1.0.5 for CB3)
  params_mult = socket_read_binary_integer(8, "rs", ...)
  params_mult[0]  count          always 8 (prepended by URScript runtime)
  params_mult[1]  timeout_ms     robot sets read_timeout = this / 1000.0
  params_mult[2]  q[0] × MULT
  params_mult[3]  q[1] × MULT
  params_mult[4]  q[2] × MULT
  params_mult[5]  q[3] × MULT
  params_mult[6]  q[4] × MULT
  params_mult[7]  q[5] × MULT
  params_mult[8]  control_mode   MUST be the 8th (last) data integer

  timeout_ms first, control_mode last — any other layout causes the robot
  to read mode from the wrong field and servoj is never called.
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

REVERSE_PORT = 50001
PRIMARY_PORT = 30001   # UR primary interface — accepts URScript text
STEP_TIME    = 0.008   # 125 Hz
MULT         = 1_000_000
MODE_SERVOJ  = 1
TIMEOUT_MS   = 200     # ms  — robot stops servoj if no packet for this long

# read_timeout = 0 in URScript means BLOCK UNTIL DATA ARRIVES (CB3 docs).
# The robot blocks on the first read until our control loop sends the first
# packet (within 8 ms of connection).  After that, read_timeout is driven by
# TIMEOUT_MS from the packet header.
_URSCRIPT = """\
def extControl():
  MULT = 1000000
  while True:
    socket_close("rs")
    if socket_open("{host}", {port}, "rs"):
      read_timeout = 0
      ctrl = -1
      misses = 0
      keep_going = True
      while keep_going:
        p = socket_read_binary_integer(8, "rs", read_timeout)
        if p[0] > 0:
          misses = 0
          read_timeout = p[1] / 1000.0
          if ctrl != p[8]:
            ctrl = p[8]
          end
          if ctrl == 1:
            q = [p[2]/MULT, p[3]/MULT, p[4]/MULT, p[5]/MULT, p[6]/MULT, p[7]/MULT]
            servoj(q, t=get_steptime(), lookahead_time=0.1, gain=300)
          end
        else:
          # A single missed read is NOT a disconnect — the Python control
          # loop can stall briefly under GIL contention.  Only tear the
          # socket down after a long, real silence (~10 s at 0.2 s timeout).
          misses = misses + 1
          if misses > 50:
            keep_going = False
          end
        end
      end
    end
    sleep(0.5)
  end
end
extControl()
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

        self._robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value
        self._port     = self.get_parameter("reverse_port").get_parameter_value().integer_value

        # Current robot joint positions (from RTDE).
        self._q_current: list[float] = [0.0] * 6
        self._q_lock    = threading.Lock()

        # Gate: set when first JointState arrives; _control_loop waits on this.
        self._js_ready  = threading.Event()

        # 125 Hz command target — written by _execute_cb, read by _control_loop.
        self._q_target: list[float] = [0.0] * 6
        self._tgt_lock  = threading.Lock()

        # Active robot connection — None when disconnected.
        self._conn: socket.socket | None = None
        self._conn_lock = threading.Lock()

        # Single-flight guard: only one heavy re-upload (dashboard stop +
        # primary-interface upload) may run at a time.  Without this, every
        # disconnect spawns a re-upload whose own `dashboard stop` causes
        # another disconnect — a self-amplifying recovery storm.
        self._recovery_lock = threading.Lock()

        cb = ReentrantCallbackGroup()
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
            execute_callback = self._execute_cb,
            goal_callback    = lambda _: GoalResponse.ACCEPT,
            cancel_callback  = lambda _: CancelResponse.ACCEPT,
            callback_group   = cb,
        )

        threading.Thread(target=self._server_loop,  daemon=True, name="ri-server").start()
        threading.Thread(target=self._control_loop, daemon=True, name="ri-ctrl").start()

        self.get_logger().info(
            f"URServoController: reverse interface on :{self._port}, robot at {self._robot_ip}"
        )

    # ── Joint states ──────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        ntp = dict(zip(msg.name, msg.position))
        try:
            q = [ntp[j] for j in UR_JOINT_ORDER]
        except KeyError:
            return
        with self._q_lock:
            self._q_current = q
        if not self._js_ready.is_set():
            with self._tgt_lock:
                self._q_target = list(q)
            self._js_ready.set()
            self.get_logger().info("[ctrl] first RTDE joint states received — control loop active")

    # ── Dashboard helpers ─────────────────────────────────────────────────────

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

    # ── PC IP auto-detect ─────────────────────────────────────────────────────

    def _pc_ip(self) -> str:
        """Return the local IP that routes to the robot."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect((self._robot_ip, 29999))
            return s.getsockname()[0]

    # ── Auto-play ─────────────────────────────────────────────────────────────

    def _play_ext_program(self) -> None:
        """Upload URScript to the robot via the primary interface and verify
        that the robot connects back to our reverse-interface server.

        Sequence per attempt:
          1. Wait for RTDE joint states (control loop must be running before
             the robot connects — avoids a blocking-read stall on first packet).
          2. Check robotmode via Dashboard Server; wait if not RUNNING.
          3. dashboard stop → 2 s pause → upload URScript via port 30001.
             The URScript contains the auto-detected PC IP so it always
             reaches our server regardless of the URCap configuration.
          4. Wait up to 30 s for the robot to connect on port 50001.
             Success is confirmed by self._conn becoming non-None.
          5. Retry up to 20 times on any failure.
        """
        ip   = self._robot_ip
        host = self._pc_ip()
        port = self._port
        script = _URSCRIPT.format(host=host, port=port)

        self.get_logger().info(f"[auto] waiting for RTDE joint states ...")
        if not self._js_ready.wait(timeout=30.0):
            self.get_logger().error(
                "[auto] no joint states after 30 s — is ur_rtde_joint_pub running?"
            )
            return

        self.get_logger().info(
            f"[auto] PC IP detected as {host} — uploading URScript to {ip}"
        )

        for attempt in range(20):
            if not rclpy.ok():
                return
            dash = None
            try:
                dash = socket.create_connection((ip, 29999), timeout=10)
                dash.settimeout(5)
                self._recv_line(dash)  # consume welcome banner

                mode = self._dashboard_cmd(dash, "robotmode").upper()
                self.get_logger().info(f"[auto] attempt {attempt+1}: robotmode={mode}")
                if "RUNNING" not in mode:
                    dash.close(); dash = None
                    self.get_logger().warn("[auto] robot not RUNNING, waiting 3 s ...")
                    time.sleep(3.0)
                    continue

                # Stop whatever is running and give PolyScope time to settle.
                # 2 s is intentional — 0.5 s caused URScript to be rejected.
                self._dashboard_cmd(dash, "stop")
                dash.close(); dash = None
                time.sleep(2.0)

                # Upload URScript via primary interface (port 30001).
                pri = socket.create_connection((ip, PRIMARY_PORT), timeout=10)
                pri.sendall(script.encode("utf-8"))
                pri.close()
                self.get_logger().info(
                    f"[auto] URScript sent (connect-back {host}:{port}) — "
                    f"waiting up to 30 s for robot to connect ..."
                )

                # Wait for the robot to call socket_open() and connect back.
                deadline = time.monotonic() + 30.0
                while time.monotonic() < deadline and rclpy.ok():
                    with self._conn_lock:
                        connected = self._conn is not None
                    if connected:
                        self.get_logger().info(
                            "[auto] robot connected — 125 Hz servoj active"
                        )
                        return
                    time.sleep(0.1)

                self.get_logger().warn(
                    "[auto] robot did not connect within 30 s — retrying ..."
                )

            except Exception as exc:
                self.get_logger().warn(
                    f"[auto] attempt {attempt+1}/20: {exc} — retry in 2 s"
                )
                if dash is not None:
                    try: dash.close()
                    except Exception: pass
                time.sleep(2.0)

        self.get_logger().error("[auto] gave up after 20 attempts")

    # ── TCP server ────────────────────────────────────────────────────────────

    def _server_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self._port))
        srv.listen(1)
        self.get_logger().info(f"[server] listening on :{self._port}")

        threading.Thread(target=self._play_ext_program, daemon=True).start()

        while rclpy.ok():
            try:
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.get_logger().info(f"[server] robot connected from {addr[0]}")
                with self._conn_lock:
                    old, self._conn = self._conn, conn
                if old:
                    try: old.close()
                    except Exception: pass
                threading.Thread(
                    target=self._drain_then_replay,
                    args=(conn,), daemon=True,
                ).start()
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().warn(f"[server] accept error: {exc}")

    def _drain_then_replay(self, conn: socket.socket) -> None:
        """Block until the robot closes the connection, then recover.

        Recovery is delegated to the robot's own URScript outer loop, which
        re-opens the reverse socket every 0.5 s.  We only fall back to a heavy
        re-upload (dashboard stop + primary-interface upload) if the robot does
        NOT come back on its own within the grace period — and even then only
        one re-upload runs at a time (self._recovery_lock).  This is what stops
        the connect/stop/reconnect oscillation that chopped motion into ~3 s
        steps.
        """
        try:
            conn.settimeout(None)
            while conn.recv(256):
                pass
        except Exception:
            pass
        with self._conn_lock:
            if self._conn is conn:
                self._conn = None
        self.get_logger().warn("[server] robot disconnected — awaiting auto-reconnect")
        threading.Thread(target=self._watchdog, daemon=True).start()

    def _watchdog(self) -> None:
        """Wait for the robot's URScript to reconnect; re-upload only if it
        fails to within the grace period."""
        deadline = time.monotonic() + 10.0
        while time.monotonic() < deadline and rclpy.ok():
            with self._conn_lock:
                if self._conn is not None:
                    return  # robot came back on its own — nothing to do
            time.sleep(0.2)

        if not self._recovery_lock.acquire(blocking=False):
            return  # a re-upload is already in progress
        try:
            if rclpy.ok():
                self.get_logger().warn(
                    "[server] no auto-reconnect after 10 s — re-uploading URScript"
                )
                self._play_ext_program()
        finally:
            self._recovery_lock.release()

    # ── 125 Hz control loop ───────────────────────────────────────────────────

    def _control_loop(self) -> None:
        self._js_ready.wait()
        sent_count = 0
        next_t = time.monotonic()
        while rclpy.ok():
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
            f"[exec] {len(waypoints)}-pt traj, {total_time:.2f} s, "
            f"robot connected={connected}"
        )

        if not connected:
            self.get_logger().warn(
                "[exec] robot not connected — trajectory will be queued in _q_target "
                "but robot may not move until URScript reconnects"
            )

        t_start   = time.monotonic()
        next_tick = t_start

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return FollowJointTrajectory.Result()

            t_now = time.monotonic() - t_start
            if t_now >= total_time:
                with self._tgt_lock:
                    self._q_target = list(waypoints[-1][1])
                break

            # Find which segment we're in.
            seg = len(waypoints) - 2
            for i in range(len(waypoints) - 1):
                if t_now < waypoints[i + 1][0]:
                    seg = i
                    break

            t0, q0 = waypoints[seg]
            t1, q1 = waypoints[seg + 1]
            alpha  = (t_now - t0) / (t1 - t0) if t1 > t0 else 1.0
            alpha  = max(0.0, min(1.0, alpha))
            q_interp = [q0[j] + alpha * (q1[j] - q0[j]) for j in range(6)]

            with self._tgt_lock:
                self._q_target = q_interp

            next_tick += STEP_TIME
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)

        goal_handle.succeed()
        r = FollowJointTrajectory.Result()
        r.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info("[exec] trajectory complete")
        return r


def main() -> None:
    rclpy.init()
    node = URServoController()
    executor = MultiThreadedExecutor()
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
