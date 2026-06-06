#!/usr/bin/env python3
"""
FollowJointTrajectory action server using the External Control URCap
reverse-interface protocol at 125 Hz.

Start-up sequence (handled internally, no external auto-play node needed):
  1. TCP server opens on port 50001.
  2. Dashboard Server (port 29999) is asked to load + play ext.urp.
  3. External Control URCap on the robot connects back to port 50001.
  4. 125 Hz control loop sends MODE_SERVOJ packets — robot holds position.
  5. If the robot disconnects for any reason, ext.urp is replayed after 3 s.

Packet format (ur_client_library/control/reverse_interface.h):
  8 × int32 big-endian:
    [0]   control_mode  (MODE_SERVOJ = 1)
    [1-6] joint positions × MULT_JOINTSTATE (1 000 000)
    [7]   robot receive timeout in ms  (20 ms = 2.5 × step)
"""
import asyncio
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
STEP_TIME    = 0.008       # 125 Hz
MULT         = 1_000_000
MODE_SERVOJ  = 1
TIMEOUT_MS   = 20          # robot aborts if silent for 20 ms


def _pack(q: list[float]) -> bytes:
    return struct.pack(">8i",
        MODE_SERVOJ,
        int(q[0] * MULT), int(q[1] * MULT), int(q[2] * MULT),
        int(q[3] * MULT), int(q[4] * MULT), int(q[5] * MULT),
        TIMEOUT_MS,
    )


class URServoController(Node):

    def __init__(self) -> None:
        super().__init__("ur_servo_controller")
        self.declare_parameter("robot_ip",     "192.168.10.2")
        self.declare_parameter("reverse_port", REVERSE_PORT)
        self.declare_parameter("ext_program",  "ext.urp")

        self._robot_ip   = self.get_parameter("robot_ip").get_parameter_value().string_value
        self._port       = self.get_parameter("reverse_port").get_parameter_value().integer_value
        self._ext_prog   = self.get_parameter("ext_program").get_parameter_value().string_value

        self._q_current: list[float] = [0.0] * 6
        self._q_lock    = threading.Lock()
        self._js_ready  = threading.Event()

        self._q_target: list[float] = [0.0] * 6
        self._tgt_lock  = threading.Lock()

        self._conn: socket.socket | None = None
        self._conn_lock = threading.Lock()

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
            f"Reverse interface on :{self._port} — "
            f"will auto-play {self._ext_prog} via Dashboard Server"
        )

    # ── joint states ──────────────────────────────────────────────────────────

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

    # ── Dashboard auto-play ───────────────────────────────────────────────────

    def _play_ext_program(self) -> None:
        """Tell the robot's Dashboard Server to load and play ext_program."""
        ip, prog = self._robot_ip, self._ext_prog
        try:
            self.get_logger().info(
                f"Dashboard: connecting to {ip}:29999 to play {prog} ..."
            )
            s = socket.create_connection((ip, 29999), timeout=10)
            s.settimeout(5)
            s.recv(1024)                          # welcome banner
            s.sendall(f"load {prog}\n".encode()); time.sleep(0.3)
            r = s.recv(1024).decode().strip()
            self.get_logger().info(f"Dashboard load → {r}")
            s.sendall(b"play\n");                 time.sleep(0.3)
            r = s.recv(1024).decode().strip()
            self.get_logger().info(f"Dashboard play → {r}")
            s.close()
        except Exception as exc:
            self.get_logger().warn(f"Dashboard play failed: {exc} (will retry on next disconnect)")

    # ── TCP server ────────────────────────────────────────────────────────────

    def _server_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self._port))
        srv.listen(1)
        self.get_logger().info(f"Listening on :{self._port}")

        # Trigger first play now that the server socket is open
        threading.Thread(target=self._play_ext_program, daemon=True).start()

        while rclpy.ok():
            try:
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.get_logger().info(f"Robot connected from {addr[0]}")
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
                    self.get_logger().warn(f"Accept error: {exc}")

    def _drain_then_replay(self, conn: socket.socket) -> None:
        """Drain robot keepalives; when connection drops, replay ext.urp."""
        try:
            conn.settimeout(1.0)
            while True:
                if not conn.recv(256):
                    break
        except Exception:
            pass
        with self._conn_lock:
            if self._conn is conn:
                self._conn = None
        self.get_logger().warn(
            f"Robot disconnected — replaying {self._ext_prog} in 3 s ..."
        )
        time.sleep(3.0)
        if rclpy.ok():
            self._play_ext_program()

    # ── 125 Hz control loop ───────────────────────────────────────────────────

    def _control_loop(self) -> None:
        self._js_ready.wait()   # don't start until we have real joint positions
        next_t = time.monotonic()
        while rclpy.ok():
            with self._tgt_lock:
                q = list(self._q_target)
            with self._conn_lock:
                conn = self._conn
            if conn is not None:
                try:
                    conn.sendall(_pack(q))
                except Exception:
                    with self._conn_lock:
                        if self._conn is conn:
                            self._conn = None
            next_t += STEP_TIME
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()

    # ── FollowJointTrajectory ─────────────────────────────────────────────────

    @staticmethod
    def _reorder(names: list[str], pos: list[float]) -> list[float]:
        return [pos[names.index(j)] for j in UR_JOINT_ORDER]

    async def _execute_cb(self, goal_handle):
        traj = goal_handle.request.trajectory
        if not traj.points:
            goal_handle.abort()
            r = FollowJointTrajectory.Result()
            r.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            return r

        jnames    = list(traj.joint_names)
        waypoints = [
            (
                pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9,
                self._reorder(jnames, list(pt.positions)),
            )
            for pt in traj.points
        ]
        total_time = waypoints[-1][0]
        self.get_logger().info(
            f"Executing {len(waypoints)}-pt trajectory ({total_time:.2f} s) at 125 Hz"
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
                    self._q_target = waypoints[-1][1]
                break

            # find segment
            seg = len(waypoints) - 2
            for i in range(len(waypoints) - 1):
                if t_now < waypoints[i + 1][0]:
                    seg = i
                    break

            t0, q0 = waypoints[seg]
            t1, q1 = waypoints[seg + 1]
            alpha   = max(0.0, min(1.0, (t_now - t0) / (t1 - t0) if t1 > t0 else 1.0))
            q_interp = [q0[j] + alpha * (q1[j] - q0[j]) for j in range(6)]

            with self._tgt_lock:
                self._q_target = q_interp

            next_tick += STEP_TIME
            await asyncio.sleep(max(0.0, next_tick - time.monotonic()))

        goal_handle.succeed()
        r = FollowJointTrajectory.Result()
        r.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        self.get_logger().info("Trajectory complete")
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
