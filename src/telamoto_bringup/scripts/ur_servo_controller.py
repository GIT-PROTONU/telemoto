#!/usr/bin/env python3
"""
FollowJointTrajectory action server using the External Control URCap
reverse-interface protocol at 125 Hz.

Architecture
------------
1. TCP server on port 50001 waits for the robot to connect.
   (ur_dashboard_autoplay.py plays ext.urp once this port is open.)
2. A 125 Hz control loop sends 32-byte packets every 8 ms.
3. The FollowJointTrajectory action server interpolates trajectory
   waypoints and feeds them to the loop; when idle, the loop holds
   the last commanded position with servoj.

Packet format (from ur_client_library/control/reverse_interface.h):
  8 × int32 big-endian:
    [0]   control_mode  (MODE_SERVOJ = 1)
    [1-6] joint positions × MULT_JOINTSTATE (= 1 000 000)
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

REVERSE_PORT  = 50001
STEP_TIME     = 0.008        # 125 Hz
MULT          = 1_000_000
MODE_SERVOJ   = 1
MODE_STOPPED  = -2
TIMEOUT_MS    = 20           # robot aborts if no packet in 20 ms


def _pack(q: list[float], mode: int = MODE_SERVOJ) -> bytes:
    return struct.pack(">8i",
        mode,
        int(q[0] * MULT), int(q[1] * MULT), int(q[2] * MULT),
        int(q[3] * MULT), int(q[4] * MULT), int(q[5] * MULT),
        TIMEOUT_MS,
    )


class URServoController(Node):

    def __init__(self) -> None:
        super().__init__("ur_servo_controller")
        self.declare_parameter("robot_ip",     "192.168.10.2")
        self.declare_parameter("reverse_port", REVERSE_PORT)

        self._port = self.get_parameter("reverse_port").get_parameter_value().integer_value

        # Shared state updated by /joint_states subscriber
        self._q_current: list[float] = [0.0] * 6
        self._q_lock    = threading.Lock()
        self._js_ready  = threading.Event()

        # Target fed to the 125 Hz loop; lock protects concurrent writes
        self._q_target: list[float] = [0.0] * 6
        self._tgt_lock  = threading.Lock()

        # Robot connection socket (None when robot is not connected)
        self._conn: socket.socket | None = None
        self._conn_lock = threading.Lock()

        cb = ReentrantCallbackGroup()
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
            execute_callback   = self._execute_cb,
            goal_callback      = lambda _: GoalResponse.ACCEPT,
            cancel_callback    = lambda _: CancelResponse.ACCEPT,
            callback_group     = cb,
        )

        threading.Thread(target=self._server_loop,  daemon=True, name="ri-server").start()
        threading.Thread(target=self._control_loop, daemon=True, name="ri-loop").start()

        self.get_logger().info(
            f"Reverse interface listening on :{self._port} — "
            "waiting for External Control URCap to connect ..."
        )

    # ── /joint_states subscriber ──────────────────────────────────────────────

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

    # ── TCP server ────────────────────────────────────────────────────────────

    def _server_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self._port))
        srv.listen(1)
        while rclpy.ok():
            try:
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.get_logger().info(f"Robot connected from {addr[0]}:{addr[1]}")
                with self._conn_lock:
                    old = self._conn
                    self._conn = conn
                if old:
                    try: old.close()
                    except Exception: pass
                # drain any robot-side keepalives
                threading.Thread(
                    target=self._drain, args=(conn,), daemon=True
                ).start()
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().warn(f"Accept: {exc}")

    def _drain(self, conn: socket.socket) -> None:
        try:
            conn.settimeout(1.0)
            while True:
                if not conn.recv(256):
                    break
        except Exception:
            pass
        self.get_logger().warn("Robot disconnected from reverse interface")
        with self._conn_lock:
            if self._conn is conn:
                self._conn = None

    # ── 125 Hz control loop ───────────────────────────────────────────────────

    def _control_loop(self) -> None:
        # Wait for first joint state before driving the robot
        self._js_ready.wait()
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
                next_t = time.monotonic()  # reset on overrun

    # ── FollowJointTrajectory action ──────────────────────────────────────────

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

        jnames = list(traj.joint_names)
        waypoints = [
            (
                pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9,
                self._reorder(jnames, list(pt.positions)),
            )
            for pt in traj.points
        ]
        total_time = waypoints[-1][0]
        self.get_logger().info(
            f"Executing {len(waypoints)}-point trajectory "
            f"({total_time:.2f} s) at 125 Hz"
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

            # Find segment and interpolate
            seg = len(waypoints) - 2
            for i in range(len(waypoints) - 1):
                if t_now < waypoints[i + 1][0]:
                    seg = i
                    break

            t0, q0 = waypoints[seg]
            t1, q1 = waypoints[seg + 1]
            alpha = (t_now - t0) / (t1 - t0) if t1 > t0 else 1.0
            alpha = max(0.0, min(1.0, alpha))
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
