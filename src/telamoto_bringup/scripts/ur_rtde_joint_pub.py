#!/usr/bin/env python3
"""Minimal RTDE reader: publishes /joint_states from actual_q, plus the pendant
speed-slider setting (target_speed_fraction) on /telamoto/speed_fraction.

CB3 3.15 PolyScopeX holds all RTDE INPUT variables internally, making ur_robot_driver's
on_configure() always fail. We bypass the driver: only RTDE OUTPUTS (read-only) are used.
(So we can READ the speed slider here, but not set it — there's no RTDE-input path.)

RTDE protocol notes:
  DATA_PACKAGE cmd = 85 (0x55 = 'U'), NOT 82
  SETUP_OUTPUTS response: recipe_id uint8 (1 byte) + comma-separated ASCII type names
  DATA_PACKAGE: recipe_id uint8 (1 byte) + packed binary variable data
"""
import gc
import socket
import struct
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

CMD_PROTOCOL_VERSION = 86   # 0x56 'V'
CMD_SETUP_OUTPUTS    = 79   # 0x4F 'O'
CMD_START            = 83   # 0x53 'S'
CMD_DATA_PACKAGE     = 85   # 0x55 'U'

# Request order fixes the byte layout of each DATA_PACKAGE: actual_q (VECTOR6D,
# 48 B) then target_speed_fraction (DOUBLE, 8 B). The speed var is optional — if
# the robot doesn't expose it, joint states still stream unchanged.
OUTPUT_VARS = "actual_q,target_speed_fraction"
Q_BYTES     = 1 + 48        # recipe_id + 6×float64
SPEED_BYTES = Q_BYTES + 8   #            + 1×float64
STREAM_FREQ = 125.0


def send_rtde(s: socket.socket, cmd: int, payload: bytes = b"") -> None:
    s.sendall(struct.pack(">HB", 3 + len(payload), cmd) + payload)


def recv_rtde(s: socket.socket) -> tuple[int, bytes]:
    hdr = b""
    while len(hdr) < 3:
        chunk = s.recv(3 - len(hdr))
        if not chunk:
            raise ConnectionError("RTDE connection closed by robot")
        hdr += chunk
    size, cmd = struct.unpack(">HB", hdr)
    data = b""
    remaining = size - 3
    while len(data) < remaining:
        chunk = s.recv(remaining - len(data))
        if not chunk:
            raise ConnectionError("RTDE connection closed mid-packet")
        data += chunk
    return cmd, data


class URRTDEJointPublisher(Node):
    def __init__(self):
        super().__init__("ur_rtde_joint_pub")
        self.declare_parameter("robot_ip", "192.168.10.2")
        self._robot_ip = self.get_parameter("robot_ip").get_parameter_value().string_value

        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        # Reused message — name is constant; stamp + position are refreshed per
        # packet. Avoids a fresh allocation on every 125 Hz publish.
        self._msg = JointState()
        self._msg.name = JOINT_NAMES

        # Pendant speed-slider fraction [0,1], LATCHED so a late subscriber (the
        # web UI in ur_servo_controller) gets the last value immediately. Published
        # only on change — the slider rarely moves, so this is near-silent.
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._speed_pub = self.create_publisher(Float64, "/telamoto/speed_fraction", latched)

        # Read AND publish straight off the RTDE stream (full 125 Hz): no
        # downsampling timer, so /joint_states is as fresh as the robot makes it
        # — the smoothest possible state seed for MoveIt Servo and move_group.
        # Reading every packet also keeps the socket buffer drained (no backlog).
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    # ------------------------------------------------------------------
    def _connect(self) -> tuple[socket.socket, int, bool]:
        """Open RTDE, negotiate v2, setup outputs, start.
        Returns (sock, recipe_id, have_speed) — have_speed False if the robot
        didn't accept target_speed_fraction (joint states still work)."""
        while rclpy.ok():
            try:
                self.get_logger().info(f"Connecting to RTDE at {self._robot_ip}:30004 ...")
                s = socket.create_connection((self._robot_ip, 30004), timeout=5)
                # Realtime link: no Nagle batching — deliver every 125 Hz RTDE
                # packet the instant it arrives, no coalescing delay.
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                s.settimeout(2.0)

                send_rtde(s, CMD_PROTOCOL_VERSION, struct.pack(">H", 2))
                cmd, data = recv_rtde(s)
                if data != b"\x01":
                    raise RuntimeError(f"Protocol negotiation returned {data.hex()}")

                payload = struct.pack(">d", STREAM_FREQ) + OUTPUT_VARS.encode()
                send_rtde(s, CMD_SETUP_OUTPUTS, payload)
                cmd, data = recv_rtde(s)
                # data[0] = recipe_id (uint8), data[1:] = comma-separated type names.
                # An unrecognised var comes back "NOT_FOUND" and is dropped from the
                # package, so confirm target_speed_fraction was accepted as DOUBLE.
                if len(data) < 1 or data[0] == 0:
                    raise RuntimeError(f"SETUP_OUTPUTS failed: {data!r}")
                recipe_id = data[0]
                types = data[1:].decode(errors="replace").split(",")
                have_speed = len(types) >= 2 and types[1] == "DOUBLE"
                if not have_speed:
                    self.get_logger().warn(
                        "RTDE: target_speed_fraction unavailable — speed slider not shown")

                send_rtde(s, CMD_START)
                cmd, data = recv_rtde(s)
                if data != b"\x01":
                    raise RuntimeError(f"START failed: {data.hex()}")

                self.get_logger().info(
                    f"RTDE streaming actual_q{'+speed' if have_speed else ''} at "
                    f"{STREAM_FREQ} Hz (recipe_id={recipe_id})"
                )
                return s, recipe_id, have_speed
            except Exception as exc:
                self.get_logger().warn(f"RTDE connect failed: {exc} — retry in 2 s")
                try:
                    s.close()
                except Exception:
                    pass
                time.sleep(2.0)
        raise RuntimeError("rclpy shutdown during connect")

    # ------------------------------------------------------------------
    def _reader_loop(self) -> None:
        """Background loop: reads RTDE data packets as fast as the robot sends them."""
        while rclpy.ok():
            try:
                sock, recipe_id, have_speed = self._connect()
                last_frac = None                       # republish on each reconnect
                while rclpy.ok():
                    cmd, data = recv_rtde(sock)
                    if cmd != CMD_DATA_PACKAGE or data[0] != recipe_id:
                        continue
                    # DATA_PACKAGE: recipe_id (uint8) + VECTOR6D (48 B) [+ DOUBLE (8 B)].
                    # Joint states are CRITICAL (they seed MoveIt Servo) and must never
                    # depend on the optional speed var — publish on actual_q alone.
                    if len(data) >= Q_BYTES:
                        self._msg.header.stamp = self.get_clock().now().to_msg()
                        self._msg.position = list(struct.unpack(">6d", data[1:Q_BYTES]))
                        self._pub.publish(self._msg)
                    # Speed slider: a bonus, only when the extra bytes are present.
                    if have_speed and len(data) >= SPEED_BYTES:
                        frac = struct.unpack(">d", data[Q_BYTES:SPEED_BYTES])[0]
                        if last_frac is None or abs(frac - last_frac) > 1e-4:
                            last_frac = frac
                            self._speed_pub.publish(Float64(data=frac))
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().warn(f"RTDE stream lost: {exc} — reconnecting")
                try:
                    sock.close()
                except Exception:
                    pass


def main() -> None:
    rclpy.init()
    node = URRTDEJointPublisher()
    # Freeze startup objects out of the GC scan set: the 125 Hz reader/publisher
    # then runs without collection pauses (steady-state allocs are non-cyclic).
    gc.collect()
    gc.freeze()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
