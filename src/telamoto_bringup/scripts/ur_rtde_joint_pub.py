#!/usr/bin/env python3
"""Minimal RTDE reader that subscribes to actual_q outputs only and publishes /joint_states.

CB3 3.15 PolyScopeX holds all RTDE INPUT variables internally, making ur_robot_driver's
on_configure() always fail. We bypass the driver: only RTDE OUTPUTS (read-only) are used.

RTDE protocol notes:
  DATA_PACKAGE cmd = 85 (0x55 = 'U'), NOT 82
  SETUP_OUTPUTS response: recipe_id uint8 (1 byte) + ASCII type names
  DATA_PACKAGE: recipe_id uint8 (1 byte) + packed binary variable data
"""
import gc
import socket
import struct
import threading
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

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

OUTPUT_VARS = "actual_q"
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

        # Read AND publish straight off the RTDE stream (full 125 Hz): no
        # downsampling timer, so /joint_states is as fresh as the robot makes it
        # — the smoothest possible state seed for MoveIt Servo and move_group.
        # Reading every packet also keeps the socket buffer drained (no backlog).
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

    # ------------------------------------------------------------------
    def _connect(self) -> tuple[socket.socket, int]:
        """Open RTDE, negotiate v2, setup outputs, start. Returns (sock, recipe_id)."""
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
                # data[0] = recipe_id (uint8), data[1:] = ASCII type name(s)
                if len(data) < 1 or data[0] == 0:
                    raise RuntimeError(f"SETUP_OUTPUTS failed: {data!r}")
                recipe_id = data[0]

                send_rtde(s, CMD_START)
                cmd, data = recv_rtde(s)
                if data != b"\x01":
                    raise RuntimeError(f"START failed: {data.hex()}")

                self.get_logger().info(
                    f"RTDE streaming actual_q at {STREAM_FREQ} Hz (recipe_id={recipe_id})"
                )
                return s, recipe_id
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
                sock, recipe_id = self._connect()
                while rclpy.ok():
                    cmd, data = recv_rtde(sock)
                    # DATA_PACKAGE: recipe_id (uint8) + VECTOR6D (6×float64 = 48 bytes)
                    if cmd == CMD_DATA_PACKAGE and len(data) >= 49 and data[0] == recipe_id:
                        self._msg.header.stamp = self.get_clock().now().to_msg()
                        self._msg.position = list(struct.unpack(">6d", data[1:49]))
                        self._pub.publish(self._msg)
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
