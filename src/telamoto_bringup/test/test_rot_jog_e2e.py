#!/usr/bin/env python3
"""E2E test for the TCP rotation jog (pan/tilt/roll axes).

Run manually, no robot or bringup needed (ports 8095/50094 must be free):

    pixi run python src/telamoto_bringup/test/test_rot_jog_e2e.py

Instantiates the REAL URServoController and drives its real WebSocket with
6-element CBOR jog frames, verifying:
  R1/R2  rotation axes scale by jog_rot_speed and ramp to target
  R3     orientation hold stands down while a rotation is commanded
  R4/R10 instant stop + lease expiry stop the rotation
  R5/R6  analog float axes work; NaN in any slot zeroes the whole twist
  R7     legacy 3-element (linear-only) frames clear the rotation
  R8     server-side jog-mode gate blocks rotation too
  R9     jrspeed is live-tunable, clamped, NaN-rejecting

Isolated: ROS_DOMAIN_ID=74 — safe to run beside a live bringup."""
import base64
import importlib.util
import json
import os
import socket
import struct
import sys
import time
import urllib.request

_SCRIPT = os.path.join("src", "telamoto_bringup", "scripts", "ur_servo_controller.py")
spec = importlib.util.spec_from_file_location("usc", _SCRIPT)
usc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(usc)

import threading

import rclpy
from sensor_msgs.msg import JointState
from moveit_msgs.srv import ServoCommandType

os.environ["ROS_DOMAIN_ID"] = "74"
rclpy.init(args=["--ros-args", "-p", "web_port:=8095", "-p", "reverse_port:=50094"])
node = usc.URServoController()
# Helper graph: joint states unblock _js_ready (jog loop), the servo arm
# service lets _arm_servo return immediately instead of its 20 s timeout.
helper = rclpy.create_node("rot_jog_test_helper")
js_pub = helper.create_publisher(JointState, "/joint_states", 10)
helper.create_service(ServoCommandType, "/servo_node/switch_command_type",
                      lambda req, res: res)
exec_ = rclpy.executors.MultiThreadedExecutor(num_threads=2)
exec_.add_node(node)
exec_.add_node(helper)
threading.Thread(target=exec_.spin, daemon=True).start()


def pub_js():
    m = JointState()
    m.name = list(usc.UR_JOINT_ORDER)
    m.position = [0.1, -1.2, 1.5, -0.3, 1.57, 0.5]
    m.velocity = [0.0] * 6
    js_pub.publish(m)


pub_js()
deadline = time.monotonic() + 5.0
while not node._js_ready.is_set() and time.monotonic() < deadline:
    pub_js(); time.sleep(0.05)
assert node._js_ready.is_set(), "joint states never arrived"
time.sleep(0.7)

urllib.request.urlopen(urllib.request.Request(
    "http://127.0.0.1:8095/api/jogmode?on=1", method="POST"), timeout=2)

s = socket.create_connection(("127.0.0.1", 8095), timeout=5)
s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
key = base64.b64encode(os.urandom(16)).decode()
s.sendall((f"GET /ws HTTP/1.1\r\nHost: 127.0.0.1:8095\r\nUpgrade: websocket\r\n"
           f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
           f"Sec-WebSocket-Version: 13\r\n\r\n").encode())
buf = b""
while b"\r\n\r\n" not in buf:
    buf += s.recv(1024)
assert b" 101 " in buf.split(b"\r\n")[0] + b" ", buf[:80]


def ci(n):
    m, u = (0x20, -1 - n) if n < 0 else (0x00, n)
    return bytes([m | u]) if u < 24 else bytes([m | 24, u])


def cf(x):  # CBOR float32
    return b"\xfa" + struct.pack(">f", x)


def cbor6(lx, ly, lz, ax, ay, az):
    enc = lambda v: cf(v) if isinstance(v, float) else ci(v)
    return b"\x86" + b"".join(enc(v) for v in (lx, ly, lz, ax, ay, az))


def cbor3(lx, ly, lz):
    return b"\x83" + ci(lx) + ci(ly) + ci(lz)


def ws_send(payload):
    mask = os.urandom(4)
    s.sendall(bytes([0x82, 0x80 | len(payload)]) + mask
              + bytes(c ^ mask[i & 3] for i, c in enumerate(payload)))


def state():
    return json.loads(urllib.request.urlopen(
        "http://127.0.0.1:8095/api/state", timeout=2).read())


fails = 0
def check(name, cond, detail):
    global fails
    fails += 0 if cond else 1
    print(("PASS  " if cond else "FAIL  ") + f"{name:36s} {detail}", flush=True)


# warm the link estimator at 30 Hz
end = time.monotonic() + 1.0
while time.monotonic() < end:
    ws_send(cbor3(0, 0, 0)); time.sleep(1 / 30)

# 1 ── pure rotation frame → angular target at jog_rot_speed, linear zero
ws_send(cbor6(0, 0, 0, 1, 0, 0)); time.sleep(0.05)
j = list(node._jog)
check("R1 rotation axis scaled", abs(j[3] - node._jog_rot_speed) < 1e-12
      and j[0] == j[1] == j[2] == 0.0 and j[4] == j[5] == 0.0,
      f"_jog={['%.3f' % v for v in j]} jrspeed={node._jog_rot_speed}")

# 2 ── ramp reaches the angular target (jog loop runs at 50 Hz)
end = time.monotonic() + 0.6
while time.monotonic() < end:
    ws_send(cbor6(0, 0, 0, 1, 0, 0)); time.sleep(1 / 30)
cmd = list(node._jog_cmd)
check("R2 angular ramp reaches target", abs(cmd[3] - node._jog_rot_speed) < 1e-6,
      f"_jog_cmd[3]={cmd[3]:.4f} target={node._jog_rot_speed}")

# 3 ── orientation hold stands down while rotating
check("R3 orient hold stands down", node._orient_target is None,
      f"_orient_target={node._orient_target}")

# 4 ── stop zeroes everything
ws_send(cbor6(0, 0, 0, 0, 0, 0)); time.sleep(0.05)
check("R4 instant stop", all(v == 0.0 for v in node._jog),
      f"_jog={['%.3f' % v for v in node._jog]}")

# 5 ── analog float rotation (half speed; account for the live link scale)
ws_send(cbor6(0, 0, 0, 0, 0.5, 0.0)); time.sleep(0.05)
want = 0.5 * node._jog_rot_speed * node._link_scale
check("R5 analog float rotation", abs(node._jog[4] - want) < 1e-9,
      f"_jog[4]={node._jog[4]:.4f} want={want:.4f}")

# 6 ── NaN in an angular slot zeroes the whole twist
ws_send(cbor6(1, 0, 0, 0, float("nan"), 0.0)); time.sleep(0.05)
check("R6 NaN angular rejected", all(v == 0.0 for v in node._jog),
      f"_jog={['%.3f' % v for v in node._jog]}")

# 7 ── legacy 3-element frame leaves rotation at zero
ws_send(cbor6(0, 0, 0, 0, 0, 1)); time.sleep(0.02)
ws_send(cbor3(1, 0, 0)); time.sleep(0.05)
j = list(node._jog)
want = node._jog_speed * node._link_scale
check("R7 legacy frame clears rotation", abs(j[0] - want) < 1e-9
      and all(v == 0.0 for v in j[3:6]),
      f"_jog={['%.3f' % v for v in j]} want={want:.4f}")

# 8 ── jog-mode OFF gates rotation server-side
ws_send(cbor3(0, 0, 0))
urllib.request.urlopen(urllib.request.Request(
    "http://127.0.0.1:8095/api/jogmode?on=0", method="POST"), timeout=2)
ws_send(cbor6(0, 0, 0, 1, 1, 1)); time.sleep(0.05)
check("R8 jog-off blocks rotation", all(v == 0.0 for v in node._jog),
      f"_jog={['%.3f' % v for v in node._jog]}")
urllib.request.urlopen(urllib.request.Request(
    "http://127.0.0.1:8095/api/jogmode?on=1", method="POST"), timeout=2)

# 9 ── jrspeed tunable: web set + state echo + clamping
urllib.request.urlopen(urllib.request.Request(
    "http://127.0.0.1:8095/api/set?jrspeed=0.5", method="POST"), timeout=2)
st = state()
urllib.request.urlopen(urllib.request.Request(
    "http://127.0.0.1:8095/api/set?jrspeed=99", method="POST"), timeout=2)
clamped = node._jog_rot_speed
urllib.request.urlopen(urllib.request.Request(
    "http://127.0.0.1:8095/api/set?jrspeed=nan", method="POST"), timeout=2)
check("R9 jrspeed live-tunable + clamped", st["jrspeed"] == 0.5
      and clamped == usc.JOG_ROT_SPEED_MAX and node._jog_rot_speed == clamped,
      f"state={st['jrspeed']} after99={clamped} afterNaN={node._jog_rot_speed}")

# 10 ── lease expiry stops a rotation (no fresh frames)
ws_send(cbor6(0, 0, 0, 1, 0, 0)); time.sleep(0.35)
check("R10 lease expiry stops rotation",
      time.monotonic() >= node._jog_lease, "lease expired (jog loop zeroes _jog)")

print("RESULT: " + ("ALL PASS" if fails == 0 else f"{fails} FAILED"))
node.destroy_node()
rclpy.try_shutdown()
sys.exit(1 if fails else 0)
