#!/usr/bin/env python3
"""E2E test for the singularity-amplification guard + runaway latch.

Run manually, no robot or bringup needed (ports 8096/50095 must be free):

    pixi run python src/telamoto_bringup/test/test_qd_guard_e2e.py

Instantiates the REAL URServoController with a FAKE robot (a socket pulling the
real 11-int packets from the control loop), real WS jog frames, helper-published
TF / joint states (with synthetic actual_qd) / fresh ServoStatus, verifying:
  Q1  honest jog (qd 0) streams speedl at full commanded speed
  Q2  amplified qd (constant 0.7 rad/s regardless of command) spirals the gate
      down to the fixed point g = (BASE + SLOPE·jspeed·g)/qd — slowed, NOT stopped
  Q3  runaway qd (1.5 rad/s) trips the LATCH: speedj zeros while the key is held,
      /api/state reports qdLatch (and the inward direction is captured)
  Q4  latch holds through key release while qd stays high
  Q5  qd settles → latch releases; the OPPOSITE (escape) jog runs at full speed
  Q6  the INWARD jog stays hard-blocked (zero twist, qdBlockAxis in /api/state)
  Q7  escape with elevated qd (1.2 rad/s — would have latched a fresh tap) keeps
      running under the raised escape allowance/latch line
  Q8  5 cm of TCP travel clears the block; the inward direction works again
  Q9  travel DURING the runaway must not clear the block (the 2026-06-12
      protective stop: each lurch covered 5-30 cm, self-clearing the block
      mid-excursion) — the anchor re-sets to the rest pose at latch release
  Q10 the inward direction is still blocked at the rest pose
  Q11 deliberate travel at honest joint speed clears it; inward works again
  S1  a singularity halt mid-jog captures the approach dir (singAxis in
      /api/state — drives the web escape-pad UI) and zeroes the stream
  S2  a sentinel feed flap mid-escape must NOT invert the captured direction
  S3  the halt clearing drops singAxis

The 2026-06-12 boundary-stall trip is the regression scenario: allowance computed
from the pre-gate target let qd reach 3.2 rad/s → UR protective stop.

Isolated: ROS_DOMAIN_ID=73 — safe to run beside a live bringup."""
import base64
import importlib.util
import json
import math
import os
import socket
import struct
import sys
import threading
import time
import urllib.request

_SCRIPT = os.path.join(os.path.dirname(__file__), os.pardir,
                       "scripts", "ur_servo_controller.py")
spec = importlib.util.spec_from_file_location("usc", _SCRIPT)
usc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(usc)

import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from moveit_msgs.msg import ServoStatus
from moveit_msgs.srv import ServoCommandType

os.environ["ROS_DOMAIN_ID"] = "73"
rclpy.init(args=["--ros-args", "-p", "web_port:=8096", "-p", "reverse_port:=50095"])
node = usc.URServoController()
helper = rclpy.create_node("qd_guard_test_helper")
js_pub = helper.create_publisher(JointState, "/joint_states", 10)
tf_pub = helper.create_publisher(TFMessage, "/tf", 10)
st_pub = helper.create_publisher(ServoStatus, usc.SERVO_STATUS_TOPIC, 10)
helper.create_service(ServoCommandType, "/servo_node/switch_command_type",
                      lambda req, res: res)

QD = [0.0]                      # synthetic measured joint speed, mutated per phase
POS = [0.5]                     # synthetic TCP x (TF), mutated to simulate travel
CODE = [ServoStatus.NO_WARNING]  # synthetic Servo verdict, mutated per phase


def feeder():
    """30 Hz: joint states (with synthetic actual_qd), TF base_link→tool0,
    fresh ServoStatus (CODE) — everything the jog stack needs to run."""
    while rclpy.ok():
        m = JointState()
        m.header.stamp = helper.get_clock().now().to_msg()
        m.name = list(usc.UR_JOINT_ORDER)
        m.position = [0.1, -1.2, 1.5, -0.3, 1.57, 0.5]
        m.velocity = [QD[0]] + [0.0] * 5
        js_pub.publish(m)
        t = TransformStamped()
        t.header.stamp = m.header.stamp
        t.header.frame_id = "base_link"
        t.child_frame_id = "tool0"
        t.transform.translation.x, t.transform.translation.z = POS[0], 0.5
        t.transform.rotation.w = 1.0
        tf_pub.publish(TFMessage(transforms=[t]))
        st_pub.publish(ServoStatus(code=CODE[0]))
        time.sleep(1 / 30)


exec_ = rclpy.executors.MultiThreadedExecutor(num_threads=2)
exec_.add_node(node)
exec_.add_node(helper)
threading.Thread(target=exec_.spin, daemon=True).start()
threading.Thread(target=feeder, daemon=True).start()

deadline = time.monotonic() + 5.0
while not node._js_ready.is_set() and time.monotonic() < deadline:
    time.sleep(0.05)
assert node._js_ready.is_set(), "joint states never arrived"
time.sleep(0.7)                                   # web thread bind

urllib.request.urlopen(urllib.request.Request(
    "http://127.0.0.1:8096/api/jogmode?on=1", method="POST"), timeout=2)
urllib.request.urlopen(urllib.request.Request(
    "http://127.0.0.1:8096/api/set?jspeed=0.1", method="POST"), timeout=2)

# ── fake robot: pull packets like the URScript does ─────────────────────────
samples = []                    # (t, mode, lin_mag m/s) per pulled packet


def fake_robot():
    r = socket.create_connection(("127.0.0.1", 50095), timeout=5)
    r.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    while rclpy.ok():
        r.sendall(struct.pack(">i", 1))
        buf = b""
        while len(buf) < 44:
            chunk = r.recv(44 - len(buf))
            if not chunk:
                return
            buf += chunk
        p = struct.unpack(">11i", buf)
        v = [x / 1e6 for x in p[1:7]]
        samples.append((time.monotonic(), p[7],
                        math.sqrt(v[0]**2 + v[1]**2 + v[2]**2)))
        time.sleep(0.006)


threading.Thread(target=fake_robot, daemon=True).start()

# ── real WS jog client streaming at 30 Hz ───────────────────────────────────
s = socket.create_connection(("127.0.0.1", 8096), timeout=5)
s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
key = base64.b64encode(os.urandom(16)).decode()
s.sendall((f"GET /ws HTTP/1.1\r\nHost: 127.0.0.1:8096\r\nUpgrade: websocket\r\n"
           f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
           f"Sec-WebSocket-Version: 13\r\n\r\n").encode())
buf = b""
while b"\r\n\r\n" not in buf:
    buf += s.recv(1024)
assert b" 101 " in buf.split(b"\r\n")[0] + b" ", buf[:80]

VEC = [0]                       # current lx streamed by the 30 Hz thread


def streamer():
    def ci(n):
        m, u = (0x20, -1 - n) if n < 0 else (0x00, n)
        return bytes([m | u])
    while rclpy.ok():
        payload = b"\x83" + ci(VEC[0]) + ci(0) + ci(0)
        mask = os.urandom(4)
        s.sendall(bytes([0x82, 0x80 | len(payload)]) + mask
                  + bytes(c ^ mask[i & 3] for i, c in enumerate(payload)))
        time.sleep(1 / 30)


threading.Thread(target=streamer, daemon=True).start()


def state():
    return json.loads(urllib.request.urlopen(
        "http://127.0.0.1:8096/api/state", timeout=2).read())


def recent(window=0.5):
    """Samples from the last `window` s."""
    t0 = time.monotonic() - window
    return [x for x in samples if x[0] >= t0]


fails = 0
def check(name, cond, detail):
    global fails
    fails += 0 if cond else 1
    print(("PASS  " if cond else "FAIL  ") + f"{name:36s} {detail}", flush=True)


time.sleep(1.5)                                   # estimator warm, robot pulling

# Q1 ── honest jog: full speed streamed as speedl ─────────────────────────────
VEC[0] = 1
time.sleep(1.5)                                   # ramp + settle
sm = recent()
speedl = [m for _, mode, m in sm if mode == usc.MODE_SPEEDL]
mag = sum(speedl) / max(1, len(speedl))
check("Q1 honest jog full speed", len(speedl) > 10 and 0.085 <= mag <= 0.105,
      f"mode3={len(speedl)}/{len(sm)} mean={mag*1000:.1f}mm/s (cmd 100)")

# Q2 ── amplified qd: gate spirals to the fixed point, slowed NOT stopped ─────
# Constant qd regardless of command = amplification. Fixed point of
# g = (BASE + SLOPE·0.1·g)/qd for qd=0.7: g = 0.3/0.45 = 0.667.
QD[0] = 0.7
time.sleep(2.5)
sm = recent()
speedl = [m for _, mode, m in sm if mode == usc.MODE_SPEEDL]
mag = sum(speedl) / max(1, len(speedl))
lat = state()["qdLatch"]
check("Q2 amplified -> spiral to crawl", len(speedl) > 10 and 0.045 <= mag <= 0.085
      and not lat,
      f"mean={mag*1000:.1f}mm/s (fixed point ~67) latch={lat}")

# Q3 ── runaway qd: LATCH — speedj zeros while the key is still held ─────────
QD[0] = 1.5
time.sleep(1.5)
sm = recent()
zeros = [m for _, mode, m in sm if mode == usc.MODE_SPEEDJ]
speedl = [m for _, mode, m in sm if mode == usc.MODE_SPEEDL]
lat = state()["qdLatch"]
check("Q3 runaway latches to zero-hold", lat and len(zeros) > 10
      and len(speedl) == 0 and all(m < 1e-9 for m in zeros),
      f"latch={lat} speedj-zeros={len(zeros)} speedl={len(speedl)}")

# Q4 ── latch survives key release while qd is still high ────────────────────
VEC[0] = 0
time.sleep(1.0)
lat = state()["qdLatch"]
check("Q4 latch holds while qd high", lat, f"latch={lat} (qd still 1.5)")

# Q5 ── qd settles -> latch releases; ESCAPE (opposite) jog runs full speed ───
QD[0] = 0.0
time.sleep(1.0)
lat = state()["qdLatch"]
VEC[0] = -1                                       # opposite of the latched inward +lx
time.sleep(1.5)
sm = recent()
speedl = [m for _, mode, m in sm if mode == usc.MODE_SPEEDL]
mag = sum(speedl) / max(1, len(speedl))
check("Q5 settle -> unlatch -> escape full", not lat and len(speedl) > 10
      and 0.085 <= mag <= 0.105,
      f"latch={lat} mean={mag*1000:.1f}mm/s (cmd 100)")
VEC[0] = 0
time.sleep(0.5)

# Q6 ── the INWARD direction stays hard-blocked after the latch ───────────────
ax = state()["qdBlockAxis"]
VEC[0] = 1                                        # the direction that latched
time.sleep(1.2)
sm = recent()
mags = [m for _, mode, m in sm if mode == usc.MODE_SPEEDL]
blocked = len(mags) > 10 and max(mags) < 1e-6
check("Q6 inward stays blocked", ax == "+X" and blocked,
      f"qdBlockAxis={ax} speedl={len(mags)} max={max(mags) if mags else 0:.6f}")
VEC[0] = 0
time.sleep(0.5)

# Q7 ── escape under elevated qd: raised allowance/latch line keeps it running ─
# qd 1.2 rad/s would latch a fresh tap (line 0.75) — escaping it must survive
# (floor 0.8 ⇒ gate ~0.67, latch line 2.0) and crawl out, not stop.
QD[0] = 1.2
VEC[0] = -1
time.sleep(2.0)
sm = recent()
speedl = [m for _, mode, m in sm if mode == usc.MODE_SPEEDL]
mag = sum(speedl) / max(1, len(speedl))
lat = state()["qdLatch"]
check("Q7 escape survives elevated qd", not lat and len(speedl) > 10
      and 0.03 <= mag <= 0.09,
      f"latch={lat} mean={mag*1000:.1f}mm/s (fixed point ~67)")
VEC[0] = 0
QD[0] = 0.0
time.sleep(0.8)

# Q8 ── 5 cm of TCP travel clears the block; inward works again ──────────────
POS[0] = 0.56                                     # 6 cm from the latch point
VEC[0] = -1                                       # any allowed jog evaluates the clear
time.sleep(0.8)
ax = state()["qdBlockAxis"]
VEC[0] = 1                                        # formerly blocked direction
time.sleep(1.2)
sm = recent()
speedl = [m for _, mode, m in sm if mode == usc.MODE_SPEEDL]
mag = sum(speedl) / max(1, len(speedl))
check("Q8 travel clears block -> inward ok", ax is None and len(speedl) > 10
      and mag > 0.05, f"qdBlockAxis={ax} mean={mag*1000:.1f}mm/s")
VEC[0] = 0
time.sleep(0.5)

# Q9 ── runaway travel must NOT clear the block (anchor re-sets at release) ───
VEC[0] = 1                                        # jog inward again
time.sleep(0.6)
QD[0] = 1.5                                       # runaway → latch (dir +X)
time.sleep(0.8)
POS[0] = 0.65                                     # 9 cm "lurch" WHILE latched
time.sleep(0.5)
VEC[0] = 0                                        # release…
QD[0] = 0.0                                       # …joints settle → latch off,
time.sleep(1.0)                                   # anchor re-set to rest (0.65)
ax = state()["qdBlockAxis"]
check("Q9 lurch does not clear the block", ax == "+X", f"qdBlockAxis={ax}")

# Q10 ── inward still hard-blocked at the rest pose ───────────────────────────
VEC[0] = 1
time.sleep(1.2)
sm = recent()
mags = [m for _, mode, m in sm if mode == usc.MODE_SPEEDL]
check("Q10 inward still blocked after lurch", len(mags) > 10
      and max(mags) < 1e-6,
      f"speedl={len(mags)} max={max(mags) if mags else 0:.6f}")
VEC[0] = 0
time.sleep(0.5)

# Q11 ── deliberate post-settle travel clears it ──────────────────────────────
VEC[0] = -1                                       # honest escape jog (qd 0)
POS[0] = 0.59                                     # 6 cm from the 0.65 rest pose
time.sleep(0.8)
ax = state()["qdBlockAxis"]
VEC[0] = 1
time.sleep(1.2)
sm = recent()
speedl = [m for _, mode, m in sm if mode == usc.MODE_SPEEDL]
mag = sum(speedl) / max(1, len(speedl))
check("Q11 honest travel clears -> inward ok", ax is None and len(speedl) > 10
      and mag > 0.05, f"qdBlockAxis={ax} mean={mag*1000:.1f}mm/s")
VEC[0] = 0
time.sleep(0.5)

# S1 ── singularity halt mid-jog: approach dir captured for the web escape UI ─
VEC[0] = 1
time.sleep(0.8)                                   # ramp up, _last_lin fresh
CODE[0] = ServoStatus.HALT_FOR_SINGULARITY
time.sleep(0.6)
st = state()
sm = recent()
mags = [m for _, mode, m in sm if mode == usc.MODE_SPEEDL]
check("S1 sing halt captures approach dir",
      st["servoCode"] == int(ServoStatus.HALT_FOR_SINGULARITY)
      and st["singAxis"] == "+X" and (not mags or max(mags) < 1e-6),
      f"servoCode={st['servoCode']} singAxis={st['singAxis']} "
      f"maxlin={max(mags) if mags else 0:.6f}")

# S2 ── feed flap mid-escape must NOT invert the captured direction ───────────
# A jittery sentinel can interleave clears between halt codes; re-capturing
# from the ESCAPE stream would flip singAxis and the web UI would disable the
# very pad backing the arm out (_halt_dir anti-inversion guard).
VEC[0] = -1                                       # escaping
CODE[0] = ServoStatus.NO_WARNING                  # brief clear…
time.sleep(0.4)
CODE[0] = ServoStatus.HALT_FOR_SINGULARITY        # …flaps back mid-escape
time.sleep(0.6)
st = state()
check("S2 flap mid-escape keeps inward dir", st["singAxis"] == "+X",
      f"singAxis={st['singAxis']} (escape stream was -X)")

# S3 ── halt clears -> hint gone ──────────────────────────────────────────────
CODE[0] = ServoStatus.NO_WARNING
VEC[0] = 0
time.sleep(0.6)
st = state()
check("S3 clear drops singAxis",
      st["singAxis"] is None and st["servoCode"] == int(ServoStatus.NO_WARNING),
      f"singAxis={st['singAxis']} servoCode={st['servoCode']}")

print("RESULT: " + ("ALL PASS" if fails == 0 else f"{fails} FAILED"))
node.destroy_node()
helper.destroy_node()
rclpy.try_shutdown()
sys.exit(1 if fails else 0)
