#!/usr/bin/env python3
"""E2E test for the embedded 3D twin (web viewer backend).

Run manually, no robot or bringup needed (ports 8094/50092 must be free):

    pixi run python src/telamoto_bringup/test/test_twin_e2e.py

Instantiates the REAL URServoController + a helper publisher node, verifies:
  A. page embeds the viewer (importmap, view3d container, lazy viewer.js)
  B. /api/urdf serves the latched /robot_description
  C. /static/* serves the vendored JS with module MIME + caching
  D. /pkg/<package>/<path> serves URDF meshes from the ament share dir
  E. path traversal on /pkg and /static is rejected
  F. /ws3d pushes binary twin frames: joints match /joint_states, wall box
     matches the /planning_scene CollisionObject
"""
import base64
import http.client
import importlib.util
import os
import socket
import struct
import sys
import threading
import time

_SCRIPT = os.path.join(os.path.dirname(__file__), os.pardir,
                       "scripts", "ur_servo_controller.py")
spec = importlib.util.spec_from_file_location("usc", _SCRIPT)
usc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(usc)

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene

# Full isolation from a live bringup: own ROS domain (a real stack publishes
# /joint_states at 125 Hz and would beat the test helper — and the test must
# never publish onto the live graph), and own web/reverse ports (an open
# web-UI tab reconnects to :8080 every second; its 30 Hz keepalive would
# pollute the link-jitter estimator under test).
os.environ["ROS_DOMAIN_ID"] = "78"
rclpy.init(args=["--ros-args", "-p", "web_port:=8094", "-p", "reverse_port:=50092"])
node = usc.URServoController()
helper = Node("twin_test_pub")
executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(node)
threading.Thread(target=executor.spin, daemon=True).start()
time.sleep(0.7)

Q = [0.1, -1.2, 1.5, -0.3, 1.57, 0.5]
latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Minimal URDF that still exercises the real mesh path (package:// → /pkg/…,
# ColladaLoader) and one revolute joint for setJointValue.
URDF = """<robot name='t'>
  <link name='base_link'><visual><geometry>
    <mesh filename='package://ur_description/meshes/ur10/visual/base.dae'/>
  </geometry></visual></link>
  <link name='tool0'/>
  <joint name='shoulder_pan_joint' type='revolute'>
    <parent link='base_link'/><child link='tool0'/><axis xyz='0 0 1'/>
    <limit lower='-6.28' upper='6.28' effort='1' velocity='1'/>
  </joint>
</robot>"""
helper.create_publisher(String, "/robot_description", latched).publish(String(data=URDF))
js_pub = helper.create_publisher(JointState, "/joint_states", 10)
sc_pub = helper.create_publisher(PlanningScene, "/planning_scene", 10)

js = JointState(name=list(usc.UR_JOINT_ORDER), position=Q)
wall = CollisionObject(id="test_wall")
wall.header.frame_id = "world"
wall.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[0.05, 1.2, 1.2])]
pose = Pose()
pose.position.x, pose.position.z, pose.orientation.w = 0.8, 0.6, 1.0
wall.primitive_poses = [pose]
scene = PlanningScene(is_diff=True)
scene.world.collision_objects = [wall]
for _ in range(10):                       # ride over discovery
    js_pub.publish(js)
    sc_pub.publish(scene)
    time.sleep(0.1)

fails = 0
def check(name, cond, detail=""):
    global fails
    fails += 0 if cond else 1
    print(("PASS  " if cond else "FAIL  ") + f"{name:34s} {detail}", flush=True)

def get(path):
    c = http.client.HTTPConnection("127.0.0.1", 8094, timeout=5)
    c.request("GET", path)               # raw path — no client-side normalizing
    r = c.getresponse()
    body = r.read()
    c.close()
    return r.status, dict((k.lower(), v) for k, v in r.getheaders()), body

st, _, body = get("/")
check("A page embeds viewer",
      st == 200 and b"importmap" in body and b"view3d" in body and b"viewer.js" in body)
check("A mobile jog pads + tuning panel",
      all(f'data-k="{k}"'.encode() in body for k in "wasdqe")
      and b'id="pads"' in body and b'id="panelbtn"' in body and b'id="panel"' in body)
check("A fullscreen bar + deep links (tilt pad removed)",
      b'id="fsbar"' in body and b'id="fsbtn"' in body and b'id="jb-t"' not in body
      and b'location.hash==="#fs"' in body and b"deviceorientation" not in body)
check("A wall-visibility toggles",
      b'id="wallbtn"' in body and b'id="fswalls"' in body)
# The UI is plain-HTTP-only by design: the page must carry the https→http
# bounce with the REAL configured port injected (8094 here, not the 8080
# default), every WebSocket must be pinned to ws://, and a TLS handshake
# against the port must be closed immediately (that fast failure is what
# makes a browser's HTTPS-First mode fall back to http).
check("A https bounce with injected port",
      b'location.protocol==="https:"' in body
      and b':8094"+location.pathname' in body and b"__WEB_PORT__" not in body)
check("A WebSockets pinned to ws://",
      b'"ws://"+location.host' in body and b"wss://" not in body
      and b"'ws://' + location.host" in get("/static/viewer.js")[2])
tls = socket.create_connection(("127.0.0.1", 8094), timeout=5)
tls.sendall(bytes([0x16, 0x03, 0x01, 0x00, 0x05]) + b"hello")  # fake ClientHello
tls.settimeout(3)
try:
    closed = tls.recv(64) == b""
except socket.timeout:
    closed = False
tls.close()
check("A TLS probe refused instantly", closed,
      "(plain server must close, not answer, a ClientHello)")
# Abrupt mid-request reset (internet scanners do this constantly): must be
# swallowed silently — socketserver used to dump a full traceback — and the
# server must stay healthy.
rst = socket.create_connection(("127.0.0.1", 8094), timeout=5)
rst.sendall(b"GET / HTTP/1.1\r\nHost: x")                     # partial request…
rst.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack("ii", 1, 0))
rst.close()                                                   # …then a hard RST
time.sleep(0.3)
check("A survives mid-request RST", get("/")[0] == 200)

st, _, body = get("/api/urdf")
check("B /api/urdf", st == 200 and b"<robot name='t'>" in body, f"status={st}")
check("B deep link #3d in page", b'location.hash==="#3d"' in get("/")[2])

st, h, body = get("/static/viewer.js")
check("C viewer.js served UNcached", st == 200
      and h.get("content-type") == "text/javascript"
      and "max-age" not in h.get("cache-control", "") and b"initTwin" in body,
      "(our code: a day-stale cached copy broke new page buttons once)")
st, h, body = get("/static/three.module.js")
check("C three.module.js cached", st == 200 and len(body) > 1_000_000
      and "max-age" in h.get("cache-control", ""), f"{len(body)} B")

st, h, body = get("/pkg/ur_description/meshes/ur10/visual/base.dae")
check("D /pkg mesh serve", st == 200 and b"COLLADA" in body[:200],
      f"status={st} {len(body)} B {h.get('content-type')}")

st1, _, b1 = get("/pkg/ur_description/../../../../../../etc/passwd")
st2, _, b2 = get("/static/../../scripts/ur_servo_controller.py")
st3, _, _ = get("/pkg/ur_description/..%2f..%2fpasswd")
check("E traversal rejected", st1 == 404 and st2 == 404 and st3 == 404
      and b"root:" not in b1 and b"URServoController" not in b2,
      f"{st1}/{st2}/{st3}")

# F ── /ws3d binary stream ──────────────────────────────────────────────────
s = socket.create_connection(("127.0.0.1", 8094), timeout=5)
key = base64.b64encode(os.urandom(16)).decode()
s.sendall((f"GET /ws3d HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\n"
           f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
           f"Sec-WebSocket-Version: 13\r\n\r\n").encode())
buf = b""
while b"\r\n\r\n" not in buf:
    buf += s.recv(1024)
def ws_recv():
    hdr = s.recv(2)
    n = hdr[1] & 0x7f                     # server frames are unmasked
    if n == 126:
        n = struct.unpack(">H", s.recv(2))[0]
    data = b""
    while len(data) < n:
        data += s.recv(n - len(data))
    return hdr[0] & 0x0f, data
op, frame = ws_recv()
ok = op == 0x2 and frame[0] == 1
qf = struct.unpack("<6f", frame[1:25])
nbox = frame[25]
check("F twin frame joints", ok and all(abs(a - b) < 1e-5 for a, b in zip(qf, Q)),
      f"op={op} q={[round(v, 2) for v in qf]}")
bx = struct.unpack("<10f", frame[26:66]) if nbox == 1 else None
check("F twin frame wall box", nbox == 1 and bx is not None
      and abs(bx[0] - 0.8) < 1e-5 and abs(bx[9] - 1.2) < 1e-5,
      f"boxes={nbox} box={[round(v, 2) for v in bx] if bx else None}")
# stream keeps flowing at ~25 Hz
t0 = time.monotonic()
for _ in range(5):
    ws_recv()
hz = 5 / (time.monotonic() - t0)
check("F stream rate ~25 Hz", 15 < hz < 40, f"{hz:.0f} Hz")

# G ── real browser: headless Chromium loads /#3d, runs the ES-module graph
# (importmap → three/urdf-loader/viewer), fetches URDF + mesh, renders.
# Success = a <canvas> in the DOM and the "waiting for robot_description"
# overlay cleared. Skipped (not failed) if no Chromium on this host.
import shutil
import subprocess
chrome = next((c for c in ("chromium", "chromium-browser", "google-chrome",
                           "google-chrome-stable") if shutil.which(c)), None)
if chrome:
    # --enable-unsafe-swiftshader: software WebGL — with --disable-gpu the
    # WebGLRenderer constructor throws on headless hosts and no canvas appears.
    cmd = [chrome, "--headless=new", "--no-sandbox", "--enable-unsafe-swiftshader",
           "--virtual-time-budget=10000", "--enable-logging=stderr", "--v=0",
           "--dump-dom", "http://127.0.0.1:8094/#3d"]
    try:
        r = subprocess.run(cmd, capture_output=True, timeout=60)
        dom = r.stdout.decode("utf8", "replace")
        log = r.stderr.decode("utf8", "replace")
        check("G browser renders twin",
              "<canvas" in dom and "ColladaLoader: File version" in log,
              f"canvas={'<canvas' in dom} mesh-parsed="
              f"{'ColladaLoader: File version' in log}")
    except subprocess.TimeoutExpired:
        check("G browser renders twin", False, "chromium timed out")
else:
    print("SKIP  G browser renders twin (no chromium found)", flush=True)

print("RESULT:", "ALL PASS" if fails == 0 else f"{fails} FAILURES", flush=True)
sys.stdout.flush()
os._exit(fails)
