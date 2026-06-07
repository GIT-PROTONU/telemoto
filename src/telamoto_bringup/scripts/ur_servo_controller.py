#!/usr/bin/env python3
"""
FollowJointTrajectory action server that drives a UR10 CB3 via the External
Control URCap — no ur_robot_driver, no RTDE inputs (PolyScope owns every RTDE
input register on this CB3 and there's no Remote Control mode to release them).

Flow: pressing Play on the pendant makes the URCap connect to REVERSE_PORT and
send "request_program"; we reply with a servoj URScript that connects back to
the same port and then PULLS one target per cycle (sends a 4-byte request, reads
a reply, servoj()s). Robot-paced pull keeps servoj gap-free (no vibration) with
no TCP backlog, and we stream the measured cycle time as servoj's duration so
commanded velocity == planned velocity despite jitter (else → protective stops).

A tiny web UI on :8080 tunes Speed / Stiffness (gain) / Smoothness (lookahead).

Reply packet — 11 × int32 big-endian, read as p[1..11]:
  p[1] timeout_ms · p[2..7] q×MULT · p[8] mode(1=servoj) · p[9] gain ·
  p[10] lookahead×1000 (ms) · p[11] servoj duration×1e6 (µs)
"""
import json
import socket
import struct
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory
from moveit_msgs.srv import ServoCommandType
from rcl_interfaces.msg import (
    FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult,
)

UR_JOINT_ORDER = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
REVERSE_PORT = 50001        # URCap requests the script here AND robot connects back
WEB_PORT     = 8080
MULT         = 1_000_000
MODE_SERVOJ  = 1
TIMEOUT_MS   = 200          # robot counts a missed read after this long
# servoj-safe ranges (UR limits) for clamping + the web sliders.
GAIN_MIN, GAIN_MAX           = 100, 2000
LOOKAHEAD_MIN, LOOKAHEAD_MAX = 0.03, 0.2
SPEED_MIN, SPEED_MAX         = 0.25, 3.0

# WASD jogging via MoveIt Servo. Web sends per-axis direction in [-1,1]; we
# scale to these speeds and publish a TwistStamped in JOG_FRAME. The robot
# stops if no jog command arrives within JOG_DEADMAN (browser-crash safety).
JOG_LINEAR  = 0.12          # m/s at full axis
JOG_ANGULAR = 0.5           # rad/s at full axis
JOG_FRAME   = "tool0"       # jog in the TOOL frame (Servo transforms the twist);
                            # use "base_link" for base-frame jogging instead
JOG_DEADMAN = 0.3           # s
SERVO_TWIST_TOPIC = "/servo_node/delta_twist_cmds"
SERVO_OUT_TOPIC   = "/telamoto/servo_command"
SERVO_TYPE_SRV    = "/servo_node/switch_command_type"

# Served on request_program. The "# HEADER_*" anchors are mandatory (the URCap
# splits header/body); we keep the header empty. A single missed read is
# tolerated — the loop exits only after ~10 s of real silence.
_URSCRIPT = """\
# HEADER_BEGIN
# HEADER_END
textmsg("telamoto: external control active")
socket_open("{host}", {port}, "reverse_socket")
read_timeout = 0
misses = 0
keep_going = True
while keep_going:
  socket_send_int(1, "reverse_socket")
  p = socket_read_binary_integer(11, "reverse_socket", read_timeout)
  if p[0] > 0:
    misses = 0
    read_timeout = p[1] / 1000.0
    if p[8] == 1:
      q = [p[2]/1000000.0, p[3]/1000000.0, p[4]/1000000.0, p[5]/1000000.0, p[6]/1000000.0, p[7]/1000000.0]
      servoj(q, t=p[11]/1000000.0, lookahead_time=p[10]/1000.0, gain=p[9])
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

_WEB_PAGE = """<!doctype html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Telamoto motion tuning</title>
<style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:auto;padding:1.2rem;max-width:560px}
 h1{font-size:1.25rem;font-weight:600}
 #status{font-size:.85rem;padding:.3rem .6rem;border-radius:.4rem;display:inline-block;margin-bottom:.5rem}
 .ok{background:#13391b;color:#6ee787}.bad{background:#3a1414;color:#ff7b72}
 .row{margin:1.5rem 0}
 .row label{display:flex;justify-content:space-between;font-size:1.05rem;margin-bottom:.45rem}
 .val{color:#4ea1ff;font-variant-numeric:tabular-nums}
 input[type=range]{width:100%;height:2.2rem}
 .hint{color:#888;font-size:.8rem;margin-top:.25rem}
 button{background:#222;color:#eee;border:1px solid #444;border-radius:.5rem;padding:.6rem 1.1rem;font-size:.9rem;margin-top:.6rem}
</style></head>
<body>
 <h1>Telamoto &mdash; motion tuning</h1>
 <div id="status" class="bad">connecting&hellip;</div>
 <div class="row"><label>Speed <span class="val" id="speedv"></span></label>
   <input type="range" id="speed" min="0.25" max="3" step="0.05">
   <div class="hint">Overall move speed (also scales acceleration). Next move.</div></div>
 <div class="row"><label>Stiffness <span class="val" id="gainv"></span></label>
   <input type="range" id="gain" min="100" max="2000" step="25">
   <div class="hint">Higher = crisper tracking, lower = softer. Live.</div></div>
 <div class="row"><label>Smoothness <span class="val" id="lookaheadv"></span></label>
   <input type="range" id="lookahead" min="0.03" max="0.2" step="0.005">
   <div class="hint">Higher = smoother but more lag; raise if it buzzes. Live.</div></div>
 <button onclick="reset()">Reset to defaults</button>
 <div class="row" style="border-top:1px solid #333;padding-top:1.2rem;margin-top:1.2rem">
   <label>Jog (WASD) <span><input type="checkbox" id="jogon"> enable</span></label>
   <div class="hint"><b>W/S</b> forward/back along the tool &middot; <b>A/D</b> across &middot;
     <b>Q/E</b> across (tool frame). Hold to move, release to stop. Robot must be connected.</div>
 </div>
<script>
 const fmt={speed:v=>(+v).toFixed(2)+"\\u00d7",gain:v=>Math.round(v),lookahead:v=>(+v).toFixed(3)+" s"};
 const ids=["speed","gain","lookahead"];
 function show(k,v){document.getElementById(k).value=v;document.getElementById(k+"v").textContent=fmt[k](v);}
 function send(k){const v=document.getElementById(k).value;
   document.getElementById(k+"v").textContent=fmt[k](v);fetch("/api/set?"+k+"="+v,{method:"POST"});}
 async function refreshStatus(){try{const s=await(await fetch("/api/state")).json();
   const st=document.getElementById("status");
   st.textContent=s.connected?"robot connected":"robot not connected \\u2014 press Play on the pendant";
   st.className=s.connected?"ok":"bad";}catch(e){}}
 async function init(){try{const s=await(await fetch("/api/state")).json();ids.forEach(k=>show(k,s[k]));}catch(e){}
   ids.forEach(k=>document.getElementById(k).addEventListener("input",()=>send(k)));
   refreshStatus();setInterval(refreshStatus,2000);}
 function reset(){show("speed",1);show("gain",300);show("lookahead",0.1);
   fetch("/api/set?speed=1&gain=300&lookahead=0.1",{method:"POST"});}
 // WASD jog
 let jogOn=false; const held=new Set(); let hb=null;
 // tool frame: Z = along the tool (forward/back), X/Y = across the flange.
 function jogVec(){return {lx:(held.has("q")?1:0)-(held.has("e")?1:0),
   ly:(held.has("a")?1:0)-(held.has("d")?1:0),lz:(held.has("w")?1:0)-(held.has("s")?1:0)};}
 function sendJog(){const v=jogVec();
   fetch("/api/jog?lx="+v.lx+"&ly="+v.ly+"&lz="+v.lz,{method:"POST"});}
 function stopJog(){held.clear();if(hb){clearInterval(hb);hb=null;}sendJog();}
 document.getElementById("jogon").addEventListener("change",e=>{jogOn=e.target.checked;if(!jogOn)stopJog();});
 document.addEventListener("keydown",e=>{if(!jogOn)return;const k=e.key.toLowerCase();
   if("wasdqe".includes(k)&&!held.has(k)){held.add(k);e.preventDefault();sendJog();if(!hb)hb=setInterval(sendJog,100);}});
 document.addEventListener("keyup",e=>{if(!jogOn)return;const k=e.key.toLowerCase();
   if(held.has(k)){held.delete(k);sendJog();if(held.size===0&&hb){clearInterval(hb);hb=null;}}});
 window.addEventListener("blur",()=>{if(jogOn)stopJog();});
 init();
</script>
</body></html>
"""


def _pack(q, gain, lookahead_s, t_s) -> bytes:
    return struct.pack(">11i",
        TIMEOUT_MS,
        *[int(x * MULT) for x in q],
        MODE_SERVOJ, int(gain),
        int(round(lookahead_s * 1000)), int(round(t_s * 1_000_000)),
    )


def _clamp(lo, hi, v):
    return max(lo, min(hi, v))


def _result(code):
    r = FollowJointTrajectory.Result()
    r.error_code = code
    return r


class URServoController(Node):

    def __init__(self) -> None:
        super().__init__("ur_servo_controller")
        self.declare_parameter("robot_ip", "192.168.10.2")
        self.declare_parameter("reverse_port", REVERSE_PORT)
        self.declare_parameter("web_port", WEB_PORT)
        # gain/lookahead stream in every packet → tunable live (web UI / params).
        self.declare_parameter("servoj_gain", 300, ParameterDescriptor(
            description="servoj gain / stiffness (live)",
            integer_range=[IntegerRange(from_value=GAIN_MIN, to_value=GAIN_MAX, step=10)]))
        self.declare_parameter("servoj_lookahead", 0.1, ParameterDescriptor(
            description="servoj lookahead s; higher = smoother/laggier (live)",
            floating_point_range=[FloatingPointRange(
                from_value=LOOKAHEAD_MIN, to_value=LOOKAHEAD_MAX, step=0.005)]))
        self.declare_parameter("speed_scale", 1.0)   # trajectory time-scale (next move)

        g = self.get_parameter
        self._robot_ip = g("robot_ip").get_parameter_value().string_value
        self._port     = g("reverse_port").get_parameter_value().integer_value
        self._web_port = g("web_port").get_parameter_value().integer_value
        self._gain      = g("servoj_gain").get_parameter_value().integer_value
        self._lookahead = g("servoj_lookahead").get_parameter_value().double_value
        self._speed     = g("speed_scale").get_parameter_value().double_value
        self.add_on_set_parameters_callback(self._on_set_params)

        self._js_ready = threading.Event()           # set on first JointState
        self._q_target = [0.0] * 6                    # held pose when idle
        self._tgt_lock = threading.Lock()
        self._traj = None                             # (waypoints, total_time, t_start)
        self._traj_lock = threading.Lock()
        self._traj_done = threading.Event()
        self._conn: socket.socket | None = None       # live reverse socket
        self._conn_lock = threading.Lock()

        # WASD jog (MoveIt Servo): _jog is the desired twist, valid until _jog_deadline.
        self._jog = [0.0] * 6
        self._jog_deadline = 0.0
        self._servo_started = False

        self._js_sub = self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        # MoveIt Servo's JointTrajectory output → our stream (during jog).
        self.create_subscription(JointTrajectory, SERVO_OUT_TOPIC, self._servo_cb, 10)
        self._twist_pub = self.create_publisher(TwistStamped, SERVO_TWIST_TOPIC, 10)
        self._servo_cli = self.create_client(ServoCommandType, SERVO_TYPE_SRV)
        ActionServer(
            self, FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
            execute_callback=self._execute_cb,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )
        for fn, name in ((self._server_loop, "srv"), (self._control_loop, "ctrl"),
                         (self._web_loop, "web"), (self._jog_loop, "jog")):
            threading.Thread(target=fn, daemon=True, name="ri-" + name).start()
        self.get_logger().info(
            f"URServoController: reverse :{self._port}, robot {self._robot_ip} "
            f"— press Play on the pendant to start")

    # ── Live tuning (params + web) ──────────────────────────────────────────────

    def _on_set_params(self, params) -> SetParametersResult:
        for p in params:
            if p.name == "servoj_gain":
                self._gain = int(_clamp(GAIN_MIN, GAIN_MAX, p.value))
            elif p.name == "servoj_lookahead":
                self._lookahead = float(_clamp(LOOKAHEAD_MIN, LOOKAHEAD_MAX, p.value))
            elif p.name == "speed_scale":
                self._speed = float(_clamp(SPEED_MIN, SPEED_MAX, p.value))
        return SetParametersResult(successful=True)

    def _web_set(self, q: dict) -> None:
        if "speed" in q:     self._speed = _clamp(SPEED_MIN, SPEED_MAX, float(q["speed"][0]))
        if "gain" in q:      self._gain = int(_clamp(GAIN_MIN, GAIN_MAX, float(q["gain"][0])))
        if "lookahead" in q: self._lookahead = _clamp(LOOKAHEAD_MIN, LOOKAHEAD_MAX, float(q["lookahead"][0]))

    def _web_jog(self, q: dict) -> None:
        ax = lambda k: _clamp(-1.0, 1.0, float(q.get(k, ["0"])[0]))
        self._jog = [JOG_LINEAR * ax("lx"), JOG_LINEAR * ax("ly"), JOG_LINEAR * ax("lz"),
                     JOG_ANGULAR * ax("ax"), JOG_ANGULAR * ax("ay"), JOG_ANGULAR * ax("az")]
        self._jog_deadline = time.monotonic() + JOG_DEADMAN

    # ── WASD jog: bridge web → MoveIt Servo → our stream ────────────────────────

    def _servo_cb(self, msg: JointTrajectory) -> None:
        # Servo's joint command becomes our target — but only while jogging and
        # not during a planned move (which the control loop prioritises anyway).
        with self._traj_lock:
            if self._traj is not None:
                return
        if not msg.points:
            return
        try:
            q = self._reorder(list(msg.joint_names), list(msg.points[0].positions))
        except ValueError:
            return
        with self._tgt_lock:
            self._q_target = q

    def _jog_loop(self) -> None:
        self._js_ready.wait()
        while rclpy.ok():
            time.sleep(0.02)                                  # 50 Hz
            if time.monotonic() >= self._jog_deadline or not any(self._jog):
                continue
            if not self._servo_started:                       # one-time: select TWIST mode
                if not self._servo_cli.service_is_ready():
                    continue
                req = ServoCommandType.Request()
                req.command_type = ServoCommandType.Request.TWIST
                self._servo_cli.call_async(req)
                self._servo_started = True
                self.get_logger().info("[jog] MoveIt Servo command type → TWIST")
            m = TwistStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = JOG_FRAME
            m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = self._jog[0:3]
            m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = self._jog[3:6]
            self._twist_pub.publish(m)

    def _web_loop(self) -> None:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_): pass

            def _send(self, body, ctype="text/plain"):
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                with node._conn_lock:
                    connected = node._conn is not None
                if self.path.startswith("/api/state"):
                    self._send(json.dumps({"speed": round(node._speed, 2), "gain": node._gain,
                        "lookahead": round(node._lookahead, 3), "connected": connected}).encode(),
                        "application/json")
                else:
                    self._send(_WEB_PAGE.encode(), "text/html; charset=utf-8")

            def do_POST(self):
                q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
                try:
                    if self.path.startswith("/api/set"):
                        node._web_set(q)
                    elif self.path.startswith("/api/jog"):
                        node._web_jog(q)
                except (ValueError, KeyError):
                    pass
                self._send(b"ok")

        try:
            srv = ThreadingHTTPServer(("0.0.0.0", self._web_port), Handler)
        except Exception as exc:
            self.get_logger().warn(f"[web] could not start on :{self._web_port}: {exc}")
            return
        self.get_logger().info(f"[web] tuning UI at http://{self._pc_ip()}:{self._web_port}")
        srv.serve_forever()

    # ── Joint states (only the first sample is needed to seed the hold pose) ─────

    def _js_cb(self, msg: JointState) -> None:
        ntp = dict(zip(msg.name, msg.position))
        try:
            q = [ntp[j] for j in UR_JOINT_ORDER]
        except KeyError:
            return
        with self._tgt_lock:
            self._q_target = list(q)
        if not self._js_ready.is_set():
            self._js_ready.set()
            self.get_logger().info("[ctrl] first joint states received")
            self.destroy_subscription(self._js_sub)   # avoid 125 Hz GIL contention

    # ── Reverse server: serves the script on request, then hands the socket to
    #    the control loop (this robot's URCap Custom Port == the reverse port) ───

    def _pc_ip(self) -> str:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect((self._robot_ip, self._port))
            return s.getsockname()[0]

    def _serve_script(self, conn, addr) -> None:
        host = self._pc_ip()
        conn.sendall(_URSCRIPT.format(host=host, port=self._port).encode())
        self.get_logger().info(f"[script] served to {addr[0]} (connect-back {host}:{self._port})")

    def _server_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self._port))
        srv.listen(1)
        self.get_logger().info(f"[server] listening on :{self._port}")
        while rclpy.ok():
            try:
                conn, addr = srv.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                # The URCap sends "request_program" (text); the control script
                # sends a binary request int. Disambiguate by peeking.
                conn.settimeout(2.0)
                try:
                    is_request = conn.recv(32, socket.MSG_PEEK).startswith(b"request_program")
                except Exception:
                    is_request = False
                if is_request:
                    with conn:
                        conn.settimeout(5)
                        try:
                            while conn.recv(1) not in (b"", b"\n"):  # consume request line
                                pass
                            self._serve_script(conn, addr)
                        except Exception as exc:
                            self.get_logger().warn(f"[script] serve failed: {exc}")
                    continue
                self.get_logger().info(f"[server] robot connected from {addr[0]}")
                with self._conn_lock:
                    old, self._conn = self._conn, conn
                if old:
                    try: old.close()
                    except Exception: pass
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().warn(f"[server] accept error: {exc}")

    @staticmethod
    def _recvall(conn, n) -> bytes | None:
        buf = b""
        while len(buf) < n:
            chunk = conn.recv(n - len(buf))
            if not chunk:
                return None
            buf += chunk
        return buf

    def _drop(self, conn) -> None:
        with self._conn_lock:
            if self._conn is conn:
                self._conn = None
        try: conn.close()
        except Exception: pass
        self.get_logger().warn("[server] robot disconnected — press Play to resume")

    # ── Control loop: robot-paced (request → reply) ─────────────────────────────

    @staticmethod
    def _interp(waypoints, t_now: float) -> list[float]:
        """Cubic Hermite (position + velocity) → C1-continuous; linear if MoveIt
        gave no velocities."""
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
        s = _clamp(0.0, 1.0, (t_now - t0) / h)
        if v0 is None or v1 is None:
            return [q0[j] + s * (q1[j] - q0[j]) for j in range(6)]
        s2, s3 = s * s, s * s * s
        h00, h10, h01, h11 = 2*s3 - 3*s2 + 1, s3 - 2*s2 + s, -2*s3 + 3*s2, s3 - s2
        return [h00*q0[j] + h10*h*v0[j] + h01*q1[j] + h11*h*v1[j] for j in range(6)]

    def _next_target(self, now: float) -> list[float]:
        with self._traj_lock:
            traj = self._traj
        if traj is None:
            with self._tgt_lock:
                return list(self._q_target)
        waypoints, total_time, t_start = traj
        if now - t_start >= total_time:
            q = list(waypoints[-1][1])
            with self._traj_lock:
                if self._traj is traj:
                    self._traj = None
            self._traj_done.set()
        else:
            q = self._interp(waypoints, now - t_start)
        with self._tgt_lock:
            self._q_target = q
        return q

    def _control_loop(self) -> None:
        self._js_ready.wait()
        first = True
        health_t = last_req = time.monotonic()
        cycles = 0
        max_gap = 0.0
        while rclpy.ok():
            with self._conn_lock:
                conn = self._conn
            if conn is None:
                first = True
                time.sleep(0.01)
                continue
            try:                                  # block until the robot requests
                conn.settimeout(1.0)
                req = self._recvall(conn, 4)
            except socket.timeout:
                continue
            except Exception:
                self._drop(conn); continue
            if req is None:
                self._drop(conn); continue

            now = time.monotonic()
            dt = now - last_req                   # measured cycle → servoj duration
            last_req = now
            cycles += 1; max_gap = max(max_gap, dt)
            if now - health_t >= 5.0:
                self.get_logger().info(
                    f"[ctrl] {cycles} cycles/5s, max gap {max_gap*1000:.1f}ms")
                cycles = 0; max_gap = 0.0; health_t = now

            q = self._next_target(now)
            # servoj duration = real cycle time → commanded velocity == planned
            # (clamp only extreme outliers; never below dt, which would spike it).
            try:
                conn.sendall(_pack(q, self._gain, self._lookahead, _clamp(0.002, 0.5, dt)))
                if first:
                    first = False
                    self.get_logger().info("[ctrl] streaming to robot")
            except Exception:
                self._drop(conn)

    # ── FollowJointTrajectory action ────────────────────────────────────────────

    @staticmethod
    def _reorder(names, values) -> list[float]:
        try:
            return [values[names.index(j)] for j in UR_JOINT_ORDER]
        except (ValueError, IndexError) as e:
            raise ValueError(f"trajectory missing UR joints {names}: {e}") from e

    def _execute_cb(self, goal_handle):
        Res = FollowJointTrajectory.Result
        traj = goal_handle.request.trajectory
        if not traj.points:
            goal_handle.abort(); return _result(Res.INVALID_GOAL)
        try:
            names = list(traj.joint_names)
            waypoints = [(
                pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9,
                self._reorder(names, list(pt.positions)),
                self._reorder(names, list(pt.velocities)) if pt.velocities else None,
            ) for pt in traj.points]
        except ValueError as e:
            self.get_logger().error(f"[exec] {e}")
            goal_handle.abort(); return _result(Res.INVALID_JOINTS)

        # "Speed": time-scale (compress time, scale velocities to match).
        speed = _clamp(SPEED_MIN, SPEED_MAX, self._speed)
        if abs(speed - 1.0) > 1e-3:
            waypoints = [(t / speed, q, [vi * speed for vi in v] if v else None)
                         for (t, q, v) in waypoints]
        total_time = waypoints[-1][0]

        with self._conn_lock:
            connected = self._conn is not None
        self.get_logger().info(
            f"[exec] {len(waypoints)} pts, {total_time:.2f}s, connected={connected}")
        if not connected:   # the loop only advances while the robot is pulling
            self.get_logger().warn("[exec] robot not connected — press Play")
            goal_handle.abort(); return _result(Res.INVALID_GOAL)

        self._traj_done.clear()
        with self._traj_lock:
            self._traj = (waypoints, total_time, time.monotonic())
        while not self._traj_done.wait(timeout=0.05):
            if not rclpy.ok():
                return Res()
            if goal_handle.is_cancel_requested:
                with self._traj_lock: self._traj = None
                goal_handle.canceled()
                self.get_logger().info("[exec] canceled")
                return Res()
            with self._conn_lock:
                lost = self._conn is None
            if lost:
                with self._traj_lock: self._traj = None
                self.get_logger().warn("[exec] robot disconnected mid-move")
                goal_handle.abort(); return _result(Res.PATH_TOLERANCE_VIOLATED)

        goal_handle.succeed()
        self.get_logger().info("[exec] complete")
        return _result(Res.SUCCESSFUL)


def main() -> None:
    rclpy.init()
    node = URServoController()
    # 2 threads: one for the blocking execute_cb, one for goal cancellation —
    # fewer than the default (one per CPU) to spare the control-loop thread.
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
