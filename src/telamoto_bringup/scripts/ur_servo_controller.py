#!/usr/bin/env python3
"""
FollowJointTrajectory action server that drives a UR10 CB3 through the
**External Control URCap** — no ur_robot_driver, no RTDE inputs.

Why this design
===============
ur_robot_driver cannot run on this CB3: PolyScope owns every RTDE *input*
register (speed_slider, all digital/analog outputs) and there is no Remote
Control mode to release them, so the hardware interface always aborts at
configure ("Variable '...' is currently controlled by another RTDE client";
an empty recipe is rejected too). This node never touches RTDE inputs — it
speaks the External Control URCap's script protocol over plain TCP.

How the External Control URCap works (FZI externalcontrol-1.0.5)
===============================================================
When the External Control program node runs, the URCap:
  1. connects to the configured Host IP : Custom Port,
  2. sends the literal request  "request_program",
  3. reads a URScript back until the socket closes,
  4. splits it on the "# HEADER_BEGIN" / "# HEADER_END" anchors and runs it.

So this node:
  1. Serves the servoj URScript on request_program — on SCRIPT_SENDER_PORT
     (50002) and, because this robot's URCap Custom Port is set to the reverse
     port, also on REVERSE_PORT (50001) via a first-bytes peek.
  2. Reverse server on REVERSE_PORT — the running URScript connects back here
     and we stream 8×int32 packets that the robot feeds to servoj().
  3. Start it by pressing Play on the pendant. A real GUI compile makes the
     URCap re-fetch our script; a dashboard "play" would reuse its cached copy,
     so dashboard auto-play is intentionally not implemented.
  4. Serves a tiny web UI (port 8080) to tune motion live — Speed (trajectory
     time-scale), Stiffness (servoj gain) and Smoothness (servoj lookahead).

Packet format — 10 × int32 big-endian
  params[0]      count          always 10 (prepended by the URScript runtime)
  params[1]      timeout_ms     robot sets read_timeout = this / 1000.0
  params[2..7]   q[0..5] × MULT
  params[8]      control_mode   1 == MODE_SERVOJ
  params[9]      servoj gain               live-tunable (rqt_reconfigure)
  params[10]     servoj lookahead × 1000   live-tunable, milliseconds
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
from rcl_interfaces.msg import (
    FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult,
)

# servoj safe ranges (UR servoj limits) used for clamping + the web sliders.
GAIN_MIN, GAIN_MAX           = 100, 2000
LOOKAHEAD_MIN, LOOKAHEAD_MAX = 0.03, 0.2
SPEED_MIN, SPEED_MAX         = 0.25, 3.0   # time-scale of executed trajectories
WEB_PORT                     = 8080

UR_JOINT_ORDER = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]

REVERSE_PORT       = 50001   # robot's running URScript connects back here
SCRIPT_SENDER_PORT = 50002   # External Control URCap requests the script here
MULT               = 1_000_000
MODE_SERVOJ        = 1
TIMEOUT_MS         = 200      # robot counts a missed read if no packet for this long
STEP_TIME          = 0.010   # 100 Hz send — deliberately a touch SLOWER than the
                             # robot's servoj consume rate so packets never back up
                             # in the TCP buffer (a backlog caused growing lag and
                             # the robot "stopping" until the program was re-played).

# Control URScript served on request_program. The "# HEADER_*" anchors are
# mandatory: the URCap splits global definitions (header) from the body it
# injects at the program node; we keep the header empty and the body
# self-contained. A single missed read is NOT a disconnect (the non-realtime
# sender can stall briefly) — the body only exits after ~10 s of real silence.
_URSCRIPT = """\
# HEADER_BEGIN
# telamoto external control — no global definitions required
# HEADER_END
textmsg("telamoto: external control active")
socket_open("{host}", {port}, "reverse_socket")
read_timeout = 0
misses = 0
keep_going = True
while keep_going:
  p = socket_read_binary_integer(10, "reverse_socket", read_timeout)
  if p[0] > 0:
    misses = 0
    read_timeout = p[1] / 1000.0
    if p[8] == 1:
      q = [p[2]/1000000.0, p[3]/1000000.0, p[4]/1000000.0, p[5]/1000000.0, p[6]/1000000.0, p[7]/1000000.0]
      # gain (p[9]) and lookahead (p[10] ms) are streamed live so they can be
      # tuned from the PC without re-playing the program.
      servoj(q, t={servoj_time}, lookahead_time=p[10]/1000.0, gain=p[9])
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
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:0;padding:1.2rem;max-width:560px;margin:auto}
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
 <div class="row">
   <label>Speed <span class="val" id="speedv"></span></label>
   <input type="range" id="speed" min="0.25" max="3" step="0.05">
   <div class="hint">Overall move speed (also scales acceleration). Takes effect on the next move.</div>
 </div>
 <div class="row">
   <label>Stiffness <span class="val" id="gainv"></span></label>
   <input type="range" id="gain" min="100" max="2000" step="25">
   <div class="hint">Higher = crisper/stiffer tracking, lower = softer. Live.</div>
 </div>
 <div class="row">
   <label>Smoothness <span class="val" id="lookaheadv"></span></label>
   <input type="range" id="lookahead" min="0.03" max="0.2" step="0.005">
   <div class="hint">Higher = smoother but more lag; raise this if it buzzes. Live.</div>
 </div>
 <button onclick="reset()">Reset to defaults</button>
<script>
 const fmt={speed:v=>(+v).toFixed(2)+"\\u00d7",gain:v=>Math.round(v),lookahead:v=>(+v).toFixed(3)+" s"};
 const ids=["speed","gain","lookahead"];
 function show(k,v){document.getElementById(k).value=v;document.getElementById(k+"v").textContent=fmt[k](v);}
 function send(k){const v=document.getElementById(k).value;
   document.getElementById(k+"v").textContent=fmt[k](v);
   fetch("/api/set?"+k+"="+v,{method:"POST"});}
 async function refreshStatus(){try{const s=await(await fetch("/api/state")).json();
   const st=document.getElementById("status");
   st.textContent=s.connected?"robot connected":"robot not connected \\u2014 press Play on the pendant";
   st.className=s.connected?"ok":"bad";}catch(e){}}
 async function init(){try{const s=await(await fetch("/api/state")).json();
   ids.forEach(k=>show(k,s[k]));}catch(e){}
   ids.forEach(k=>document.getElementById(k).addEventListener("input",()=>send(k)));
   refreshStatus(); setInterval(refreshStatus,2000);}
 function reset(){show("speed",1);show("gain",300);show("lookahead",0.1);
   fetch("/api/set?speed=1&gain=300&lookahead=0.1",{method:"POST"});}
 init();
</script>
</body></html>
"""


def _pack(q: list[float], gain: int, lookahead_s: float) -> bytes:
    return struct.pack(">10i",
        TIMEOUT_MS,
        int(q[0] * MULT), int(q[1] * MULT), int(q[2] * MULT),
        int(q[3] * MULT), int(q[4] * MULT), int(q[5] * MULT),
        MODE_SERVOJ,
        int(gain),
        int(round(lookahead_s * 1000)),
    )


class URServoController(Node):

    def __init__(self) -> None:
        super().__init__("ur_servo_controller")
        self.declare_parameter("robot_ip",          "192.168.10.2")
        self.declare_parameter("reverse_port",       REVERSE_PORT)
        self.declare_parameter("script_sender_port", SCRIPT_SENDER_PORT)
        # servoj_time is baked into the served script (it's coupled to the
        # STEP_TIME send rate — keep it below STEP_TIME so the robot never backs
        # up). Changing it takes effect on the next Play.
        self.declare_parameter("servoj_time", 0.008)
        # gain (stiffness) and lookahead (smoothness) are streamed in every
        # packet — tune them LIVE from the web UI (http://<pc>:8080, or
        # `pixi run tune`); effect on the next servoj cycle.
        self.declare_parameter(
            "servoj_gain", 300,
            ParameterDescriptor(
                description="servoj proportional gain / stiffness (live)",
                integer_range=[IntegerRange(from_value=GAIN_MIN, to_value=GAIN_MAX, step=10)],
            ),
        )
        self.declare_parameter(
            "servoj_lookahead", 0.1,
            ParameterDescriptor(
                description="servoj lookahead time, seconds; higher = smoother/laggier (live)",
                floating_point_range=[FloatingPointRange(
                    from_value=LOOKAHEAD_MIN, to_value=LOOKAHEAD_MAX, step=0.005)],
            ),
        )

        # Overall speed: time-scale applied to executed trajectories (1.0 = as
        # MoveIt planned). Lives in the controller so it works with RViz planning.
        self.declare_parameter("speed_scale", 1.0)
        self.declare_parameter("web_port", WEB_PORT)

        self._robot_ip    = self.get_parameter("robot_ip").get_parameter_value().string_value
        self._port        = self.get_parameter("reverse_port").get_parameter_value().integer_value
        self._sender_port = self.get_parameter("script_sender_port").get_parameter_value().integer_value
        self._servoj_time      = self.get_parameter("servoj_time").get_parameter_value().double_value
        self._servoj_gain      = self.get_parameter("servoj_gain").get_parameter_value().integer_value
        self._servoj_lookahead = self.get_parameter("servoj_lookahead").get_parameter_value().double_value
        self._speed_scale      = self.get_parameter("speed_scale").get_parameter_value().double_value
        self._web_port         = self.get_parameter("web_port").get_parameter_value().integer_value
        self.add_on_set_parameters_callback(self._on_set_params)

        # Gate: set when the first JointState arrives; the control loop waits on it.
        self._js_ready = threading.Event()

        # Command target — held position when no trajectory is active.
        self._q_target: list[float] = [0.0] * 6
        self._tgt_lock = threading.Lock()

        # Active trajectory: (waypoints, total_time, t_start) or None. The single
        # control loop interpolates AND sends, so there are no two competing
        # Python loops to jitter against.
        self._traj = None
        self._traj_lock = threading.Lock()
        self._traj_done = threading.Event()

        # Active reverse-socket connection — None when the robot is disconnected.
        self._conn: socket.socket | None = None
        self._conn_lock = threading.Lock()

        self._js_sub = self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self._action_server = ActionServer(
            self, FollowJointTrajectory,
            "joint_trajectory_controller/follow_joint_trajectory",
            execute_callback = self._execute_cb,
            goal_callback    = lambda _: GoalResponse.ACCEPT,
            cancel_callback  = lambda _: CancelResponse.ACCEPT,
            callback_group   = ReentrantCallbackGroup(),
        )

        threading.Thread(target=self._script_sender_loop,  daemon=True, name="ri-script").start()
        threading.Thread(target=self._reverse_server_loop, daemon=True, name="ri-server").start()
        threading.Thread(target=self._control_loop,        daemon=True, name="ri-ctrl").start()
        threading.Thread(target=self._web_loop,            daemon=True, name="ri-web").start()

        self.get_logger().info(
            f"URServoController: script sender :{self._sender_port}, reverse :{self._port}, "
            f"robot at {self._robot_ip} — press Play on the pendant to start"
        )

    # ── Live tuning ─────────────────────────────────────────────────────────────

    def _on_set_params(self, params) -> SetParametersResult:
        """gain/lookahead apply on the next servoj cycle (streamed in packets);
        speed on the next move; servoj_time on the next Play (baked into script)."""
        for p in params:
            if p.name == "servoj_gain":
                self._servoj_gain = int(max(GAIN_MIN, min(GAIN_MAX, p.value)))
            elif p.name == "servoj_lookahead":
                self._servoj_lookahead = float(max(LOOKAHEAD_MIN, min(LOOKAHEAD_MAX, p.value)))
            elif p.name == "speed_scale":
                self._speed_scale = float(max(SPEED_MIN, min(SPEED_MAX, p.value)))
            elif p.name == "servoj_time":
                self._servoj_time = float(p.value)
                self.get_logger().info("[tune] servoj_time set — re-Play to apply")
        return SetParametersResult(successful=True)

    # ── Web tuning UI ───────────────────────────────────────────────────────────

    def _web_state(self) -> dict:
        with self._conn_lock:
            connected = self._conn is not None
        return {
            "speed":     round(self._speed_scale, 2),
            "gain":      self._servoj_gain,
            "lookahead": round(self._servoj_lookahead, 3),
            "connected": connected,
        }

    def _web_set(self, q: dict) -> None:
        if "speed" in q:
            self._speed_scale = max(SPEED_MIN, min(SPEED_MAX, float(q["speed"][0])))
        if "gain" in q:
            self._servoj_gain = int(max(GAIN_MIN, min(GAIN_MAX, float(q["gain"][0]))))
        if "lookahead" in q:
            self._servoj_lookahead = max(LOOKAHEAD_MIN, min(LOOKAHEAD_MAX, float(q["lookahead"][0])))

    def _web_loop(self) -> None:
        node = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *_args):
                pass

            def _send(self, code, body, ctype):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path.startswith("/api/state"):
                    self._send(200, json.dumps(node._web_state()).encode(), "application/json")
                else:
                    self._send(200, _WEB_PAGE.encode("utf-8"), "text/html; charset=utf-8")

            def do_POST(self):
                if self.path.startswith("/api/set"):
                    try:
                        node._web_set(urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query))
                    except (ValueError, KeyError):
                        pass
                    self._send(200, b"ok", "text/plain")
                else:
                    self._send(404, b"not found", "text/plain")

        try:
            srv = ThreadingHTTPServer(("0.0.0.0", self._web_port), Handler)
        except Exception as exc:
            self.get_logger().warn(f"[web] could not start on :{self._web_port}: {exc}")
            return
        self.get_logger().info(
            f"[web] tuning UI at http://{self._pc_ip()}:{self._web_port}  (or http://localhost:{self._web_port})"
        )
        srv.serve_forever()

    # ── Joint states ───────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        # We only need the robot's start pose to seed the hold target. Control is
        # open-loop servoj streaming, so after the first sample we drop this
        # subscription — processing it at 125 Hz starves the GIL and makes the
        # control loop stutter. (MoveIt keeps its own /joint_states sub.)
        ntp = dict(zip(msg.name, msg.position))
        try:
            q = [ntp[j] for j in UR_JOINT_ORDER]
        except KeyError:
            return
        with self._tgt_lock:
            self._q_target = list(q)
        if not self._js_ready.is_set():
            self._js_ready.set()
            self.get_logger().info("[ctrl] first joint states received — control loop active")
            self.destroy_subscription(self._js_sub)

    # ── Script sender (External Control URCap protocol) ─────────────────────────

    @staticmethod
    def _recv_line(s: socket.socket) -> str:
        buf = bytearray()
        while True:
            ch = s.recv(1)
            if not ch or ch == b"\n":
                break
            buf += ch
        return buf.decode("utf-8", errors="replace").strip()

    def _pc_ip(self) -> str:
        """Local IP that routes to the robot (baked into the served script)."""
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect((self._robot_ip, self._port))
            return s.getsockname()[0]

    def _serve_script(self, conn: socket.socket, addr) -> None:
        # gain/lookahead are streamed per-packet (live), so only servoj_time is
        # baked into the script here.
        host   = self._pc_ip()
        script = _URSCRIPT.format(host=host, port=self._port, servoj_time=self._servoj_time)
        conn.sendall(script.encode("utf-8"))
        self.get_logger().info(
            f"[script] served control script to {addr[0]} (connect-back {host}:{self._port})"
        )

    def _script_sender_loop(self) -> None:
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind(("0.0.0.0", self._sender_port))
        srv.listen(1)
        self.get_logger().info(f"[script] listening on :{self._sender_port}")

        while rclpy.ok():
            try:
                conn, addr = srv.accept()
                with conn:
                    conn.settimeout(5)
                    try:
                        self._recv_line(conn)   # consume "request_program"
                    except Exception:
                        pass
                    self._serve_script(conn, addr)
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().warn(f"[script] error: {exc}")

    # ── Reverse server (robot's URScript connects back here) ────────────────────

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

                # This robot's URCap Custom Port == the reverse port, so a
                # request_program may arrive here too. Peek: a script request is
                # text; the robot's reverse connection sends nothing first (it
                # waits to read), so the peek times out.
                is_request = False
                try:
                    conn.settimeout(2.0)
                    is_request = conn.recv(32, socket.MSG_PEEK).startswith(b"request_program")
                except Exception:
                    pass

                if is_request:
                    try:
                        with conn:
                            conn.settimeout(5)
                            self._recv_line(conn)
                            self._serve_script(conn, addr)
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
                threading.Thread(target=self._watch_connection, args=(conn,), daemon=True).start()
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().warn(f"[server] accept error: {exc}")

    def _watch_connection(self, conn: socket.socket) -> None:
        """Block until the robot closes the reverse socket, then mark it gone.
        The program ends when the URScript exits, so recovery is a manual Play
        on the pendant (which re-fetches a fresh script)."""
        try:
            conn.settimeout(None)
            while conn.recv(256):
                pass
        except Exception:
            pass
        with self._conn_lock:
            if self._conn is conn:
                self._conn = None
        self.get_logger().warn("[server] robot disconnected — press Play on the pendant to resume")

    # ── 125 Hz control loop (single timing authority) ───────────────────────────

    @staticmethod
    def _interp(waypoints, t_now: float) -> list[float]:
        """Cubic Hermite interpolation using each waypoint's position AND
        velocity → continuous velocity (C1), no per-waypoint jerk. Falls back to
        linear if MoveIt didn't supply velocities."""
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
        first_packet = True
        next_t = time.monotonic()
        # Loop-health diagnostics: report cadence every 5 s (healthy ≈ 500
        # ticks/5s at 100 Hz, 0 late, max gap ≈ STEP_TIME).
        last_tick = next_t
        health_t  = next_t
        ticks = overruns = 0
        max_gap = 0.0
        while rclpy.ok():
            now = time.monotonic()
            gap = now - last_tick
            last_tick = now
            ticks += 1
            max_gap = max(max_gap, gap)
            if gap > STEP_TIME * 1.5:
                overruns += 1
            if now - health_t >= 5.0:
                self.get_logger().info(
                    f"[ctrl] rate health: {ticks} ticks/5s, {overruns} late, "
                    f"max gap {max_gap * 1000:.1f}ms"
                )
                ticks = overruns = 0
                max_gap = 0.0
                health_t = now

            # Interpolate the active trajectory here (not in a second loop) so
            # timing stays uniform; otherwise hold the last target.
            with self._traj_lock:
                traj = self._traj
            if traj is not None:
                waypoints, total_time, t_start = traj
                if now - t_start >= total_time:
                    q = list(waypoints[-1][1])
                    with self._tgt_lock:
                        self._q_target = q
                    with self._traj_lock:
                        if self._traj is traj:
                            self._traj = None
                    self._traj_done.set()
                else:
                    q = self._interp(waypoints, now - t_start)
                    with self._tgt_lock:
                        self._q_target = q
            else:
                with self._tgt_lock:
                    q = list(self._q_target)

            with self._conn_lock:
                conn = self._conn
            if conn is not None:
                try:
                    conn.sendall(_pack(q, self._servoj_gain, self._servoj_lookahead))
                    if first_packet:
                        first_packet = False
                        self.get_logger().info("[ctrl] first packet sent to robot")
                except Exception:
                    with self._conn_lock:
                        if self._conn is conn:
                            self._conn = None
                    first_packet = True
                    self.get_logger().warn("[ctrl] send failed — connection lost")

            next_t += STEP_TIME
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()

    # ── FollowJointTrajectory action ────────────────────────────────────────────

    @staticmethod
    def _reorder(names: list[str], values: list[float]) -> list[float]:
        try:
            return [values[names.index(j)] for j in UR_JOINT_ORDER]
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

        # Apply the web "Speed" knob by time-scaling: compress time by `speed`
        # and scale velocities by `speed` so the cubic interp stays consistent.
        speed = max(SPEED_MIN, min(SPEED_MAX, self._speed_scale))
        if abs(speed - 1.0) > 1e-3:
            waypoints = [
                (t / speed, q, [vi * speed for vi in v] if v else None)
                for (t, q, v) in waypoints
            ]

        total_time = waypoints[-1][0]
        with self._conn_lock:
            connected = self._conn is not None
        self.get_logger().info(
            f"[exec] {len(waypoints)}-pt traj, {total_time:.2f} s, robot connected={connected}"
        )
        if not connected:
            self.get_logger().warn("[exec] robot not connected — press Play on the pendant")

        # Hand the trajectory to the control loop and wait for it to finish.
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
    # 2 threads: one runs the (blocking) execute_cb, one services goal
    # cancellation concurrently. Fewer threads = less GIL contention with the
    # control-loop thread (the default of one-per-CPU starves it → stutter).
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
