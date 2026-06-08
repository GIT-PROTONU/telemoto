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
import base64
import gc
import hashlib
import json
import socket
import struct
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"   # RFC 6455 handshake magic

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
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
MODE_SERVOJ  = 1            # position control (planned moves, hold)
MODE_SPEEDJ  = 2            # joint-velocity control (WASD jog + idle self-hold)
JOG_SPEED_DEF = 0.05       # m/s at full axis (default; low for fine work)
JOG_ACCEL_DEF = 1.0        # rad/s^2 — speedj acceleration (default; gentle ramp)
MAX_JOG_QD    = 0.5        # rad/s — HARD per-joint speed cap during jogging. The
                           # arm CANNOT exceed this no matter what Servo emits
                           # (singularity amplification, glitches). Safety net.
TIMEOUT_MS   = 200          # robot counts a missed read after this long
_ZERO6       = [0.0] * 6
# servoj-safe ranges (UR limits) for clamping + the web sliders.
GAIN_MIN, GAIN_MAX           = 100, 2000
LOOKAHEAD_MIN, LOOKAHEAD_MAX = 0.03, 0.2
SPEED_MIN, SPEED_MAX         = 0.25, 3.0

# WASD jogging via MoveIt Servo. The web streams per-axis direction in [-1,1]
# at ~30 Hz while a key is held; we scale to these speeds and publish a
# TwistStamped in JOG_FRAME. Each command leases the twist for JOG_LEASE.
JOG_FRAME   = "tool0"       # jog in the TOOL frame (Servo transforms the twist);
                            # use "base_link" for base-frame jogging instead
JOG_LEASE   = 0.1           # s — each jog command is a lease; if no new command
                            # arrives within this the robot stops (latency/drop
                            # safety + instant stop on key release)
JOG_SPEED_MIN,  JOG_SPEED_MAX  = 0.005, 0.5   # m/s at full axis (web slider);
                                              # 5 mm/s floor → sub-mm taps for fine work
JOG_ACCEL_MIN,  JOG_ACCEL_MAX  = 0.3,  20.0   # rad/s^2 speedj accel (web slider);
                                              # lower = gentler start/stop
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
last_mode = 0
keep_going = True
while keep_going:
  socket_send_int(1, "reverse_socket")
  p = socket_read_binary_integer(11, "reverse_socket", read_timeout)
  if p[0] > 0:
    misses = 0
    read_timeout = p[1] / 1000.0
    last_mode = p[8]
    if p[8] == 1:
      q = [p[2]/1000000.0, p[3]/1000000.0, p[4]/1000000.0, p[5]/1000000.0, p[6]/1000000.0, p[7]/1000000.0]
      servoj(q, t=p[11]/1000000.0, lookahead_time=p[10]/1000.0, gain=p[9])
    elif p[8] == 2:
      qd = [p[2]/1000000.0, p[3]/1000000.0, p[4]/1000000.0, p[5]/1000000.0, p[6]/1000000.0, p[7]/1000000.0]
      # speedj acceleration is streamed live in p[9] (×100), so it's tunable.
      speedj(qd, p[9]/100.0, p[11]/1000000.0)
    end
  else:
    misses = misses + 1
    if last_mode == 2:
      stopj(4.0)
      last_mode = 0
    end
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
 <div class="row"><label>Jog speed <span class="val" id="jspeedv"></span></label>
   <input type="range" id="jspeed" min="0.005" max="0.5" step="0.005">
   <div class="hint">Cartesian jog velocity &mdash; lower for mm-precise work. Live.</div></div>
 <div class="row"><label>Jog acceleration <span class="val" id="jaccelv"></span></label>
   <input type="range" id="jaccel" min="0.3" max="20" step="0.1">
   <div class="hint">speedj ramp rate &mdash; lower = gentler start/stop. Live.</div></div>
<script>
 const fmt={speed:v=>(+v).toFixed(2)+"\\u00d7",gain:v=>Math.round(v),lookahead:v=>(+v).toFixed(3)+" s",
   jspeed:v=>Math.round(+v*1000)+" mm/s",jaccel:v=>(+v).toFixed(1)+" rad/s\\u00b2"};
 const ids=["speed","gain","lookahead","jspeed","jaccel"];
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
 // WASD jog: stream the direction as a CBOR binary frame at 30 Hz over a
 // WebSocket while a key is held, zero on release. WS is ordered + low-overhead.
 let jogOn=false; const held=new Set(); let stream=null, ws=null;
 // tool frame: Z = along the tool (forward/back), X/Y = across the flange.
 function jogVec(){return {lx:(held.has("q")?1:0)-(held.has("e")?1:0),
   ly:(held.has("a")?1:0)-(held.has("d")?1:0),lz:(held.has("w")?1:0)-(held.has("s")?1:0)};}
 function wsOpen(){ws=new WebSocket((location.protocol==="https:"?"wss://":"ws://")+location.host+"/ws");
   ws.onclose=()=>{ws=null;setTimeout(wsOpen,1000);};}
 // Minimal CBOR (RFC 8949): encode [lx,ly,lz] as an array of small signed ints.
 function cborInt(n){n=Math.round(n);const m=n<0?0x20:0x00,u=n<0?-1-n:n;
   if(u<24)return[m|u];if(u<256)return[m|24,u];return[m|25,(u>>8)&0xff,u&0xff];}
 function cborJog(v){return new Uint8Array([0x83].concat(cborInt(v.lx),cborInt(v.ly),cborInt(v.lz)));}
 function sendJog(){if(ws&&ws.readyState===1)ws.send(cborJog(jogVec()));}
 function startStream(){if(!stream)stream=setInterval(sendJog,33);}   // ~30 Hz
 function stopAll(){held.clear();if(stream){clearInterval(stream);stream=null;}sendJog();}  // sends zero
 document.getElementById("jogon").addEventListener("change",e=>{jogOn=e.target.checked;
   fetch("/api/jogmode?on="+(jogOn?1:0),{method:"POST"});if(!jogOn)stopAll();});
 document.addEventListener("keydown",e=>{if(!jogOn)return;const k=e.key.toLowerCase();
   if("wasdqe".includes(k)&&!held.has(k)){held.add(k);e.preventDefault();sendJog();startStream();}});
 document.addEventListener("keyup",e=>{if(!jogOn)return;const k=e.key.toLowerCase();
   if(held.has(k)){held.delete(k);
     if(held.size===0){clearInterval(stream);stream=null;sendJog();}else sendJog();}});
 window.addEventListener("blur",()=>{if(jogOn)stopAll();});
 wsOpen(); init();
</script>
</body></html>
"""


def _pack(values, mode, gain, lookahead_s, t_s) -> bytes:
    # `values` are joint positions (servoj) or joint velocities (speedj).
    return struct.pack(">11i",
        TIMEOUT_MS,
        *[int(x * MULT) for x in values],
        mode, int(gain),
        int(round(lookahead_s * 1000)), int(round(t_s * 1_000_000)),
    )


def _clamp(lo, hi, v):
    return max(lo, min(hi, v))


def _cap_speed(qd, lim):
    """Hard-cap a joint-velocity vector to `lim` (rad/s) on its fastest joint,
    scaling the whole vector so direction is preserved. Safety guard against
    singularity amplification — the robot can never be told to move faster than
    `lim` per joint during a jog."""
    m = max((abs(x) for x in qd), default=0.0)
    if m > lim:
        s = lim / m
        return [x * s for x in qd]
    return qd


def _cbor_decode(buf, i=0):
    """Minimal RFC 8949 decoder for the jog frame: ints and arrays (major
    types 0, 1, 4). Returns (value, next_index)."""
    head = buf[i]; major, minor = head >> 5, head & 0x1f; i += 1
    if minor < 24:
        val = minor
    elif minor == 24:
        val = buf[i]; i += 1
    elif minor == 25:
        val = int.from_bytes(buf[i:i + 2], "big"); i += 2
    elif minor == 26:
        val = int.from_bytes(buf[i:i + 4], "big"); i += 4
    elif minor == 27:
        val = int.from_bytes(buf[i:i + 8], "big"); i += 8
    else:
        raise ValueError("cbor: reserved length")
    if major == 0:
        return val, i
    if major == 1:
        return -1 - val, i
    if major == 4:
        out = []
        for _ in range(val):
            item, i = _cbor_decode(buf, i)
            out.append(item)
        return out, i
    raise ValueError(f"cbor: unsupported major type {major}")


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
        # WASD jog tuning (live): Cartesian linear speed and speedj acceleration.
        self.declare_parameter("jog_speed", JOG_SPEED_DEF, ParameterDescriptor(
            description="WASD jog linear speed m/s (live)",
            floating_point_range=[FloatingPointRange(
                from_value=JOG_SPEED_MIN, to_value=JOG_SPEED_MAX, step=0.005)]))
        self.declare_parameter("jog_accel", JOG_ACCEL_DEF, ParameterDescriptor(
            description="WASD jog speedj acceleration rad/s^2; lower = gentler (live)",
            floating_point_range=[FloatingPointRange(
                from_value=JOG_ACCEL_MIN, to_value=JOG_ACCEL_MAX, step=0.1)]))

        g = self.get_parameter
        self._robot_ip = g("robot_ip").get_parameter_value().string_value
        self._port     = g("reverse_port").get_parameter_value().integer_value
        self._web_port = g("web_port").get_parameter_value().integer_value
        self._gain      = g("servoj_gain").get_parameter_value().integer_value
        self._lookahead = g("servoj_lookahead").get_parameter_value().double_value
        self._speed     = g("speed_scale").get_parameter_value().double_value
        self._jog_speed = g("jog_speed").get_parameter_value().double_value
        self._jog_accel = g("jog_accel").get_parameter_value().double_value
        self.add_on_set_parameters_callback(self._on_set_params)

        self._js_ready = threading.Event()           # set on first JointState
        self._tgt_lock = threading.Lock()
        self._traj = None                             # (waypoints, total_time, t_start)
        self._traj_lock = threading.Lock()
        self._traj_done = threading.Event()
        self._conn: socket.socket | None = None       # live reverse socket
        self._conn_lock = threading.Lock()

        # WASD jog (MoveIt Servo): the web streams the desired twist at ~30 Hz
        # while a key is held; each command leases the twist for JOG_LEASE, so it
        # zeroes itself on release / latency / drop. _qd_target is Servo's joint
        # VELOCITY, streamed as speedj while jogging — open-loop velocity is the
        # natural primitive for human-in-the-loop teleop (no reference-pose to
        # snap back to, robust to PC timing jitter).
        self._jog = [0.0] * 6
        self._jog_lease = 0.0                          # twist valid until this time
        self._jog_mode = False                         # web toggle: stay in jog mode
        self._pub_lock = threading.Lock()              # serialise twist publishes
        self._qd_target = [0.0] * 6
        self._servo_active_until = 0.0
        self._dbg_pub_nz = False                        # diagnostics
        self._dbg_rx = 0
        self._dbg_log_t = 0.0

        # Control streams carry only the latest sample (keep_last 1): for a jog
        # at 30–250 Hz a stale command is worthless, so we never want a queue to
        # build during a brief stall — always act on the freshest value.
        ctrl_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE)
        self._js_sub = self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        # MoveIt Servo's JointTrajectory output → our stream (during jog).
        self.create_subscription(JointTrajectory, SERVO_OUT_TOPIC, self._servo_cb, ctrl_qos)
        self._twist_pub = self.create_publisher(TwistStamped, SERVO_TWIST_TOPIC, ctrl_qos)
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
            elif p.name == "jog_speed":
                self._jog_speed = float(_clamp(JOG_SPEED_MIN, JOG_SPEED_MAX, p.value))
            elif p.name == "jog_accel":
                self._jog_accel = float(_clamp(JOG_ACCEL_MIN, JOG_ACCEL_MAX, p.value))
        return SetParametersResult(successful=True)

    def _web_set(self, q: dict) -> None:
        if "speed" in q:     self._speed = _clamp(SPEED_MIN, SPEED_MAX, float(q["speed"][0]))
        if "gain" in q:      self._gain = int(_clamp(GAIN_MIN, GAIN_MAX, float(q["gain"][0])))
        if "lookahead" in q: self._lookahead = _clamp(LOOKAHEAD_MIN, LOOKAHEAD_MAX, float(q["lookahead"][0]))
        if "jspeed" in q:    self._jog_speed = _clamp(JOG_SPEED_MIN, JOG_SPEED_MAX, float(q["jspeed"][0]))
        if "jaccel" in q:    self._jog_accel = _clamp(JOG_ACCEL_MIN, JOG_ACCEL_MAX, float(q["jaccel"][0]))

    def _apply_jog(self, lx: float, ly: float, lz: float) -> None:
        # Axes in [-1,1] (tool frame), decoded from the CBOR jog frame.
        # Lease model: each message renews the twist for JOG_LEASE; a zero msg
        # (key release) stops instantly, and a stalled/closed socket lets the
        # lease expire so the robot stops on its own.
        sp = self._jog_speed
        self._jog = [sp * _clamp(-1.0, 1.0, lx),
                     sp * _clamp(-1.0, 1.0, ly),
                     sp * _clamp(-1.0, 1.0, lz), 0.0, 0.0, 0.0]
        self._jog_lease = time.monotonic() + JOG_LEASE
        self._dbg_rx += 1
        self._publish_twist()                       # publish NOW, don't wait for the loop

    def _publish_twist(self) -> None:
        if not self._jog_mode:
            return
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = JOG_FRAME
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = self._jog[0:3]
        m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = self._jog[3:6]
        with self._pub_lock:
            self._twist_pub.publish(m)

    def _web_jogmode(self, q: dict) -> None:
        self._jog_mode = q.get("on", ["0"])[0] in ("1", "true", "on")
        if not self._jog_mode:
            self._jog = _ZERO6
        self.get_logger().info(f"[jog] velocity mode {'ON' if self._jog_mode else 'OFF'}")

    # ── WASD jog: bridge web → MoveIt Servo → our stream ────────────────────────

    def _servo_cb(self, msg: JointTrajectory) -> None:
        # Servo's joint velocities drive the speedj jog. Ignored during planned
        # moves (the control loop prioritises the trajectory anyway).
        with self._traj_lock:
            if self._traj is not None:
                return
        if not msg.points or not msg.points[0].velocities:
            return
        try:
            qd = self._reorder(list(msg.joint_names), list(msg.points[0].velocities))
        except ValueError:
            return
        with self._tgt_lock:
            self._qd_target = qd
        self._servo_active_until = time.monotonic() + 0.12   # ~jog-active window

    def _arm_servo(self) -> None:
        # Put MoveIt Servo in TWIST mode up front so even a quick tap is acted on
        # immediately (it ignores twists until the command type is selected).
        deadline = time.monotonic() + 20.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self._servo_cli.service_is_ready():
                req = ServoCommandType.Request()
                req.command_type = ServoCommandType.Request.TWIST
                self._servo_cli.call_async(req)
                self.get_logger().info("[jog] MoveIt Servo armed (TWIST mode)")
                return
            time.sleep(0.2)
        self.get_logger().warn("[jog] servo_node not available — jog will be delayed")

    def _jog_loop(self) -> None:
        self._js_ready.wait()
        self._arm_servo()
        while rclpy.ok():
            time.sleep(0.02)                                  # 50 Hz
            now = time.monotonic()
            if now >= self._jog_lease:                        # lease expired → stop
                self._jog = _ZERO6
            # Lightweight diagnostics: a sparse on/off line + a 2 s summary, so
            # the realtime control loop isn't starved by per-command logging.
            pub_nz = any(self._jog)
            if pub_nz != self._dbg_pub_nz:
                self._dbg_pub_nz = pub_nz
                self.get_logger().info(f"[jog] {'moving' if pub_nz else 'stopped'}")
            if now - self._dbg_log_t >= 2.0:
                if self._dbg_rx:
                    self.get_logger().info(f"[jog] 2s: {self._dbg_rx} ws cmds")
                self._dbg_rx = 0
                self._dbg_log_t = now
            # Background publisher: keeps Servo warm when idle and re-publishes
            # the zero on lease expiry. Active commands are published instantly in
            # _apply_jog, so this is just the steady fallback stream.
            self._publish_twist()

    def _web_loop(self) -> None:
        node = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"              # required for the WS 101 upgrade
            def log_message(self, *_): pass

            def _send(self, body, ctype="text/plain"):
                self.send_response(200)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self):
                if self.path == "/ws" and self.headers.get("Upgrade", "").lower() == "websocket":
                    self._serve_ws()
                    return
                if self.path.startswith("/api/state"):
                    with node._conn_lock:
                        connected = node._conn is not None
                    self._send(json.dumps({"speed": round(node._speed, 2), "gain": node._gain,
                        "lookahead": round(node._lookahead, 3),
                        "jspeed": round(node._jog_speed, 3), "jaccel": round(node._jog_accel, 1),
                        "connected": connected}).encode(),
                        "application/json")
                else:
                    self._send(_WEB_PAGE.encode(), "text/html; charset=utf-8")

            def do_POST(self):
                q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
                try:
                    if self.path.startswith("/api/set"):
                        node._web_set(q)
                    elif self.path.startswith("/api/jogmode"):
                        node._web_jogmode(q)
                except (ValueError, KeyError):
                    pass
                self._send(b"ok")

            # ── WebSocket: ordered, low-overhead jog stream ────────────────────
            def _serve_ws(self):
                self.close_connection = True               # this socket is now a WS
                key = self.headers.get("Sec-WebSocket-Key", "")
                accept = base64.b64encode(
                    hashlib.sha1((key + _WS_GUID).encode()).digest()).decode()
                self.send_response(101)
                self.send_header("Upgrade", "websocket")
                self.send_header("Connection", "Upgrade")
                self.send_header("Sec-WebSocket-Accept", accept)
                self.end_headers()
                try:
                    while True:
                        op, data = self._ws_read()
                        if op is None or op == 0x8:        # closed
                            break
                        if op == 0x2:                      # binary frame = CBOR jog cmd
                            try:
                                v, _ = _cbor_decode(data)
                                node._apply_jog(float(v[0]), float(v[1]), float(v[2]))
                            except (ValueError, IndexError, TypeError):
                                pass
                except Exception:
                    pass
                node._apply_jog(0.0, 0.0, 0.0)             # stop on disconnect

            def _ws_read(self):
                rd = self.rfile
                hdr = rd.read(2)
                if len(hdr) < 2:
                    return None, b""
                ln = hdr[1] & 0x7f
                if ln == 126:
                    ln = struct.unpack(">H", rd.read(2))[0]
                elif ln == 127:
                    ln = struct.unpack(">Q", rd.read(8))[0]
                mask = rd.read(4) if hdr[1] & 0x80 else b"\x00\x00\x00\x00"
                data = rd.read(ln) if ln else b""
                return hdr[0] & 0x0f, bytes(c ^ mask[i & 3] for i, c in enumerate(data))

        try:
            srv = ThreadingHTTPServer(("0.0.0.0", self._web_port), Handler)
        except Exception as exc:
            self.get_logger().warn(f"[web] could not start on :{self._web_port}: {exc}")
            return
        self.get_logger().info(f"[web] tuning UI at http://{self._pc_ip()}:{self._web_port}")
        srv.serve_forever()

    # ── Joint states: only gate startup (the robot holds its own pose via speedj
    #    when idle, so we don't track positions here) ──────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        if not self._js_ready.is_set():
            self._js_ready.set()
            self.get_logger().info("[ctrl] first joint states received")
            self.destroy_subscription(self._js_sub)

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

    def _traj_target(self, now: float):
        """servoj position for an active planned move, or None if none active."""
        with self._traj_lock:
            traj = self._traj
        if traj is None:
            return None
        waypoints, total_time, t_start = traj
        if now - t_start >= total_time:
            with self._traj_lock:
                if self._traj is traj:
                    self._traj = None
            self._traj_done.set()
            return list(waypoints[-1][1])
        return self._interp(waypoints, now - t_start)

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

            # step duration = real cycle time so velocity == planned/commanded
            # (clamp only extreme outliers; never below dt, which would spike it).
            step_t = _clamp(0.002, 0.5, dt)
            q_traj = self._traj_target(now)
            if q_traj is not None:
                # planned move: servoj to the interpolated position.
                pkt = _pack(q_traj, MODE_SERVOJ, self._gain, self._lookahead, step_t)
            else:
                # jog / hold: speedj to Servo's joint velocity while jogging, else
                # zero (which also gives the idle self-hold). speedj's own
                # acceleration (jog_accel) shapes the ramp — lower it for a gentler
                # start/stop. SAFETY: _cap_speed hard-limits the joint speed so a
                # singularity-amplified Servo output can never make the arm shoot.
                with self._tgt_lock:
                    qd = self._qd_target if now < self._servo_active_until else _ZERO6
                qd = _cap_speed(qd, MAX_JOG_QD)
                pkt = _pack(qd, MODE_SPEEDJ, int(self._jog_accel * 100),
                            self._lookahead, step_t)
            try:
                conn.sendall(pkt)
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
    # Realtime: take everything alive after setup out of the GC's scan set so the
    # 125 Hz control loop never eats a collection pause (per-cycle lists are
    # non-cyclic → freed immediately by refcounting; GC only adds jitter here).
    gc.collect()
    gc.freeze()
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
