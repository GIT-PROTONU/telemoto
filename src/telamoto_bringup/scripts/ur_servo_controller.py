#!/usr/bin/env python3
"""
FollowJointTrajectory action server that drives a UR10 CB3 via the External
Control URCap — no ur_robot_driver, no RTDE inputs (PolyScope owns every RTDE
input register on this CB3 and there's no Remote Control mode to release them).

Flow: pressing Play on the pendant makes the URCap connect to REVERSE_PORT and
send "request_program"; we reply with a URScript that connects back to the same
port and PIPELINES targets: it requests the next one, runs speedj/servoj on the
current one (continuous motion, ~100% duty), then reads the reply that arrived
DURING that motion (4-byte request, 11-int reply). Overlapping the socket round
trip with the motion removes the idle gap of the old block-then-move loop (which
made the arm average ~50% of commanded speed → "deviates from path" trip); the
loop now runs ~125 Hz with no added latency. See _URSCRIPT.

Two modes (packet p[8]): servoj (position) for planned moves + the idle hold,
speedj (joint velocity) for WASD jogging via MoveIt Servo. A web UI on :8080
tunes speed/stiffness/smoothness + the jog and streams WASD over a WebSocket.

Reply packet — 11 x int32 big-endian, read as p[1..11]:
  p[1] timeout_ms . p[2..7] q x MULT . p[8] mode (1=servoj, 2=speedj) .
  p[9] servoj gain OR speedj accel x100 . p[10] lookahead x1000 (ms) .
  p[11] step duration x1e6 (us)
"""
import base64
import gc
import hashlib
import json
import os
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
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
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
_WS_GUID     = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"   # RFC 6455 handshake magic
MULT         = 1_000_000
MODE_SERVOJ  = 1            # position control (planned moves, hold)
MODE_SPEEDJ  = 2            # joint-velocity control (WASD jog + idle self-hold)
JOG_SPEED_DEF = 0.05       # m/s at full axis (default; low for fine work)
JOG_ACCEL_DEF = 1.0        # rad/s^2 — speedj acceleration (default; gentle ramp)
JOG_COAST_DEF = 0.03       # s — how long a Servo velocity stays valid after the
                           # last sample before falling back to zero (stop crispness)
MAX_JOG_QD    = 0.5        # rad/s — HARD per-joint speed cap during jogging. The
                           # arm CANNOT exceed this no matter what Servo emits
                           # (singularity amplification, glitches). Safety net.
TIMEOUT_MS   = 200          # robot counts a missed read after this long
STEP_T       = 0.008        # s — servoj/speedj duration we command each cycle (=1
                            # CB3 125 Hz cycle). The robot's pull loop runs at ≈2×
                            # this (measured: 8 ms → 16 ms cycle ≈ 62 Hz; 50 ms →
                            # 104 ms ≈ 10 Hz), so this constant sets the loop rate.
                            # NOT a safety bound — the runaway guard is the URScript
                            # read_timeout (TIMEOUT_MS) + miss→stopj. Fixed, not
                            # dt-tracked: with the 2× robot behavior, feeding dt back
                            # into step_t is positive feedback that pegs the rate low.
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
JOG_COAST_MIN,  JOG_COAST_MAX  = 0.001, 0.2   # s coast window (web slider); lower =
                                              # crisper stop. Floor >0 so the jog
                                              # never self-zeros between Servo samples.
                                              # NOTE: below ~the 4 ms Servo period /
                                              # ~8 ms robot cycle the jog stutters
                                              # (fails safe → stops), so keep ≥0.01 in use.
STEP_T_MIN,     STEP_T_MAX     = 0.008, 0.05  # s speedj/servoj duration per cycle.
                                              # Higher = the robot spends MORE of each
                                              # cycle executing motion vs blocked on
                                              # socket I/O → higher duty cycle → actual
                                              # speed tracks commanded (fixes the ~50%
                                              # velocity deficit) at the cost of a lower
                                              # update rate. Tune live vs the cmd/act log.
SERVO_TWIST_TOPIC = "/servo_node/delta_twist_cmds"
SERVO_OUT_TOPIC   = "/telamoto/servo_command"
SERVO_TYPE_SRV    = "/servo_node/switch_command_type"

# Live-tunable knobs — single source of truth for the ROS-param callback, the web
# setter, and clamping: (ros_param, web_key, attr, lo, hi, cast).
_TUNE = (
    ("speed_scale",      "speed",     "_speed",     SPEED_MIN,     SPEED_MAX,     float),
    ("servoj_gain",      "gain",      "_gain",      GAIN_MIN,      GAIN_MAX,      int),
    ("servoj_lookahead", "lookahead", "_lookahead", LOOKAHEAD_MIN, LOOKAHEAD_MAX, float),
    ("jog_speed",        "jspeed",    "_jog_speed", JOG_SPEED_MIN, JOG_SPEED_MAX, float),
    ("jog_accel",        "jaccel",    "_jog_accel", JOG_ACCEL_MIN, JOG_ACCEL_MAX, float),
    ("jog_coast",        "jcoast",    "_jog_coast", JOG_COAST_MIN, JOG_COAST_MAX, float),
    ("step_t",           "stept",     "_step_t",    STEP_T_MIN,    STEP_T_MAX,    float),
)

# Served on request_program. The "# HEADER_*" anchors are mandatory (the URCap
# splits header/body); we keep the header empty.
#
# PIPELINED pull loop. The old loop was: request → BLOCK on reply → speedj(8ms)
# → repeat. speedj (the only moving part) ran ~8ms of every ~17ms cycle; the
# other ~9ms was socket I/O with no active motion, and the velocity did NOT
# persist across that gap → the arm averaged ~50% of the commanded speed → a
# constant velocity deficit → position fell behind linearly → "deviates from
# path" protective stop at higher speed.
#
# Now we overlap the socket round-trip WITH the motion: send the NEXT request,
# run speedj/servoj on the CURRENT target (continuous, ~100% duty), then read the
# reply — which already arrived during the motion — with a short timeout. No idle
# gap → actual speed tracks commanded, no deficit, no trip, and the loop runs
# faster (~125 Hz) so latency is the same or better. Single 4-byte request per
# consumed reply (the `pending` guard) keeps the host handshake 1:1.
#
# SAFETY: motion is now continuous, so a dead host would otherwise coast on a
# stale qd. The miss watchdog stopj's hard (8 rad/s²) after ~8 missed reads
# (~60 ms) and the host-side MAX_JOG_QD cap + Servo singularity decel still apply.
_URSCRIPT = """\
# HEADER_BEGIN
# HEADER_END
textmsg("telamoto: external control active")
socket_open("{host}", {port}, "reverse_socket")
mode = 0
qd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
accel = 1.0
gain = 300
lookahead = 0.1
step_t = 0.008
misses = 0
pending = False
keep_going = True
while keep_going:
  if not pending:
    socket_send_int(1, "reverse_socket")
    pending = True
  end
  # Execute the current target — this is where the arm moves, continuously.
  if mode == 2:
    speedj(qd, accel, step_t)
  elif mode == 1:
    servoj(q, t=step_t, lookahead_time=lookahead, gain=gain)
  else:
    sync()
  end
  # Consume the reply that arrived during the motion. Normally it is already
  # buffered → returns immediately whatever the timeout. The 0.008 s timeout only
  # bounds the wait on a genuine miss; it is deliberately ONE full 125 Hz tick:
  # large enough to never round to 0 (which would block forever and defeat the
  # watchdog), small enough that a dead host is caught in ~128 ms (8 misses).
  p = socket_read_binary_integer(11, "reverse_socket", 0.008)
  if p[0] > 0:
    pending = False
    misses = 0
    mode = p[8]
    gain = p[9]
    accel = p[9] / 100.0
    lookahead = p[10] / 1000.0
    step_t = p[11] / 1000000.0
    if mode == 2:
      qd = [p[2]/1000000.0, p[3]/1000000.0, p[4]/1000000.0, p[5]/1000000.0, p[6]/1000000.0, p[7]/1000000.0]
    elif mode == 1:
      q = [p[2]/1000000.0, p[3]/1000000.0, p[4]/1000000.0, p[5]/1000000.0, p[6]/1000000.0, p[7]/1000000.0]
    end
  else:
    misses = misses + 1
    if misses > 8:
      if mode == 2:
        stopj(8.0)
      end
      mode = 0
    end
    if misses > 250:
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
 <div id="pendant" class="hint">pendant speed slider: &mdash;</div>
 <div id="qd" class="hint">joint speed: &mdash; &nbsp;|&nbsp; 1.5s peak: &mdash; rad/s</div>
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
 <div class="row"><label>Jog coast <span class="val" id="jcoastv"></span></label>
   <input type="range" id="jcoast" min="0.001" max="0.2" step="0.001">
   <div class="hint">Motion held after Servo goes quiet &mdash; lower = crisper stop. Live.</div></div>
<script>
 const fmt={speed:v=>(+v).toFixed(2)+"\\u00d7",gain:v=>Math.round(v),lookahead:v=>(+v).toFixed(3)+" s",
   jspeed:v=>Math.round(+v*1000)+" mm/s",jaccel:v=>(+v).toFixed(1)+" rad/s\\u00b2",
   jcoast:v=>Math.round(+v*1000)+" ms"};
 const ids=["speed","gain","lookahead","jspeed","jaccel","jcoast"];
 function show(k,v){document.getElementById(k).value=v;document.getElementById(k+"v").textContent=fmt[k](v);}
 function send(k){const v=document.getElementById(k).value;
   document.getElementById(k+"v").textContent=fmt[k](v);fetch("/api/set?"+k+"="+v,{method:"POST"});}
 async function refreshStatus(){try{const s=await(await fetch("/api/state")).json();
   const st=document.getElementById("status");
   st.textContent=s.connected?"robot connected":"robot not connected \\u2014 press Play on the pendant";
   st.className=s.connected?"ok":"bad";
   document.getElementById("pendant").textContent="pendant speed slider: "
     +(s.speedfrac==null?"\\u2014":Math.round(s.speedfrac*100)+"%");
   const qd=document.getElementById("qd");
   qd.textContent="joint speed: "+(s.qdnow==null?"\\u2014":s.qdnow.toFixed(3))
     +" \\u2014 1.5s peak: "+(s.qdpeak==null?"\\u2014":s.qdpeak.toFixed(3))+" rad/s";
   qd.style.color=(s.qdpeak>0.02)?"#ff7b72":"#888";}catch(e){}}
 async function init(){try{const s=await(await fetch("/api/state")).json();ids.forEach(k=>show(k,s[k]));}catch(e){}
   ids.forEach(k=>document.getElementById(k).addEventListener("input",()=>send(k)));
   refreshStatus();setInterval(refreshStatus,300);}
 function reset(){show("speed",1);show("gain",300);show("lookahead",0.1);
   fetch("/api/set?speed=1&gain=300&lookahead=0.1",{method:"POST"});}
 // WASD jog: stream the direction as a CBOR binary frame at 30 Hz over a
 // WebSocket while a key is held, zero on release. WS is ordered + low-overhead.
 let jogOn=false; const held=new Set(); let stream=null, ws=null;
 // tool frame: Z = along the tool (forward/back), X/Y = across the flange.
 function jogVec(){return {lx:(held.has("a")?1:0)-(held.has("d")?1:0),
   ly:(held.has("q")?1:0)-(held.has("e")?1:0),lz:(held.has("w")?1:0)-(held.has("s")?1:0)};}
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
        self.declare_parameter("jog_coast", JOG_COAST_DEF, ParameterDescriptor(
            description="WASD jog coast window s; lower = crisper stop (live)",
            floating_point_range=[FloatingPointRange(
                from_value=JOG_COAST_MIN, to_value=JOG_COAST_MAX, step=0.001)]))
        self.declare_parameter("step_t", STEP_T, ParameterDescriptor(
            description="speedj/servoj duration per cycle s; higher = better speed "
                        "tracking, lower update rate (live)",
            floating_point_range=[FloatingPointRange(
                from_value=STEP_T_MIN, to_value=STEP_T_MAX, step=0.001)]))

        g = self.get_parameter
        self._robot_ip = g("robot_ip").get_parameter_value().string_value
        self._port     = g("reverse_port").get_parameter_value().integer_value
        self._web_port = g("web_port").get_parameter_value().integer_value
        self._gain      = g("servoj_gain").get_parameter_value().integer_value
        self._lookahead = g("servoj_lookahead").get_parameter_value().double_value
        self._speed     = g("speed_scale").get_parameter_value().double_value
        self._jog_speed = g("jog_speed").get_parameter_value().double_value
        self._jog_accel = g("jog_accel").get_parameter_value().double_value
        self._jog_coast = g("jog_coast").get_parameter_value().double_value
        self._step_t    = g("step_t").get_parameter_value().double_value
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
        self._jog_moving = False                        # for the moving/stopped log
        self._speed_fraction = None                     # pendant speed slider [0,1], read-only
        # Trip diagnostics: when the connection drops (often a protective stop),
        # _drop() classifies it against the last commanded motion. Lets us tell a
        # mid-motion following-error from an on-release settle oscillation from a
        # plain idle/user stop — see _drop.
        self._last_motion_t = 0.0                       # monotonic t of last nonzero speedj
        self._last_peak_qd = 0.0                        # |joint speed| at that moment (rad/s)
        # ACTUAL joint speed (from RTDE actual_qd in /joint_states.velocity), for
        # the web readout: lets us SEE the standstill ring / real jog speed instead
        # of inferring it. _qd_now = current max |joint speed|; _qd_peak = ~1.5 s
        # decaying peak-hold so a brief ring still registers between UI polls.
        self._qd_now = 0.0
        self._qd_peak = 0.0
        self._qd_peak_t = 0.0
        # Slew-limited jog command: we ramp the velocity WE send toward Servo's
        # target at jog_accel rad/s², so the command never outruns what the robot
        # can physically reach (no following-error trip) while still climbing to
        # full speed. Gentle start + fast top speed; the felt acceleration is
        # jog_accel. Reset to 0 whenever not jogging / between connects.
        self._qd_cmd = [0.0] * 6

        # Control streams carry only the latest sample (keep_last 1): for a jog
        # at 30–250 Hz a stale command is worthless, so we never want a queue to
        # build during a brief stall — always act on the freshest value.
        ctrl_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                              reliability=ReliabilityPolicy.RELIABLE)
        self._js_sub = self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        # Pendant speed-slider fraction from ur_rtde_joint_pub — latched (matches
        # the publisher) so we get the last value on subscribe; display only.
        latched = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float64, "/telamoto/speed_fraction", self._speed_cb, latched)
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
        spec = {ros: (attr, lo, hi, cast) for ros, _, attr, lo, hi, cast in _TUNE}
        for p in params:
            if p.name in spec:
                attr, lo, hi, cast = spec[p.name]
                setattr(self, attr, cast(_clamp(lo, hi, float(p.value))))
        return SetParametersResult(successful=True)

    def _web_set(self, q: dict) -> None:
        for _, web, attr, lo, hi, cast in _TUNE:
            if web in q:
                setattr(self, attr, cast(_clamp(lo, hi, float(q[web][0]))))

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
        # Staleness window (jog_coast, live via web): command this velocity only
        # until here, then fall back to zero. Tight (default 30 ms ≈
        # max_expected_latency, ~7 Servo cycles at the 250 Hz publish_period) so the
        # jog stops crisply with no coast, while still zeroing on Servo silence
        # (crash/halt/drop) — the runaway guard. Floor (JOG_COAST_MIN) keeps it >0.
        self._servo_active_until = time.monotonic() + self._jog_coast

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
            if time.monotonic() >= self._jog_lease:           # lease expired → stop
                self._jog = _ZERO6
            moving = any(self._jog)                           # log only on transition
            if moving != self._jog_moving:
                self._jog_moving = moving
                self.get_logger().info(f"[jog] {'moving' if moving else 'stopped'}")
            # Background publisher: keeps Servo warm when idle and re-publishes the
            # zero on lease expiry. Active commands publish instantly in _apply_jog,
            # so this is just the steady fallback stream.
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
                    sf = node._speed_fraction
                    self._send(json.dumps({"speed": round(node._speed, 2), "gain": node._gain,
                        "lookahead": round(node._lookahead, 3),
                        "jspeed": round(node._jog_speed, 3), "jaccel": round(node._jog_accel, 1),
                        "jcoast": round(node._jog_coast, 3),
                        "speedfrac": round(sf, 3) if sf is not None else None,
                        "qdnow": round(node._qd_now, 3), "qdpeak": round(node._qd_peak, 3),
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
        # Kept subscribed (no longer self-destructs): track the actual measured
        # joint speed for the web readout. msg.velocity is RTDE actual_qd.
        if msg.velocity:
            m = max(abs(v) for v in msg.velocity)
            self._qd_now = m
            now = time.monotonic()
            if m >= self._qd_peak or now - self._qd_peak_t > 1.5:
                self._qd_peak = m
                self._qd_peak_t = now

    def _speed_cb(self, msg: Float64) -> None:
        self._speed_fraction = msg.data            # pendant speed slider, display only

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
        # Classify the drop against the last commanded motion. A protective stop
        # closes the socket just like a user stop, so timing is the only tell:
        #   MID-MOTION  → following error while moving (the classic high-speed trip)
        #   ON-RELEASE  → tripped while decelerating/settling (overshoot/resonance)
        #   IDLE        → no recent motion → almost certainly user stop / E-stop
        dt = time.monotonic() - self._last_motion_t if self._last_motion_t else 1e9
        if dt < 0.25:
            kind = f"*** TRIP? MID-MOTION (peak {self._last_peak_qd:.3f} rad/s) ***"
        elif dt < 2.0:
            kind = f"*** TRIP? ON-RELEASE/SETTLE {dt:.2f}s after motion (peak {self._last_peak_qd:.3f} rad/s) ***"
        else:
            kind = f"IDLE ({dt:.1f}s since motion) — likely user/E-stop"
        self.get_logger().warn(
            f"[server] robot disconnected — press Play to resume | {kind}")

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

    def _log_protective_stop(self, last_req: float) -> None:
        """Classify a robot-stopped-requesting event (almost always a protective
        stop) against the last commanded motion, so the trip self-labels:
          MID-MOTION  → tripped while moving (following error)
          ON-RELEASE  → tripped while decelerating/settling (overshoot/resonance)
          IDLE        → no recent motion → user / E-stop / pendant pause."""
        dt = last_req - self._last_motion_t if self._last_motion_t else 1e9
        if dt < 0.25:
            kind = f"MID-MOTION (peak {self._last_peak_qd:.3f} rad/s)"
        elif dt < 2.0:
            kind = f"ON-RELEASE/SETTLE {dt:.2f}s after motion (peak {self._last_peak_qd:.3f} rad/s)"
        else:
            kind = f"IDLE ({dt:.1f}s since motion) — user/E-stop"
        self.get_logger().warn(
            f"[ctrl] *** robot stopped requesting — likely PROTECTIVE STOP | {kind} ***")

    def _promote_realtime(self) -> None:
        """Put THIS thread (the robot pull-loop) on SCHED_FIFO so it preempts the
        node's ~45 other threads the instant the robot's request arrives, instead of
        waiting out a SCHED_OTHER timeslice. This keeps the loop timing steady (was
        24–60 ms jitter → ~8 ms) AND is what makes the pipeline reliable: the reply
        must be sent within the robot's speedj window, so a prompt RT wake-up keeps
        replies on time and misses near zero. (The high-speed trip itself turned out
        to be the speedj duty-cycle, fixed in the URScript — but steady timing still
        matters here.) The loop is almost always blocked on the socket recv (GIL
        released), so RT priority starves nothing and any GIL priority-inversion is
        bounded to ~one GIL switch interval. Priority 20 is deliberate: every sibling
        thread is SCHED_OTHER so even a low FIFO priority preempts them, while staying
        BELOW MoveIt Servo's RT thread (40) and kernel net/IRQ threads. Best-effort:
        needs the rtprio ulimit (setup/realtime-limits.sh); logs + continues if not."""
        try:
            os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(20))
            self.get_logger().info("[ctrl] control loop promoted to SCHED_FIFO:20")
        except (PermissionError, OSError) as exc:
            self.get_logger().warn(
                f"[ctrl] could not set SCHED_FIFO ({exc}); staying SCHED_OTHER — "
                "high-speed jog may trip on scheduling gaps. "
                "Check rtprio ulimit (setup/realtime-limits.sh).")

    def _control_loop(self) -> None:
        self._js_ready.wait()
        self._promote_realtime()
        first = True
        stalled = False                           # robot stopped pulling (trip detect)
        last_diag = 0.0                           # throttle for cmd-vs-actual log
        health_t = last_req = time.monotonic()
        cycles = 0
        max_gap = 0.0
        while rclpy.ok():
            with self._conn_lock:
                conn = self._conn
            if conn is None:
                first = True
                self._qd_cmd = [0.0] * 6           # robot is at rest → slew from 0 on reconnect
                last_req = time.monotonic()         # avoid a huge dt (slew/health) on reconnect
                time.sleep(0.01)
                continue
            try:                                  # block until the robot requests
                conn.settimeout(1.0)
                req = self._recvall(conn, 4)
            except socket.timeout:
                # Connected, but the robot stopped sending pull-requests — the
                # signature of a PROTECTIVE STOP (URScript halts; it does NOT drop
                # our socket, so we'd otherwise stream into the void unaware). Log
                # once per stall, classified against the LAST successful request
                # (detection-latency-independent).
                if not stalled:
                    stalled = True
                    self._qd_cmd = [0.0] * 6       # robot halted → slew from 0 on resume
                    self._log_protective_stop(last_req)
                continue
            except Exception:
                self._drop(conn); continue
            if req is None:
                self._drop(conn); continue
            if stalled:                           # robot came back (Play pressed)
                stalled = False
                self.get_logger().info("[ctrl] robot resumed pull-requests")

            now = time.monotonic()
            dt = now - last_req                   # measured cycle time (health only)
            last_req = now
            cycles += 1; max_gap = max(max_gap, dt)
            if now - health_t >= 5.0:
                self.get_logger().info(
                    f"[ctrl] {cycles} cycles/5s, max gap {max_gap*1000:.1f}ms")
                cycles = 0; max_gap = 0.0; health_t = now

            # speedj/servoj duration per cycle = the loop period. With the pipelined
            # URScript the motion runs continuously (≈100% duty) regardless of this,
            # so step_t just sets the rate: 8 ms is the robot's 125 Hz floor (lowest
            # latency) and is the right default — live-tunable, but no need to touch.
            step_t = self._step_t
            q_traj = self._traj_target(now)
            if q_traj is not None:
                # planned move: servoj to the interpolated position.
                self._qd_cmd = [0.0] * 6              # reset slew state for next jog
                pkt = _pack(q_traj, MODE_SERVOJ, self._gain, self._lookahead, step_t)
            else:
                # jog / hold: speedj to Servo's joint velocity while jogging, else
                # zero (which also gives the idle self-hold). speedj's own
                # acceleration (jog_accel) shapes the ramp — lower it for a gentler
                # start/stop. SAFETY: _cap_speed hard-limits the joint speed so a
                # singularity-amplified Servo output can never make the arm shoot.
                #
                # Gate on ACTUAL jog intent (self._jog non-zero), not just on Servo
                # freshness: the jog loop keeps Servo warm with zero-twist, so at
                # standstill Servo still publishes tiny non-zero residual velocities.
                # Streaming those as speedj makes the arm chase the residual = the
                # standstill ring. When released, command a clean exact zero instead.
                jogging = any(self._jog)
                with self._tgt_lock:
                    target = self._qd_target if (jogging and now < self._servo_active_until) else _ZERO6
                target = _cap_speed(target, MAX_JOG_QD)
                # Slew-limit our command toward the target at jog_accel rad/s² (clamp
                # dt so a stall can't make it jump). The pipelined URScript is what
                # fixed the trip (the robot now holds the commanded velocity — no
                # duty-cycle deficit); this slew is purely for FEEL: a gentle ramp UP
                # and a clean ramp DOWN to exact zero on release (smooth stop, no
                # residual ring). jog_accel is the single smoothness knob.
                step = self._jog_accel * min(dt, 0.05)
                qd = [c + _clamp(-step, step, t - c) for c, t in zip(self._qd_cmd, target)]
                self._qd_cmd = qd
                peak = max((abs(x) for x in qd), default=0.0)
                if peak > 1e-4:                          # remember last real motion (for _drop)
                    self._last_motion_t = now
                    self._last_peak_qd = peak
                # Diagnostic (throttled): target (Servo demand) vs cmd (our slewed
                # command) vs act (RTDE actual_qd). Healthy = act tracks cmd closely;
                # cmd lags target only during the ramp, then converges.
                if (peak > 1e-4 or any(target)) and now - last_diag >= 0.3:
                    last_diag = now
                    tgt = max((abs(x) for x in target), default=0.0)
                    self.get_logger().info(
                        f"[jog] tgt={tgt:.3f} cmd={peak:.3f} act={self._qd_now:.3f} rad/s "
                        f"(accel={self._jog_accel:.1f} step_t={self._step_t*1000:.0f}ms)")
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
