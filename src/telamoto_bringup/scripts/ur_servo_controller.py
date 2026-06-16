#!/usr/bin/env python3
"""
Motion bridge for a UR10 CB3 via the External Control URCap — no ur_robot_driver,
no RTDE inputs (PolyScope 3.x owns every RTDE input register on this CB3 and has
no Remote Control mode to release them). Joint states come from ur_rtde_joint_pub
(RTDE outputs, 125 Hz); this node provides:

  • a FollowJointTrajectory action server (planned MoveIt moves → servoj), and
  • Cartesian jogging from a web UI on :8080 (→ speedl): WASD/QE translation +
    I/K (tilt) J/L (pan) U/O (roll) TCP rotation, both also on the touch pads.

Transport: pressing Play on the pendant makes the URCap connect to REVERSE_PORT
and send "request_program"; we reply with a URScript that connects back to the
same port and PIPELINES targets: it requests the next one, executes the current
one (continuous motion, ~100% duty), then reads the reply that arrived DURING
that motion (4-byte request, 11-int reply, ~125 Hz). See _URSCRIPT.

Jog architecture — why speedl: a PC-side Cartesian→joint conversion (MoveIt
Servo) works on a joint state that is ~15–30 ms stale by execution time, which
steers the realized direction a few degrees off the commanded one, proportional
to speed (measured: 21 mm lateral bow over a 343 mm push at 500 mm/s). speedl
hands the twist to the ROBOT's own controller — onboard IK on fresh state, the
same mechanism as the pendant jog → straight by construction (verified: ~0.1 mm
median off-axis). Riding on the commanded twist as trim:
  • orientation hold — proportional lock on the tool orientation captured at jog
    start (re-captured after REORIENT_IDLE of standstill),
  • straight-line hold — PI lock (KI = KP²/4, critically damped) on the line
    through the jog-start point along the commanded direction.
Jog axes are tool-frame or base-frame (web toggle). ROTATION jog rides the speedl
angular slots; while it is active the orientation hold stands down (re-locks at the
new pose on stop) and the line hold degenerates to a position hold (TCP pivots in
place).

SAFETY (layered — see the singularity incident in the project notes):
  1. MoveIt Servo runs as a kinematic SENTINEL: it receives the jog twist and its
     /servo_node/status gates the speedl command (×1 clean, ×0.25 decelerate,
     ×0 halt) — its own joint output is unused.
  2. Amplification guard: measured joint speed (RTDE actual_qd) beyond what the
     SENT TCP speed justifies (QD_ALLOW_BASE + QD_ALLOW_SLOPE·|sent|) shrinks a
     smooth slew-limited gate — tight at low speed where J⁻¹ blowup lives,
     permissive at honest high speed. Beyond QD_LATCH_FACTOR × allowance the
     gate has lost (boundary/singular amplification is unbounded): latch to the
     speedj-zero hold until key release + joint settle.
  3. The URScript stopj's after ~8 missed replies (~64 ms) — dead-host watchdog.
  4. UR's own safety limits backstop everything.

Reply packet — 11 x int32 big-endian, read as p[1..11]:
  p[1] timeout_ms . p[2..7] payload x MULT (servoj: q rad; speedj: qd rad/s;
  speedl: base-frame twist m/s+rad/s) . p[8] mode (1=servoj, 2=speedj,
  3=speedl) . p[9] servoj gain OR accel x100 . p[10] lookahead x1000 (ms) .
  p[11] step duration x1e6 (us)
"""
import asyncio
import base64
import gc
import hashlib
import json
import math
import mimetypes
import os
import socket
import struct
import threading
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Float64, String
from geometry_msgs.msg import TwistStamped
from moveit_msgs.msg import CollisionObject, PlanningScene, ServoStatus
from moveit_msgs.srv import ServoCommandType
from rcl_interfaces.msg import (
    FloatingPointRange, IntegerRange, ParameterDescriptor, SetParametersResult,
)
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter as RclpyParameter

try:
    # WebRTC datachannel = the browser's only true-UDP transport (the jog rides
    # an UNORDERED/NO-RETRANSMIT channel so a lossy WAN path drops stale twists
    # instead of queueing them behind TCP retransmits). The same peer machinery
    # also serves the WEBCAM as a video track (RTP/UDP — lowest-latency feed a
    # browser can play). Optional: without aiortc the page's /api/rtc offer
    # gets a 503 and the jog stays on the WS (no webcam).
    from aiortc import (RTCConfiguration, RTCIceServer, RTCPeerConnection,
                        RTCSessionDescription)
    from aiortc.contrib.media import MediaPlayer, MediaRelay
except ImportError:
    RTCPeerConnection = None

UR_JOINT_ORDER = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
REVERSE_PORT = 50001        # URCap requests the script here AND robot connects back
WEB_PORT     = 8080
_WS_GUID     = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"   # RFC 6455 handshake magic
RTC_STUN     = "stun:stun.l.google.com:19302"  # srflx candidates: the UI is used
                                               # straight off the public IP (user
                                               # decision 2026-06-12 — no VPN), so
                                               # both ends usually sit behind NAT
RTC_MAX_PCS  = 16           # refuse offers beyond this many live peer connections
CAM_DEVICE   = "/dev/video0"   # webcam for the live video track ("" disables)
CAM_SIZE     = "640x480"       # capture size — 480p30 keeps the VP8 encode cheap
CAM_FPS      = 30              # on this host and the WAN bitrate well under 1 Mb/s
CAM_IDLE_CLOSE = 5.0           # s with no viewers before the device is released:
                               # a quick off→on toggle must REUSE the open camera
                               # (closing v4l2 is async — instant reopen hits
                               # EBUSY = black screen) and gets its first frame
                               # instantly instead of paying the USB open again
MULT         = 1_000_000
MODE_SERVOJ  = 1            # position control (planned moves, hold)
MODE_SPEEDJ  = 2            # joint-velocity control (the idle self-hold: exact zeros)
MODE_SPEEDL  = 3            # CARTESIAN velocity control (WASD jog). speedl() lets the
                            # ROBOT's own controller do the Cartesian→joint conversion
                            # onboard at 125 Hz with fresh state — same mechanism as
                            # the pendant jog, which is why the pendant tracks straight
                            # lines at speed and a PC-side J⁻¹ (MoveIt Servo, ~15–30 ms
                            # stale by execution time) never quite does.
SPEEDL_ACCEL = 2.5          # m/s² — robot-side tool accel for speedl: must outrun the
                            # Cartesian ramp (jog_accel) + corrections so the robot
                            # faithfully TRACKS the smooth command.
# Singularity-amplification guard for speedl (replaces a fixed joint-speed cap, which
# bang-banged at 500 mm/s where joints LEGITIMATELY need >0.5 rad/s → stutter). The
# danger signature from the incident is a SMALL Cartesian command producing LARGE
# joint speeds (J⁻¹ blowup near a singularity), so the allowance scales with the
# TCP speed actually SENT to the robot last cycle (post-gate):
#   allowed_qd = BASE + SLOPE·|sent lin| + ROT_SLOPE·|sent ang|.
# SENT, not the operator's pre-gate target: in the 2026-06-12 boundary-stall trip
# Servo had the jog at ×0.25 (62 mm/s sent) while the allowance was computed from
# the 248 mm/s target — 4× too blind exactly when another layer was already
# braking (guard fired at 1.71 rad/s instead of ~0.5; UR tripped at 3.2). Judging
# against the sent speed also makes the gate self-tightening under amplification
# (gate ↓ → sent ↓ → allowance ↓): honest motion (qd ∝ sent, slope < SLOPE) always
# recovers, amplified motion spirals down to a crawl — DELIBERATE, the operator
# prefers a slowed/deviating jog over a stop. Applied as a SMOOTH gate — brakes
# fast, recovers gently — never a hard cut, so no limit-cycle stutter.
QD_ALLOW_BASE  = 0.3        # rad/s allowed at zero commanded speed
QD_ALLOW_SLOPE = 2.5        # rad/s additional per m/s of SENT linear TCP speed
QD_GATE_DOWN   = 0.20       # max gate decrease per 125 Hz cycle (~40 ms to full stop)
QD_GATE_UP     = 0.02       # max gate increase per cycle (~0.4 s to full recovery)
# RUNAWAY latch — the guard's last resort. The proportional gate can only scale
# the command, never zero it (tgt = allowed/qd > 0), and at a workspace boundary /
# deep singularity the onboard speedl IK amplifies ANY nonzero twist without bound
# (2026-06-12: qd kept climbing 2.6 → 3.2 rad/s with only 20 mm/s sent → UR
# protective stop). If measured qd exceeds LATCH_FACTOR × allowance the jog drops
# to the exact speedj-zero hold and stays there until the operator releases the
# key AND the joints settle — a self-recovering host-side stop instead of UR's
# pendant-bound protective stop. Honest motion can't reach it (at 0.5 m/s sent
# the latch sits at ~3.9 rad/s, beyond UR's own limits).
QD_LATCH_FACTOR = 2.5       # × allowance: beyond this the gate has lost — latch
QD_LATCH_SETTLE = 0.15      # rad/s: joints considered settled → latch may release
# Latch DIRECTION memory (2026-06-12 second on-robot run): a plain latch forgets
# which way the trouble was — releasing and re-pressing the same key ratcheted
# the arm DEEPER into the boundary, and escape taps kept re-latching (at the
# singular pose even OUTWARD motion needs ~1 rad/s for the first few cm, far
# over a fresh tap's 0.75 rad/s latch line → "tap five times to get out"). So
# the latch captures the inward direction (base frame, from the pre-gate
# command) and the TCP position: afterwards the inward cone is HARD-blocked,
# while the opposite (escape) cone runs with a raised allowance floor and a
# raised latch line — outward motion monotonically leaves the amplification
# zone and UR's own limits (~2.1+ rad/s) still backstop. Everything clears
# after QD_BLOCK_CLEAR_M of TCP travel from the REST pose where the latch
# released (any route: escape jog, lateral jog, planned move). Measuring from
# the latch ONSET was a hole (2026-06-12 protective stop): each runaway
# excursion itself travels 5–30 cm, self-clearing the block mid-lurch — six
# taps ratcheted in until qd 2.56 rad/s tripped UR's own joint-speed limit.
QD_BLOCK_DOT     = 0.5      # cone half-angle cos: ≥ this aligned with the inward
                            # direction = blocked; ≤ −this = escaping
QD_ESCAPE_ALLOW  = 0.8      # rad/s allowance floor while escaping
QD_ESCAPE_LATCH  = 1.5      # rad/s latch line while escaping
QD_BLOCK_CLEAR_M = 0.05     # m of TCP travel from the latch-release rest pose
                            # → block clears
JOG_SPEED_DEF = 0.5        # m/s at full axis (default)
JOG_ACCEL_DEF = 1.0        # m/s^2 — CARTESIAN start/stop ramp rate (default; gentle).
                           # Ramps the TWIST magnitude (direction-preserving), never
                           # the per-joint velocity — a per-joint slew distorts the
                           # commanded direction and bends the path. See _jog_loop.
SPEEDJ_ACCEL  = 8.0        # rad/s^2 — speedj accel for the idle hold's zero command
                           # (how hard the robot brakes into standstill); matches the
                           # URScript miss-watchdog stopj(8).
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
# TwistStamped in JOG_FRAME. Each command leases the twist for the link-adaptive
# lease (JOG_LEASE floor — see the laggy-link block below).
JOG_FRAME   = "tool0"       # jog in the TOOL frame (Servo transforms the twist);
                            # use "base_link" for base-frame jogging instead
BASE_FRAME  = "base_link"   # fixed frame the TCP orientation is held against
# Orientation hold: Servo's twist mode is OPEN-LOOP resolved-rate (q̇ = J⁻¹·twist).
# Commanding zero angular velocity only zeroes it for THAT instant; over discrete
# 4 ms cycles with a configuration-dependent Jacobian a tiny orientation error is
# left each step and nothing corrects it → the TCP visibly drifts/tilts over a
# long jog. We close the loop: capture the orientation when a jog starts and inject
# a proportional corrective angular velocity (in JOG_FRAME) to hold it.
# Orientation-hold proportional gain (rad/s of correction per rad of error), LIVE-
# tunable via the web slider — it's the one knob that governs hold quality.
# Too HIGH → limit-cycle "waddle": with the feedback latency (TF→Servo→speedj→TF),
# a position→velocity P-loop oscillates once KP·delay is large, and a high KP also
# saturates the correction at ORIENT_LOCK_MAX → bang-bang, which sustains it (KP=20
# waddled ±1.5–2°). Too LOW → soft hold, steady-state drift ∝ drift_rate / KP.
# Default 6 is well inside the stable regime; tune live to taste.
ORIENT_KP_DEF             = 6.0
ORIENT_KP_MIN, ORIENT_KP_MAX = 0.0, 30.0
ORIENT_LOCK_MAX = 0.5       # rad/s — cap on the correction (≤ servo.yaml rotational
                            # cap 1.0; the amplification guard backstops the joints)
# Straight-line hold: same closed-loop idea as the orientation lock, applied to the
# PATH — a PI lock on the line through the jog-start point along the commanded
# direction, feeding the perpendicular position error back as a corrective linear
# velocity. Along-track motion is untouched (the error is perpendicular-only), so the
# hold can neither slow nor delay the jog itself.
#
# PI, not P: any residual lateral disturbance is (approximately) a constant VELOCITY
# proportional to speed, and a P position loop can never zero a constant-rate
# disturbance (steady state = rate/KP — measured exactly that in the speedj era:
# 14 mm at 500 mm/s with KP=4). The INTEGRAL term learns the disturbance velocity and
# cancels it outright → lateral error converges to ~0 at ANY speed. KI is derived
# from KP for CRITICAL damping (KI = KP²/4 → ζ=1): no overshoot, no oscillation,
# smooth by construction. With speedl the disturbance is near-zero to begin with;
# this hold mops up the residual (TF noise, accel transients).
PATH_KP_DEF             = 6.0   # 1/s — P gain; with KI=KP²/4 the error converges to 0
                                # with time constant ≈ 2/KP (~0.33 s) regardless of
                                # speed. Stability: KP·loop-delay ≪ 1 (delay ~30–50 ms).
PATH_KP_MIN, PATH_KP_MAX = 0.0, 15.0
PATH_LOCK_MAX = 0.15        # m/s — cap on the TOTAL correction; small vs jog speed
                            # so a TF glitch can never fling the arm sideways.
PATH_I_MAX    = 0.12        # m/s — separate clamp on the integral part (anti-windup:
                            # it can hold the full ~60 mm/s disturbance with margin,
                            # but can never wind past what motion could justify).
REORIENT_IDLE   = 1.0       # s — only re-capture the locked orientation after the jog
                            # has been idle THIS long. Rapid consecutive taps keep the
                            # SAME reference (so residual drift is pulled back, not
                            # ratcheted in); a deliberate pause re-locks the new pose.
JOG_LEASE   = 0.1           # s — each jog command is a lease; if no new command
                            # arrives within this the robot stops (latency/drop
                            # safety + instant stop on key release)
# Laggy-link compensation (remote/VPN jogging). A fixed 0.1 s lease assumes the
# ~30 Hz frame stream never gaps more than ~67 ms — true on a LAN, false over the
# internet, where jitter expires the lease mid-hold and the jog stutters at FULL
# speed (ramp down → ramp up). Instead the lease ADAPTS to the measured
# frame-arrival gap, and the jog speed is scaled DOWN by the same factor, holding
#   scaled_speed × lease  =  jog_speed × JOG_LEASE
# so the worst-case uncommanded travel after the last received frame (expiry,
# dead client) never exceeds what the 0.1 s lease allows on a LAN — a laggy link
# degrades to a slower jog, never a less safe one. The client streams at a fixed
# 30 Hz whenever jog mode is on (zeros while no key is held), so the gap between
# frames IS the link jitter and the estimate is warm before the first key press.
JOG_LEASE_MAX     = 0.4     # s — lease ceiling; links gapping beyond this still
                            # stutter (correctly: unusable for jogging)
LINK_GAP_TOL      = 0.05    # s — frame gaps up to this are normal even on a LAN
                            # (33 ms period + timer jitter) and do NOT stretch the
                            # lease, so LAN behavior is bit-identical
LINK_LEASE_FACTOR = 3.0     # lease = JOG_LEASE + factor × (recent-max gap − TOL)
LINK_GAP_NOMINAL  = 1.0 / 30.0  # s — the client's frame period (30 Hz stream)
LINK_GAP_TAU      = 10.0    # s — peak-hold decay: a bad burst stretches the
                            # lease, a recovered link relaxes back in ~10 s
LINK_BIG_WINDOW   = 2.0     # s — de-twitch: an ISOLATED big gap (browser GC,
                            # scheduler hiccup) must not modulate the jog speed —
                            # it keeps the old fixed-lease behavior (at worst one
                            # stutter). Only a SECOND big gap within this window
                            # confirms real link degradation and feeds the peak.
LINK_GAP_RESET    = 0.45    # s — gaps beyond this are ignored by the estimator:
                            # just past the lease ceiling, so they are unbridgeable
                            # anyway (already stuttered) and far more likely a
                            # stream (re)start (jog toggle, tab flick, WS
                            # reconnect) than jitter worth adapting to
JOG_SPEED_MIN,  JOG_SPEED_MAX  = 0.005, 0.5   # m/s at full axis (web slider);
                                              # 5 mm/s floor → sub-mm taps for fine work
JOG_ACCEL_MIN,  JOG_ACCEL_MAX  = 0.3,  20.0   # m/s^2 Cartesian ramp (web slider);
                                              # lower = gentler start/stop
# Rotational jog (pan/tilt/roll of the TCP): three more axes in the SAME CBOR jog
# frame, through the same lease/ramp/gate machinery as the linear jog (sentinel
# feed, amplification guard, link scaling — the blind-travel invariant holds for
# rotation too: scaled_rot_speed × lease = jog_rot_speed × 0.1 s). While a rotation
# is commanded the orientation hold stands DOWN (the user owns the orientation) and
# re-locks at the new pose when the rotation stops; the straight-line hold
# degenerates to a POSITION hold (start point, no direction) so a pure rotation
# pivots about tool0 even if the pendant TCP is set elsewhere.
JOG_ROT_SPEED_DEF = 0.5     # rad/s at full axis (~29°/s). Was 0.25 while the
                            # axis signs were unverified on the robot; verified
                            # 2026-06-12, raised — the slider still caps at 1.0.
JOG_ROT_SPEED_MIN, JOG_ROT_SPEED_MAX = 0.01, 1.0  # ≤ ur_servo.yaml rotational cap
JOG_ROT_RAMP = 2.0          # rad/s² of rotational ramp per m/s² of jog_accel —
                            # one accel knob drives both ramps
QD_ALLOW_ROT_SLOPE = 2.0    # rad/s of amplification-guard allowance per rad/s of
                            # commanded TCP rotation (a pure wrist roll needs ~1:1;
                            # margin for poses where pan/tilt recruit big joints)
# Servo collision-monitor proximity thresholds [m] (web sliders): inside the
# distance the jog gate drops to ×0.25 (DECELERATE_FOR_COLLISION), at contact ×0
# (HALT_FOR_COLLISION). Pushed live to /servo_node's parameters — the controller
# owns the UI value, servo owns the check.
# ⚠ self default must stay < ~0.015: the UR10's forearm↔wrist_2 surfaces sit
# 1.5–2 cm apart at EVERY pose (measured via test_selfcoll_e2e.py / servo's
# collision monitor), so a bigger threshold = Servo permanently in
# DECELERATE_FOR_COLLISION = jog stuck at ×0.25 with a constant warn banner
# (the old 0.30 default did exactly that — the jog never ran at full speed).
# ⚠ scene default must stay ≤ ~0.10 for the same reason: with the full cage
# (6 walls + floor + base column) some surface is nearly always within 0.5 m,
# so the old 0.50 default pinned Servo at DECELERATE_FOR_COLLISION for entire
# sessions (2026-06-12: 403 gate-log entries at ×0.25, ZERO at ×1.0 — the
# user's "rotation is very slow" was the whole jog quartered).
SELF_COLL_MIN,  SELF_COLL_MAX,  SELF_COLL_DEF  = 0.01, 0.5, 0.01
SCENE_COLL_MIN, SCENE_COLL_MAX, SCENE_COLL_DEF = 0.01, 1.0, 0.10
# Collision-halt ESCAPE: Servo's collision scale is computed from the robot's
# ACTUAL state vs the scene (not from the command), so once the model touches a
# scene object the status pins at HALT_FOR_COLLISION in every direction and a
# plain ×0 gate would deadlock the jog at the wall. Escape rule: remember the
# approach direction at the moment of the halt; commands that genuinely REVERSE
# (within a 60° cone of the exact opposite) pass at the crawl gate, everything
# else stays hard-blocked. Singularity halts are NOT escapable this way — Servo
# itself is direction-aware there: a jog-caused halt fires at the hard-stop
# boundary, where the REVERSED twist re-evaluates as DECELERATE_FOR_LEAVING
# (×0.25 via the normal gate map) and backs out (verified by
# test_singularity_e2e.py). Past the boundary (arm parked deep by a planned
# move) Servo halts EVERY direction — deliberate: blind crawling where J⁻¹
# amplification lives is the original incident; recover with a planned move.
COLL_ESCAPE_GATE = 0.25     # crawl factor while backing out of a collision halt
COLL_ESCAPE_DOT  = 0.5      # cos(60°): how opposed to the approach a command must be
STEP_T_MIN,     STEP_T_MAX     = 0.008, 0.05  # s speedl/servoj duration per cycle.
                                              # Higher = the robot spends MORE of each
                                              # cycle executing motion vs blocked on
                                              # socket I/O → higher duty cycle → actual
                                              # speed tracks commanded (fixes the ~50%
                                              # velocity deficit) at the cost of a lower
                                              # update rate. Tune live vs the cmd/act log.
SERVO_TWIST_TOPIC  = "/servo_node/delta_twist_cmds"   # sentinel input (jog twist)
SERVO_STATUS_TOPIC = "/servo_node/status"             # sentinel verdict (gates speedl)
SERVO_TYPE_SRV     = "/servo_node/switch_command_type"

# Live-tunable knobs — single source of truth for the ROS-param callback, the web
# setter, and clamping: (ros_param, web_key, attr, lo, hi, cast).
_TUNE = (
    ("speed_scale",      "speed",     "_speed",     SPEED_MIN,     SPEED_MAX,     float),
    ("servoj_gain",      "gain",      "_gain",      GAIN_MIN,      GAIN_MAX,      int),
    ("servoj_lookahead", "lookahead", "_lookahead", LOOKAHEAD_MIN, LOOKAHEAD_MAX, float),
    ("jog_speed",        "jspeed",    "_jog_speed", JOG_SPEED_MIN, JOG_SPEED_MAX, float),
    ("jog_accel",        "jaccel",    "_jog_accel", JOG_ACCEL_MIN, JOG_ACCEL_MAX, float),
    ("jog_rot_speed",    "jrspeed",   "_jog_rot_speed",
                                                    JOG_ROT_SPEED_MIN, JOG_ROT_SPEED_MAX, float),
    ("step_t",           "stept",     "_step_t",    STEP_T_MIN,    STEP_T_MAX,    float),
    ("orient_lock_kp",   "okp",       "_orient_kp", ORIENT_KP_MIN, ORIENT_KP_MAX, float),
    ("path_lock_kp",     "pkp",       "_path_kp",   PATH_KP_MIN,   PATH_KP_MAX,   float),
    ("self_collision_dist",  "selfd", "_self_coll",  SELF_COLL_MIN,  SELF_COLL_MAX,  float),
    ("scene_collision_dist", "walld", "_scene_coll", SCENE_COLL_MIN, SCENE_COLL_MAX, float),
)

# Tunables that live in /servo_node, not here: attr → servo parameter name.
# Changes are coalesced and pushed via /servo_node/set_parameters (see
# _push_collision_params) — both are dynamic in moveit_servo 2.12.
_SERVO_PARAM_ATTRS = {
    "_self_coll":  "moveit_servo.self_collision_proximity_threshold",
    "_scene_coll": "moveit_servo.scene_collision_proximity_threshold",
}

# Served on request_program. The "# HEADER_*" anchors are mandatory (the URCap
# splits header/body); we keep the header empty.
#
# PIPELINED pull loop: send the NEXT request, execute the CURRENT target
# (continuous motion, ~100% duty), then read the reply that already arrived during
# the motion. A naive request→block→move loop leaves the motion command idle during
# the socket round-trip (~50% duty → the arm averages half the commanded speed →
# "deviates from path" protective stop at higher speeds). Single 4-byte request per
# consumed reply (the `pending` guard) keeps the host handshake 1:1; ~125 Hz.
#
# SAFETY: motion is continuous, so a dead host would otherwise coast on a stale
# command. The miss watchdog stopj's hard (8 rad/s²) after ~8 missed reads (~64 ms);
# the host-side sentinel + amplification gates apply before anything is sent.
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
  elif mode == 3:
    speedl(qd, accel, step_t)
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
    if mode == 2 or mode == 3:
      qd = [p[2]/1000000.0, p[3]/1000000.0, p[4]/1000000.0, p[5]/1000000.0, p[6]/1000000.0, p[7]/1000000.0]
    elif mode == 1:
      q = [p[2]/1000000.0, p[3]/1000000.0, p[4]/1000000.0, p[5]/1000000.0, p[6]/1000000.0, p[7]/1000000.0]
    end
  else:
    misses = misses + 1
    if misses > 8:
      if mode >= 2:
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
<meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Telamoto motion tuning</title>
<style>
 body{font-family:system-ui,sans-serif;background:#111;color:#eee;margin:auto;
  padding:1.2rem 1.2rem 5.5rem;max-width:640px}
 .ok{background:#13391b;color:#6ee787}.bad{background:#3a1414;color:#ff7b72}
 .row{margin:1.5rem 0}
 .row label{display:flex;justify-content:space-between;font-size:1.05rem;margin-bottom:.45rem}
 .val{color:#4ea1ff;font-variant-numeric:tabular-nums}
 input[type=range]{width:100%;height:2.2rem}
 .hint{color:#888;font-size:.8rem;margin-top:.25rem}
 button{background:#222;color:#eee;border:1px solid #444;border-radius:.5rem;padding:.6rem 1.1rem;font-size:.9rem;margin-top:.6rem}
 /* Touch jog pads (mobile: no keyboard). touch-action:none stops scroll/zoom
    gestures from stealing the press mid-jog. */
 #pads{display:flex;gap:2rem;justify-content:center;align-items:center;margin:1rem 0}
 #padgrid{display:grid;grid-template-areas:". q ." "a t d" ". e .";gap:.45rem}
 #padz{display:flex;flex-direction:column;gap:.45rem}
 .jb{width:64px;height:64px;margin:0;padding:0;font-size:.95rem;border-radius:.7rem;
  background:#1c2733;border:1px solid #38506a;color:#cfe3ff;line-height:1.2;
  touch-action:none;user-select:none;-webkit-user-select:none}
 .jb.on{background:#2d5a8e;border-color:#4ea1ff}
 /* Rotate mode: the SAME pads drive the angular axes — purple tint as the cue. */
 #pads.rot .jb{background:#2b2138;border-color:#7a5cff;color:#dccdff}
 #pads.rot .jb.on{background:#6e2da8;border-color:#c89bff}
 #jb-q{grid-area:q}#jb-a{grid-area:a}#jb-d{grid-area:d}#jb-e{grid-area:e}
 /* Mode toggle in the grid CENTER (the old tilt-pad spot): one thumb-reach tap
    flips linear/rotation jog. Round + dashed = "not a jog pad"; NOT class .jb,
    so the collision-halt pad disabling and the pointer jog handlers skip it. */
 #rotpad{grid-area:t;width:64px;height:64px;margin:0;padding:0;font-size:.8rem;
  border-radius:50%;background:#15202b;border:1px dashed #4a5a70;color:#8fa8c8;
  touch-action:none;user-select:none;-webkit-user-select:none}
 #rotpad.on{background:#6e2da8;border:1px solid #c89bff;color:#dccdff}
 #pads.off .jb,#pads.off #rotpad{opacity:.35;pointer-events:none}
 .jb:disabled{opacity:.15;pointer-events:none}
 /* Big 3D view (mobile-first): most of the viewport, capped on desktop. */
 #view3d{width:100%;height:min(56vh,560px);height:min(56dvh,560px);
  background:#181818;border-radius:.5rem;display:none;overflow:hidden}
 /* Fullscreen 3D (body.fs): the view owns the viewport; a translucent top bar
    (status + frame/jog toggles) and the jog pads overlay it. CSS-fixed
    rather than the Fullscreen API — iPhone Safari doesn't allow that on divs. */
 /* Fullscreen top bar: one glassy strip — status dot, ping + transport,
    compact equal buttons, and (when active) the collision chip. */
 #fsbar{display:none;position:fixed;top:0;left:0;right:0;z-index:27;
  gap:.45rem;row-gap:.4rem;flex-wrap:wrap;align-items:center;justify-content:center;
  padding:calc(.55rem + env(safe-area-inset-top)) .65rem .55rem;
  background:#0e131adb;backdrop-filter:blur(12px);-webkit-backdrop-filter:blur(12px);
  border-bottom:1px solid #ffffff14}
 #fsbar button{flex:1 1 0;max-width:6.5rem;margin:0;padding:.55rem 0;text-align:center;
  background:#1c2733;border:1px solid #38506a;color:#cfe3ff;
  border-radius:.55rem;font-size:.85rem;white-space:nowrap}
 #fsbar button.on{background:#2d5a8e;border-color:#4ea1ff}
 #fsstat{flex:0 0 auto;font-size:.8rem;white-space:nowrap;background:none!important;
  padding:0 .2rem 0 0}
 #fsstat.ok{color:#6ee787}#fsstat.bad{color:#ff7b72}
 #fsping{flex:0 0 auto;font-size:.72rem;color:#9ab;white-space:nowrap;
  font-variant-numeric:tabular-nums;padding-right:.1rem}
 /* Webcam overlay: floats under the top bar, never over the pads; tap to
    toggle small ↔ large. Works in both layouts (position:fixed). */
 #camv{display:none;position:fixed;right:.5rem;z-index:26;background:#000;
  top:calc(3.6rem + env(safe-area-inset-top));width:min(44vw,340px);
  border:1px solid #38506a;border-radius:.6rem}
 #camv.big{width:min(94vw,720px);left:50%;right:auto;transform:translateX(-50%)}
 body.fs{overflow:hidden}
 body.fs #view3d{position:fixed;inset:0;z-index:25;height:100%;border-radius:0}
 body.fs #fsbar{display:flex}
 body.fs #pads{position:fixed;left:0;right:0;bottom:0;z-index:27;margin:0;
  justify-content:space-between;
  padding:0 max(1rem,env(safe-area-inset-left)) calc(1rem + env(safe-area-inset-bottom))
   max(1rem,env(safe-area-inset-right));pointer-events:none}
 body.fs #pads .jb,body.fs #pads #rotpad{pointer-events:auto;
  width:clamp(58px,16vw,76px);height:clamp(58px,16vw,76px)}
 body.fs #pads .jb{background:#1c2733e8;font-size:1rem}
 body.fs #pads.rot .jb{background:#2b2138e8}
 body.fs #pads.off .jb,body.fs #pads.off #rotpad{pointer-events:none}
 body.fs #panel{z-index:28}
 /* Tuning panel: bottom sheet toggled by the top-bar gear — keeps every
    slider one tap away while the 3D view + jog pads own the screen. */
 #panel{position:fixed;left:0;right:0;bottom:0;z-index:20;margin:auto;max-width:640px;
  background:#191f26;border:1px solid #333;border-bottom:none;border-radius:1rem 1rem 0 0;
  padding:0 1.2rem 1.2rem;max-height:75vh;max-height:75dvh;overflow-y:auto;
  transform:translateY(105%);transition:transform .25s ease;box-shadow:0 -4px 24px #000c}
 #panel.open{transform:none}
 #panelhdr{position:sticky;top:0;z-index:1;background:#191f26;display:flex;
  justify-content:space-between;align-items:center;padding:.8rem 0 .4rem}
 #panelhdr button{margin:0;padding:.35rem .8rem}
 #panel .row{margin:1.1rem 0}
 #fsCollAlert{display:none;flex-basis:100%;font-size:.78rem;line-height:1.5;
  padding:.35rem .65rem;border-radius:.45rem;border:1px solid transparent;
  max-width:560px;margin:0 auto;text-align:center}
 #fsCollAlert.warn{display:block;background:#3d3000;border-color:#e3b341;color:#e3b341}
 #fsCollAlert.halt{display:block;background:#3d1414;border-color:#ff7b72;color:#ff7b72}
</style></head>
<body class="fs">
 <!-- The fullscreen mobile layout is the ONLY interface — there is no classic
      scroll page. These hidden checkboxes are state holders: jog mode is forced
      ON at load (the SERVER-side jog-mode gate still exists for API/tests),
      Base/Tool + rotate are driven by the top bar + center pad, and the 3D view
      is always on. Diagnostics moved into the tuning panel. -->
 <input type="checkbox" id="viewon" style="display:none">
 <input type="checkbox" id="basef" style="display:none">
 <input type="checkbox" id="rotf" style="display:none">
 <input type="checkbox" id="jogon" style="display:none">
 <div id="view3d"></div>
 <div id="fsbar"><span id="fsstat" class="bad">&#9679;</span><span id="fsping">&mdash;</span>
   <button id="fswalls">walls</button>
   <button id="fscam">cam</button>
   <button id="fsframe">tool</button>
   <button id="fstune">&#9881;</button>
   <div id="fsCollAlert"></div></div>
 <video id="camv" autoplay muted playsinline></video>
 <div id="pads" class="off">
   <div id="padgrid">
     <button class="jb" id="jb-q" data-k="q"></button>
     <button class="jb" id="jb-a" data-k="a"></button>
     <button id="rotpad">rot</button>
     <button class="jb" id="jb-d" data-k="d"></button>
     <button class="jb" id="jb-e" data-k="e"></button>
   </div>
   <div id="padz">
     <button class="jb" id="jb-w" data-k="w"></button>
     <button class="jb" id="jb-s" data-k="s"></button>
   </div>
 </div>
 <div id="panel">
 <div id="panelhdr"><b>Tuning</b><button id="panelclose">&#10005; close</button></div>
 <div class="hint" id="joghint"></div>
 <div class="row"><label>Jog speed <span class="val" id="jspeedv"></span></label>
   <input type="range" id="jspeed" min="0.005" max="0.5" step="0.005">
   <div class="hint">Cartesian jog velocity &mdash; lower for mm-precise work. Live.</div></div>
 <div class="row"><label>Jog acceleration <span class="val" id="jaccelv"></span></label>
   <input type="range" id="jaccel" min="0.3" max="20" step="0.1">
   <div class="hint">Cartesian start/stop ramp &mdash; lower = gentler start/stop. Live.</div></div>
 <div class="row"><label>Rotate speed <span class="val" id="jrspeedv"></span></label>
   <input type="range" id="jrspeed" min="0.01" max="1" step="0.01">
   <div class="hint">TCP rotation jog speed (pan/tilt/roll &mdash; <b>I/K</b> <b>J/L</b> <b>U/O</b>
     or the pads with <b>rot</b> on). Live.</div></div>
 <div class="row"><label>Orientation hold <span class="val" id="okpv"></span></label>
   <input type="range" id="okp" min="0" max="30" step="0.5">
   <div class="hint">TCP orientation-hold stiffness &mdash; raise for tighter hold, lower if it
     waddles/oscillates; 0 = off. Live.</div></div>
 <div class="row"><label>Straight-line hold <span class="val" id="pkpv"></span></label>
   <input type="range" id="pkp" min="0" max="15" step="0.5">
   <div class="hint">Steers the TCP back onto the commanded line while jogging &mdash; raise for a
     tighter line, lower if it wiggles; 0 = off. Live.</div></div>
 <div class="row"><label>Self-collision distance <span class="val" id="selfdv"></span></label>
   <input type="range" id="selfd" min="0.01" max="0.5" step="0.01">
   <div class="hint">Jog slows when robot links get this close to each other, stops at
     contact. Live.</div></div>
 <div class="row"><label>Wall distance <span class="val" id="walldv"></span></label>
   <input type="range" id="walld" min="0.01" max="1" step="0.01">
   <div class="hint">Jog slows this far from scene objects (the cage walls), stops at
     contact. With the full cage keep this &le; ~10 cm or jog is permanently slowed.
     Live.</div></div>
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
 <div class="hint" id="pendant" style="border-top:1px solid #333;margin-top:1.1rem;padding-top:.9rem">pendant speed slider: &mdash;</div>
 <div class="hint" id="qd">joint speed: &mdash; &nbsp;|&nbsp; 1.5s peak: &mdash; rad/s</div>
 <div class="hint" id="link">link: &mdash;</div>
 </div>
<script type="importmap">
{"imports":{"three":"/static/three.module.js",
 "three/examples/jsm/loaders/STLLoader.js":"/static/STLLoader.js",
 "three/examples/jsm/loaders/ColladaLoader.js":"/static/ColladaLoader.js"}}
</script>
<script>
 // The UI is plain HTTP by DESIGN (LAN/VPN, no certs — see CLAUDE.md). If an
 // HTTPS proxy or a browser upgrade ever lands us here over https, bounce to
 // the http origin (the jog/twin WebSockets are pinned to ws:// and would be
 // mixed-content-blocked under https anyway). __WEB_PORT__ is injected by the
 // server at serve time = the real configured port.
 if(location.protocol==="https:")
   location.replace("http://"+location.hostname+":__WEB_PORT__"+location.pathname+location.hash);
 const fmt={speed:v=>(+v).toFixed(2)+"\\u00d7",gain:v=>Math.round(v),lookahead:v=>(+v).toFixed(3)+" s",
   jspeed:v=>Math.round(+v*1000)+" mm/s",jaccel:v=>(+v).toFixed(1)+" m/s\\u00b2",
   jrspeed:v=>Math.round(+v*57.2958)+"\\u00b0/s",
   okp:v=>(+v).toFixed(1),pkp:v=>(+v).toFixed(1),
   selfd:v=>Math.round(+v*100)+" cm",walld:v=>Math.round(+v*100)+" cm"};
 const ids=["speed","gain","lookahead","jspeed","jaccel","jrspeed","okp","pkp","selfd","walld"];
 function show(k,v){document.getElementById(k).value=v;document.getElementById(k+"v").textContent=fmt[k](v);}
 function send(k){const v=document.getElementById(k).value;
   document.getElementById(k+"v").textContent=fmt[k](v);fetch("/api/set?"+k+"="+v,{method:"POST"});}
 async function refreshStatus(){try{const t0=performance.now();
   const s=await(await fetch("/api/state")).json();
   const rtt=Math.round(performance.now()-t0);
   const fst=document.getElementById("fsstat");
   fst.textContent=s.connected?"\\u25CF":"\\u25CF press Play";
   fst.className=s.connected?"ok":"bad";
   document.getElementById("fscam").style.display=s.cam?"":"none";
   document.getElementById("pendant").textContent="pendant speed slider: "
     +(s.speedfrac==null?"\\u2014":Math.round(s.speedfrac*100)+"%");
   const qd=document.getElementById("qd");
   qd.textContent="joint speed: "+(s.qdnow==null?"\\u2014":s.qdnow.toFixed(3))
     +" \\u2014 1.5s peak: "+(s.qdpeak==null?"\\u2014":s.qdpeak.toFixed(3))+" rad/s";
   qd.style.color=(s.qdpeak>0.02)?"#ff7b72":"#888";
   // Link health: rtt is browser-measured (this fetch); gap/lease/scale come from
   // the server's jog-frame estimator. Amber/red = laggy link, jog auto-slowed.
   const lk=document.getElementById("link");
   lk.textContent="link: rtt "+rtt+" ms \\u00b7 frame gap "+s.linkgap+" ms \\u00b7 lease "
     +s.lease+" ms \\u00b7 jog speed \\u00d7"+s.linkscale.toFixed(2)
     +" \\u00b7 "+trans();
   lk.style.color=s.linkscale>0.99?"#888":(s.linkscale>0.3?"#e3b341":"#ff7b72");
   // Fullscreen ping: browser-measured rtt; appends the auto-slow factor when
   // the link estimator has scaled the jog down.
   const fp=document.getElementById("fsping");
   fp.textContent=rtt+" ms "+trans()
     +(s.linkscale>0.99?"":" \\u00d7"+s.linkscale.toFixed(2));
   fp.style.color=(rtt>250||s.linkscale<=0.3)?"#ff7b72"
     :(rtt>100||s.linkscale<=0.99)?"#e3b341":"#9ab";
   // Collision alert — updated in both normal and fullscreen elements.
   setCollAlert(s);}catch(e){}}
 // escInfo: base-frame key + pad label (base-frame pad) for each approach axis.
 const escInfo={
   "+X":{key:"D",pad:"\\u2212X"},"-X":{key:"A",pad:"+X"},
   "+Y":{key:"E",pad:"\\u2212Y"},"-Y":{key:"Q",pad:"+Y"},
   "+Z":{key:"S",pad:"\\u2212Z"},"-Z":{key:"W",pad:"+Z"}};
 const axLabel={"+X":"front","-X":"back","+Y":"left","-Y":"right","+Z":"up","-Z":"down"};
 const fmtWall=w=>w?(w==="wall_column"?"base column"
   :w.replace("wall_","").replace("_"," ")+" wall"):null;
 const isMobile=window.matchMedia("(pointer:coarse)").matches;
 function escHint(axis){
   const info=escInfo[axis];if(!info)return null;
   return isMobile?"tap <b>"+info.pad+"</b>":"press <b>"+info.key+"</b>";}
 // Escape info for backing out along base-frame `axis`: hint text (pointing
 // tool-frame users at the Base toggle first — escape keys are base-frame)
 // + the key for setPadBlock. null when the axis is unknown.
 function escOut(axis){
   const h=escHint(axis);if(!h)return null;
   const base=document.getElementById("basef").checked;
   return {hint:(base?h:"switch to <b>Base frame</b> and "+h)+" to back out",
     key:(escInfo[axis]||{}).key?.toLowerCase()??null};}
 // Disable all pad buttons except escKey (lowercase); re-enable all when escKey=null.
 // Also evicts any blocked key from `held` so a held-button mid-halt stops instantly.
 // Pads are compared via their EFFECTIVE key (padKey): in rotate mode every pad
 // sends a rotation key, none of which can be the (linear) escape.
 function setPadBlock(escKey){
   let changed=false;
   document.querySelectorAll("#pads .jb").forEach(b=>{
     const ek=padKey(b.dataset.k);
     const block=!!(escKey&&ek!==escKey);
     b.disabled=block;
     if(block&&held.has(ek)){held.delete(ek);changed=true;}
   });
   if(changed)sendJog();}
 function setCollAlert(s){
   let msg="",cls="",escKey=null;
   if(s.qdLatch){
     // Host-side runaway latch — checked FIRST: during the 2026-06-12 incident
     // the collision banner outranked this one, so the operator was told to
     // "back out of a wall" while the actual hazard was joint-speed
     // amplification, and kept tapping into it. Runaway is always the story
     // that must win the banner.
     msg="\\u26D4 joint-speed runaway \\u2014 jog stopped; release all keys, then jog"
       +" AWAY from the boundary";cls="halt";}
   else if(s.collBlocked&&s.collAxis){
     const wall=fmtWall(s.collWall)||"collision";   // no wall that way = e.g. self-collision
     const e=escOut(s.collAxis);
     msg="\\u26D4 "+wall+" ("+(axLabel[s.collAxis]||s.collAxis)+")"
       +(e?" &mdash; "+e.hint:"");cls="halt";escKey=e?.key??null;}
   else if(s.collBlocked){
     const near=fmtWall(s.nearWall);
     msg="\\u26D4 collision"+(near?" &mdash; nearest: "+near:"")
       +(isMobile?" &mdash; jog any direction to back out":" &mdash; jog any direction, or move wall in RViz");
     cls="halt";}
   else if(s.servoCode===2&&s.singAxis){
     // HALT_FOR_SINGULARITY with the approach direction captured (the JOG drove
     // in, so the halt fired at the boundary): same UI as a wall hit \\u2014 name the
     // direction and disable every pad except the escape. The reverse jog really
     // moves: Servo re-evaluates it as DECELERATE_FOR_LEAVING (\\u00d70.25) because
     // the sentinel feed is pre-gate.
     const e=escOut(s.singAxis);
     msg="\\u26D4 singularity ("+(axLabel[s.singAxis]||s.singAxis)+")"
       +(e?" &mdash; "+e.hint:"");cls="halt";escKey=e?.key??null;}
   else if(s.servoCode===2){
     // DEEP singularity halt (no approach direction \\u2014 the arm was parked here
     // by a planned move): EVERY direction halts by design and only a planned
     // move gets out. Shown even with nothing held \\u2014 the pose itself is the
     // condition. Pads stay enabled: Servo arbitrates which direction may
     // leave, we can't know it client-side.
     msg="\\u26D4 singularity \\u2014 jog halted; reverse to back out"
       +" (if stuck: planned move in RViz)";cls="halt";}
   else if(s.qdBlockAxis){
     // Post-latch directional block: only the inward cone is dead; the
     // opposite direction escapes with a relaxed guard, lateral moves pass
     // \\u2014 so pads stay enabled here (no escKey), unlike the full halts.
     const e=escOut(s.qdBlockAxis);
     msg="\\u26A0 singularity boundary \\u2014 "+(axLabel[s.qdBlockAxis]||s.qdBlockAxis)
       +" blocked"+(e?"; "+e.hint:"")+" (clears after 5 cm of travel)";cls="warn";}
   else if((held.size>0||wasMoving)&&s.servoGate!=null&&s.servoGate<1.0){
     // wasMoving covers the just-stopped frame. servoCode picks the
     // honest reason: 4=collision decel (scene OR self), 1/3=singularity, 6=bound.
     cls="warn";
     if(s.servoCode===4)
       msg="\\u26A0 collision slow-down"+(s.nearWall?" \\u2014 near "+fmtWall(s.nearWall):"")
         +" \\u00d7"+s.servoGate.toFixed(2);
     else if(s.servoCode===6)msg="\\u26A0 jog slowed \\u2014 joint limit";
     else if(s.servoCode===3)msg="\\u26A0 leaving singularity \\u2014 crawl \\u00d70.25";
     else msg="\\u26A0 jog slowed \\u2014 approaching singularity (reverse to clear)";}
   // Escape directions are LINEAR — drop the pads out of rotate mode so the
   // escape pad actually exists on screen.
   if(escKey&&rotOn){const r=document.getElementById("rotf");
     r.checked=false;r.dispatchEvent(new Event("change"));}
   setPadBlock(escKey);
   for(const id of["fsCollAlert"]){
     const el=document.getElementById(id);
     if(msg){el.className=cls;el.innerHTML=msg;}
     else{el.className="";el.textContent="";}}}
 const hints={tool:"<b>W/S</b> forward/back along the tool \\u00b7 <b>A/D</b> across \\u00b7 <b>Q/E</b> across (tool frame). Rotate: <b>I/K</b> tilt \\u00b7 <b>J/L</b> pan \\u00b7 <b>U/O</b> roll. Hold to move, release to stop.",
   base:"<b>A/D</b> \\u00b1X \\u00b7 <b>Q/E</b> \\u00b1Y \\u00b7 <b>W/S</b> \\u00b1Z \\u00b7 rotate <b>I/K</b> \\u00b1RX \\u00b7 <b>J/L</b> \\u00b1RY \\u00b7 <b>U/O</b> \\u00b1RZ (robot BASE frame). Hold to move, release to stop."};
 function showHint(){const b=document.getElementById("basef").checked;
   document.getElementById("joghint").innerHTML=hints[b?"base":"tool"];
   const fb=document.getElementById("fsframe");
   fb.classList.toggle("on",b);fb.textContent=b?"base":"tool";}
 async function init(){try{const s=await(await fetch("/api/state")).json();ids.forEach(k=>show(k,s[k]));
   document.getElementById("basef").checked=!!s.basef;
   }catch(e){}
   // Jog mode is ALWAYS ON from the page (no UI toggle since 2026-06-12): tell
   // the server so even if another client turned it off — and do it OUTSIDE
   // the state-fetch try, a failed first fetch must not leave jog dead.
   const jc=document.getElementById("jogon");jc.checked=true;
   jc.dispatchEvent(new Event("change"));
   showHint();padRelabel();
   ids.forEach(k=>document.getElementById(k).addEventListener("input",()=>send(k)));
   document.getElementById("basef").addEventListener("change",e=>{showHint();padRelabel();
     fetch("/api/jogframe?base="+(e.target.checked?1:0),{method:"POST"});});
   refreshStatus();setInterval(refreshStatus,300);}
 function reset(){show("speed",1);show("gain",300);show("lookahead",0.1);
   fetch("/api/set?speed=1&gain=300&lookahead=0.1",{method:"POST"});}
 // WASD jog: stream the direction as a CBOR binary frame at a fixed 30 Hz over a
 // WebSocket the whole time jog mode is ON (zeros while no key is held) — the
 // server measures the gap between frames to estimate link jitter and adapt the
 // jog lease/speed for laggy (remote) connections, so the stream must run even at
 // idle to keep that estimate warm. Paused while the tab is hidden: browsers
 // throttle background timers to ~1 Hz, which would poison the estimate.
 let jogOn=false; const held=new Set(); let stream=null, ws=null;
 let dc=null, rtcRetryT=null, seqN=0;          // UDP transport state (WebRTC)
 const trans=()=>dc&&dc.readyState==="open"?"udp":"tcp";   // live jog transport
 // tool frame: Z = along the tool (forward/back), X/Y = across the flange.
 // Keys give ±1, clamped (the server clamps again; never trust the client).
 // ax/ay/az are the ROTATION axes (I/K tilt, J/L pan, U/O roll — or the pads
 // with "rot" on).
 function jogVec(){const c=v=>Math.max(-1,Math.min(1,v));
   return {lx:c((held.has("a")?1:0)-(held.has("d")?1:0)),
     ly:c((held.has("q")?1:0)-(held.has("e")?1:0)),
     lz:c((held.has("w")?1:0)-(held.has("s")?1:0)),
     ax:c((held.has("i")?1:0)-(held.has("k")?1:0)),
     ay:c((held.has("j")?1:0)-(held.has("l")?1:0)),
     az:c((held.has("u")?1:0)-(held.has("o")?1:0))};}
 // ws:// unconditionally: the page redirects itself off https (top of script),
 // so a wss branch would only mask a misconfigured proxy.
 function wsOpen(){ws=new WebSocket("ws://"+location.host+"/ws");
   ws.onclose=()=>{ws=null;setTimeout(wsOpen,1000);};}
 // UDP transport: an unordered/no-retransmit WebRTC datachannel — on a lossy
 // WAN path TCP queues stale twists behind retransmits (head-of-line blocking),
 // UDP just loses them; the seq element lets the server drop reordered frames.
 // The WS above stays connected as the automatic fallback (and is the only path
 // when the server lacks aiortc: /api/rtc answers 503). Same frames, same
 // server-side _handle_jog_frame — a 2nd transport, never a 2nd command path.
 function rtcRetry(){if(!rtcRetryT)rtcRetryT=setTimeout(()=>{rtcRetryT=null;rtcOpen();},5000);}
 function rtcPc(){return new RTCPeerConnection({iceServers:[{urls:"stun:stun.l.google.com:19302"}]});}
 async function rtcSignal(pc){                 // non-trickle: offer → /api/rtc → answer
   await pc.setLocalDescription(await pc.createOffer());
   await new Promise(res=>{                    // wait for the full candidate set,
     if(pc.iceGatheringState==="complete")return res();
     const t=setTimeout(res,2000);             // but cap the wait (srflx is slow)
     pc.addEventListener("icegatheringstatechange",
       ()=>{if(pc.iceGatheringState==="complete"){clearTimeout(t);res();}});});
   const r=await fetch("/api/rtc",{method:"POST",
     body:JSON.stringify({sdp:pc.localDescription.sdp,type:pc.localDescription.type})});
   if(!r.ok)throw 0;
   await pc.setRemoteDescription(await r.json());}
 function rtcOpen(){
   if(!window.RTCPeerConnection)return;        // http page without RTC → WS only
   let pc;
   try{pc=rtcPc();}catch(e){return;}
   const ch=pc.createDataChannel("jog",{ordered:false,maxRetransmits:0});
   ch.binaryType="arraybuffer";
   let down=false;
   const fail=()=>{if(down)return;down=true;if(dc===ch)dc=null;
     try{pc.close();}catch(e){} rtcRetry();};
   ch.onopen=()=>{dc=ch;};
   ch.onclose=fail; ch.onerror=fail;
   pc.onconnectionstatechange=()=>{
     if(pc.connectionState==="failed"||pc.connectionState==="closed")fail();};
   rtcSignal(pc).catch(fail);}
 rtcOpen();
 // Webcam viewer: a SEPARATE recvonly peer connection (the jog channel is
 // never renegotiated; video over RTP/UDP = the lowest-latency feed a browser
 // can play). Zero playout/jitter buffering — the server already drops frames
 // a slow link can't carry (relay buffered=false), so buffering only adds lag.
 let camPc=null;
 function camStop(){const pc=camPc;camPc=null;
   if(pc){try{pc.close();}catch(e){}}
   const v=document.getElementById("camv");
   v.srcObject=null;v.style.display="none";
   document.getElementById("fscam").classList.remove("on");}
 async function camStart(){
   const pc=camPc=rtcPc();
   document.getElementById("fscam").classList.add("on");   // instant feedback;
   const tr=pc.addTransceiver("video",{direction:"recvonly"});  // camStop clears
   try{tr.receiver.jitterBufferTarget=0;}catch(e){}
   try{tr.receiver.playoutDelayHint=0;}catch(e){}
   // Every late event guards on camPc===pc: after a quick off→on toggle the
   // OLD connection's ontrack/state events still fire and used to attach a
   // dead stream to the element (black screen). Reveal on the TRACK's unmute
   // (= first RTP packet, fires regardless of element state) with a timed
   // fallback — NEVER gate on video-element events like loadeddata: Chrome
   // doesn't decode for a display:none element, so that gating deadlocked
   // (hidden until a frame, no frame while hidden = "no feed at all").
   pc.ontrack=e=>{if(camPc!==pc)return;
     const v=document.getElementById("camv");
     v.srcObject=e.streams[0];
     const show=()=>{if(camPc===pc&&v.style.display!=="block"){
       v.style.display="block";v.play().catch(()=>{});}};
     e.track.onunmute=show;
     setTimeout(show,1500);};
   pc.onconnectionstatechange=()=>{
     if((pc.connectionState==="failed"||pc.connectionState==="closed")&&camPc===pc)camStop();};
   try{await rtcSignal(pc);}
   catch(e){if(camPc===pc)camStop();return;}    // fail of THIS attempt only
   if(camPc!==pc){try{pc.close();}catch(e){}}   // toggled off mid-signaling
   }
 document.getElementById("fscam").addEventListener("click",
   ()=>{camPc?camStop():camStart();});
 document.getElementById("camv").addEventListener("click",   // tap: small ↔ large
   e=>e.target.classList.toggle("big"));
 // Minimal CBOR (RFC 8949): encode [lx,ly,lz] as an array of small signed ints.
 function cborInt(n){n=Math.round(n);const m=n<0?0x20:0x00,u=n<0?-1-n:n;
   if(u<24)return[m|u];if(u<256)return[m|24,u];return[m|25,(u>>8)&0xff,u&0xff];}
 function cborNum(n){if(Number.isInteger(n))return cborInt(n);   // analog → float32
   const d=new DataView(new ArrayBuffer(4));d.setFloat32(0,n);
   return [0xfa,d.getUint8(0),d.getUint8(1),d.getUint8(2),d.getUint8(3)];}
 // 7th element: mod-2^16 sequence number — the server drops a frame older than
 // the newest seen, so UDP reordering can't replay a pre-release twist after
 // the keyup zero (harmless no-op on the ordered WS path).
 function cborJog(v){seqN=(seqN+1)&0xffff;
   return new Uint8Array([0x87].concat(cborNum(v.lx),cborNum(v.ly),cborNum(v.lz),
   cborNum(v.ax),cborNum(v.ay),cborNum(v.az),cborInt(seqN)));}
 // Idle ticks send a 1-byte KEEPALIVE (CBOR empty array): it feeds the server's
 // link-jitter estimator but never the jog command — so a second open tab can't
 // fight the active tab's jog with a 30 Hz stream of zeros (= choppy motion).
 // A real [0,0,0] is still sent on the moving->stopped transition (instant stop).
 const KEEPALIVE=new Uint8Array([0x80]); let wasMoving=false;
 function sendJog(){
   const v=jogVec(),m=!!(v.lx||v.ly||v.lz||v.ax||v.ay||v.az);
   const f=m||wasMoving?cborJog(v):KEEPALIVE;wasMoving=m;
   if(dc&&dc.readyState==="open"){try{dc.send(f);return;}catch(e){}}  // UDP first,
   if(ws&&ws.readyState===1)ws.send(f);}                              // WS fallback
 function startStream(){if(!stream&&!document.hidden)stream=setInterval(sendJog,33);}   // ~30 Hz
 function stopStream(){if(stream){clearInterval(stream);stream=null;}}
 function stopAll(){held.clear();clearPads();stopStream();sendJog();}  // sends zero
 document.getElementById("jogon").addEventListener("change",e=>{jogOn=e.target.checked;
   document.getElementById("pads").classList.toggle("off",!jogOn);
   fetch("/api/jogmode?on="+(jogOn?1:0),{method:"POST"});if(jogOn)startStream();else stopAll();});
 document.addEventListener("keydown",e=>{if(!jogOn)return;const k=e.key.toLowerCase();
   if("wasdqeikjluo".includes(k)&&!held.has(k)){held.add(k);e.preventDefault();sendJog();startStream();}});
 document.addEventListener("keyup",e=>{if(!jogOn)return;const k=e.key.toLowerCase();
   if(held.has(k)){held.delete(k);sendJog();}});
 window.addEventListener("blur",()=>{if(jogOn){held.clear();clearPads();sendJog();}});
 document.addEventListener("visibilitychange",()=>{if(document.hidden){held.clear();clearPads();
   sendJog();stopStream();}else if(jogOn)startStream();});
 // Touch jog pads (mobile): hold-to-jog buttons feeding the SAME `held` set and
 // 30 Hz stream as the keyboard, so every fail-safe (lease, keepalive isolation,
 // blur/hidden clearing) applies unchanged.
 const padLabels={tool:{w:"\\u25b2 fwd",s:"\\u25bc back",a:"\\u25c0",d:"\\u25b6",q:"\\u2191 up",e:"\\u2193 down"},
   base:{w:"+Z \\u25b2",s:"\\u2212Z \\u25bc",a:"+X",d:"\\u2212X",q:"+Y",e:"\\u2212Y"},
   rtool:{w:"\\u27f2 roll",s:"\\u27f3 roll",a:"\\u25c0 pan",d:"pan \\u25b6",q:"tilt \\u25b2",e:"tilt \\u25bc"},
   rbase:{w:"+RZ",s:"\\u2212RZ",a:"+RY",d:"\\u2212RY",q:"+RX",e:"\\u2212RX"}};
 // Rotate mode: the SAME pads (and the same `held`/stream path) drive the angular
 // axes — each pad's key is remapped to its rotation twin.
 const rotMap={q:"i",e:"k",a:"j",d:"l",w:"u",s:"o"};
 let rotOn=false;
 function padKey(k){return rotOn&&rotMap[k]?rotMap[k]:k;}
 // Desktop (mouse-primary): the keyboard key rides above the label so the pads
 // double as a WASD/IJKL legend — the fullscreen layout is the default view there too.
 function padRelabel(){const base=document.getElementById("basef").checked;
   const L=padLabels[(rotOn?"r":"")+(base?"base":"tool")];
   document.querySelectorAll("#pads .jb").forEach(b=>{const k=b.dataset.k;
     b.innerHTML=(!isMobile?"<small style='opacity:.55'>"+padKey(k).toUpperCase()+"</small><br>":"")+L[k];});}
 function clearPads(){document.querySelectorAll("#pads .jb.on").forEach(b=>b.classList.remove("on"));}
 function press(k,on){if(!jogOn)return;
   if(on&&!held.has(k)){held.add(k);sendJog();startStream();}
   else if(!on&&held.has(k)){held.delete(k);sendJog();}}
 document.querySelectorAll("#pads .jb").forEach(b=>{const k=b.dataset.k;
   b.addEventListener("pointerdown",e=>{e.preventDefault();
     try{b.setPointerCapture(e.pointerId);}catch(err){}
     b.classList.add("on");b._k=padKey(k);press(b._k,true);});   // remember the SENT
   const up=()=>{b.classList.remove("on");if(b._k)press(b._k,false);};  // key: a mode
   b.addEventListener("pointerup",up);b.addEventListener("pointercancel",up);  // flip
   b.addEventListener("contextmenu",e=>e.preventDefault());});  // mid-press must still
                                                                // release the same key
 document.getElementById("rotf").addEventListener("change",e=>{rotOn=e.target.checked;
   document.getElementById("pads").classList.toggle("rot",rotOn);
   document.getElementById("rotpad").classList.toggle("on",rotOn);
   held.clear();clearPads();sendJog();              // never carry motion across a remap
   padRelabel();});
 padRelabel();
 // (Tilt-to-jog — hold the center pad and tilt the phone — was removed 2026-06-12;
 // the server still accepts analog float axes in the CBOR frame, so an analog
 // input source can come back without a protocol change.)
 // 3D twin: loaded lazily on first enable (the three.js stack is ~1.4 MB once,
 // then browser-cached) — slider-only sessions never pay for it.
 let twin=null;
 document.getElementById("viewon").addEventListener("change",async e=>{
   const d=document.getElementById("view3d");
   if(e.target.checked){d.style.display="block";
     try{if(twin)twin.resume();
       // ?v= busts copies cached before viewer.js became no-cache.
       else{twin=(await import("/static/viewer.js?v=2")).initTwin(d);
         if(twin&&!wallsOn)twin.walls(false);}}
     catch(err){d.style.height="auto";
       d.innerHTML="<div style='padding:1rem;color:#ff7b72;font-size:.85rem'>3D view failed: "
         +err.message+"</div>";}}
   else{d.style.display="none";if(twin)twin.pause();}});
 // The fullscreen layout IS the interface: the 3D view fills the viewport, the
 // jog pads + a slim top bar (status, frame/jog toggles that proxy the hidden
 // checkboxes) overlay it. setFS just guarantees the 3D view is on.
 function setFS(on){document.body.classList.toggle("fs",on);
   if(on){const v=document.getElementById("viewon");
     if(!v.checked){v.checked=true;v.dispatchEvent(new Event("change"));}}}
 // Safety-wall visibility (3D view ONLY — collision checking is unaffected).
 let wallsOn=false;
 function setWalls(on){wallsOn=on;
   document.getElementById("fswalls").classList.toggle("on",on);
   if(twin&&twin.walls)twin.walls(on);}   // guard: a cache-stale twin has no walls()
 document.getElementById("fswalls").addEventListener("click",()=>setWalls(!wallsOn));
 const fsProxy=(btn,box)=>document.getElementById(btn).addEventListener("click",()=>{
   const c=document.getElementById(box);c.checked=!c.checked;
   c.dispatchEvent(new Event("change"));});
 fsProxy("fsframe","basef");fsProxy("rotpad","rotf");
 // Tuning panel (bottom sheet): keeps the screen for the 3D view + jog pads.
 const panel=document.getElementById("panel");
 document.getElementById("fstune").addEventListener("click",()=>panel.classList.toggle("open"));
 document.getElementById("panelclose").addEventListener("click",()=>panel.classList.remove("open"));
 wsOpen(); init();
 // Deep link …:8080/#tune opens with the tuning panel already up.
 if(location.hash==="#tune")panel.classList.add("open");
 // The fullscreen mobile layout is the only interface on EVERY device (body
 // ships class="fs"); this turns the 3D view on at boot.
 setFS(true);
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


def _static_dir() -> str:
    """Directory of the vendored web assets (three.js, urdf-loader, viewer.js):
    the installed share dir, or the source tree when running uninstalled."""
    try:
        d = os.path.join(get_package_share_directory("telamoto_bringup"), "web", "static")
        if os.path.isdir(d):
            return d
    except Exception:
        pass
    return os.path.normpath(os.path.join(
        os.path.dirname(os.path.abspath(__file__)), os.pardir, "web", "static"))


def _quat_mul(a, b):
    """Hamilton product of two quaternions, each as (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_conj(q):
    """Conjugate = inverse for a unit quaternion. (x,y,z,w) → (-x,-y,-z,w)."""
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_to_rotvec(q):
    """Unit quaternion (x,y,z,w) → rotation vector (axis × angle), expressed in the
    quaternion's own frame. Picks the shorter rotation (|angle| ≤ π)."""
    x, y, z, w = q
    if w < 0.0:                         # shorter way round
        x, y, z, w = -x, -y, -z, -w
    v = math.sqrt(x * x + y * y + z * z)
    if v < 1e-9:                        # ~identity → no rotation
        return (0.0, 0.0, 0.0)
    s = 2.0 * math.atan2(v, w) / v
    return (x * s, y * s, z * s)


def _quat_rotate(q, v):
    """Rotate vector v=(x,y,z) by unit quaternion q=(x,y,z,w). Used to express a
    tool-frame jog direction in the base frame."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # t = 2 * (q.xyz × v); v' = v + qw*t + q.xyz × t
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def _quat_to_rpy_deg(q):
    """Unit quaternion (x,y,z,w) → roll/pitch/yaw in degrees (ZYX convention).
    Human-readable orientation for the jog log."""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))     # clamp for asin domain
    pitch = math.asin(sp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


def _quat_angle_deg(a, b):
    """Angle in degrees between two orientations (x,y,z,w), or None if either is
    missing. Used to report how far the TCP orientation drifted over a jog."""
    if a is None or b is None:
        return None
    rx, ry, rz = _quat_to_rotvec(_quat_mul(_quat_conj(a), b))
    return math.degrees(math.sqrt(rx * rx + ry * ry + rz * rz))


def _cbor_decode(buf, i=0):
    """Minimal RFC 8949 decoder for the jog frame: ints, arrays, and floats
    (major types 0, 1, 4, 7) — floats carry analog jog axes.
    Returns (value, next_index)."""
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
    if major == 7 and minor == 26:                    # IEEE 754 single
        return struct.unpack(">f", val.to_bytes(4, "big"))[0], i
    if major == 7 and minor == 27:                    # IEEE 754 double
        return struct.unpack(">d", val.to_bytes(8, "big"))[0], i
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
        self.declare_parameter("rtc_stun", RTC_STUN)   # "" = host candidates only
        self.declare_parameter("cam_device", CAM_DEVICE)   # "" = no webcam track
        self.declare_parameter("cam_size", CAM_SIZE)
        self.declare_parameter("cam_fps", CAM_FPS)
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
            description="WASD jog Cartesian start/stop ramp m/s^2; lower = gentler (live)",
            floating_point_range=[FloatingPointRange(
                from_value=JOG_ACCEL_MIN, to_value=JOG_ACCEL_MAX, step=0.1)]))
        self.declare_parameter("jog_rot_speed", JOG_ROT_SPEED_DEF, ParameterDescriptor(
            description="rotational jog speed rad/s (pan/tilt/roll, live)",
            floating_point_range=[FloatingPointRange(
                from_value=JOG_ROT_SPEED_MIN, to_value=JOG_ROT_SPEED_MAX, step=0.01)]))
        self.declare_parameter("step_t", STEP_T, ParameterDescriptor(
            description="speedl/servoj duration per cycle s; higher = better speed "
                        "tracking, lower update rate (live)",
            floating_point_range=[FloatingPointRange(
                from_value=STEP_T_MIN, to_value=STEP_T_MAX, step=0.001)]))
        self.declare_parameter("orient_lock_kp", ORIENT_KP_DEF, ParameterDescriptor(
            description="TCP orientation-hold gain; higher = stiffer but waddles, "
                        "lower = softer (live)",
            floating_point_range=[FloatingPointRange(
                from_value=ORIENT_KP_MIN, to_value=ORIENT_KP_MAX, step=0.5)]))
        self.declare_parameter("path_lock_kp", PATH_KP_DEF, ParameterDescriptor(
            description="TCP straight-line hold gain; higher = tighter line, "
                        "lower if it oscillates; 0 = off (live)",
            floating_point_range=[FloatingPointRange(
                from_value=PATH_KP_MIN, to_value=PATH_KP_MAX, step=0.5)]))
        self.declare_parameter("self_collision_dist", SELF_COLL_DEF, ParameterDescriptor(
            description="Servo self-collision proximity m: jog slows inside this "
                        "link-to-link distance, halts at contact (live)",
            floating_point_range=[FloatingPointRange(
                from_value=SELF_COLL_MIN, to_value=SELF_COLL_MAX, step=0.01)]))
        self.declare_parameter("scene_collision_dist", SCENE_COLL_DEF, ParameterDescriptor(
            description="Servo scene-collision proximity m: jog slows inside this "
                        "distance to world objects (e.g. test wall), halts at "
                        "contact (live)",
            floating_point_range=[FloatingPointRange(
                from_value=SCENE_COLL_MIN, to_value=SCENE_COLL_MAX, step=0.01)]))

        g = self.get_parameter
        self._robot_ip = g("robot_ip").get_parameter_value().string_value
        self._port     = g("reverse_port").get_parameter_value().integer_value
        self._web_port = g("web_port").get_parameter_value().integer_value
        self._rtc_stun = g("rtc_stun").get_parameter_value().string_value
        self._cam_device = g("cam_device").get_parameter_value().string_value
        self._cam_size   = g("cam_size").get_parameter_value().string_value
        self._cam_fps    = g("cam_fps").get_parameter_value().integer_value
        self._gain      = g("servoj_gain").get_parameter_value().integer_value
        self._lookahead = g("servoj_lookahead").get_parameter_value().double_value
        self._speed     = g("speed_scale").get_parameter_value().double_value
        self._jog_speed = g("jog_speed").get_parameter_value().double_value
        self._jog_accel = g("jog_accel").get_parameter_value().double_value
        self._jog_rot_speed = g("jog_rot_speed").get_parameter_value().double_value
        self._step_t    = g("step_t").get_parameter_value().double_value
        self._orient_kp = g("orient_lock_kp").get_parameter_value().double_value
        self._path_kp   = g("path_lock_kp").get_parameter_value().double_value
        self._self_coll  = g("self_collision_dist").get_parameter_value().double_value
        self._scene_coll = g("scene_collision_dist").get_parameter_value().double_value
        self.add_on_set_parameters_callback(self._on_set_params)
        # Collision distances live in /servo_node — coalesce slider drags and push
        # via its parameter service. Dirty at startup so the first push syncs servo
        # to OUR values regardless of what ur_servo.yaml shipped.
        self._servo_param_cli = self.create_client(SetParameters, "/servo_node/set_parameters")
        self._servo_params_dirty = True
        self._servo_srv_was_ready = False
        self.create_timer(0.25, self._push_collision_params)

        self._js_ready = threading.Event()           # set on first JointState
        self._traj = None                             # (waypoints, total_time, t_start)
        self._traj_lock = threading.Lock()
        self._traj_done = threading.Event()
        self._conn: socket.socket | None = None       # live reverse socket
        self._conn_lock = threading.Lock()

        # WASD jog: the web streams per-axis direction at ~30 Hz while a key is
        # held; each command leases the twist for JOG_LEASE, so it zeroes itself on
        # release / latency / drop. Open-loop velocity is the natural primitive for
        # human-in-the-loop teleop (no reference pose to snap back to).
        self._jog = [0.0] * 6
        self._jog_lease = 0.0                          # twist valid until this time
        # Link-health estimator for the adaptive lease (see JOG_LEASE_MAX block):
        # recent-max gap between received jog frames (peak-hold, decaying) and the
        # lease/speed-scale derived from it. With several WS clients streaming at
        # once the interleaved gaps look better than any single link — the same
        # last-writer-wins ambiguity the jog command itself already has.
        self._link_last_t = 0.0
        self._link_big_t = 0.0
        self._link_gap = LINK_GAP_NOMINAL
        self._link_lease = JOG_LEASE
        self._link_scale = 1.0
        self._jog_mode = True                          # web toggle: stay in jog mode
        self._jog_base = True                          # web toggle: jog axes in BASE
                                                       # frame (True) or TOOL (False)
        self._pub_lock = threading.Lock()              # serialise twist publishes
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
        # Full-rate TCP path log: every /joint_states (125 Hz RTDE rate) we append the
        # exact TCP pose to a CSV (NOT the ROS log — that would flood it / add latency).
        # Truncated each run so "latest" is always this file. Read it back to analyse
        # path straightness at full resolution. See _js_cb.
        self._tcp_csv = None
        self._tcp_csv_n = 0
        try:
            self._tcp_csv_path = os.path.join(os.path.expanduser("~/.ros/log"), "tcp_path.csv")
            self._tcp_csv = open(self._tcp_csv_path, "w", buffering=1 << 16)
            self._tcp_csv.write("t_mono,moving,cmd_mm_s,x_mm,y_mm,z_mm,qx,qy,qz,qw\n")
        except Exception as exc:
            self.get_logger().warn(f"[log] TCP path csv disabled: {exc}")
        # Cartesian start/stop ramp: the jog twist (linear + angular, in the
        # selected jog frame) actually commanded, ramped toward the target
        # self._jog at jog_accel m/s² (× JOG_ROT_RAMP for the angular axes) in
        # _jog_loop. Ramping the TWIST magnitude preserves the commanded
        # DIRECTION (a per-joint ramp would distort it and bend the path).
        self._jog_cmd = [0.0] * 6

        # Orientation hold for the jog: TF gives the live tool0 pose in base_link;
        # _orient_target is the (x,y,z,w) orientation captured when a jog starts and
        # is None whenever stopped (→ re-captured on each new key press). See
        # _publish_twist / _orient_lock_angular.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._orient_target = None
        self._jog_start_orient = None      # locked orientation at jog start, for the
                                           # before/after drift log in _jog_loop
        self._jog_stop_t = 0.0             # monotonic t the jog last stopped (gates
                                           # re-capture: see REORIENT_IDLE)
        self._jog_start_pos = None         # TCP position at jog start + the commanded
        self._jog_dir = None               # direction (base frame, unit) → lets the
                                           # trace measure how far the PATH bends off
                                           # the commanded straight line (lat/angle)
        self._path_i = [0.0, 0.0, 0.0]     # integral of lateral error (base frame,
                                           # m/s) — the learned disturbance velocity;
                                           # reset on baseline anchor + at standstill
        self._path_i_t = time.monotonic()  # last integration timestamp

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
        # 3D twin for the web UI: the browser renders the robot ITSELF (three.js +
        # urdf-loader) from the URDF and a ~25 Hz binary joint/scene stream — a few
        # kB/s instead of streaming pixels, so it stays smooth on the same laggy
        # links the jog compensation targets. robot_state_publisher latches the
        # URDF on /robot_description; scene_wall keepalives the wall as a BOX
        # CollisionObject on /planning_scene (1 Hz).
        self._urdf = None                  # served at /api/urdf
        self._q_actual = None              # latest joint positions, UR_JOINT_ORDER
        self._js_names = None              # joint-name → UR-order cache for _js_cb
        self._js_map = None
        self._scene_boxes = {}             # id → 10 floats: xyz quat(xyzw) size
        self._web_static = _static_dir()
        self.create_subscription(String, "/robot_description", self._urdf_cb, latched)
        self.create_subscription(PlanningScene, "/planning_scene", self._scene_cb, 10)
        self._twist_pub = self.create_publisher(TwistStamped, SERVO_TWIST_TOPIC, ctrl_qos)
        # SAFETY SENTINEL: MoveIt Servo receives every jog twist and keeps evaluating
        # the kinematics, but its joint output is unused (the robot executes speedl) —
        # its status gates the speedl command (slow near a singularity, zero at the
        # hard threshold), so the singularity protection from the incident stays live.
        self._servo_gate = 1.0
        self._qd_gate = 1.0          # smooth singularity-amplification gate [0,1]
        self._qd_guard_t = 0.0
        # Amplification-guard state: the post-gate twist magnitudes actually SENT
        # last cycle (the allowance baseline) and the runaway latch (see the
        # QD_LATCH_* block). A spurious latch (e.g. jogging the instant a planned
        # move's joints are still settling) errs SAFE and self-clears on release.
        self._sent_lin = 0.0
        self._sent_rot = 0.0
        self._qd_latched = False
        self._qd_block_dir = None    # inward (base-frame unit) direction captured
        self._qd_block_pos = None    # at the runaway latch + the TCP point — the
                                     # post-latch directional block (QD_BLOCK_*)
        self._qd_block_log_t = 0.0
        self._code_log_t = 0.0       # throttle for same-gate status-code changes
        # Halt-escape state: the base_link-frame jog linear command last
        # streamed (pre-gate) + its time, the approach direction captured at a
        # collision/singularity halt (None = no escape direction known), the
        # just-cleared direction (anti-inversion guard in _halt_dir), and a
        # log throttle.
        self._last_lin = (0.0, 0.0, 0.0)
        self._last_lin_t = 0.0
        self._coll_block_dir = None
        self._coll_prev = None        # (dir, t) at the last collision-halt clear
        self._coll_near_wall = None   # wall name at last collision halt
        self._sing_block_dir = None   # approach dir at a singularity halt (web UI
        self._sing_prev = None        # escape hint only — Servo gates the motion)
        self._servo_code = ServoStatus.NO_WARNING
        self._escape_log_t = 0.0
        # Sentinel liveness: jog is only allowed while Servo's status is FRESH.
        # Starts at 0 → jog blocked until the first status arrives, and a dead
        # servo_node fails SAFE (gate 0) instead of coasting on the last verdict.
        self._servo_status_t = 0.0
        self._sentinel_warn_t = 0.0
        self._https_warn_t = 0.0     # throttle for refused-HTTPS log lines
        self._web_seen = {}          # client ip -> last-seen t (connect logging)
        # WebRTC jog transport: a dedicated asyncio loop thread for aiortc —
        # signaling requests hop onto it via run_coroutine_threadsafe. None =
        # aiortc missing, /api/rtc answers 503 and the page stays on the WS.
        self._rtc_pcs = set()
        self._rtc_loop = None
        # Webcam: opened lazily on the FIRST video viewer, closed when the
        # LAST one disconnects — no capture/encode CPU while nobody watches.
        self._cam_player = None
        self._cam_pcs = set()
        if RTCPeerConnection is not None:
            self._cam_relay = MediaRelay()
            self._rtc_loop = asyncio.new_event_loop()
            threading.Thread(target=self._rtc_loop.run_forever,
                             daemon=True, name="ri-rtc").start()
        else:
            self.get_logger().warn(
                "[web] aiortc not installed — jog runs WS-only (TCP)")
        self.create_subscription(ServoStatus, SERVO_STATUS_TOPIC, self._servo_status_cb, 10)
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
                v = float(p.value)
                if not math.isfinite(v):     # nan slips through _clamp as MAX
                    continue
                setattr(self, attr, cast(_clamp(lo, hi, v)))
                if attr in _SERVO_PARAM_ATTRS:
                    self._servo_params_dirty = True
        return SetParametersResult(successful=True)

    def _web_set(self, q: dict) -> None:
        for _, web, attr, lo, hi, cast in _TUNE:
            if web in q:
                v = float(q[web][0])
                if not math.isfinite(v):     # "?speed=nan" must not clamp to MAX
                    continue
                setattr(self, attr, cast(_clamp(lo, hi, v)))
                if attr in _SERVO_PARAM_ATTRS:
                    self._servo_params_dirty = True

    def _push_collision_params(self) -> None:
        # 4 Hz coalescer: slider "input" events fire per pixel of drag; servo only
        # needs the latest value. Stays dirty (retries) until servo's parameter
        # service is up, so values survive a servo_node restart race.
        ready = self._servo_param_cli.service_is_ready()
        # servo_node restarted (service came back): its YAML defaults may not match
        # the current sliders — re-push ours.
        if ready and not self._servo_srv_was_ready:
            self._servo_params_dirty = True
        self._servo_srv_was_ready = ready
        if not self._servo_params_dirty or not ready:
            return
        self._servo_params_dirty = False
        req = SetParameters.Request(parameters=[
            RclpyParameter(name, RclpyParameter.Type.DOUBLE,
                           float(getattr(self, attr))).to_parameter_msg()
            for attr, name in _SERVO_PARAM_ATTRS.items()
        ])
        self._servo_param_cli.call_async(req)
        self.get_logger().info(
            f"[jog] collision distances → self {self._self_coll:.2f} m, "
            f"scene {self._scene_coll:.2f} m (pushed to servo)")

    def _apply_jog(self, lx: float, ly: float, lz: float,
                   ax: float = 0.0, ay: float = 0.0, az: float = 0.0) -> None:
        # Linear + angular axes in [-1,1] (selected jog frame), decoded from the
        # CBOR jog frame (legacy 3-element frames leave the rotation at zero).
        # Lease model: each message renews the twist for the (link-adaptive) lease;
        # a zero msg (key release) stops instantly, and a stalled/closed socket
        # lets the lease expire so the robot stops on its own.
        # SERVER-side jog-mode gate: the browser also checks its toggle, but any
        # WS client can send frames — without this, jog would move the robot with
        # the sentinel feed off (no fresh Servo status). Never trust the client.
        axes = (lx, ly, lz, ax, ay, az)
        # Floats can arrive from the wire (analog axes): a NaN would slip THROUGH
        # _clamp (NaN comparisons are false → full speed) — drop to zero.
        if not self._jog_mode or not all(map(math.isfinite, axes)):
            axes = (0.0,) * 6
        now = self._link_tick()
        sp  = self._jog_speed * self._link_scale
        rsp = self._jog_rot_speed * self._link_scale   # same scale → the blind-
                                                       # travel invariant holds for
                                                       # rotation too
        self._jog = [s * _clamp(-1.0, 1.0, v)
                     for s, v in zip((sp, sp, sp, rsp, rsp, rsp), axes)]
        self._jog_lease = now + self._link_lease
        self._publish_twist()                       # publish NOW, don't wait for the loop

    def _link_tick(self) -> float:
        """Feed the link-health estimator with one frame arrival (jog command or
        idle keepalive) and refresh the adaptive lease/speed-scale — see the
        JOG_LEASE_MAX block. Returns the current monotonic time. NEVER touches
        the jog twist or its lease: a keepalive must not keep a stale command
        alive. The Cartesian ramp (jog_accel) smooths any scale change mid-hold."""
        now = time.monotonic()
        gap = now - self._link_last_t
        self._link_last_t = now
        if gap < LINK_GAP_RESET:
            decayed = self._link_gap * math.exp(-gap / LINK_GAP_TAU)
            if gap > LINK_GAP_TOL:
                # De-twitch: only the SECOND big gap inside LINK_BIG_WINDOW feeds
                # the peak — an isolated spike costs at most one old-style lease
                # expiry instead of seconds of reduced jog speed (= choppiness).
                if now - self._link_big_t <= LINK_BIG_WINDOW:
                    decayed = max(decayed, gap)
                self._link_big_t = now
            self._link_gap = max(LINK_GAP_NOMINAL, decayed)
            excess = max(0.0, self._link_gap - LINK_GAP_TOL)
            self._link_lease = _clamp(JOG_LEASE, JOG_LEASE_MAX,
                                      JOG_LEASE + LINK_LEASE_FACTOR * excess)
            self._link_scale = JOG_LEASE / self._link_lease
        return now

    def _handle_jog_frame(self, data: bytes, st: dict) -> None:
        """One binary jog frame from EITHER transport — the /ws read loop and
        the RTC datachannel share this single decode path so every fail-safe
        (jog-mode gate, NaN rejection, lease, link estimator) stays transport-
        agnostic. `st` is per-connection state: the RTC channel is unordered/
        no-retransmit by design, so frames carry a 7th element — a mod-2^16
        sequence number — and a frame older than the newest seen is DROPPED
        (a reordered pre-release twist must never resurrect motion after the
        keyup zero). WS frames carry the seq too, but TCP ordering means the
        check can never trip there."""
        try:
            v, _ = _cbor_decode(data)
            if len(v) >= 7:
                seq, last = int(v[6]) & 0xffff, st.get("seq")
                if last is not None and (seq - last) & 0xffff >= 0x8000:
                    return                     # older than the newest seen
                st["seq"] = seq
            if len(v) >= 6:                    # linear + rotation axes
                self._apply_jog(*(float(x) for x in v[:6]))
            elif len(v) >= 3:                  # legacy linear-only frame
                self._apply_jog(float(v[0]), float(v[1]), float(v[2]))
            else:                              # empty array = idle KEEPALIVE:
                self._link_tick()              # estimator only — never the twist
                                               # or its lease, so an idle second
                                               # tab can't fight the active jog
        except (ValueError, IndexError, TypeError):
            pass

    def _tcp_pose(self):
        """Live pose of JOG_FRAME (tool0) in BASE_FRAME: ((x,y,z), (qx,qy,qz,qw))
        — the same transform RViz renders — or None if TF isn't available yet."""
        try:
            tf = self._tf_buffer.lookup_transform(BASE_FRAME, JOG_FRAME, Time())
        except Exception:
            return None
        t, r = tf.transform.translation, tf.transform.rotation
        return ((t.x, t.y, t.z), (r.x, r.y, r.z, r.w))

    def _tcp_orientation(self):
        """Live orientation of tool0 in BASE_FRAME (qx,qy,qz,qw), or None."""
        pose = self._tcp_pose()
        return pose[1] if pose else None

    def _orient_lock_angular(self, cur):
        """Proportional orientation hold: corrective angular velocity (rad/s,
        expressed in JOG_FRAME) that drives tool0 back to self._orient_target.
        `cur` = current tool orientation quaternion. (0,0,0) if no target."""
        target = self._orient_target
        if target is None or cur is None:
            return (0.0, 0.0, 0.0)
        # Error rotation in the CURRENT tool frame: R_cur⁻¹ · R_target. Its rotation
        # vector is the axis/angle (in tool coords) that returns us to the target,
        # so ω = KP · rotvec rotates the tool straight back toward it.
        ex, ey, ez = _quat_to_rotvec(_quat_mul(_quat_conj(cur), target))
        kp = self._orient_kp
        wx, wy, wz = ex * kp, ey * kp, ez * kp
        m = math.sqrt(wx * wx + wy * wy + wz * wz)
        if m > ORIENT_LOCK_MAX:                       # gentle cap, direction kept
            s = ORIENT_LOCK_MAX / m
            wx, wy, wz = wx * s, wy * s, wz * s
        return (wx, wy, wz)

    def _set_path_baseline(self, pose, jog_vec) -> None:
        """Anchor the straight-line hold: line = current TCP position along the
        commanded jog direction (expressed in base frame; tool-frame jogs are rotated
        via the current orientation). Cleared (None) if TF is unavailable. A ZERO
        linear vector (pure rotation jog) keeps the start point with no direction —
        _path_lock_linear then acts as a position hold pinning the pivoting TCP."""
        self._path_i = [0.0, 0.0, 0.0]     # new line → forget the learned disturbance
        self._path_i_t = time.monotonic()
        if pose is None:
            self._jog_start_pos = self._jog_dir = None
            return
        self._jog_start_pos = pose[0]
        d = jog_vec if self._jog_base else _quat_rotate(pose[1], jog_vec)
        n = math.sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2])
        self._jog_dir = tuple(c / n for c in d) if n > 1e-9 else None

    def _path_lock_linear(self, pose):
        """PI straight-line hold: corrective linear velocity (m/s, expressed in the
        BASE frame) that steers tool0 back onto the line through the jog-start point
        along the commanded direction. P pulls back on the current error; I learns the
        constant lateral-lag velocity and cancels it → error converges to ~0 at ANY
        speed. KI = KP²/4 (critical damping: no overshoot/oscillation by construction).
        Only the PERPENDICULAR error is fed back — along-track motion (the jog itself)
        is untouched. Returns (0,0,0) if there's no baseline or pose."""
        start, u = self._jog_start_pos, self._jog_dir
        if start is None or pose is None:
            return (0.0, 0.0, 0.0)
        pos, q = pose
        disp = (pos[0] - start[0], pos[1] - start[1], pos[2] - start[2])
        if u is None:
            # No commanded direction = pure ROTATION jog: the line degenerates to a
            # point and the full displacement is error — a position hold that keeps
            # the TCP pivoting in place (even if the pendant TCP, which UR speedl
            # rotates about, is set away from tool0).
            e = disp
        else:
            along = disp[0] * u[0] + disp[1] * u[1] + disp[2] * u[2]
            # Lateral error = displacement minus its along-line component (base frame).
            e = (disp[0] - along * u[0], disp[1] - along * u[1], disp[2] - along * u[2])
        kp = self._path_kp
        # Integrate the error (base frame). dt from a real clock so the two publisher
        # threads (50 Hz loop + ~30 Hz websocket) integrate correctly between them;
        # clamped so a stall can't produce a windup step.
        now = time.monotonic()
        dt = _clamp(0.0, 0.1, now - self._path_i_t)
        self._path_i_t = now
        ki = kp * kp / 4.0
        i = self._path_i
        i[0] -= ki * e[0] * dt
        i[1] -= ki * e[1] * dt
        i[2] -= ki * e[2] * dt
        n = math.sqrt(i[0] * i[0] + i[1] * i[1] + i[2] * i[2])
        if n > PATH_I_MAX:                            # anti-windup, direction kept
            s = PATH_I_MAX / n
            i[0], i[1], i[2] = i[0] * s, i[1] * s, i[2] * s
        vx = -kp * e[0] + i[0]
        vy = -kp * e[1] + i[1]
        vz = -kp * e[2] + i[2]
        n = math.sqrt(vx * vx + vy * vy + vz * vz)
        if n > PATH_LOCK_MAX:                         # gentle cap, direction kept
            s = PATH_LOCK_MAX / n
            vx, vy, vz = vx * s, vy * s, vz * s
        return (vx, vy, vz)                           # BASE frame

    def _publish_twist(self) -> None:
        if not self._jog_mode:
            return
        # Publish the RAMPED Cartesian jog to MoveIt Servo — the SENTINEL feed. The
        # robot executes our speedl command directly (control loop); Servo only needs
        # the plain twist to keep assessing the kinematics for the status gate, so no
        # corrections ride here.
        moving = any(abs(v) > 1e-6 for v in self._jog_cmd)
        if not moving:
            self._path_i = [0.0, 0.0, 0.0]    # standstill → drop the learned
            self._path_i_t = time.monotonic()  # disturbance (it's speed-specific)
        m = TwistStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = BASE_FRAME if self._jog_base else JOG_FRAME
        m.twist.linear.x, m.twist.linear.y, m.twist.linear.z = self._jog_cmd[0:3]
        m.twist.angular.x, m.twist.angular.y, m.twist.angular.z = self._jog_cmd[3:6]
        with self._pub_lock:
            self._twist_pub.publish(m)

    def _web_jogmode(self, q: dict) -> None:
        self._jog_mode = q.get("on", ["0"])[0] in ("1", "true", "on")
        if not self._jog_mode:
            self._jog = _ZERO6
        self.get_logger().info(f"[jog] velocity mode {'ON' if self._jog_mode else 'OFF'}")

    def _web_jogframe(self, q: dict) -> None:
        self._jog_base = q.get("base", ["0"])[0] in ("1", "true", "on")
        self.get_logger().info(f"[jog] frame: {'BASE' if self._jog_base else 'TOOL'}")

    # ── WASD jog: web → ramp → speedl (sentinel: Servo) ─────────────────────────

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
        last_orient_log = 0.0
        last_ramp = time.monotonic()
        prev_intent = False
        jog_ref = (False, (0.0,) * 3, (0.0,) * 3)  # (frame, lin, ang) — change detect
        while rclpy.ok():
            time.sleep(0.02)                                  # 50 Hz
            now = time.monotonic()
            if now >= self._jog_lease:                        # lease expired → stop
                self._jog = _ZERO6
            jog_vec = tuple(self._jog[0:3])
            ang_vec = tuple(self._jog[3:6])
            rotating = any(ang_vec)
            intent = any(jog_vec) or rotating                 # user is holding a jog key
            # Capture / re-capture the locked orientation on the RISING edge of jog
            # intent. Re-capture only after a real pause (REORIENT_IDLE) so you can
            # hand-reposition / planned-move between jogs and re-lock, while rapid taps
            # keep the SAME reference (residual drift is pulled back, not ratcheted in).
            # The idle clock starts on release (falling edge).
            # Re-anchor key includes the jog FRAME: toggling Base/Tool mid-jog changes
            # what the same key vector means, so it must re-anchor like a key switch.
            # ROTATION jog: while any angular axis is commanded the user OWNS the
            # orientation — the hold stands down (target None) and re-locks at the
            # NEW orientation the moment the rotation stops (deliberate reorientation
            # must never be "corrected" back).
            jog_key = (self._jog_base, jog_vec, ang_vec)
            if intent and not prev_intent:
                pose = self._tcp_pose()
                o = pose[1] if pose else None
                if rotating:
                    self._orient_target = None
                elif self._orient_target is None or now - self._jog_stop_t > REORIENT_IDLE:
                    self._orient_target = o
                self._jog_start_orient = self._orient_target
                self._set_path_baseline(pose, jog_vec)
            elif intent and jog_key != jog_ref:
                # Commanded direction changed MID-jog (key switch / slider / frame):
                # re-anchor the line at the current pose along the new direction, or
                # the path lock would fight the new motion as "lateral error".
                pose = self._tcp_pose()
                if rotating:
                    self._orient_target = None
                elif self._orient_target is None:
                    # rotation released mid-jog → lock the NEW orientation
                    self._orient_target = pose[1] if pose else None
                self._set_path_baseline(pose, jog_vec)
            elif not intent and prev_intent:
                self._jog_stop_t = now
            jog_ref = jog_key
            prev_intent = intent

            # Cartesian twist ramp toward the target. Ramping the TWIST magnitude —
            # never per-joint — keeps the commanded direction intact through both
            # ramp-up and ramp-down (smooth start/stop feel; jog_accel is the single
            # knob, scaled by JOG_ROT_RAMP for the angular axes).
            acc = self._jog_accel * min(now - last_ramp, 0.05)
            racc = acc * JOG_ROT_RAMP
            last_ramp = now
            self._jog_cmd = [c + _clamp(-a, a, t - c)
                             for c, t, a in zip(self._jog_cmd, self._jog,
                                                (acc, acc, acc, racc, racc, racc))]

            # Logging tracks ACTUAL command motion (_jog_cmd), so the ramp-down after
            # release is captured too. Periodic trace: orientation as RPY° + drift from
            # the locked reference, reviewable in the log afterward.
            moving = any(abs(v) > 1e-6 for v in self._jog_cmd)
            if moving and now - last_orient_log >= 0.5:
                last_orient_log = now
                pose = self._tcp_pose()
                if pose:
                    pos, o = pose
                    r, p, yw = _quat_to_rpy_deg(o)
                    d = _quat_angle_deg(self._orient_target, o)
                    drift = f"{d:.2f}°" if d is not None else "n/a"
                    # Path straightness: split the displacement-since-start into the
                    # commanded direction (along) vs perpendicular (lat). A straight
                    # jog has lat≈0; "moves at an angle" shows as lat / angle growing.
                    path = ""
                    if self._jog_start_pos and self._jog_dir:
                        disp = tuple(pos[i] - self._jog_start_pos[i] for i in range(3))
                        along = sum(disp[i] * self._jog_dir[i] for i in range(3))
                        lat = math.sqrt(sum((disp[i] - along * self._jog_dir[i]) ** 2
                                            for i in range(3)))
                        ang = math.degrees(math.atan2(lat, abs(along))) if abs(along) > 1e-4 else 0.0
                        path = (f" | pos [{pos[0]*1000:.0f} {pos[1]*1000:.0f} {pos[2]*1000:.0f}]mm"
                                f" along={along*1000:.0f} lat={lat*1000:.1f}mm angle={ang:.1f}°")
                    self.get_logger().info(
                        f"[jog] TCP rpy [{r:.2f} {p:.2f} {yw:.2f}]° | drift {drift}{path}")
            if moving != self._jog_moving:
                self._jog_moving = moving
                o = self._tcp_orientation()
                oq = ("[%.4f %.4f %.4f %.4f]" % o) if o else "n/a"
                if moving:
                    self.get_logger().info(f"[jog] moving — TCP quat before {oq}")
                else:
                    d = _quat_angle_deg(self._jog_start_orient, o)
                    drift = f"{d:.2f}°" if d is not None else "n/a"
                    self.get_logger().info(
                        f"[jog] stopped — TCP quat after {oq} | orientation drift {drift}")
            # Steady sentinel feed: keeps MoveIt Servo assessing the kinematics so its
            # status gate is always fresh. Active commands also publish instantly in
            # _apply_jog for low latency.
            self._publish_twist()

    # ── WebRTC jog transport (browser UDP) ──────────────────────────────────────
    # The page opens an UNORDERED, NO-RETRANSMIT datachannel ("jog") carrying the
    # same CBOR frames as /ws: on a lossy WAN path TCP queues stale twists behind
    # retransmits (head-of-line blocking), UDP just loses them and the next frame
    # arrives 33 ms later. Signaling is one POST /api/rtc (offer in, answer out —
    # aiortc gathers ICE before answering, no trickle). The channel feeds
    # _handle_jog_frame → the same jog-mode gate / lease / estimator / sentinel
    # machinery: a new TRANSPORT, never a 2nd command path.

    def _rtc_offer(self, body: dict, ip: str) -> dict | None:
        """Blocking signaling entry, called from an HTTP handler thread."""
        if self._rtc_loop is None:
            return None
        fut = asyncio.run_coroutine_threadsafe(
            self._rtc_answer(str(body["sdp"]), str(body["type"]), ip),
            self._rtc_loop)
        return fut.result(timeout=10.0)

    async def _rtc_answer(self, sdp: str, typ: str, ip: str) -> dict:
        if len(self._rtc_pcs) >= RTC_MAX_PCS:
            raise RuntimeError(f"{len(self._rtc_pcs)} peer connections open")
        # Empty rtc_stun param = host candidates only (the loopback E2E uses it
        # so an unreachable STUN server can't stall the answer).
        pc = RTCPeerConnection(RTCConfiguration(
            iceServers=[RTCIceServer(urls=self._rtc_stun)]
                       if self._rtc_stun else []))
        self._rtc_pcs.add(pc)
        st = {}                                # per-connection seq state

        @pc.on("datachannel")
        def on_channel(ch):
            # Unthrottled, like the WS jog stream: this is a CONTROL channel —
            # knowing exactly who held it and when matters.
            self.get_logger().info(f"[web] rtc jog stream connected from {ip}")

            @ch.on("message")
            def on_message(data):
                if isinstance(data, (bytes, bytearray)):
                    self._handle_jog_frame(bytes(data), st)

        @pc.on("connectionstatechange")
        async def on_state():
            # "disconnected" can be transient (ICE consent blips) — only
            # failed/closed is terminal. Mirrors the WS disconnect semantics:
            # zero the jog, let the lease machinery do the rest.
            if pc.connectionState in ("failed", "closed"):
                self._rtc_pcs.discard(pc)
                kind = ("webcam viewer" if pc in self._cam_pcs
                        else "rtc jog stream")
                self.get_logger().info(
                    f"[web] {kind} disconnected from {ip} "
                    f"({pc.connectionState})")
                if pc not in self._cam_pcs:        # jog peer: stop like a WS
                    self._apply_jog(0.0, 0.0, 0.0)  # drop would
                self._cam_release(pc)
                await pc.close()

        await pc.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=typ))
        # An offer with a video m-line is a WEBCAM VIEWER (the page opens a
        # separate recvonly peer connection for it): attach the live track.
        if any(t.kind == "video" for t in pc.getTransceivers()):
            track = await self._cam_track()
            if track is not None:
                pc.addTrack(track)
                self._cam_pcs.add(pc)
                self.get_logger().info(f"[web] webcam viewer connected from {ip}")
        await pc.setLocalDescription(await pc.createAnswer())
        # aiortc resolves setLocalDescription only after ICE gathering, so the
        # localDescription below already carries the full candidate set.
        return {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}

    async def _cam_track(self):
        """A fresh relay subscription onto the (lazily opened) webcam, or None
        if there is no camera. buffered=False is the low-latency knob: a
        viewer that can't keep up gets frames DROPPED, never queued. The open
        is retried: the fd of a just-stopped player releases asynchronously,
        so a reopen right after the idle-close can hit EBUSY for a moment."""
        if not self._cam_device:
            return None
        if self._cam_player is None:
            for attempt in range(4):
                try:
                    self._cam_player = MediaPlayer(
                        self._cam_device, format="v4l2",
                        options={"framerate": str(self._cam_fps),
                                 "video_size": self._cam_size})
                    break
                except Exception as exc:
                    err = exc
                    await asyncio.sleep(0.3)
            else:
                self.get_logger().warn(f"[web] webcam open failed: {err}")
                return None
            self.get_logger().info(
                f"[web] webcam opened ({self._cam_device} "
                f"{self._cam_size}@{self._cam_fps})")
        return self._cam_relay.subscribe(self._cam_player.video, buffered=False)

    def _cam_release(self, pc) -> None:
        """Called when a peer closes: release the camera once NOBODY has
        watched for CAM_IDLE_CLOSE — never instantly, so an off→on toggle
        keeps the open device (the instant-stop version raced its own v4l2
        close on reopen = black screen). Piled-up idle tasks are harmless:
        whichever fires first stops, the rest see player None / new viewers."""
        self._cam_pcs.discard(pc)
        if self._cam_pcs or self._cam_player is None:
            return

        async def stop_if_idle():
            await asyncio.sleep(CAM_IDLE_CLOSE)
            if not self._cam_pcs and self._cam_player is not None:
                self._cam_player.video.stop()
                self._cam_player = None
                self.get_logger().info("[web] webcam closed (no viewers)")

        asyncio.ensure_future(stop_if_idle())

    def _web_loop(self) -> None:
        node = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"              # required for the WS 101 upgrade
            # Nagle + delayed ACK turns the 25 Hz /ws3d push into RTT-paced
            # bursts once the link RTT exceeds the frame interval (VPN) —
            # the twin stutters while the jog itself is fine (browsers
            # already set TCP_NODELAY on their end of every WebSocket).
            disable_nagle_algorithm = True
            def log_message(self, *_): pass

            def handle(self):
                # The UI is plain HTTP by design. A browser auto-upgrading to
                # HTTPS (HTTPS-First mode / stale HSTS / a typed https://) opens
                # a TLS handshake here — first byte 0x16 (ClientHello), which
                # the plain server can't answer. Close immediately: a FAST TLS
                # failure is exactly what makes the browser fall back to
                # http:// on its own; parsing it as a request would instead
                # stall the upgrade and show a cryptic 400. Log the right URL.
                try:
                    first = self.connection.recv(1, socket.MSG_PEEK)
                except OSError:
                    return
                if first == b"\x16":
                    node._log_https_refused()
                    return                             # close → browser retries as http
                try:
                    super().handle()
                except (ConnectionResetError, ConnectionAbortedError,
                        BrokenPipeError, TimeoutError):
                    pass    # client vanished mid-request — routine (scanners,
                            # phones changing networks), not a server error;
                            # socketserver would dump a full traceback otherwise

            def _client_ip(self):
                ip = self.client_address[0]
                return ip[7:] if ip.startswith("::ffff:") else ip  # dual-stack v4

            def _send(self, body, ctype="text/plain", code=200, cache=False):
                self.send_response(code)
                self.send_header("Content-Type", ctype)
                self.send_header("Content-Length", str(len(body)))
                if cache:    # vendored JS + meshes are immutable per install
                    self.send_header("Cache-Control", "public, max-age=86400")
                self.end_headers()
                self.wfile.write(body)

            # Static/mesh files for the 3D twin: /static/<file> (vendored JS,
            # viewer) and /pkg/<package>/<path> (URDF mesh resources — URDFLoader
            # maps package://ur_description/… here). Path-traversal guarded: the
            # normalized relative part may not escape the root (realpath is NOT
            # used — --symlink-install share dirs legitimately point elsewhere).
            def _serve_file(self):
                path = urllib.parse.unquote(urllib.parse.urlparse(self.path).path)
                try:
                    if path.startswith("/static/"):
                        root, rel = node._web_static, path[len("/static/"):]
                    else:                                  # /pkg/<package>/<path>
                        _, _, pkg, rel = path.split("/", 3)
                        root = get_package_share_directory(pkg)
                    rel = os.path.normpath(rel)
                    if os.path.isabs(rel) or rel.startswith(".."):
                        raise FileNotFoundError(path)
                    with open(os.path.join(root, rel), "rb") as f:
                        body = f.read()
                except Exception:
                    self._send(b"not found", code=404)
                    return
                ctype = (mimetypes.guess_type(rel)[0] or "application/octet-stream")
                if rel.endswith(".js"):
                    ctype = "text/javascript"          # modules need a JS MIME
                elif rel.endswith(".dae"):
                    ctype = "model/vnd.collada+xml"
                # Vendored libs + meshes are immutable per install; viewer.js is
                # OUR code and changes — a day-stale copy once made a new page
                # call APIs the cached module didn't have yet (dead buttons).
                self._send(body, ctype, cache=(rel != "viewer.js"))

            def do_GET(self):
                node._log_web_client(self._client_ip(), self.path,
                                     self.headers.get("User-Agent", ""))
                if self.headers.get("Upgrade", "").lower() == "websocket":
                    if self.path == "/ws3d":
                        self._serve_ws(push=True)      # twin state stream (send-only)
                    else:
                        self._serve_ws()               # jog command stream
                    return
                if self.path == "/api/urdf":
                    u = node._urdf
                    if u is None:
                        self._send(b"robot_description not received yet", code=503)
                    else:
                        self._send(u.encode(), "application/xml")
                elif self.path.startswith(("/static/", "/pkg/")):
                    self._serve_file()
                elif self.path.startswith("/api/state"):
                    with node._conn_lock:
                        connected = node._conn is not None
                    sf = node._speed_fraction
                    # Name a wall only when it's actually inside the scene
                    # threshold — otherwise a collision decel is more likely a
                    # SELF-collision and must not claim "near front wall".
                    nw = node._nearest_cage_wall()
                    near_wall = nw[0] if nw and nw[1] <= node._scene_coll else None
                    self._send(json.dumps({"speed": round(node._speed, 2), "gain": node._gain,
                        "lookahead": round(node._lookahead, 3),
                        "jspeed": round(node._jog_speed, 3), "jaccel": round(node._jog_accel, 1),
                        "jrspeed": round(node._jog_rot_speed, 3),
                        "okp": round(node._orient_kp, 1),
                        "pkp": round(node._path_kp, 1),
                        "selfd": round(node._self_coll, 2), "walld": round(node._scene_coll, 2),
                        "basef": node._jog_base,
                        "speedfrac": round(sf, 3) if sf is not None else None,
                        "qdnow": round(node._qd_now, 3), "qdpeak": round(node._qd_peak, 3),
                        "linkgap": round(node._link_gap * 1000),
                        "lease": round(node._link_lease * 1000),
                        "linkscale": round(node._link_scale, 2),
                        "connected": connected,
                        "cam": bool(node._cam_device
                                    and os.path.exists(node._cam_device)),
                        "jogon": node._jog_mode,
                        "servoGate": round(node._servo_gate, 2),
                        "servoCode": int(node._servo_code),
                        "qdLatch": node._qd_latched,
                        "qdBlockAxis": node._dominant_axis(node._qd_block_dir),
                        "collBlocked": node._servo_code == ServoStatus.HALT_FOR_COLLISION,
                        "collWall": node._coll_near_wall,
                        "collAxis": node._dominant_axis(node._coll_block_dir),
                        "singAxis": node._dominant_axis(node._sing_block_dir),
                        "nearWall": near_wall}).encode(),
                        "application/json")
                else:
                    self._send(_WEB_PAGE.replace(
                        "__WEB_PORT__", str(node._web_port)).encode(),
                        "text/html; charset=utf-8")

            def do_POST(self):
                node._log_web_client(self._client_ip(), self.path,
                                     self.headers.get("User-Agent", ""))
                if self.path.startswith("/api/rtc"):
                    # WebRTC signaling: JSON offer in the body, JSON answer
                    # back. Any failure (no aiortc, bad SDP, ICE trouble) is a
                    # 503 — the page silently stays on the WS fallback.
                    try:
                        n = int(self.headers.get("Content-Length") or 0)
                        ans = node._rtc_offer(json.loads(self.rfile.read(n)),
                                              self._client_ip())
                    except Exception as exc:
                        node.get_logger().warn(f"[web] rtc offer failed: {exc}")
                        ans = None
                    if ans is None:
                        self._send(b"rtc unavailable", code=503)
                    else:
                        self._send(json.dumps(ans).encode(), "application/json")
                    return
                q = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
                try:
                    if self.path.startswith("/api/set"):
                        node._web_set(q)
                    elif self.path.startswith("/api/jogmode"):
                        node._web_jogmode(q)
                    elif self.path.startswith("/api/jogframe"):
                        node._web_jogframe(q)
                except (ValueError, KeyError):
                    pass
                self._send(b"ok")

            # ── WebSocket: ordered, low-overhead streams ───────────────────────
            # /ws   — jog commands IN (CBOR), nothing out.
            # /ws3d — twin state OUT (binary, ~25 Hz), incoming frames ignored —
            #         keeps the viewer off the jog path AND out of the link-jitter
            #         estimator (it never calls _apply_jog).
            def _serve_ws(self, push=False):
                self.close_connection = True               # this socket is now a WS
                # WS connects are logged UNthrottled: /ws is the jog CONTROL
                # channel — knowing exactly who held it and when matters.
                ip = self._client_ip()
                kind = "3D-twin stream" if push else "jog stream"
                node.get_logger().info(f"[web] {kind} connected from {ip}")
                key = self.headers.get("Sec-WebSocket-Key", "")
                accept = base64.b64encode(
                    hashlib.sha1((key + _WS_GUID).encode()).digest()).decode()
                self.send_response(101)
                self.send_header("Upgrade", "websocket")
                self.send_header("Connection", "Upgrade")
                self.send_header("Sec-WebSocket-Accept", accept)
                self.end_headers()
                stop = threading.Event()
                if push:
                    threading.Thread(target=self._ws_push, args=(stop,),
                                     daemon=True, name="ri-ws3d").start()
                st = {}                                    # per-connection seq state
                try:
                    while True:
                        op, data = self._ws_read()
                        if op is None or op == 0x8:        # closed
                            break
                        if op == 0x2 and not push:         # binary frame = CBOR jog cmd
                            node._handle_jog_frame(data, st)
                except Exception:
                    pass
                stop.set()
                node.get_logger().info(f"[web] {kind} disconnected from {ip}")
                if not push:
                    node._apply_jog(0.0, 0.0, 0.0)         # stop on disconnect

            def _ws_push(self, stop):
                # Single writer per connection: the read loop never writes after
                # the 101, so unlocked sequential writes are safe. Frames are
                # server→client, hence unmasked (RFC 6455).
                try:
                    while not stop.is_set():
                        payload = node._twin_frame()
                        if payload is not None:
                            n = len(payload)
                            hdr = (bytes([0x82, n]) if n < 126
                                   else bytes([0x82, 126]) + struct.pack(">H", n))
                            self.wfile.write(hdr + payload)
                        stop.wait(0.04)                    # ~25 Hz
                except Exception:
                    pass                                   # client gone → read loop ends too

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

        class DualStackHTTPServer(ThreadingHTTPServer):
            address_family = socket.AF_INET6
            def server_bind(self):
                self.socket.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
                super().server_bind()

        try:
            srv = DualStackHTTPServer(("::", self._web_port), Handler)
        except Exception as exc:
            self.get_logger().warn(f"[web] could not start on :{self._web_port}: {exc}")
            return
        self.get_logger().info(f"[web] tuning UI at http://{self._pc_ip()}:{self._web_port}")
        srv.serve_forever()

    # ── Joint states: only gate startup (the robot holds its own pose via speedj
    #    when idle, so we don't track positions here) ──────────────────────────────

    def _urdf_cb(self, msg: String) -> None:
        if self._urdf is None:
            self.get_logger().info(f"[web] robot_description received ({len(msg.data)} B)")
        self._urdf = msg.data

    def _scene_cb(self, msg: PlanningScene) -> None:
        # Track world BOX collision objects (the draggable test wall) for the 3D
        # twin. scene_wall replace-ADDs at 1 Hz, so a (re)started viewer converges.
        for obj in msg.world.collision_objects:
            if obj.operation == CollisionObject.REMOVE:
                self._scene_boxes.pop(obj.id, None)
            elif (obj.primitives and obj.primitive_poses
                  and obj.primitives[0].type == SolidPrimitive.BOX):
                p, dims = obj.primitive_poses[0], obj.primitives[0].dimensions
                self._scene_boxes[obj.id] = (
                    p.position.x, p.position.y, p.position.z,
                    p.orientation.x, p.orientation.y, p.orientation.z,
                    p.orientation.w, dims[0], dims[1], dims[2])

    def _twin_frame(self) -> bytes | None:
        """Binary 3D-twin state for the /ws3d push: 0x01, 6×f32 joint positions
        (UR_JOINT_ORDER), box count, then 10×f32 per box (xyz quat-xyzw size).
        Little-endian. With the robot OFF (no joint states yet) the joints are
        zeros — the stream must still run or the cage walls never render
        ("walls disappeared" bug: returning None here starved /ws3d entirely
        whenever the UI was opened before pressing Play)."""
        q = self._q_actual or [0.0] * 6
        boxes = list(self._scene_boxes.values())[:8]
        return (struct.pack("<B6fB", 1, *q, len(boxes))
                + b"".join(struct.pack("<10f", *b) for b in boxes))

    def _js_cb(self, msg: JointState) -> None:
        if not self._js_ready.is_set():
            self._js_ready.set()
            self.get_logger().info("[ctrl] first joint states received")
        # Latest positions in UR order for the 3D twin (name→index map cached;
        # names are stable per publisher).
        if msg.position and msg.name:
            names = list(msg.name)
            if names != self._js_names:
                self._js_names = names
                try:
                    self._js_map = [names.index(j) for j in UR_JOINT_ORDER]
                except ValueError:
                    self._js_map = None        # not a UR joint-state message
            if self._js_map is not None and len(msg.position) >= 6:
                self._q_actual = [msg.position[i] for i in self._js_map]
        # Kept subscribed (no longer self-destructs): track the actual measured
        # joint speed for the web readout. msg.velocity is RTDE actual_qd.
        if msg.velocity:
            m = max(abs(v) for v in msg.velocity)
            self._qd_now = m
            now = time.monotonic()
            if m >= self._qd_peak or now - self._qd_peak_t > 1.5:
                self._qd_peak = m
                self._qd_peak_t = now
        # Full-rate TCP path log (125 Hz, while jogging): exact pose + commanded
        # speed, so path straightness is analysable at full resolution. This runs on
        # the executor thread (not the RT control loop); the TF read + buffered write
        # are cheap. Flush ~1×/s so the file is readable mid-run.
        if self._tcp_csv is not None and self._jog_mode:
            pose = self._tcp_pose()
            if pose:
                (x, y, z), (qx, qy, qz, qw) = pose
                cmd = math.sqrt(sum(v * v for v in self._jog_cmd[0:3])) * 1000.0
                mv = 1 if any(abs(v) > 1e-6 for v in self._jog_cmd) else 0
                self._tcp_csv.write(
                    f"{time.monotonic():.4f},{mv},{cmd:.1f},"
                    f"{x*1000:.2f},{y*1000:.2f},{z*1000:.2f},"
                    f"{qx:.5f},{qy:.5f},{qz:.5f},{qw:.5f}\n")
                self._tcp_csv_n += 1
                if self._tcp_csv_n % 125 == 0:
                    self._tcp_csv.flush()

    def _speed_cb(self, msg: Float64) -> None:
        self._speed_fraction = msg.data            # pendant speed slider, display only

    def _nearest_cage_wall(self, direction=None):
        """(id, surface_distance_m) of the cage wall closest to the TCP, or None.
        Distance to box centers is useless for the cage's huge slabs (the front
        wall's center can be "nearest" from almost anywhere → every banner said
        front wall). With `direction` (unit vector, base frame) only walls that
        lie roughly that way from the TCP count — used at a collision halt to
        name the wall the jog was actually moving toward."""
        pose = self._tcp_pose()
        if not pose or not self._scene_boxes:
            return None
        px, py, pz = pose[0]
        best, best_d = None, float("inf")
        for wid, box in self._scene_boxes.items():
            if not wid.startswith("wall_"):
                continue
            bx, by, bz, qx, qy, qz, qw, sx, sy, sz = box
            # TCP in the box frame → closest point on the box → offset back in base.
            lx, ly, lz = _quat_rotate((-qx, -qy, -qz, qw), (px - bx, py - by, pz - bz))
            ox, oy, oz = _quat_rotate((qx, qy, qz, qw),
                                      (_clamp(-sx / 2, sx / 2, lx) - lx,
                                       _clamp(-sy / 2, sy / 2, ly) - ly,
                                       _clamp(-sz / 2, sz / 2, lz) - lz))
            d = math.sqrt(ox * ox + oy * oy + oz * oz)
            if direction is not None and d > 1e-9:
                dot = (ox * direction[0] + oy * direction[1] + oz * direction[2]) / d
                if dot < 0.5:                  # not the way the jog was moving
                    continue
            if d < best_d:
                best_d, best = d, wid
        return (best, best_d) if best is not None else None

    def _halt_dir(self, prev):
        """Approach direction for a halt that just fired: the normalized last
        streamed jog command. `prev` is the (dir, t) remembered when the same
        halt class last cleared: a jittery sentinel feed can flap halt↔clear
        mid-escape, and re-capturing then would record the ESCAPE direction as
        "inward" — inverting the block and disabling the very pad backing the
        arm out. If the fresh direction opposes a direction cleared moments
        ago, the remembered inward direction is restored instead."""
        lx, ly, lz = self._last_lin
        mag = math.sqrt(lx * lx + ly * ly + lz * lz)
        if mag <= 1e-6:
            return None
        d = (lx / mag, ly / mag, lz / mag)
        if prev is not None:
            pd, pt = prev
            if (time.monotonic() - pt < 2.0
                    and d[0] * pd[0] + d[1] * pd[1] + d[2] * pd[2] <= -0.5):
                return pd
        return d

    @staticmethod
    def _dominant_axis(bd) -> str | None:
        """Dominant base-frame axis ("±X/Y/Z") of a direction vector, or None."""
        if bd is None:
            return None
        x, y, z = bd
        if max(abs(x), abs(y), abs(z)) < 1e-6:
            return None
        if abs(x) >= abs(y) and abs(x) >= abs(z):
            return "+X" if x > 0 else "-X"
        if abs(y) >= abs(x) and abs(y) >= abs(z):
            return "+Y" if y > 0 else "-Y"
        return "+Z" if z > 0 else "-Z"

    def _servo_status_cb(self, msg: ServoStatus) -> None:
        # Map Servo's kinematic assessment to a speedl gate: full speed when clean,
        # crawl while approaching/leaving a singularity or a joint bound, hard zero at
        # the halt thresholds. Transitions are logged so trips self-explain.
        if msg.code in (ServoStatus.HALT_FOR_SINGULARITY, ServoStatus.HALT_FOR_COLLISION):
            gate = 0.0
        elif msg.code == ServoStatus.NO_WARNING:
            gate = 1.0
        else:
            gate = 0.25
        self._servo_status_t = time.monotonic()
        # Halt-escape bookkeeping (collision AND singularity). Capture the approach
        # direction ONLY on the transition into the halt, and only if a jog was
        # actually streaming (≤0.5 s old) — a halt that appears while idle (e.g.
        # the wall dragged onto the robot in RViz, or a planned move parking the
        # arm deep in a singular zone) has no trustworthy "out" direction, so no
        # escape hint: fix the scene / plan out instead. Cleared as soon as Servo
        # reports anything other than that halt class. The collision direction
        # also relaxes the gate for opposing commands (jog loop); the singularity
        # direction is a WEB-UI HINT only — Servo itself re-evaluates the reversed
        # twist as DECELERATE_FOR_LEAVING (the sentinel feed is pre-gate).
        prev, self._servo_code = self._servo_code, msg.code
        jog_fresh = time.monotonic() - self._last_lin_t < 0.5
        if msg.code == ServoStatus.HALT_FOR_COLLISION:
            if prev != ServoStatus.HALT_FOR_COLLISION and jog_fresh:
                self._coll_block_dir = self._halt_dir(self._coll_prev)
                if self._coll_block_dir is not None:
                    # Name the wall the jog was moving TOWARD — but only if it is
                    # plausibly close (a self-collision halt must not get blamed
                    # on some distant wall that happens to lie ahead).
                    hit = self._nearest_cage_wall(self._coll_block_dir)
                    self._coll_near_wall = (
                        hit[0] if hit and hit[1] <= max(0.3, self._scene_coll)
                        else None)
                    self.get_logger().warn(
                        "[jog] collision halt — approach dir captured, jog the "
                        "OPPOSITE way to back out (×%.2f crawl)" % COLL_ESCAPE_GATE)
        elif self._coll_block_dir is not None:
            self._coll_prev = (self._coll_block_dir, time.monotonic())
            self._coll_block_dir = None
            self._coll_near_wall = None
            self.get_logger().info("[jog] collision cleared — full jog restored")
        if msg.code == ServoStatus.HALT_FOR_SINGULARITY:
            if prev != ServoStatus.HALT_FOR_SINGULARITY and jog_fresh:
                self._sing_block_dir = self._halt_dir(self._sing_prev)
                if self._sing_block_dir is not None:
                    self.get_logger().warn(
                        "[jog] singularity halt — approach dir captured, jog the "
                        "OPPOSITE way to back out (Servo crawls it ×0.25)")
        elif self._sing_block_dir is not None:
            self._sing_prev = (self._sing_block_dir, time.monotonic())
            self._sing_block_dir = None
            self.get_logger().info("[jog] singularity halt cleared")
        if gate != self._servo_gate:
            self._servo_gate = gate
            self.get_logger().warn(
                f"[jog] servo sentinel code {msg.code} → speed gate ×{gate}")
        elif msg.code != prev:
            # Same gate, different verdict (e.g. collision-decel ↔ singularity-
            # decel, both ×0.25) was invisible in the log — the 2026-06-12
            # boundary trip could not tell what Servo thought. Throttled: a
            # jittery feed flaps codes at stream rate.
            t = time.monotonic()
            if t - self._code_log_t > 0.25:
                self._code_log_t = t
                self.get_logger().info(
                    f"[jog] servo status code {prev} → {msg.code} (gate ×{gate})")

    # ── Reverse server: serves the script on request, then hands the socket to
    #    the control loop (this robot's URCap Custom Port == the reverse port) ───

    def _pc_ip(self) -> str:
        """Best local IP for the log line and the URScript connect-back. NEVER
        raises: with the robot dark its direct link is down and the UDP connect
        probe fails with 'Network is unreachable' — that used to kill the web
        thread before serve_forever(), taking the whole UI offline. The robot
        probe stays first so the connect-back host is correct whenever a robot
        is actually there (it must be: _serve_script only runs after the robot
        reached us, so the route exists)."""
        for probe in ((self._robot_ip, self._port), ("8.8.8.8", 53)):
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
                    s.connect(probe)
                    return s.getsockname()[0]
            except OSError:
                continue
        return "localhost"

    def _log_web_client(self, ip: str, path: str, ua: str) -> None:
        """One "client connected" line per IP per 10-min idle gap — a single
        page load fires dozens of requests (assets, meshes, /api polls), so
        per-request logging would be pure noise. Shows what they fetched first
        and their browser; an unexpected IP here = check who has access."""
        now = time.monotonic()
        last = self._web_seen.get(ip)
        self._web_seen[ip] = now
        if last is None or now - last > 600.0:
            self.get_logger().info(
                f"[web] client connected: {ip} → {path}"
                + (f" — {ua}" if ua else ""))

    def _log_https_refused(self) -> None:
        """Throttled notice that a TLS handshake hit the plain-HTTP web port."""
        now = time.monotonic()
        if now - self._https_warn_t > 5.0:
            self._https_warn_t = now
            self.get_logger().warn(
                "[web] HTTPS connection attempt refused — the tuning UI is "
                f"plain HTTP only: http://{self._pc_ip()}:{self._web_port}")

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
                last_req = time.monotonic()         # avoid a huge dt (health) on reconnect
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
                pkt = _pack(q_traj, MODE_SERVOJ, self._gain, self._lookahead, step_t)
            else:
                # jog / hold. Gate on the RAMPED command (_jog_cmd): stream only while
                # it is non-zero (covers the smooth ramp-down after release); at
                # standstill command exact speedj zeros — the active idle hold (also
                # kills the old standstill ring from Servo's near-zero residuals).
                moving = any(abs(v) > 1e-6 for v in self._jog_cmd)
                pose = self._tcp_pose() if moving else None
                # Amplification allowance vs what was actually SENT last cycle
                # (see the QD_ALLOW/QD_LATCH blocks), evaluated every cycle so the
                # runaway latch can fire even while another gate has the command
                # at a crawl — that was exactly the 2026-06-12 trip.
                allowed = (QD_ALLOW_BASE + QD_ALLOW_SLOPE * self._sent_lin
                           + QD_ALLOW_ROT_SLOPE * self._sent_rot)
                # Post-latch block bookkeeping: cleared once the TCP has left
                # the rest pose; while it stands, a command opposing the
                # captured inward direction is an ESCAPE — raised allowance +
                # latch line (one press must walk out, not five taps — see
                # QD_BLOCK_*). Travel counts ONLY while not latched and the
                # joints move at honest speeds (≤ the escape allowance floor):
                # in the 2026-06-12 incident the runaway lurches themselves
                # covered 5–30 cm, self-clearing the block 50 ms before each
                # re-latch — involuntary travel must never clear it. (The
                # anchor is also re-set to the rest pose at latch release.)
                bd_qd, escaping = self._qd_block_dir, False
                if (bd_qd is not None and pose is not None
                        and not self._qd_latched
                        and self._qd_now <= QD_ESCAPE_ALLOW):
                    bp = self._qd_block_pos
                    if (bp is not None and math.dist(pose[0], bp) > QD_BLOCK_CLEAR_M):
                        bd_qd = self._qd_block_dir = self._qd_block_pos = None
                        self.get_logger().info(
                            "[jog] singularity-boundary block cleared "
                            f"(TCP moved {QD_BLOCK_CLEAR_M*1000:.0f}+ mm away "
                            "at honest joint speed)")
                # Current commanded direction (base frame) from the jog TARGET —
                # not the last SENT twist: at the start of a tap nothing was sent
                # yet, and an escape tap pressed while the joints still ring
                # (qd elevated) must be recognized as an escape from its very
                # first cycle or it re-latches instantly ("tap five times").
                cur_dir = None
                jt = tuple(self._jog[0:3])
                if pose is not None and any(jt):
                    cd = jt if self._jog_base else _quat_rotate(pose[1], jt)
                    m = math.sqrt(cd[0] * cd[0] + cd[1] * cd[1] + cd[2] * cd[2])
                    if m > 1e-6:
                        cur_dir = (cd[0] / m, cd[1] / m, cd[2] / m)
                if bd_qd is not None and cur_dir is not None:
                    d = (cur_dir[0] * bd_qd[0] + cur_dir[1] * bd_qd[1]
                         + cur_dir[2] * bd_qd[2])
                    escaping = d <= -QD_BLOCK_DOT
                if escaping:
                    allowed = max(allowed, QD_ESCAPE_ALLOW)
                latch_at = QD_LATCH_FACTOR * allowed
                if escaping:
                    latch_at = max(latch_at, QD_ESCAPE_LATCH)
                if (pose is not None and not self._qd_latched
                        and self._qd_now > latch_at):
                    self._qd_latched = True
                    # Remember which way the trouble was — but NEVER from an
                    # escape (overwriting the inward direction with the outward
                    # one would invert the block).
                    if not escaping and cur_dir is not None:
                        self._qd_block_dir = cur_dir
                        self._qd_block_pos = pose[0]
                    self.get_logger().warn(
                        f"[jog] RUNAWAY LATCH: qd {self._qd_now:.2f} rad/s vs "
                        f"{allowed:.2f} allowed (sent {self._sent_lin*1000:.0f} mm/s "
                        f"+ {math.degrees(self._sent_rot):.0f}°/s"
                        f"{', escaping' if escaping else ''}) — zero-hold "
                        "until key release + joints settle")
                if pose is not None and not self._qd_latched:
                    # CARTESIAN jog (speedl): the robot's own controller converts the
                    # twist to joint motion onboard, 125 Hz, zero-staleness — straight
                    # lines like the pendant, by construction. The orientation + line
                    # holds ride along as trim. Built in base_link, then rotated into
                    # the UR-native Base frame (= base_link yawed π in ur_description:
                    # x→−x, y→−y; angular likewise) which speedl expects.
                    q = pose[1]
                    lin_cmd = tuple(self._jog_cmd[0:3])
                    ang_cmd = tuple(self._jog_cmd[3:6])
                    lin = lin_cmd if self._jog_base else _quat_rotate(q, lin_cmd)
                    ang_jog = ang_cmd if self._jog_base else _quat_rotate(q, ang_cmd)
                    lat = self._path_lock_linear(pose)
                    hold = _quat_rotate(q, self._orient_lock_angular(q))
                    ang = (ang_jog[0] + hold[0], ang_jog[1] + hold[1],
                           ang_jog[2] + hold[2])
                    # SAFETY GATE 1: Servo's kinematic sentinel (singularity/collision).
                    # SAFETY GATE 2: singularity-AMPLIFICATION guard — measured joint
                    # speed (RTDE actual_qd) vs what the SENT TCP speed justifies
                    # (`allowed`, computed above). Smooth braking (slew-limited gate),
                    # never a hard cut: a fixed cap bang-banged at full speed (cut →
                    # re-ramp → cut = stutter). The runaway latch above is the
                    # backstop for when proportional braking cannot win.
                    tgt = 1.0 if self._qd_now <= allowed else allowed / self._qd_now
                    self._qd_gate += _clamp(-QD_GATE_DOWN, QD_GATE_UP, tgt - self._qd_gate)
                    if self._qd_gate < 0.6 and now - self._qd_guard_t > 1.0:
                        self._qd_guard_t = now
                        self.get_logger().warn(
                            f"[jog] amplification guard: qd {self._qd_now:.2f} rad/s vs "
                            f"{allowed:.2f} allowed at sent {self._sent_lin*1000:.0f} mm/s "
                            f"— gate ×{self._qd_gate:.2f}")
                    # Record the pre-gate commanded direction (base_link) — the
                    # collision-halt escape needs the approach dir even though the
                    # gated output is zero.
                    lin_mag = math.sqrt(lin[0]**2 + lin[1]**2 + lin[2]**2)
                    if lin_mag > 0.002:
                        self._last_lin = lin
                        self._last_lin_t = now
                    # Collision-halt ESCAPE: while halted for collision, a command
                    # genuinely opposing the captured approach direction passes at
                    # the crawl gate so the operator can back out of the (virtual)
                    # wall. Anything else stays hard-blocked. Singularity halts
                    # never set _coll_block_dir, so they are unaffected here —
                    # their captured direction (_sing_block_dir) is a web-UI hint
                    # only; Servo itself lets the reverse out (pre-gate feed).
                    g_servo = self._servo_gate
                    # SENTINEL LIVENESS: with the steady 50 Hz feed Servo verdicts
                    # arrive continuously; a silent sentinel (servo_node dead, not
                    # armed yet, or feed off) means NO protection — fail safe to a
                    # hard block rather than coasting on the last verdict. The
                    # collision-halt escape below must NOT apply here: it may only
                    # relax a verdict from a LIVE sentinel.
                    sentinel_fresh = now - self._servo_status_t <= 0.5
                    if not sentinel_fresh:
                        g_servo = 0.0
                        if now - self._sentinel_warn_t > 2.0:
                            self._sentinel_warn_t = now
                            self.get_logger().warn(
                                "[jog] servo sentinel SILENT (no status for "
                                f"{now - self._servo_status_t:.1f}s) — jog blocked")
                    bd = self._coll_block_dir
                    if sentinel_fresh and g_servo == 0.0 and lin_mag > 1e-6:
                        if bd is not None:
                            dot = (lin[0]*bd[0] + lin[1]*bd[1] + lin[2]*bd[2]) / lin_mag
                            if dot <= -COLL_ESCAPE_DOT:
                                g_servo = COLL_ESCAPE_GATE
                                if now - self._escape_log_t > 1.0:
                                    self._escape_log_t = now
                                    self.get_logger().info(
                                        f"[jog] collision-halt escape: reversing out "
                                        f"×{COLL_ESCAPE_GATE} (dot {dot:.2f})")
                        elif self._servo_code == ServoStatus.HALT_FOR_COLLISION:
                            # Halt appeared at startup/idle — no approach direction
                            # captured, so no "bad" direction is known. Allow crawl
                            # in any direction; Servo re-blocks instantly if the user
                            # moves further into the collision rather than away.
                            g_servo = COLL_ESCAPE_GATE
                            if now - self._escape_log_t > 1.0:
                                self._escape_log_t = now
                                self.get_logger().info(
                                    "[jog] collision-halt escape (no dir): "
                                    f"crawl ×{COLL_ESCAPE_GATE} — all axes allowed")
                    g = g_servo * self._qd_gate
                    # Post-latch INWARD block: re-pressing the key that drove the
                    # arm into the boundary must not ratchet it deeper. Only the
                    # inward cone is dead — lateral and escape directions pass
                    # (collision escape is the inverse shape: all-blocked except
                    # the escape cone, because a wall surrounds; a singular zone
                    # is left by ANY other direction).
                    if bd_qd is not None and lin_mag > 1e-6:
                        d = (lin[0] * bd_qd[0] + lin[1] * bd_qd[1]
                             + lin[2] * bd_qd[2]) / lin_mag
                        if d >= QD_BLOCK_DOT:
                            g = 0.0
                            if now - self._qd_block_log_t > 1.0:
                                self._qd_block_log_t = now
                                self.get_logger().warn(
                                    "[jog] singularity-boundary block: inward "
                                    "command zeroed — jog the other way "
                                    "(clears after "
                                    f"{QD_BLOCK_CLEAR_M*1000:.0f} mm of travel)")
                    v6 = [-(lin[0] + lat[0]) * g, -(lin[1] + lat[1]) * g,
                          (lin[2] + lat[2]) * g, -ang[0] * g, -ang[1] * g, ang[2] * g]
                    peak = math.sqrt(v6[0]**2 + v6[1]**2 + v6[2]**2)
                    rot = math.sqrt(v6[3]**2 + v6[4]**2 + v6[5]**2)
                    self._sent_lin, self._sent_rot = peak, rot
                    if peak > 1e-4 or rot > 1e-4:
                        self._last_motion_t = now
                        self._last_peak_qd = self._qd_now
                        if now - last_diag >= 0.3:
                            last_diag = now
                            self.get_logger().info(
                                f"[jog] speedl cmd={peak*1000:.0f}mm/s "
                                f"rot={math.degrees(rot):.0f}°/s "
                                f"act_qd={self._qd_now:.3f} gate={g:.2f}")
                    pkt = _pack(v6, MODE_SPEEDL, int(SPEEDL_ACCEL * 100),
                                self._lookahead, step_t)
                else:
                    # Idle, runaway-latched, or a TF gap mid-jog (fail safe to a
                    # stop): exact speedj zeros = the active self-hold.
                    self._sent_lin = self._sent_rot = 0.0
                    if not moving:
                        self._qd_gate = 1.0          # fresh gate for the next jog
                        if self._qd_latched and self._qd_now < QD_LATCH_SETTLE:
                            self._qd_latched = False
                            # Re-anchor the directional block at the REST pose:
                            # a runaway excursion itself travels far more than
                            # QD_BLOCK_CLEAR_M (2026-06-12 incident: 5–30 cm
                            # lurches per tap), so measuring travel from the
                            # LATCH point let the block self-clear before the
                            # next tap and the operator ratcheted 6 excursions
                            # deep. Clearing travel must be deliberate motion
                            # made after the joints settled.
                            if self._qd_block_dir is not None:
                                p = self._tcp_pose()
                                if p is not None:
                                    self._qd_block_pos = p[0]
                            self.get_logger().info(
                                f"[jog] runaway latch released "
                                f"(qd {self._qd_now:.2f} rad/s, jog idle)")
                    pkt = _pack(_ZERO6, MODE_SPEEDJ, int(SPEEDJ_ACCEL * 100),
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
        if node._tcp_csv is not None:
            try: node._tcp_csv.flush(); node._tcp_csv.close()
            except Exception: pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
