# Telamoto — ROS2 / UR10 CB3.1 workspace

## Stack
| Layer | Package | Notes |
|---|---|---|
| Package manager | **Pixi** (RoboStack-Jazzy channel) | `pixi.toml` at repo root |
| Robot I/O | **custom** (see Motion architecture) | `ur_robot_driver` does NOT work on this CB3 |
| Motion planning | **MoveIt2** (`moveit`) + **MoveIt Servo** | Jazzy release; Servo drives the WASD jog |
| IK (planned moves) | **TRAC-IK** (`solve_type: Distance`) | `telamoto_bringup/config/kinematics.yaml`, loaded by the **vendored** `ur_moveit.launch.py` (upstream has no override hook) + `moveit_real`/`moveit_rviz`. ⚠ servo_node keeps upstream KDL — Servo is the jog safety sentinel, don't move it to TRAC-IK |
| Custom interfaces | `telamoto_msgs` | `msg/`, `srv/` |
| HW extension | `telamoto_hardware` | C++ pluginlib plugin |
| Bringup | `telamoto_bringup` | launch files, configs, control scripts |

## Robot
- Model: **UR10 CB3.1**, PolyScope 3.x (no Remote Control mode)
- Default IP: `192.168.10.2` (set `robot_ip` launch arg to override)

## Motion architecture (IMPORTANT — no standard driver)
PolyScope 3.x holds ALL RTDE **input** registers, so `ur_robot_driver` always fails
`on_configure()` on this robot. Instead (`src/telamoto_bringup/scripts/`):
- **`ur_rtde_joint_pub.py`** — reads RTDE **outputs only** (port 30004, 125 Hz):
  publishes `/joint_states` (actual_q + actual_qd) and the pendant speed slider.
- **`ur_servo_controller.py`** — moves the arm via the **External Control URCap**
  script-sender: serves a pipelined URScript on port 50001 (press **Play** on the
  pendant to connect), then streams 11-int packets at 125 Hz. Modes: `servoj`
  (planned moves, FollowJointTrajectory action), **`speedl` (WASD jog — the robot
  does the Cartesian→joint conversion onboard, pendant-grade straight lines:
  measured ~0.1 mm median off-axis at 500 mm/s)**, and `speedj` zeros (idle hold).
  Control loop runs SCHED_FIFO:20. speedl vectors are built in `base_link` then
  rotated π about Z into the UR-native Base frame.
- **Jog correctness**: the twist is ramped in Cartesian space (never per-joint — that
  bends the path), with closed-loop **orientation hold** (proportional, tf2-based) and
  **straight-line hold** (PI, critically damped) riding on the commanded twist as
  trim. Jog axes are tool- or base-frame (web toggle).
- **Rotation jog** (pan/tilt/roll of the TCP): keys **I/K** tilt · **J/L** pan ·
  **U/O** roll (tool frame; ±RX/RY/RZ in base frame), or the touch pads with the
  **rot** toggle (same `held` set + stream — never a 2nd command path). Three
  angular axes in the SAME CBOR jog frame (`[lx,ly,lz,ax,ay,az]`; legacy 3-element
  frames still accepted), through the same lease/ramp/sentinel/amplification-guard
  machinery (`QD_ALLOW_ROT_SLOPE` extends the qd allowance; link scaling preserves
  the blind-travel invariant for rotation too). While rotating, the orientation
  hold stands DOWN (the user owns the orientation; re-locks at the new pose on
  stop) and the line hold degenerates to a POSITION hold (TCP pivots in place even
  if the pendant TCP ≠ tool0). Speed via the `jrspeed` slider / `jog_rot_speed`
  param (default 0.5 rad/s ≈ 29°/s, max 1.0 = servo.yaml rotational cap). Axis
  signs VERIFIED on the real robot 2026-06-12 (the "rotation is very slow"
  report was the old 0.50 m wall-distance DEFAULT pinning Servo at ×0.25 for
  the whole session — now defaults to 0.10 m). E2E:
  `pixi run python src/telamoto_bringup/test/test_rot_jog_e2e.py`.
- **Web tuning UI** on `http://<pc>:8080` (IPv6 dual-stack; comes up with the
  robot OFF too — `_pc_ip()` must never raise, and `/ws3d` streams zero-joint
  frames before the first joint state or the cage never renders — "walls
  disappeared" bug 2026-06-12). **Plain HTTP ONLY by design**
  (no certs on the LAN/VPN): a TLS hello on the port is closed instantly so a
  browser's HTTPS-First falls back to http on its own (throttled log names the
  http URL); the page bounces itself off https (`__WEB_PORT__` injected at
  serve time = real port); every WebSocket is pinned to `ws://` — live sliders (speed/stiffness/
  jog speed/accel/orientation-hold `okp`/straight-line-hold `pkp`/self-collision
  & wall distance — pushed to `/servo_node` params), Base/Tool frame toggle,
  WASD jog over **UDP first** — a WebRTC datachannel (unordered/no-retransmit;
  aiortc server-side on asyncio thread `ri-rtc`; signaling = one POST
  `/api/rtc`, offer in/answer out; STUN via the `rtc_stun` param (default
  Google, `''` = host-only) for the public-internet deployment, link line
  shows `udp`/`tcp`) with the WebSocket as automatic
  fallback (sole path when aiortc is missing → `/api/rtc` 503). Same CBOR
  frames either way, now 7 elements: a mod-2^16 **seq** (7th) lets the server
  drop reordered frames — a stale pre-release twist must never resurrect
  motion after the keyup zero. Both transports feed ONE `_handle_jog_frame`,
  so every jog fail-safe is transport-agnostic. The web server also sets
  `disable_nagle_algorithm = True` (Nagle + delayed ACK turned the 25 Hz
  /ws3d push into RTT-paced bursts on remote links). **Webcam feed** (cam
  button → floating overlay, tap = small↔large): the page opens a SEPARATE
  recvonly peer connection through the same `/api/rtc`; the server attaches
  `/dev/video0` (params `cam_device`/`cam_size`/`cam_fps`, default 480p30
  VP8) as a WebRTC video track — RTP/UDP, `MediaRelay(buffered=False)` +
  zero receiver jitter-buffer hints, so a slow link drops frames instead of
  lagging. Camera opens on the FIRST viewer; released only after
  `CAM_IDLE_CLOSE` (5 s) with no viewers — NEVER instantly: a quick off→on
  toggle must reuse the open device (v4l2 close is async; instant reopen hit
  EBUSY = the 2026-06-12 black-screen report; open also retries on EBUSY).
  Client guards every late ontrack/state event on the CURRENT pc and shows
  the overlay only at `loadeddata` (first decoded frame — a cold camera is
  seconds behind ontrack). Cam button hidden when the device is absent. RTC E2E
  (incl. live webcam phase):
  `pixi run python src/telamoto_bringup/test/test_rtc_jog_e2e.py`.
  And an **embedded 3D twin** (three.js
  + urdf-loader vendored in `web/static/`). **The fullscreen mobile layout is
  the ONLY interface on EVERY device** (`<body class="fs">` — the classic scroll
  page and its ✕ exit were REMOVED 2026-06-16; the page never leaves fullscreen):
  full-viewport 3D view, glassy top
  bar (status dot, ping + udp/tcp transport + auto-slow readout,
  walls/frame/⚙, collision-alert chip — UI slimmed 2026-06-12: the inline
  jog-speed slider is gone, jog speed lives only in the ⚙ tuning panel; the
  **rot** toggle moved to the round CENTER pad of the jog grid (`#rotpad`,
  deliberately NOT class `.jb` so pad handlers/halt-disabling skip it); the
  jog enable button is gone — the page forces jog mode ON at load (always the
  use case; the SERVER-side jog-mode gate still exists for API/tests); the
  classic-page diagnostics — pendant speed slider, joint-speed peak, link
  rtt/gap/lease/scale — now live at the bottom of the ⚙ tuning panel; hidden
  `viewon`/`basef`/`rotf`/`jogon` checkboxes survive as state holders only),
  **touch jog pads** overlaid bottom
  (hold-to-jog buttons feeding the same `held` set + 30 Hz stream as the
  keyboard, so every jog fail-safe applies unchanged; labels follow the
  Base/Tool toggle; desktop shows the WASD key above each label), sliders in a
  bottom-sheet **tuning panel** (⚙; deep link `/#tune`). **Collision alerts**:
  banner driven by `/api/state` (`servoCode`/`collBlocked`/`collWall`/
  `collAxis`/`nearWall`) — names the wall hit (surface distance + jog-direction
  cone, never center distance), shows the escape key/pad, disables all pads
  except the escape direction during a directional halt. Defaults: jog ON, base
  frame ON, walls hidden. Twin: URDF from latched `/robot_description` via
  `/api/urdf`, meshes via `/pkg/<package>/<path>` (traversal-guarded), robot +
  planning-scene walls + TCP trail on a ~25 Hz binary `/ws3d` stream (send-only
  — never touches the jog path or the link-jitter estimator); boxes large in
  ≥2 dimensions (>1.5 m) render extra-faint — PANELS only: a slim tall box
  like the base keep-out column stays at full 0.45 opacity (the old
  any-side rule made the column near-invisible, found 2026-06-12).
  Fullscreen is CSS-fixed (iPhone Safari has no
  element-fullscreen API; it is the only layout — no `/#fs`/`/#3d` deep links
  anymore, the page always boots into it). **Tilt-to-jog: REMOVED 2026-06-12**
  (the round center pad + deviceorientation handling are gone from the page);
  the server still accepts analog float axes (CBOR float32, clamped, non-finite
  rejected) in the jog frame, so an analog input source can return without a
  protocol change. E2E (incl. headless-Chromium render):
  `pixi run python src/telamoto_bringup/test/test_twin_e2e.py`.
- **Collision cage**: `scene_wall.py` (launched in both twin modes) puts six movable
  walls in the planning scene (`wall_front/back/left/right/top/floor`; defaults
  x ±0.8, y ±0.8, top 1.6, floor −0.03 m) — drag each along its normal via its
  RViz interactive marker (namespace `/scene_wall`) — plus a FIXED
  **`wall_column`** (square, half-width `column_r` = 0.18 m, floor→top, on the
  base z-axis, no marker): the shoulder-singularity keep-out (2026-06-12
  protective stop: TCP 0.28 m radial ⇒ qd 2.5 rad/s at a 100 mm/s command —
  Servo's wrist-tuned singularity thresholds never fired there). Full height
  is correct (the singular locus is the whole vertical axis; the incident was
  at z 1.1 m) — the RADIUS is the tuning knob: hard stop at the surface, the
  wall-distance decel band then starts ≈0.28 m where it measurably turned
  violent. Kept tight because the column also blocks PLANNED moves through
  the core, which are joint-space and inherently safe near the axis. Only the
  WRIST links are checked against the column (base→forearm are ACM-allowed:
  near the axis at ordinary poses + a forearm sweeping over the base with the
  wrist out is well-conditioned). ⚠ The **floor** and the **column** are
  published only after the node fetches move_group's ACM
  (`/get_planning_scene`) and extends it (`ACM_EXCLUSIONS`): floor↔{base_link,
  base_link_inertia, shoulder_link} allowed — those links are permanently near
  z=0 and would otherwise pin Servo's collision monitor at contact (jog
  bricked). The extended ACM rides every 1 Hz keepalive (PSM ACM semantics =
  replace-on-non-empty; never publish a partial ACM). With the full
  cage keep the wall-distance slider ≤ ~10 cm or jog is permanently slowed
  (now the DEFAULT: `SCENE_COLL_DEF` 0.10 m — the old 0.50 default pinned
  every session at ×0.25; root cause of the 2026-06-12 "rotation very slow"
  report and a misleading factor in the protective-stop incident).
  **Dragged poses persist** across restarts/reboots: saved on mouse-up to
  `~/.ros/telamoto_cage_poses.yaml` (`pose_file` param; delete it for defaults).
  The web 3D view has a "walls" button to hide the cage RENDERING only —
  collision checking is unaffected.
  Cage E2E: `pixi run python src/telamoto_bringup/test/test_cage_e2e.py`.
  Planned moves avoid the walls; the jog is gated by Servo's
  collision monitor (`check_collisions` in `ur_servo.yaml`): decelerate inside the
  proximity threshold, halt at contact (self-collision too, via the SRDF ACM).
  **Collision-halt escape**: the halt is direction-blind (Servo scores state, not
  command), so the controller captures the approach direction at the halt and lets
  roughly-opposite commands (≥60° cone) through at ×0.25 to back out; a halt that
  appeared while idle (no captured direction) allows crawl in ANY direction —
  Servo re-halts instantly if the user moves further in. ⚠ **Self-collision
  threshold must stay < ~0.015 m** (default 0.01): the UR10's forearm↔wrist_2
  surfaces stand 1.5–2 cm apart at EVERY pose (measured), so any larger value
  pins Servo in DECELERATE_FOR_COLLISION = jog permanently ×0.25 (the old 0.30
  default did this — the jog never ran at full speed). Self-collision still
  hard-halts at model contact. Self-collision E2E (real servo_node, isolated
  domain 79): `pixi run python src/telamoto_bringup/test/test_selfcoll_e2e.py`
  — in tests always aggregate `/servo_node/status` over a window, never
  point-sample (a jittery feed interleaves NO_WARNING between collision codes).
- **Jog fail-safes** (verified by fake-robot E2E): jog frames (WS and RTC alike)
  are ignored server-side when jog mode is off; jog is hard-blocked whenever
  Servo's status is >0.5 s stale (servo_node dead/not armed = no sentinel = no
  motion); web/param tunables reject non-finite values; jog twist leases expire
  in 0.1 s; an RTC peer disconnect zeroes the twist like a WS drop.
- **Laggy-link compensation** (remote jogging — the UI is used straight off the
  public IP, user decision 2026-06-12, no VPN): the browser streams at a fixed
  30 Hz whenever jog mode is on — real `[lx,ly,lz]` frames while moving (+ one zero
  on stop), **1-byte CBOR empty-array KEEPALIVES while idle** (paused when the tab
  is hidden). Keepalives feed ONLY the link-jitter estimator — never the twist or
  its lease — so an idle second tab can't fight the active tab's jog (was: 30 Hz
  zero-flapping = choppy motion). The server peak-holds the frame-arrival gap and
  stretches the lease (floor 0.1 s, ceiling 0.4 s, only the excess over a 50 ms
  jitter tolerance counts; **de-twitch: an isolated big gap is ignored — only a
  2nd big gap within 2 s feeds the peak**, so a single browser GC pause can't
  modulate jog speed) while scaling jog speed down by the same factor — invariant:
  `scaled_speed × lease = jog_speed × 0.1 s`, so worst-case blind travel never
  exceeds the LAN case. Link stats (RTT/gap/lease/scale) live in the web UI.
  E2E test (real node + real WS, no robot needed; isolated ROS domain + ports,
  safe to run beside a live bringup):
  `pixi run python src/telamoto_bringup/test/test_jog_lag_e2e.py` — verifies
  sub-ms start/stop latency, LAN scale ×1.00, invariant, spike de-twitch,
  keepalive isolation, recovery.
- ⚠ **Launch gotcha**: never wrap `ur_moveit.launch.py` in a `TimerAction` —
  TimerAction push/pops launch configurations, and that file starts move_group from
  an `OnProcessExit` handler that fires after the pop → "launch configuration
  'warehouse_sqlite_path' does not exist". Its `wait_for_robot_description` node
  already provides the start ordering. Real-mode move_group **respawns**
  (`respawn=True`, 5 s): on a robot-off cold start there are no /joint_states,
  the planning scene monitor times out (10 s) and move_group exits FATAL —
  without respawn that permanently killed planned moves AND the ACM fetch
  (floor + base column withheld) for the whole session.
- ⚠ **SAFETY** (past incident: singularity amplification made the arm shoot) —
  layered, all must stay ON: (1) MoveIt Servo runs as a **singularity sentinel**
  (its `~/status` gates the speedl command; thresholds in `config/ur_servo.yaml`);
  (2) the **amplification guard** (`QD_ALLOW_*`): measured joint speed beyond what
  the **SENT (post-gate)** TCP speed justifies shrinks a smooth gate — sent, not
  the operator target (the 2026-06-12 boundary-stall trip: allowance from the
  pre-gate 248 mm/s while Servo had the jog at ×0.25/62 mm/s = guard 4× blind,
  qd hit 3.2 rad/s → UR protective stop). Self-tightening by design: amplified
  motion spirals to a crawl (slow/deviate, NEVER stop — operator preference),
  honest motion always recovers; (3) the **runaway latch** (`QD_LATCH_*`): qd >
  2.5× allowance = proportional braking has lost (boundary/deep-singularity
  amplification is unbounded) → speedj-zero hold until key release + joints
  settle — a self-recovering host-side stop instead of UR's pendant-bound
  protective stop (web banner: "joint-speed runaway" — checked FIRST in
  setCollAlert, ABOVE the collision banners: in the 2026-06-12 incident the
  collision story outranked it and the operator kept tapping into the
  amplification zone). The latch has
  **direction memory** (`QD_BLOCK_*`, 2nd on-robot run: re-pressing the inward
  key ratcheted deeper, escape taps re-latched ~5× before getting out): the
  inward cone stays HARD-blocked after release (banner names it + the escape
  key), the opposite cone escapes under a RAISED allowance floor (0.8) and
  latch line (1.5 rad/s) — direction judged from the live jog TARGET, never
  the stale sent twist, so an escape pressed while joints still ring counts
  as an escape from its first cycle; block clears after 5 cm of TCP travel
  from the REST pose where the latch released (any route incl. planned moves),
  and the travel counts ONLY while unlatched at honest joint speed (≤ the 0.8
  escape-allowance floor) — measuring from the latch ONSET let the 2026-06-12
  runaway lurches (5–30 cm each) self-clear the block mid-excursion, 6 taps
  ratcheted in, qd 2.56 rad/s tripped UR's 120°/s base-joint limit; (4) the URScript
  miss-watchdog `stopj`; (5) UR's own safety limits. Same-gate Servo status-code
  changes are now logged too (a code-4↔code-1 flip used to be invisible).
  Guard E2E (fake robot pulling real packets, synthetic actual_qd):
  `pixi run python src/telamoto_bringup/test/test_qd_guard_e2e.py`.
- **Singularity jog semantics** (verified by
  `pixi run python src/telamoto_bringup/test/test_singularity_e2e.py` — real
  servo_node, isolated domain 75): approach decel band → gate ×0.25, hard stop
  → gate ×0. Servo is direction-aware: a halt the JOG caused fires at the
  boundary, where the reversed twist re-evaluates as DECELERATE_FOR_LEAVING
  (×0.25) — the escape works because the sentinel feed is PRE-gate. A halt the
  JOG caused captures the approach direction (`singAxis` in `/api/state`;
  `_halt_dir` is anti-inversion-guarded against sentinel-feed flaps mid-escape
  and shared with the collision capture) → wall-style escape UI: the banner
  names the direction + escape key/pad, all pads disabled except the escape
  (UI hint only — Servo still gates the motion). Deep in the singular zone
  (reachable only by a planned move) EVERY direction halts and no direction is
  captured — deliberate, recover with a planned move; banner says so, pads stay
  enabled, Servo arbitrates the way out. Capture E2E: S1–S3 in
  test_qd_guard_e2e.py. ⚠ E2E flake: this host (realtime kernel) intermittently blacks out
  loopback-UDP delivery into a fresh servo_node for tens of seconds (both
  RMWs; `grep Udp: /proc/net/snmp` RcvbufErrors climbs; also hits
  test_selfcoll_e2e.py) — the tests gate/resync on observed tracking and
  abort with an environmental message, NOT a logic failure. Production is
  safe regardless: a silent sentinel hard-blocks the jog (0.5 s staleness).

## Diagnostics
```bash
setup/jog_log.sh          # jog/TCP trace from the newest controller run log
setup/jog_log.sh -c       # 125 Hz TCP path CSV (~/.ros/log/tcp_path.csv)
```
Protective stops do NOT drop the reverse socket — the controller detects the stalled
pull-loop and logs `PROTECTIVE STOP`, classified MID-MOTION / ON-RELEASE / IDLE.

## Common commands
```bash
pixi run build            # colcon build (RelWithDebInfo)
pixi run twin             # launch with mock hardware, no physical robot
pixi run twin-real        # real robot (add launch_rviz:=false when jog-tuning)
pixi run rviz             # RViz2 + MoveIt2 plugin only
pixi run tune             # open the :8080 web tuning UI
pixi run shell            # sourced bash shell inside the pixi env
```

## Workspace layout
```
telamoto/
├── pixi.toml
├── src/
│   ├── telamoto_bringup/     # launch + config
│   ├── telamoto_hardware/    # C++ ros2_control plugin
│   └── telamoto_msgs/        # custom msg/srv definitions
├── build/   (gitignored)
├── install/ (gitignored)
└── log/     (gitignored)
```

## Adding a new ROS2 package
```bash
cd src
ros2 pkg create --build-type ament_cmake my_pkg --dependencies rclcpp
pixi run build-pkg PKG=my_pkg
```

## UR CB3.1 kinematics calibration
Before first use on the real robot, extract the calibration:
```bash
ros2 launch ur_calibration calibration_correction.launch.py \
  robot_ip:=192.168.1.10 target_filename:=config/ur10_calibration.yaml
```
`digital_twin.launch.py` picks up `config/ur10_calibration.yaml` automatically
when the file exists.
