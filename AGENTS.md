# AGENTS.md — Telamoto workspace

Detailed context lives in `CLAUDE.md` (read it). This file is the agent-facing
operating guide: what to build, what must never change.

## Project
ROS2 / UR10 CB3.1 teleop workspace. Pixi-managed (RoboStack-Jazzy), MoveIt2 +
MoveIt Servo, TRAC-IK IK (planned moves only), custom robot I/O (no
`ur_robot_driver` — it cannot work on this PolyScope 3.x controller).

Packages: `telamoto_msgs` (interfaces), `telamoto_hardware` (C++ ros2_control
plugin), `telamoto_bringup` (launch/config/web/scripts).

## Commands
```bash
pixi run build            # colcon build (RelWithDebInfo)
pixi run twin             # mock: RViz + :8080 web UI/twin (jog won't move fake arm)
pixi run twin-real        # real robot (add launch_rviz:=false when jog-tuning)
pixi run record           # LeRobot teach node (needs --ros-args -p dataset_root:= -p task:=)
pixi run rviz             # RViz2 + MoveIt2 only
pixi run tune             # open :8080 web tuning UI
pixi run shell            # sourced bash shell in pixi env
pixi run build-pkg PKG=x  # add a new ament_cmake package
```
`twin`/`twin-real` also launch the LeRobot teach node by default (disable with
`use_lerobot:=false`); its start/stop Trigger services + status feed the web
UI's red record button on `/api/record` → `/api/state` recAvail/recording/
recFrames/recEps. The node runs via `prefix="pixi run --environment lerobot python"`
(needs `lerobot_root`/`lerobot_task` args).
Tests (E2E, fake/real robot, isolated ROS domains — safe beside a live bringup):
```bash
pixi run python src/telamoto_bringup/test/test_<name>_e2e.py
```
Names: `rot_jog`, `rtc_jog`, `twin`, `cage`, `selfcoll`, `jog_lag`, `qd_guard`,
`singularity`, `lerobot_record`.

Diagnostics: `setup/jog_log.sh` (jog/TCP trace; `-c` → 125 Hz CSV).

## Motion architecture (no standard driver)
PolyScope holds all RTDE **input** registers, so custom scripts stream to the
robot instead:
- `ur_rtde_joint_pub.py` — RTDE **outputs only** (port 30004, 125 Hz) → `/joint_states`.
- `ur_servo_controller.py` — External Control URCap on port 50001; 125 Hz:
  `servoj` (planned moves), `speedl` (WASD jog), `speedj` zeros (idle). Loop runs
  SCHED_FIFO:20. Jog twists are ramped in Cartesian space (never per-joint).

## Safety invariants (MUST stay ON — past singularity incident made the arm shoot)
All five layers below keep working; new code must not bypass them. `.gitignore`
the sentinel relationship too: Servo gates EVERY speedl command.
1. MoveIt Servo = **singularity sentinel** (its `/status` gates the jog). Keep its
   default KDL IK — do NOT move it to TRAC-IK (Servo is the jog safety sentinel).
2. **Amplification guard** (`QD_ALLOW_*`) — measured joint speed beyond what the
   **SENT (post-gate)** TCP speed justifies shrinks a smooth gate. Slows spare,
   never stops; honest motion recovers.
3. **Runaway latch** (`QD_LATCH_*`) — qd > 2.5× allowance → speedj-zero hold until
   key release + joints settle (+ direction memory `QD_BLOCK_*`: inward cone stays
   HARD-blocked, opposite cone escapes under a raised floor/latch line).
4. URScript miss-watchdog `stopj`.
5. UR's own safety limits.

Invariant: `scaled_speed × lease = jog_speed × 0.1 s` (worst-case blind travel never
exceeds the LAN case).

## Critical gotchas (verified on real hardware)
- **Self-collision threshold < ~0.015 m** (default 0.01). Larger pins Servo in
  DECELERATE_FOR_COLLISION = jog permanently ×0.25.
- **Keep wall-distance slider ≤ ~10 cm** (default `SCENE_COLL_DEF` 0.10 m). Old 0.50
  default pinned every session at ×0.25.
- **Never wrap `ur_moveit.launch.py` in a TimerAction** (launch-config pop kills
  move_group). `wait_for_robot_description` already orders the start; real-mode
  move_group respawns on failure.
- **Sub-include-arg leaks**: a child launch receiving a `LaunchConfiguration` returns
  may leak its value into the parent scope (e.g. `launch_rviz`). Snapshot user inputs
  into a separate config name before they're passed to child launches.
- **Jog fail-safes**: frames ignored when jog off; hard-blocked when Servo status
  stale >0.5 s; twists lease out in 0.1 s; RTC disconnect zeroes twist; reject
  non-finite tunable values; stale/reordered jog frames dropped via a mod-2^16 seq.
- **E2E test flake**: on the Realtime kernel, loopback-UDP into a fresh servo_node
  can black out for tens of seconds. Tests must gate/resync on observed tracking
  and abort with an environmental message — NOT a logic failure. In tests, always
  aggregate `/servo_node/status` over a window, never point-sample.

## Web UI (`:8080`)
- Fullscreen mobile-only interface (`<body class="fs">`) — no scroll/exit page, no
  `/#fs`/`/#3d` deep links. 3D twin (three.js) + touch pads + ⚙ tuning panel.
- Jog: UDP (WebRTC datachannel) first, WebSocket fallback. Same CBOR frames both
  ways, both feed one `_handle_jog_frame` (transport-agnostic fail-safes).
- Plain HTTP only by design (LAN/VPN, no certs). All WebSockets pinned `ws://`.
- Webcam: opens on first viewer, released after `CAM_IDLE_CLOSE` (30 s) idle —
  never instant (v4l2 EBUSY + an idle-close can segfault PyAV during reconnect).
- 3D twin boxes >1.5 m in ≥2 dims render extra-faint (panels only); a slim tall box
  (keep-out column) stays full opacity. Server must boot + stream zero-joint frames
  even with the robot off.

## Collision cage
- Six movable walls + a fixed `wall_column` (shoulder-singularity keep-out, half-width
  0.18 m, floor→top, WRIST links only checked against it). Dragged poses persist to
  `~/.ros/telamoto_cage_poses.yaml`.
- Floor + column are published only after fetch+extend of move_group's ACM
  (base_link/shoulder kept allowed vs floor). Never publish a partial ACM.

## Standard housekeeping
- No comments in code unless asked.
- Run build/checks before finishing.
- Never commit unless explicitly asked.