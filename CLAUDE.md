# Telamoto — ROS2 / UR10 CB3.1 workspace

## Stack
| Layer | Package | Notes |
|---|---|---|
| Package manager | **Pixi** (RoboStack-Jazzy channel) | `pixi.toml` at repo root |
| Robot I/O | **custom** (see Motion architecture) | `ur_robot_driver` does NOT work on this CB3 |
| Motion planning | **MoveIt2** (`moveit`) + **MoveIt Servo** | Jazzy release; Servo drives the WASD jog |
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
- **Web tuning UI** on `http://<pc>:8080` — live sliders (speed/stiffness/jog
  speed/accel/orientation-hold `okp`/straight-line-hold `pkp`/self-collision &
  wall distance — pushed to `/servo_node` params), Base/Tool frame toggle, WASD
  jog over a WebSocket (CBOR frames), and an **embedded 3D twin** (three.js +
  urdf-loader vendored in `web/static/`, lazy-loaded; deep link `/#3d`).
  **Mobile-first layout**: big 3D view up top, **touch jog pads** (hold-to-jog
  buttons feeding the same `held` set + 30 Hz stream as the keyboard, so every
  jog fail-safe applies unchanged; labels follow the Base/Tool toggle), and all
  sliders live in a bottom-sheet **tuning panel** (floating ⚙ button; deep link
  `/#tune`). Twin: URDF from latched `/robot_description` via `/api/urdf`, meshes via
  `/pkg/<package>/<path>` (traversal-guarded), robot + planning-scene walls + TCP
  trail driven by a ~25 Hz binary stream on `/ws3d` (send-only — never touches
  the jog path or the link-jitter estimator); boxes with a side >1.5 m render
  extra-faint (the camera looks at the robot through the cage). **Fullscreen
  mode** (⛶ on the view / deep link `/#fs`): CSS-fixed (iPhone Safari has no
  element-fullscreen API), pads + a top bar (status, frame/jog toggles, exit)
  overlay the view. **Tilt-to-jog**: HOLD the round center pad and tilt the
  phone — dead-man control; orientation deltas from the hold pose (4° deadzone,
  25° full scale) become analog lx/ly (CBOR float32 in the same jog frames —
  the server clamps and rejects non-finite). Browsers only deliver orientation
  events on HTTPS (front the UI with `tailscale serve`); iOS also asks
  permission. E2E (incl. headless-Chromium render):
  `pixi run python src/telamoto_bringup/test/test_twin_e2e.py`.
- **Collision cage**: `scene_wall.py` (launched by both bringups) puts six movable
  walls in the planning scene (`wall_front/back/left/right/top/floor`; defaults
  x ±0.8, y ±0.8, top 1.6, floor −0.03 m) — drag each along its normal via its
  RViz interactive marker (namespace `/scene_wall`). ⚠ The **floor** is published
  only after the node fetches move_group's ACM (`/get_planning_scene`) and extends
  it: floor↔{base_link, base_link_inertia, shoulder_link} allowed — those links
  are permanently near z=0 and would otherwise pin Servo's collision monitor at
  contact (jog bricked). The extended ACM rides every 1 Hz keepalive (PSM ACM
  semantics = replace-on-non-empty; never publish a partial ACM). With the full
  cage keep the wall-distance slider ≤ ~10 cm or jog is permanently slowed.
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
  appears while idle stays fully blocked (move the wall instead). ⚠ The UR10's own
  collision model keeps certain wrist links ~2 cm apart at ANY pose, so a
  self-collision distance above ~2 cm = permanent ×0.25 jog (the 30 cm default
  slider value does this — lower it to ~1–2 cm for full-speed jogging).
- **Jog fail-safes** (verified by fake-robot E2E): WS jog frames are ignored
  server-side when jog mode is off; jog is hard-blocked whenever Servo's status is
  >0.5 s stale (servo_node dead/not armed = no sentinel = no motion); web/param
  tunables reject non-finite values; jog twist leases expire in 0.1 s.
- **Laggy-link compensation** (remote/VPN jogging): the browser streams at a fixed
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
  already provides the start ordering.
- ⚠ **SAFETY** (past incident: singularity amplification made the arm shoot) —
  layered, all must stay ON: (1) MoveIt Servo runs as a **singularity sentinel**
  (its `~/status` gates the speedl command; thresholds in `config/ur_servo.yaml`);
  (2) the **amplification guard** (`QD_ALLOW_*`): measured joint speed beyond what
  the commanded TCP speed justifies shrinks a smooth gate; (3) the URScript
  miss-watchdog `stopj`; (4) UR's own safety limits.

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
pixi run bringup-fake     # launch with mock hardware, no physical robot
pixi run bringup robot_ip:=192.168.10.2   # real robot (add launch_rviz:=false when jog-tuning)
pixi run rviz             # RViz2 + MoveIt2 plugin only
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
Then pass `kinematics_params_file` in `ur10_bringup.launch.py`.
