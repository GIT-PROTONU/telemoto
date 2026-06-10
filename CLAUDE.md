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
  jog over a WebSocket (CBOR frames).
- **Collision testing**: `scene_wall.py` (launched by both bringups) puts a draggable
  wall in the planning scene — move it via its RViz interactive marker
  (namespace `/scene_wall`). Planned moves avoid it; the jog is gated by Servo's
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
