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
  speed/accel/orientation-hold `okp`/straight-line-hold `pkp`), Base/Tool frame
  toggle, WASD jog over a WebSocket (CBOR frames).
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
