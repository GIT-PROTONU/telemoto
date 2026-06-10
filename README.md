# Telamoto

A ROS 2 (Jazzy) workspace for driving a **Universal Robots UR10 CB3.1** — motion
planning with MoveIt 2, real-time Cartesian (WASD) jogging from a web UI, and a
custom control bridge that talks to the robot's **External Control URCap**
directly, because the standard `ur_robot_driver` cannot be used on this robot
(see [Why no `ur_robot_driver`](#why-no-ur_robot_driver)).

This is the **first real working version**: planned moves from RViz and live WASD
jogging both run smoothly on the physical UR10.

---

## Stack

| Layer | Package | Notes |
|---|---|---|
| Environment / package manager | **Pixi** (RoboStack-Jazzy channel) | `pixi.toml` at repo root — no system ROS install needed |
| Motion planning | **MoveIt 2** | Jazzy release |
| Real-time jog | **URScript `speedl`** (onboard IK) | straight lines like the pendant — measured 0.15 mm avg off-axis deflection at up to 500 mm/s; MoveIt Servo stays on as a singularity sentinel |
| Control | `ros2_control` + `ros2_controllers` | scaled joint-trajectory controller (sim/twin) |
| Custom interfaces | `telamoto_msgs` | `msg/`, `srv/` |
| HW extension | `telamoto_hardware` | C++ pluginlib plugin |
| Bringup | `telamoto_bringup` | launch files, configs, the control bridge |

- **Robot:** UR10 **CB3.1**, PolyScope 3.x
- **Default robot IP:** `192.168.10.2` (override with the `robot_ip` launch arg)

---

## Quick start

```bash
pixi run build            # colcon build (RelWithDebInfo, Ninja, symlink-install)

pixi run twin             # digital twin: UR10 + MoveIt 2 + RViz, fake hardware (no robot)
pixi run twin-real        # REAL robot via the custom control bridge (see below)
pixi run tune             # open the motion-tuning + WASD jog web UI (also on a phone)

pixi run rviz             # RViz + MoveIt only (controllers must already be running)
pixi run shell            # sourced bash shell inside the Pixi env
```

`pixi run twin-real` is the real-robot entry point. It launches the digital-twin
stack against the physical UR10 and starts the control bridge — **no
`ur_robot_driver`**.

### Running on the real robot

1. **Network:** the robot must be reachable at `192.168.10.2`. For low-jitter
   control use the direct onboard-NIC link — see [Host setup](#host-setup-network--realtime).
2. **Pendant:** load a program named `telamoto` containing a single **External
   Control** node (FZI `externalcontrol-1.0.5`), configured with this PC's IP and
   Custom Port **50001**.
3. `pixi run twin-real`
4. **Press Play on the pendant.** The URCap then requests the control script from
   the bridge and connects back. (A dashboard `play` reuses a cached script — a
   real **Play** on the pendant is required to (re)load it.)
5. `pixi run tune` (or browse `http://<this-pc-ip>:8080` from a phone) for the
   live tuning sliders and WASD jogging.

---

## How motion actually works

### Why no `ur_robot_driver`

This UR10 runs PolyScope 3.x **without Remote Control mode**. In that
configuration the controller holds *all* RTDE inputs, so `ur_robot_driver` (which
drives the robot through RTDE inputs) can never take control — it always fails to
hand off. Joint states can still be *read* from the RTDE output stream, but
commanding the robot has to go a different way.

So instead of the driver, Telamoto speaks the **External Control URCap**
script-sender protocol directly:

- The pendant's External Control node connects to this PC and sends the literal
  `request_program`.
- The bridge (`ur_servo_controller.py`) replies with a small URScript control
  loop, which connects back to the same port.
- From then on the robot **pulls** one target per cycle and executes it.

`ur_rtde_joint_pub.py` separately reads the RTDE output stream and republishes
`/joint_states` at the robot's native 125 Hz (used to seed planning / Servo).

### The pipelined pull loop (robot-paced, ~125 Hz)

Each cycle the on-robot URScript overlaps the socket round-trip with the motion:

```
socket_send_int(1)              # request the NEXT target
speedl/speedj/servoj(...)       # execute the CURRENT one (continuous, ~100% duty)
p = socket_read_binary_integer  # consume the reply that arrived DURING the motion
```

A naive request→block→move loop leaves the motion command idle during the socket
round-trip (~50% duty → the arm averages half the commanded speed → protective
stop at higher speeds). Pipelining keeps motion back-to-back at the robot's native
**125 Hz**, one reply per request, no backlog. If the PC dies, the script
`stopj()`s after ~8 missed reads (~64 ms).

### Three modes

The reply's mode field selects the on-robot command:

- **`servoj`** (joint position) — planned moves from MoveIt.
- **`speedl`** (Cartesian velocity) — WASD jogging. The **robot's own controller**
  does the Cartesian→joint conversion onboard with fresh state — the same
  mechanism as the pendant jog, which is why it tracks straight lines at any
  speed. (A PC-side conversion works on a joint state that is ~15–30 ms stale by
  execution time, which bends the path proportionally to speed — measured 21 mm
  over a 343 mm push at 500 mm/s before this design.)
- **`speedj`** zeros — the active idle hold.

### WASD jog pipeline

```
browser (WASD) ──WebSocket(CBOR)──▶ bridge ──ramp + holds──▶ speedl ──▶ robot
                                      │                        ▲
                                      └─TwistStamped─▶ MoveIt Servo (sentinel)
                                                         └──status gate──┘
```

- The web UI streams the jog direction as a compact **CBOR** frame over a
  WebSocket (~30 Hz while a key is held; the first keypress fires immediately).
- The bridge ramps the twist in Cartesian space (direction-preserving) and adds
  two closed-loop trims: an **orientation hold** (proportional lock on the tool
  orientation captured at jog start) and a **straight-line hold** (PI lock,
  critically damped, on the line through the jog-start point).
- **MoveIt Servo is a singularity sentinel**, not the executor: it receives the
  same twist, keeps evaluating the kinematics, and its status gates the speedl
  command (full speed → crawl → zero).
- **Frames:** tool frame (`W/S` along the tool, `A/D`/`Q/E` across) or base frame
  (`A/D` = ±X, `Q/E` = ±Y, `W/S` = ±Z) — live toggle in the web UI.

Measured result on the physical UR10: **~0.1 mm median off-axis deviation** at up
to 500 mm/s, orientation held to ≤0.1°.

---

## Performance

Measured on the physical robot, control bridge on the same PC, direct NIC link.

### Loop rate — 125 Hz

The CB3 runs its servo loop at **125 Hz (8 ms)**. With the pipelined URScript the
socket round-trip overlaps the motion, so the loop runs at the robot's native rate
(measured: 626 cycles / 5 s, max gap ~10 ms; the control-loop thread runs
`SCHED_FIFO` to keep replies inside the motion window). `step_t` is fixed at
**8 ms** — one CB3 control cycle, the floor.

### End-to-end latency — keypress → robot moves

Jogging from a browser on the control PC (MoveIt Servo is out of the command
path — it only gates):

| Stage | Typical | Worst |
|---|---|---|
| keypress → browser event → `ws.send` *(est.)* | ~10 ms | ~16 ms |
| WebSocket → bridge (localhost) | <1 ms | ~1 ms |
| control loop picks up + sends (125 Hz / 8 ms) | ~4 ms | ~8 ms |
| robot reads `speedl`, first motion (8 ms cycle) | ~8 ms | ~8 ms |
| **Total** | **~23 ms** | **~33 ms** |

From a **phone over WiFi**, add ~10–30 ms for the wireless hop. This is latency to
*motion onset*; reaching the commanded velocity then ramps at `jog_accel` (a
tunable feel knob, not pipeline latency).

---

## Tuning web UI

Served by the control bridge on port **8080** while `twin-real` runs (`pixi run
tune`, or `http://<this-pc-ip>:8080` from a phone). All sliders are **live** and
clamped to UR servoj-safe ranges:

- **Speed** — trajectory time-scale for planned moves (0.25–3.0×).
- **Stiffness** — `servoj` gain.
- **Smoothness** — `servoj` lookahead.
- **Jog speed** — Cartesian jog speed (mm/s; 5 mm/s floor for fine work).
- **Jog acceleration** — Cartesian start/stop ramp (gentler ↔ snappier feel).
- **Orientation hold** (`okp`) — stiffness of the tool-orientation lock
  (raise for a tighter hold, lower if it oscillates; 0 = off).
- **Straight-line hold** (`pkp`) — stiffness of the path lock
  (same trade-off; 0 = off).
- **Base frame** checkbox — jog along robot-base axes instead of tool axes.
- A live robot-connection indicator, pendant speed-slider readout, and actual
  joint-speed readout.

In RViz, planned-move speed comes from MoveIt's **Velocity/Acceleration Scaling**
(default 0.1 — raise to 1.0 for fast moves), not the web slider.

---

## Host setup (network & realtime)

Optional but recommended for the smoothest control. Scripts in `setup/`:

- **`robot-net.sh`** — dedicated, direct link from the onboard PCIe NIC (`eno1`)
  straight into the UR10, instead of routing through a USB Ethernet adapter and
  the home router (a latency/jitter source). Creates a static-IP NetworkManager
  profile that never touches your default route / DNS / internet.
  ```bash
  sudo IFACE=eno1 bash setup/robot-net.sh
  # then on the pendant: Setup Robot > Network > Static address
  #   IP 192.168.10.2  mask 255.255.255.0  gateway (blank)
  ```

- **`realtime-limits.sh`** — grants `rtprio`/`memlock` so MoveIt Servo's
  `SCHED_FIFO` servo thread can actually use the realtime kernel (`ulimit -r` is 0
  by default, which silently demotes it). **Log out and back in afterwards.**
  ```bash
  sudo bash setup/realtime-limits.sh
  ```

---

## Safety

⚠️ **The arm once shot at high speed near a singularity.** The guards are layered
and must all stay on:

1. **MoveIt Servo singularity sentinel** — Servo receives every jog twist and its
   status gates the `speedl` command: full speed when clean, crawl approaching a
   singularity, zero at the halt threshold (`config/ur_servo.yaml` thresholds).
2. **Amplification guard** — the incident's signature is a *small* command
   producing *large* joint speeds (J⁻¹ blowup near a singularity). Measured joint
   speed beyond what the commanded TCP speed justifies
   (`QD_ALLOW_BASE + QD_ALLOW_SLOPE·|cmd|`) shrinks a smooth slew-limited gate —
   tight at low speed where the danger lives, permissive at honest high speed,
   and unable to limit-cycle (a fixed cap stuttered at full speed).
3. **Dead-host watchdog** — the on-robot URScript `stopj()`s after ~8 missed
   reads (~64 ms); a stalled or dead PC cannot run the arm away.
4. **Jog lease** — release, latency, drop, or crash zeroes the command within
   tens of ms.
5. **UR's own safety limits** backstop everything.

---

## Workspace layout

```
telamoto/
├── pixi.toml                 # env + tasks
├── setup/                    # host network + realtime setup scripts
└── src/
    ├── telamoto_bringup/     # launch, config, the control bridge
    │   ├── launch/           # digital_twin, moveit(_real), ur10_bringup, rviz
    │   ├── config/           # ur_servo.yaml, kinematics, limits, calibration, …
    │   └── scripts/
    │       ├── ur_servo_controller.py   # the control bridge (URScript sender, jog, web UI)
    │       ├── ur_rtde_joint_pub.py     # /joint_states from the RTDE output stream
    │       └── ur_dashboard_autoplay.py
    ├── telamoto_hardware/    # C++ ros2_control plugin
    └── telamoto_msgs/        # custom msg/srv
```

`build/`, `install/`, and `log/` are gitignored.

---

## Adding a ROS 2 package

```bash
cd src
ros2 pkg create --build-type ament_cmake my_pkg --dependencies rclcpp
pixi run build-pkg PKG=my_pkg
```

## Kinematics calibration

Extract the per-robot calibration before first use, then it is loaded from
`config/ur10_calibration.yaml`:

```bash
pixi run calibrate    # writes src/telamoto_bringup/config/ur10_calibration.yaml
```
