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
| Real-time jog | **MoveIt Servo** | Cartesian twist → joint velocities |
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

### The pull loop (robot-paced)

Each cycle the on-robot URScript does:

```
socket_send_int(1)              # "send me a target"  (request)
p = socket_read_binary_integer  # 11×int32 reply from the PC
servoj(...) or speedj(...)      # execute it
```

The PC's control loop blocks on the request and answers with the freshest target.
This **pull** model (rather than the PC pushing on a timer) is deliberate: pushing
either left gaps between `servoj` calls (~100 Hz vibration) or backed up the TCP
buffer (growing lag). Pulling keeps motion back-to-back and one reply per request
— no gaps, no backlog. **Do not revert to push.**

### Hybrid servoj / speedj

The reply's mode field selects the on-robot command:

- **`servoj`** (position) — planned moves from MoveIt, and the idle hold.
- **`speedj`** (joint velocity) — WASD jogging.

WASD jog uses velocity because, for continuous human-in-the-loop teleop, the
operator closes the position loop by eye: `speedj` has no reference pose to snap
back to and tolerates control-cycle jitter, whereas `servoj` would turn a position
delta over a too-short step into a velocity spike → protective stop.

### WASD jog pipeline

```
browser (WASD)  ──WebSocket(CBOR)──▶  bridge  ──TwistStamped──▶  MoveIt Servo
                                                                      │
                                                            JointTrajectory
                                                                      ▼
robot  ◀──speedj──  bridge control loop  ◀──────────────────  (joint velocities)
```

- The web UI streams the jog direction as a compact **CBOR** frame over a
  WebSocket (~30 Hz while a key is held; the first keypress fires immediately).
- The bridge publishes a `TwistStamped` to MoveIt Servo, which solves for joint
  velocities and streams them back; the bridge feeds them to `speedj`.
- **Tool frame:** `W/S` along the tool, `A/D` and `Q/E` across the flange.

---

## Performance

Measured on the physical robot, control bridge on the same PC, direct NIC link.

### Loop rate — 62 Hz setpoints over a 125 Hz robot

The CB3 runs its servo loop at **125 Hz (8 ms)** internally and `speedj` holds the
commanded velocity continuously between updates — so the *robot's motion* is always
smooth 125 Hz. What we control is the **setpoint refresh rate**.

The pull loop's cycle time is **≈ 2.1 × the commanded step duration** (`step_t`),
with essentially no fixed overhead — confirmed by a two-point fit:

| commanded `step_t` | measured cycle | rate |
|---|---|---|
| 8 ms | 16 ms | **62 Hz** |
| 50 ms | 104 ms | 10 Hz |

`step_t` is fixed at **8 ms** (one CB3 control cycle — the floor; the robot cannot
act on sub-cycle durations). That yields ~62 Hz. It is *not* fed back from the
measured cycle time: with the 2× behaviour, doing so is positive feedback that
pegs the rate at the clamp (this was a real bug — the loop was stuck at 10 Hz).

62 Hz is the architectural ceiling for a pull loop: one setpoint costs two control
cycles (one to move, one for the socket round-trip), so 125 Hz *setpoints* would
require a push/pipelined transport, which reintroduces the vibration/backlog the
pull design exists to avoid. 62 Hz of velocity refresh on top of a 125 Hz servo
loop is far more than human teleop needs.

### End-to-end latency — keypress → robot moves

Jogging from a browser on the control PC:

| Stage | Typical | Worst |
|---|---|---|
| keypress → browser event → `ws.send` *(est.)* | ~10 ms | ~16 ms |
| WebSocket → bridge (localhost) | <1 ms | ~1 ms |
| publish twist → MoveIt Servo (DDS) | ~0.3 ms | ~0.5 ms |
| Servo cycle + IK (`publish_period` 4 ms, 250 Hz) | ~2–3 ms | ~5 ms |
| Servo output → bridge target (DDS) | ~0.3 ms | ~0.5 ms |
| control loop picks up + sends (62 Hz / 16 ms) | ~8 ms | ~16 ms |
| robot reads `speedj`, first motion (8 ms cycle) | ~8 ms | ~8 ms |
| **Total** | **~30 ms** | **~47 ms** |

From a **phone over WiFi**, add ~10–30 ms for the wireless hop (~40–60 ms total).
This is latency to *motion onset*; reaching the commanded velocity then ramps at
the `jog_accel` (a tunable feel knob, not pipeline latency). The two pull cycles at
the end dominate the controllable budget. Rows above `_apply_jog` are estimates;
everything from the bridge onward is measured/configured.

---

## Tuning web UI

Served by the control bridge on port **8080** while `twin-real` runs (`pixi run
tune`, or `http://<this-pc-ip>:8080` from a phone). All sliders are **live** and
clamped to UR servoj-safe ranges:

- **Speed** — trajectory time-scale for planned moves (0.25–3.0×).
- **Stiffness** — `servoj` gain.
- **Smoothness** — `servoj` lookahead.
- **Jog speed** — Cartesian jog speed (mm/s; 5 mm/s floor for fine work).
- **Jog acceleration** — `speedj` ramp rate (gentler start/stop).
- **Jog coast** — how long a jog velocity stays valid after Servo goes quiet
  (lower = crisper stop; default 30 ms).
- A live robot-connection indicator.

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

⚠️ **The arm once shot at high speed near a singularity.** Two guards must stay on
and must not be weakened:

- **Hard per-joint speed cap** on jogging (`MAX_JOG_QD`) — the arm cannot exceed
  it no matter what Servo emits (singularity amplification, glitches).
- **Singularity deceleration** in MoveIt Servo (`config/ur_servo.yaml`).

Other safety properties of the design:

- `speedj` jog is open-loop velocity, so the on-robot URScript `stopj()`s on a
  missed read — a stalled or dead PC cannot run the arm away.
- The jog has a lease/coast window: release, latency, drop, or crash stops the
  motion within tens of ms.

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
