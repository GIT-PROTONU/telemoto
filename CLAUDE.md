# Telamoto — ROS2 / UR10 CB3.1 workspace

## Stack
| Layer | Package | Notes |
|---|---|---|
| Package manager | **Pixi** (RoboStack-Jazzy channel) | `pixi.toml` at repo root |
| Robot driver | `ur_robot_driver` | upstream UR package, included as Pixi dep |
| Motion planning | **MoveIt2** (`moveit`) | Jazzy release |
| Control | `ros2_control` + `ros2_controllers` | scaled joint-trajectory controller |
| Custom interfaces | `telamoto_msgs` | `msg/`, `srv/` |
| HW extension | `telamoto_hardware` | C++ pluginlib plugin |
| Bringup | `telamoto_bringup` | launch files, configs |

## Robot
- Model: **UR10 CB3.1**  
- Default IP: `192.168.10.2` (set `robot_ip` launch arg to override)  
- CB3 requires the `ur_robot_driver` (not the legacy `ur_modern_driver`)

## Common commands
```bash
pixi run build            # colcon build (RelWithDebInfo)
pixi run bringup-fake     # launch with mock hardware, no physical robot
pixi run bringup robot_ip:=192.168.10.2   # real robot
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
