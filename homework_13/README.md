# Antidrone Turret Control System (ROS 2 Jazzy)

A standalone ROS 2 package designed for automated FPV drone interception using a C++ business logic core and dynamic target tracking pipelines.

---

## 🛠️ System Architecture

The package splits critical navigation/firing math from the ROS 2 middleware layer to ensure modularity, deterministic execution, and full testability without a running ROS master.

### Active Nodes (Wrapper Components)
* `target_track_publisher_node` — Simulates radar/camera tracks from pre-recorded flight data.
* `turret_controller_node` — The central processing unit. Evaluates targets and issues firing orders.
* `actuator_node` — Manages weapon state (interception net/gun) and reloads.
* `gimbal_driver_node` — Controls vertical target alignment (pitch).
* `yaw_servo_driver_node` — Controls horizontal target alignment (yaw).

### Pure C++ Core
All math functions, coordinate validation, tracking status selection, and reload constraints are implemented inside `include/antidrone_turret/turret_decision.hpp` using **lowerCamelCase** naming conventions for internal logic.

---

## 🚀 Quick Start (Docker Environment)

No local ROS 2 installation is required. The system runs inside a pre-configured Docker container.

### 1. Launch the Pipeline
Spin up the default tracking workspace (runs all available tracks sequentially):
```bash
docker compose up --build
```

### 2. Connect via Secondary Terminal
To inspect topics, open a new terminal window and log into the running container:
```bash
docker compose exec antidrone_turret bash
```
Always source the workspace inside the container before running ROS tools:
```bash
source /opt/ros/jazzy/setup.bash && source /root/robot_ws/install/setup.bash
```

---

## 🎯 Tactical Scenarios (Track Testing)

You can launch specific combat flight trajectories to verify the turret's decision limits:

* **Successful Interception:** Target enters the 30-meter zone with high confidence:
  ```bash
  docker compose run --rm antidrone_turret ros2 launch antidrone_turret system.launch.py track:=approach_trigger.csv
  ```
* **Safe Tracking (Out of Range):** Target visible beyond 30 meters. Turret tracks but skips firing:
  ```bash
  docker compose run --rm antidrone_turret ros2 launch antidrone_turret system.launch.py track:=far_flyby_no_trigger.csv
  ```
* **Low Confidence Overrides:** Target is close but tracking confidence drops below 80%. Turret goes idle:
  ```bash
  docker compose run --rm antidrone_turret ros2 launch antidrone_turret system.launch.py track:=low_confidence_no_trigger.csv
  ```
* **Reload Pressure Protection:** Multiple drones attack simultaneously. Re-trigger requests are blocked until reload is complete:
  ```bash
  docker compose run --rm antidrone_turret ros2 launch antidrone_turret system.launch.py track:=reload_pressure.csv
  ```

---

## 📊 Live Telemetry Inspection

Run these inside your secondary sourced terminal to monitor raw message outputs:

```bash
# Monitor central decisions, range, and trigger outputs
ros2 topic echo /turret/status

# Monitor pitch adjustments and pixel tracking errors
ros2 topic echo /gimbal/cmd

# Monitor yaw adjustments
ros2 topic echo /servo/cmd

# Monitor weapon system availability and total shots fired
ros2 topic echo /actuator/status
```

---

## 🧪 Automated Testing Suite

The codebase includes full unit-test coverage for core decision limits and middleware message generation via `gtest`:

```bash
cd /root/robot_ws
colcon test --packages-select antidrone_turret
colcon test-result --verbose
```
*Expected output status:* `28 tests, 0 errors, 0 failures, 0 skipped`.
