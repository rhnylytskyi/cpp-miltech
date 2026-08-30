# Homework 14: Underground Trench World Mapper

Autonomous robotic system implemented in ROS 2 Jazzy for underground trench mapping and contact neutralization scenario using modern C++20.

## Overview

The solution consists of a modular ROS 2 architecture split into two separate functional packages to ensure clean system design:
1. `underground_world`: The simulation core providing environment generation and metrics validation.
2. `trench_crawler_solution`: The custom autonomous payload and navigation logic developed for the task.

## System Architecture

### 1. Navigator Node (`trench_navigator_node`)
* **Algorithm**: Implements Breadth-First Search (BFS) to continuously identify unknown cell frontiers.
* **Memory Management**: Builds an internal 2D grid representation based on sequential dynamic inputs from local laser scans.
* **Synchronization**: Utilizes boolean state flags (`m_isNewScanAvailable`) to completely avoid data race conditions inside the 20ms execution control timer.

### 2. Weapon Controller Node (`weapon_action_node`)
* **Service Contract**: Interacts with the simulated hardware via the `/payload/trigger` service.
* **Payload Automation**: Listens to real-time scanning matrices, dynamically focuses on targets, changes internal task states, and confirms eliminations over the `/payload/enemy_down` topic.

## Deployment & Execution Guide

### 1. Workspace Building
```bash
cd ~/cpp-miltech/homework_14/robot_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Execution & Recording Sequence
To guarantee full logging, always start the recorder in **Terminal 1** before launching the simulator in **Terminal 2**. Close the recorder with `Ctrl + C` once the mission reports `SUCCESS`.

#### Scenario 1: Training Corridor
* **Terminal 1**: `mkdir -p ../bags && ros2 bag record -a -o ../bags/training_corridor`
* **Terminal 2**: `ros2 launch underground_world system.launch.py scenario:=training_corridor.yaml`

#### Scenario 2: Small Rooms
* **Terminal 1**: `ros2 bag record -a -o ../bags/small_rooms`
* **Terminal 2**: `ros2 launch underground_world system.launch.py scenario:=small_rooms.yaml`

#### Scenario 3: Branching Trench
* **Terminal 1**: `ros2 bag record -a -o ../bags/branching_trench`
* **Terminal 2**: `ros2 launch underground_world system.launch.py scenario:=branching_trench.yaml`

#### Scenario 4: Dead End Bunker
* **Terminal 1**: `ros2 bag record -a -o ../bags/dead_end_bunker`
* **Terminal 2**: `ros2 launch underground_world system.launch.py scenario:=dead_end_bunker.yaml`
