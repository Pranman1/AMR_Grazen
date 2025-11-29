# Grazen System - TurtleBot3 Navigation & Mapping

A comprehensive ROS2 Humble navigation and mapping system for TurtleBot3 Burger, supporting both simulation and real-world deployment.

---

## Table of Contents
1. [Installation](#installation)
2. [Quick Start Commands](#quick-start-commands)
3. [System Architecture](#system-architecture)
4. [Parameter Reference](#parameter-reference)
5. [Troubleshooting](#troubleshooting)

---

## Installation

### Prerequisites

```bash
# Ubuntu 22.04 with ROS2 Humble
sudo apt update && sudo apt upgrade -y

# Install ROS2 Humble (if not installed)
# Follow: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# Set TurtleBot3 Model (add to ~/.bashrc)
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
source ~/.bashrc
```

### Required ROS2 Packages

```bash
# Navigation & SLAM
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-slam-toolbox \
    ros-humble-cartographer-ros \
    ros-humble-tf2-ros \
    ros-humble-tf2-tools

# TurtleBot3 Packages
sudo apt install -y \
    ros-humble-turtlebot3* \
    ros-humble-dynamixel-sdk

# Gazebo Simulation
sudo apt install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control

# Visualization
sudo apt install -y \
    ros-humble-rviz2 \
    ros-humble-rviz-default-plugins

# Exploration (for auto-mapping)
sudo apt install -y ros-humble-explore-lite
```

### Build the Workspace

```bash
cd ~/turtle_test
colcon build --symlink-install
source install/setup.bash
```

---

## Quick Start Commands

### Simulation Mode

```bash
# Always source first
source ~/turtle_test/install/setup.bash

# Manual Mapping (WASD control)
ros2 launch grazen_system system.launch.py mode:=sim task:=map auto_map:=false sim_world:=arena

# Auto Mapping (Explore Lite)
ros2 launch grazen_system system.launch.py mode:=sim task:=map auto_map:=true sim_world:=arena

# Navigation (on existing map)
ros2 launch grazen_system system.launch.py mode:=sim task:=nav sim_world:=arena
```

### Real Robot Mode

```bash
# Terminal 1: Robot Bringup (run on robot or via SSH)
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2: Manual Mapping
source ~/turtle_test/install/setup.bash
ros2 launch grazen_system system.launch.py mode:=real task:=map auto_map:=false map_name:=my_map

# Terminal 2: Auto Mapping
ros2 launch grazen_system system.launch.py mode:=real task:=map auto_map:=true map_name:=my_map

# Terminal 2: Navigation
ros2 launch grazen_system system.launch.py mode:=real task:=nav map_name:=my_map
```

### Launch Arguments Reference

| Argument | Values | Default | Description |
|----------|--------|---------|-------------|
| `mode` | `sim`, `real` | `sim` | Simulation or real robot |
| `task` | `map`, `nav` | `nav` | Mapping or navigation mode |
| `auto_map` | `true`, `false` | `true` | Auto (explore_lite) or manual (WASD) mapping |
| `sim_world` | `warehouse`, `arena` | `warehouse` | Gazebo world to load |
| `map_name` | string | auto-generated | Name for saving/loading maps |

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    GRAZEN SYSTEM LAUNCHER                        │
├─────────────────────────────────────────────────────────────────┤
│  mode:=sim/real    task:=map/nav    auto_map:=true/false        │
└─────────────────────────────────────────────────────────────────┘
                              │
         ┌────────────────────┴────────────────────┐
         │                                         │
    MAPPING MODE                              NAVIGATION MODE
    (task=map)                                (task=nav)
         │                                         │
         ├─ Gazebo/Robot Bringup                   ├─ Gazebo/Robot Bringup
         ├─ SLAM Toolbox                           ├─ Map Server (loads .yaml)
         ├─ Nav2 (no AMCL)                         ├─ Nav2 + AMCL (localization)
         ├─ RViz                                   ├─ Mission Manager
         │                                         │
         ├─ [auto_map=true]                        └─ Executes waypoints.txt
         │   ├─ Explore Lite (autonomous)
         │   └─ Auto Mapper (saves when done)
         │
         └─ [auto_map=false]
             └─ Manual Mapper (WASD + 'p' to save)
```

### Custom Nodes

| Node | File | Purpose |
|------|------|---------|
| `manual_mapper` | `manual_mapper.py` | WASD keyboard control + map saving |
| `auto_mapper` | `auto_mapper.py` | Monitors explore_lite, auto-saves map when done |
| `mission_manager` | `mission_manager.py` | Executes sequential waypoints from file |
| `scan_resampler` | `scan_resampler.py` | Fixes variable LDS-02 scan counts (real robot only) |

---

## Parameter Reference

All parameters are in `config/grazen_params.yaml` unless noted.

### Navigation Tuning

#### Goal Tolerance (How close is "close enough")

```yaml
# controller_server → goal_checker
xy_goal_tolerance: 0.10      # meters - position tolerance
yaw_goal_tolerance: 0.25     # radians - rotation tolerance (~14°)

# controller_server → FollowPath (DWB)
xy_goal_tolerance: 0.10      # Should match goal_checker
```

**Values:**
- `0.05` = Very precise (5cm) - may oscillate trying to reach exact position
- `0.10` = Recommended (10cm) - balanced precision
- `0.25` = Default/Loose (25cm) - faster but less accurate

#### Path Straightness (Curvy vs Direct paths)

```yaml
# controller_server → FollowPath
sim_time: 1.0                # seconds - lookahead time (lower = more reactive)
GoalAlign.scale: 12.0        # Lower = less early curving toward goal
PathDist.scale: 48.0         # Higher = follow global path more strictly
```

**Effect of `sim_time`:**
```
sim_time: 2.0  →  Smoother but curvier paths (looks ahead further)
sim_time: 1.0  →  More direct paths (recommended)
sim_time: 0.5  →  Very reactive, may be jerky
```

**Effect of `GoalAlign.scale`:**
```
24.0 (high)  →  Robot curves toward goal early
12.0 (low)   →  Robot follows path, turns at end (recommended)
```

#### Obstacle Avoidance (Safety vs Tight Spaces)

```yaml
# local_costmap → inflation_layer
cost_scaling_factor: 2.5     # How strongly to avoid obstacles
inflation_radius: 0.25       # meters - buffer zone around obstacles

# global_costmap → inflation_layer  
cost_scaling_factor: 2.0     # Lower for global (allows tighter planning)
inflation_radius: 0.20       # Smaller to fit through doorways
```

**Inflation Radius Visual:**
```
                    inflation_radius
                    ←───────────→
    ┌─────────────────────────────────────┐
    │         "Danger Zone"               │
    │    ┌───────────────────────┐        │
    │    │                       │        │
    │    │      OBSTACLE         │        │  Robot avoids
    │    │                       │        │  this entire area
    │    └───────────────────────┘        │
    │                                     │
    └─────────────────────────────────────┘
```

**Values:**
| `inflation_radius` | Effect |
|-------------------|--------|
| `0.15` | Tight - fits narrow spaces, risk of collision |
| `0.20` | Balanced for global planning |
| `0.25` | Safe for local avoidance |
| `0.30` | Very safe but won't enter tight spaces |

**Cost Scaling Factor:**
```
1.0-2.0  →  "Being near obstacles is okay"
2.5      →  "Prefer paths away from obstacles" (recommended)
3.0+     →  "Stay FAR from obstacles"
```

#### Robot Speed

```yaml
# controller_server → FollowPath
max_vel_x: 0.20              # m/s - forward speed (0.35 max for Burger)
max_vel_theta: 0.8           # rad/s - rotation speed
max_speed_xy: 0.20           # m/s - overall speed limit
```

**Speed vs Safety:**
| `max_vel_x` | Use Case |
|-------------|----------|
| `0.35` | Fast, open areas only |
| `0.26` | Default TurtleBot3 |
| `0.20` | Safe exploration (recommended) |
| `0.10` | Very cautious/testing |

### SLAM Tuning (`config/slam_params.yaml`)

```yaml
# Core settings
base_frame: base_footprint   # TF frame for robot base
odom_frame: odom
map_frame: map

# Update frequency
map_update_interval: 1.0     # seconds between map publishes
minimum_travel_distance: 0.2 # meters before new scan processed
minimum_travel_heading: 0.2  # radians before new scan processed

# Laser settings
max_laser_range: 12.0        # meters - ignore readings beyond this
minimum_range: 0.12          # meters - LDS-02 minimum range
```

### Auto Mapper Settings (`system.launch.py`)

```yaml
completion_timeout: 10.0     # seconds with no frontiers = done
save_interval: 0.0           # 0 = disabled, >0 = periodic backup saves
```

### Waypoints Format (`config/waypoints.txt`)

```
# Forward(X)  Left(Y)
# All values in meters, relative to PREVIOUS position (sequential)
0.0    0.0    # Stay at start
2.0    0.0    # Move 2m forward
0.0    1.0    # Move 1m left
-2.0   0.0    # Move 2m backward
0.0   -1.0    # Move 1m right (back to start)
```

**ROS Coordinate System:**
```
        +X (forward)
           ↑
           │
+Y (left) ←●→ -Y (right)
           │
           ↓
        -X (backward)
```

---

## Troubleshooting

### 1. Simulation Hanging or "Entity Already Exists"

**Symptom:** Gazebo fails to launch, hangs, or crashes with `Entity [burger] already exists`.

**Cause:** Zombie processes from previous run.

**Fix:**
```bash
# Kill everything
killall -9 gzserver gzclient
pkill -9 -f ros
ros2 daemon stop && ros2 daemon start

# If still broken, restart PC
```

### 2. Map Not Building (Real Robot)

**Symptom:** SLAM shows `LaserRangeScan contains X readings, expected Y` errors. Map doesn't fill in.

**Cause:** Real LDS-02 produces variable scan counts (250-256), SLAM expects constant.

**Fix:** The `scan_resampler` node handles this automatically in `mode:=real`. Verify it's running:
```bash
ros2 node list | grep scan_resampler
```

### 3. QoS Mismatch - No Messages Received

**Symptom:** Node logs show `New publisher discovered... offering incompatible QoS. No messages will be received.`

**Cause:** Publisher uses BEST_EFFORT, subscriber uses RELIABLE (incompatible).

**Fix:** Use sensor-compatible QoS in Python nodes:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

sensor_qos = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT
)
subscription = self.create_subscription(LaserScan, 'scan', callback, sensor_qos)
```

### 4. "Invalid frame ID odom" - Transform Missing

**Symptom:** `Timed out waiting for transform from base_link to odom`

**Cause:** Robot bringup not running (real mode) or Gazebo not started (sim mode).

**Fix:**
- Real: Start `ros2 launch turtlebot3_bringup robot.launch.py` first
- Sim: Ensure Gazebo launched successfully

### 5. Robot Goes Wrong Direction (Real Robot)

**Symptom:** Pressing 'W' makes robot go backward, or vice versa.

**Cause:** Dynamixel motor IDs swapped (ID 1 and ID 2 on wrong wheels).

**Fix:** Physically swap the Dynamixel cable connections, or reconfigure motor IDs in Dynamixel Wizard.

### 6. Corrupt Workspace (Old Code Running)

**Symptom:** Changed code but old version still runs.

**Fix:**
```bash
cd ~/turtle_test
rm -rf build/ install/ log/
colcon build --symlink-install
source install/setup.bash
```

### 7. TF / Map "Time Travel" Errors

**Symptom:** `Transform data too old` or `Lookup would require extrapolation`

**Cause:** Mixing system time with simulation time.

**Fix:** Ensure `use_sim_time` is consistent across all nodes. In `mode:=sim`, all nodes should have `use_sim_time: true`.

### 8. Network / DDS Errors

**Symptom:** `Network unreachable` or `selected interface is not multicast-capable`

**Fix:**
```bash
# For simulation (localhost)
unset CYCLONEDDS_URI
export ROS_LOCALHOST_ONLY=1

# Reset DDS
ros2 daemon stop && ros2 daemon start
```

### 9. Auto-Mapping Doesn't Save

**Symptom:** Exploration completes but map doesn't save.

**Cause:** QoS mismatch on `/explore/frontiers` topic.

**Fix:** Already fixed in `auto_mapper.py` with BEST_EFFORT QoS. Ensure you rebuilt:
```bash
colcon build --packages-select grazen_system --symlink-install
```

### 10. Kill All ROS Processes

```bash
pkill -9 -f ros2
pkill -9 -f rviz
pkill -9 -f gazebo
pkill -9 -f nav2
pkill -9 -f slam
```

---

## File Structure

```
grazen_system/
├── config/
│   ├── grazen_params.yaml    # Nav2 parameters
│   ├── slam_params.yaml      # SLAM Toolbox parameters
│   ├── explore_params.yaml   # Explore Lite parameters
│   └── waypoints.txt         # Mission waypoints
├── grazen_system/
│   ├── manual_mapper.py      # WASD control + map save
│   ├── auto_mapper.py        # Auto-save when exploration done
│   ├── mission_manager.py    # Sequential waypoint execution
│   └── scan_resampler.py     # Fix variable LDS-02 scan counts
├── launch/
│   └── system.launch.py      # Main launcher
├── maps/                     # Saved maps (.yaml + .pgm)
└── setup.py
```

---

## License

MIT License - See LICENSE file for details.
