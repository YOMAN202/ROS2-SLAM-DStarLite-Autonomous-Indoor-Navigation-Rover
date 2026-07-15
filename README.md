# ROS2-SLAM-DStarLite-Autonomous-Indoor-Navigation-Rover

An autonomous indoor navigation rover built from scratch on **ROS2 Humble** and **Raspberry Pi 4B**, featuring LiDAR-based SLAM, D* Lite global path planning, a dynamic multi-layer costmap, a reactive LiDAR safety layer, and differential drive control via ESP32 — **without Nav2**.

> Developed as part of an Embedded AI & Robotics internship at **L&T Precision Engineering & Systems (PES)**, Mumbai.

---

## Demo

The rover autonomously maps an unknown indoor environment, plans an optimal path to a user-defined goal, and navigates while avoiding static and dynamic obstacles in real time.

Live visualization via **Foxglove Studio** (WebSocket over WiFi):

- `/map` — SLAM occupancy grid
- `/costmap` — multi-layer cost map
- `/planned_path` — D* Lite global path
- `/scan` — live LiDAR scan

---

## Hardware

| Component | Specification |
|---|---|
| Compute | Raspberry Pi 4B (4GB RAM) |
| LiDAR | RPLIDAR C1 (360°, 12m range) |
| Microcontroller | ESP32 WROOM-32 |
| Motor Drivers | 2× L298N H-Bridge |
| Motors | 4× DC Gear Motors |
| Battery | 2× 11.1V LiPo |
| Power (Pi) | USB Powerbank |
| Chassis | Wooden platform |

---

## Software Architecture

```
RPLIDAR C1
    │ /scan
    ▼
RF2O Laser Odometry ──────────────────────────────┐
    │ /odom                                        │
    ▼                                              │
slam_toolbox (Online Async SLAM)                  │
    │ /map (OccupancyGrid)                         │
    ▼                                              │
Custom Costmap Node                               │
    │ /costmap                                     │
    │  Layer 1: Obstacle Inflation (Gaussian)      │
    │  Layer 2: Dynamic Obstacles (decaying)       │
    │  Layer 3: Path History (accumulating)        │
    ▼                                              │
D* Lite Global Planner (C++)  ◄───────────────────┘
    │ /cmd_vel_raw + /planned_path
    ▼
LiDAR Safety Node (3-state)
    │ SAFE → normal speed
    │ WARNING → reduced speed
    │ DANGER → stop + replan signal
    │ /cmd_vel
    ▼
cmd_vel_to_serial (Python)
    │ USB Serial → "m <left> <right>"
    ▼
ESP32 WROOM
    │ PWM via ledcWrite
    ▼
2× L298N → 4× DC Motors (Differential Drive)
```

---

## Key Components

### 1. SLAM — `slam_toolbox` (Online Async Mode)
Builds a 2D occupancy grid map in real time using LiDAR scans. RF2O Laser Odometry provides the `odom → base_link` transform required for scan matching.

### 2. Multi-Layer Costmap (`costmap_node.py`)
Three additive cost layers merged into a single `OccupancyGrid`:

| Layer | Source | Behavior |
|---|---|---|
| Obstacle Inflation | `/map` | Gaussian falloff from walls (radius 0.08m) |
| Dynamic Obstacles | `/scan` | +60 cost per LiDAR hit, 50% decay/second |
| Path History | `/odom` | +5 per visited cell, capped at 40, never decays |

Cell traversal cost: `1.0 + costmap_value / 10.0` (lethal = INF)

### 3. D* Lite Global Planner (`dstar_lite_planner.cpp`)
Custom C++ implementation of the D* Lite algorithm (Koenig & Likhachev, 2002):
- **Heuristic**: Octile distance (admissible for 8-connected grid)
- **Lazy deletion** via `entry_finder` map to prevent priority queue oscillation
- **Termination**: `g(start) == rhs(start)` (not `≤`, which causes premature exit)
- **Incremental replanning** on costmap changes without full reinitialisation
- **Recovery behaviour**: backs up 2 seconds when no path found
- **Lookahead safety**: stops if upcoming waypoints have cost ≥ 100

### 4. LiDAR Safety Node (`safety_node.py`)
High-frequency (20Hz) reactive safety layer operating independently of the planner:

| State | Condition | Action |
|---|---|---|
| SAFE | front > 0.70m | Normal speed |
| WARNING | 0.35m < front < 0.70m | Reduced speed |
| DANGER | front < 0.35m | Stop + publish `/obstacle_blocked` |

Also monitors rear distance to prevent crashes during recovery backup.

### 5. ESP32 Differential Drive Firmware
Receives `m <left> <right>` commands over USB serial (115200 baud). Uses `ledcAttach` / `ledcWrite` for proper PWM on ESP32 Arduino core 3.x. Speed range: -100 to +100 (mapped to 0–255 PWM duty cycle).

### 6. cmd_vel_to_serial (`cmd_vel_to_serial.py`)
Converts ROS2 `/cmd_vel` (`Twist`) to differential drive motor commands:
```
left_speed  = (linear.x - angular.z * wheel_base / 2) / max_linear * 100
right_speed = (linear.x + angular.z * wheel_base / 2) / max_linear * 100
```
Parameters: `wheel_base=0.31m`, `min_motor_speed=60`, `max_motor_speed=80`

---

## Installation

### Prerequisites
- Ubuntu 22.04 LTS on Raspberry Pi 4B
- ROS2 Humble

### Install ROS2 Humble
```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-ros-base ros-humble-slam-toolbox \
  ros-humble-foxglove-bridge ros-humble-rplidar-ros \
  ros-humble-tf2-ros ros-humble-tf2-tools \
  python3-colcon-common-extensions -y
```

### Build workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/MAPIRlab/rf2o_laser_odometry.git
git clone https://github.com/YOMAN202/ROS2-SLAM-DStarLite-Autonomous-Indoor-Navigation-Rover.git
cp -r ROS2-SLAM-DStarLite-Autonomous-Indoor-Navigation-Rover/dstar_lite_planner .
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
```

### Python dependencies
```bash
pip3 install scipy pyserial
```

---

## Running the Stack

Open 9 SSH terminals and run in order:

```bash
# Terminal 1 — Static TF
source /opt/ros/humble/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame

# Terminal 2 — LiDAR
source /opt/ros/humble/setup.bash
ros2 run rplidar_ros rplidar_node --ros-args \
  -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=460800 -p frame_id:=laser_frame

# Terminal 3 — RF2O Odometry
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py

# Terminal 4 — SLAM
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/home/aicoe/mapper_params_online_async.yaml

# Terminal 5 — Costmap
source /opt/ros/humble/setup.bash
python3 ~/costmap_node.py

# Terminal 6 — D* Lite Planner
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/dstar_lite_planner/share/dstar_lite_planner/local_setup.bash
ros2 run dstar_lite_planner dstar_lite_planner

# Terminal 7 — Safety Node
source /opt/ros/humble/setup.bash
python3 ~/safety_node.py --ros-args -p stop_distance:=0.35 -p warn_distance:=0.60

# Terminal 8 — Foxglove Bridge
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# Terminal 9 — Motor Control (start last)
source /opt/ros/humble/setup.bash
python3 ~/cmd_vel_to_serial.py --ros-args -p serial_port:=/dev/ttyUSB1
```

### Set a navigation goal
```bash
source /opt/ros/humble/setup.bash
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}" -r 1
```

### Visualization
Connect **Foxglove Studio** to `ws://<raspberry_pi_ip>:8765`

---

## Project Structure

```
├── costmap_node.py          # Multi-layer costmap (Python)
├── safety_node.py           # LiDAR safety layer (Python)
├── cmd_vel_to_serial.py     # ROS2 to ESP32 serial bridge (Python)
├── mapper_params_online_async.yaml  # slam_toolbox configuration
└── dstar_lite_planner/      # D* Lite global planner (C++)
    ├── CMakeLists.txt
    ├── package.xml
    └── src/
        └── dstar_lite_planner.cpp
```

ESP32 firmware (Arduino): available in `/esp32_firmware/` (see branch `esp32`)

---

## Design Decisions

**Why D* Lite instead of A*?**
D* Lite supports incremental replanning — when the costmap changes due to a new obstacle, only the affected portion of the graph is updated rather than replanning from scratch. This is critical for dynamic indoor environments.

**Why not Nav2?**
Nav2 is a production-grade framework with significant configuration overhead. Building the navigation stack from scratch provides complete architectural understanding and is more appropriate for a research/internship context.

**Why RF2O instead of wheel encoders?**
The rover does not have wheel encoders. RF2O (Range Flow-based Odometry) estimates motion directly from consecutive LiDAR scans without any additional hardware. While less accurate than encoder odometry (especially during rotation), it is sufficient for slow indoor navigation.

**Why a separate safety node?**
The safety node runs at 20Hz independently of the 5Hz planner. Even if the planner crashes or produces a bad path, the safety node will stop the rover before a collision. This separation of concerns is standard in industrial mobile robotics.

---

## Known Limitations

- RF2O heading drift during rotation causes map distortion over time
- Motors require minimum ~60 PWM to overcome static friction (threshold varies with load)
- In-place rotation not reliable due to motor torque limitations
- 2D LiDAR cannot detect obstacles below its scan plane height

## Future Work

- Wheel encoders for accurate odometry
- IMU (MPU-6050) fused with RF2O via `robot_localization` EKF
- Pure Pursuit local controller replacing direct waypoint tracking
- Map persistence (save/load SLAM map)

---

## References

- Koenig, S. & Likhachev, M. (2002). *D* Lite*. AAAI.
- Jaimez, M. et al. (2016). *Fast Visual Odometry for 3-D Range Sensors*. IEEE TRO. (RF2O)
- Macenski, S. et al. (2021). *The Marathon 2: A Navigation System*. IROS. (slam_toolbox)

---

## Author

**Akshat Mishra**
B.Tech Electrical & Electronics Engineering, Manipal Institute of Technology
Internship: Embedded AI & Robotics, L&T Precision Engineering & Systems, Mumbai (2026)
