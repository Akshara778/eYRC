# eYRC — Krishi coBot (KC) Theme 2025‑26

Solution stack for the **Krishi coBot (KC)** theme of the [e‑Yantra Robotics Competition (eYRC) 2025‑26](https://www.e-yantra.org/), organized by IIT Bombay. The bot autonomously navigates a simulated farm/warehouse environment, detects markers and fruit crates using computer vision, and performs pick‑and‑place manipulation with a UR5 arm — all built on **ROS 2**.

> This repository contains the team's task-wise submissions (Task 1 → Task 3) as required by the competition's boilerplate structure.

---

## 🧭 Overview

| Capability | Approach |
|---|---|
| **Localization & Navigation** | LiDAR (`/scan`) + odometry (`/odom`) fused with a PID-based controller publishing `/cmd_vel` |
| **Perception** | OpenCV shape/marker detection and ArUco tag recognition over `/camera` topics |
| **Manipulation** | UR5 arm servoed via `/delta_twist_cmds` and `/delta_joint_cmds`, with PI(D) closed-loop pose correction using `tf2` transforms |
| **Pick & Place** | Gazebo `linkattacher_msgs` (`AttachLink` / `DetachLink`) services to simulate gripping |

The stack is split into three progressive tasks, mirroring the competition's evaluation stages:

```
eYRC/
├── Task1/
│   ├── task1a.py     # LiDAR-based waypoint navigation (mobile base)
│   ├── task1b.py     # Depth-camera perception & /tf publishing
│   └── task1c.py      # UR5 arm PID pose control via TF targets
├── Task2/
│   ├── ebot_nav_task2a.py       # Mobile base navigation (multi-waypoint docking)
│   ├── shape_detector_task2a.py # LiDAR-based shape/marker detection
│   ├── cv_task2b.py             # OpenCV-based visual detection & /tf broadcasting
│   └── tf_task2b.py             # UR5 pick-and-place with Gazebo link attach/detach
└── Task3/
    ├── ebot_nav_task3b.py   # Extended multi-goal navigation
    ├── task3a_aruco.py      # ArUco marker detection & pose estimation
    └── task3a_fruits.py     # Fruit/crate detection pipeline
```

---

## ⚙️ Tech Stack

- **ROS 2** (`rclpy`) — node graph, pub/sub, services, timers
- **OpenCV** + **cv_bridge** — image processing, ArUco detection
- **tf2_ros** — coordinate frame transforms for arm/base pose control
- **Gazebo** (`linkattacher_msgs`) — simulated grasping via link attach/detach services
- **NumPy** — numerical/geometry utilities
- **Python 3**

---

## 🧩 Task Breakdown

### Task 1 — Foundations
- **`task1a.py`** — A `LidarProcessor` node that drives the mobile base toward a sequence of target `(x, y, θ)` waypoints using odometry feedback, with obstacle awareness from `/scan` and marker-based stop conditions on `/marker`.
- **`task1b.py`** — Boilerplate perception node (per e‑Yantra's official task template) that subscribes to aligned depth/color camera streams and publishes detected object frames to `/tf`.
- **`task1c.py`** — A `MoveArm` node that PID-corrects the UR5 end-effector toward pre-defined target poses (position + quaternion) using live `/tf` lookups, publishing Cartesian velocity commands on `/delta_twist_cmds`.

### Task 2 — Navigation + Perception + Manipulation
- **`ebot_nav_task2a.py`** — A `Navigation` node that chains multiple waypoints with proportional heading/position control, publishing a `/flag` once each docking point is reached.
- **`shape_detector_task2a.py`** — Detects geometric markers (e.g., pentagon target) via LiDAR range deltas and velocity tracking, publishing detection status and marker state.
- **`cv_task2b.py`** — Camera-based detection node (OpenCV + `tf2_ros`) that identifies objects and broadcasts their transforms for the arm to consume.
- **`tf_task2b.py`** — A `PickAndDropNode` that drives the UR5 arm through a sequence of TF-defined pick/place poses, calling Gazebo's `/attach_link` and `/detach_link` services to grip and release objects, with PI control on both Cartesian velocity and joint commands.

### Task 3 — Integrated Pipeline
- **`ebot_nav_task3b.py`** — Extended version of the Task 2 navigation stack for a longer multi-goal route.
- **`task3a_aruco.py`** — ArUco marker detection and pose estimation from the camera feed, broadcasting marker frames to `/tf`.
- **`task3a_fruits.py`** — Vision pipeline to detect and localize fruit crates for pick-and-place targeting.

---

## 📡 Key ROS 2 Topics & Services

| Name | Type | Used For |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Base pose feedback |
| `/scan` | `sensor_msgs/LaserScan` | Obstacle detection, shape/marker sensing |
| `/cmd_vel` | `geometry_msgs/Twist` | Mobile base velocity commands |
| `/marker`, `/flag` | `std_msgs/Int8` | Task-state signaling between nodes |
| `/tf` | `tf2_msgs/TFMessage` | Object & end-effector frame broadcasting |
| `/delta_twist_cmds` | `geometry_msgs/Twist` | UR5 Cartesian velocity control |
| `/delta_joint_cmds` | `std_msgs/Float64MultiArray` | UR5 joint-space control |
| `/attach_link`, `/detach_link` | `linkattacher_msgs/srv` | Simulated gripping in Gazebo |

---

## 🚀 Getting Started

### Prerequisites
- ROS 2 (Humble or later recommended)
- Gazebo (with `linkattacher_msgs` plugin available)
- Python 3 packages: `opencv-python`, `cv_bridge`, `numpy`
- A Krishi coBot Gazebo simulation world (provided by e‑Yantra) with the UR5 arm and mobile base spawned

### Running a node
```bash
# Source your ROS 2 workspace first
source /opt/ros/<distro>/setup.bash

# Example: run the Task 2 navigation node
python3 Task2/ebot_nav_task2a.py

# Example: run the Task 2 pick-and-place arm controller
python3 Task2/tf_task2b.py
```
> Each script is a standalone `rclpy` node — launch files are not included here, so nodes should be run alongside the official eYRC simulation stack (Gazebo world + robot description + camera/LiDAR bringup).

---
