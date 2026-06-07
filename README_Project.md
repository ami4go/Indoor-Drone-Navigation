# 🚁 Autonomous Indoor Drone Navigation

> **Real-time 3D perception and mapping for GPS-denied indoor flight using ROS 2, PX4, and Gazebo.**

An autonomous drone simulation pipeline that gives a simulated quadcopter the ability to **see** obstacles with a depth camera, **remember** them using a persistent 3D map, and ultimately **navigate** around them using path planning — all without GPS.

---

## 📋 Project Progress

| Status | Task | Description |
|:---:|:---|:---|
| ✅ | Environment Setup | Ubuntu 22.04, ROS 2 Humble, PX4 SITL, Gazebo Garden |
| ✅ | Indoor World Design | 10×8×3m room with walls, 3 obstacles, start/destination markers |
| ✅ | Obstacle Navigation Proof | Blind waypoint nav crashes → proved sensors are needed |
| ✅ | Keyboard Teleoperation | Manual WASD control → proved PX4 control pipeline works |
| ✅ | Depth Camera Integration | Switched to `x500_depth` model with OakD-Lite stereo camera |
| ✅ | Gazebo-ROS 2 Bridge | Bridged depth image, point cloud, camera info, and clock topics |
| ✅ | Point Cloud Filtering | Voxel Grid downsampling + Statistical Outlier Removal (307K → ~170 pts) |
| ✅ | RViz2 Visualization | Real-time 3D point cloud display with saved config |
| ✅ | OctoMap 3D Mapping | Persistent occupancy grid — drone remembers obstacles after turning away |
| ✅ | TF Broadcaster | Publishes drone position as TF transforms for map alignment |
| ✅ | OctoMap Visualization | 3D voxel cubes in RViz2 showing occupied/free space |
| 🔲 | A* Path Planning | Compute obstacle-free routes from start to destination |
| 🔲 | Autonomous Navigation | Connect planner to PX4 for fully autonomous flight |

---

## 🏗️ System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        GAZEBO GARDEN v7.9.0                        │
│  ┌──────────────┐  ┌──────────────┐  ┌───────────────────────────┐ │
│  │ Indoor Room   │  │ x500_depth   │  │ OakD-Lite Depth Camera   │ │
│  │ 10×8×3m       │  │ Drone Model  │  │ 640×480 @ 30fps          │ │
│  └──────────────┘  └──────────────┘  └───────────────────────────┘ │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ Gazebo Transport
                               ▼
                    ┌──────────────────────┐
                    │   ros_gz_bridge      │
                    │   (Topic Bridging)   │
                    └──────────┬───────────┘
                               │ ROS 2 Topics
              ┌────────────────┼────────────────┐
              ▼                ▼                ▼
   ┌──────────────┐  ┌────────────────┐  ┌──────────────┐
   │ PX4 Autopilot│  │ Point Cloud    │  │ TF           │
   │ (SITL)       │  │ Filter Node    │  │ Broadcaster  │
   │              │  │                │  │              │
   │ Odometry ────┼──┤ 307K→170 pts  │  │ map→camera   │
   └──────────────┘  └───────┬────────┘  └──────┬───────┘
                             │                   │
                             ▼                   ▼
                    ┌────────────────────────────────────┐
                    │        OctoMap Server               │
                    │   Persistent 3D Occupancy Map       │
                    │   Resolution: 10cm | Range: 8m      │
                    └────────────────┬───────────────────┘
                                     │
                                     ▼
                           ┌──────────────────┐
                           │    RViz2          │
                           │  3D Visualization │
                           └──────────────────┘
```

---

## 🖥️ Tech Stack & Versions

| Component | Version | Purpose |
|:---|:---|:---|
| **Ubuntu** | 22.04 LTS (Jammy) | Operating System |
| **ROS 2** | Humble Hawksbill | Robotics middleware |
| **PX4 Autopilot** | v1.14 (SITL) | Flight controller firmware |
| **Gazebo** | Garden v7.9.0 | 3D physics simulator |
| **Micro-XRCE-DDS Agent** | v2.4.x | PX4 ↔ ROS 2 communication bridge |
| **ros-gz-bridge** | v0.244.11 | Gazebo ↔ ROS 2 topic bridging |
| **Drone Model** | x500_depth (OakD-Lite) | Quadcopter with integrated stereo depth camera |
| **OctoMap** | v1.9.8 | 3D occupancy mapping library |
| **OctoMap Server** | v2.3.1 | ROS 2 node for building maps from point clouds |
| **OctoMap RViz Plugins** | v2.1.1 | 3D voxel visualization in RViz2 |
| **PCL-ROS** | v2.4.5 | Point Cloud Library ROS 2 integration |
| **Python** | 3.10 | Scripting language for ROS 2 nodes |
| **NumPy** | 2.2.6 | Numerical processing for point cloud filtering |

---

## 🏠 Simulation Environment

The drone operates inside a **10m × 8m × 3m** indoor room with three physical obstacles placed to force non-trivial navigation:

| Obstacle | Type | Position | Purpose |
|:---|:---|:---|:---|
| 🔴 Red Pillar | Cylinder (r=0.3m, h=2.5m) | (0, 1) | Blocks the direct path |
| 🟠 Orange Wall | Box (0.3×2×2m) | (-1.5, -1) | Blocks lower corridor |
| 🟡 Yellow Pillar | Cylinder (r=0.25m, h=2m) | (2, 0) | Forces weaving maneuver |
| 🟢 Start Marker | Green disc on floor | (-3, -2) | Takeoff zone |
| 🔵 Destination Marker | Blue disc on floor | (3, 2) | Landing target |

![Gazebo Simulation Environment — 10×8×3m indoor room with obstacles and drone](Demo_Pic/Screenshot%20from%202026-06-08%2002-14-03.png)

---

## 👁️ Perception Pipeline

### Depth Camera → Filtered Point Cloud

The drone's OakD-Lite depth camera captures **307,200 raw 3D points per frame** at 30fps. Two filters clean this data for real-time use:

1. **Voxel Grid Downsampling** — Divides space into 8cm cubes. All points inside each cube get replaced by one averaged point. Reduces volume by ~99%.
2. **Statistical Outlier Removal** — For each point, checks its 10 nearest neighbors. Points that are unusually far from neighbors are noise and get removed.

**Result:** 307,200 raw points → ~170 clean, meaningful points per frame.

![RViz2 Point Cloud — Green dots show the filtered depth camera output in real-time](Demo_Pic/Screenshot%20from%202026-06-08%2002-14-56.png)

---

## 🗺️ 3D Occupancy Mapping (OctoMap)

### Why a Map?

Without a map, the drone only knows what it sees **right now**. If it turns away from the red pillar, it forgets the pillar exists. OctoMap solves this by building a **persistent 3D memory**:

- Space is divided into 10cm cubes (voxels)
- Each cube is classified as: **Occupied** (obstacle), **Free** (safe to fly), or **Unknown** (not yet seen)
- As the drone flies and looks around, the map accumulates — it never forgets

### TF Broadcaster

For the map to be accurate, OctoMap needs to know **where the camera was** when each frame was captured. The TF Broadcaster node reads the drone's position from PX4 odometry and publishes coordinate frame transforms (`map → base_link → camera_frame`).

![OctoMap + Point Cloud — Blue cubes are the persistent 3D map, green dots are the live camera feed](Demo_Pic/Screenshot%20from%202026-06-08%2002-15-31.png)

### Full System View

The combined view shows all three systems working together: Gazebo simulation (right), keyboard controller with real-time position tracking (top-right), RViz2 with OctoMap and point cloud overlay (left).

![Full System — Gazebo + RViz2 + Keyboard Controller running simultaneously](Demo_Pic/Screenshot%20from%202026-06-08%2002-06-28.png)

---

## 📂 Project Files

| File | Description |
|:---|:---|
| `launch_sim.sh` | One-command launcher — opens 8 coordinated terminals |
| `indoor_10x8x3.sdf` | Gazebo world file — room geometry, obstacles, markers |
| `keyboard_control.py` | Manual WASD drone controller with real-time position display |
| `offboard_waypoint_nav.py` | Automated waypoint navigator (proved blind nav fails) |
| `pointcloud_filter.py` | Voxel Grid + SOR filter node (307K → 170 points) |
| `tf_broadcaster.py` | Publishes drone position as TF transforms for OctoMap |
| `octomap_params.yaml` | OctoMap server configuration (resolution, range, thresholds) |
| `drone_rviz.rviz` | RViz2 saved config — point cloud + OctoMap visualization |
| `analyze_hover.py` | Post-flight hover stability analyzer |
| `presentation.html` | Project presentation slides |

---

## 🚀 Quick Start

### Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble (`sudo apt install ros-humble-desktop`)
- PX4 Autopilot compiled for SITL
- Micro-XRCE-DDS Agent built from source

### Launch Everything
```bash
# Single command launches all 8 terminals:
~/Desktop/Drone_IP/launch_sim.sh
```

### What Opens

| Terminal | Process | What It Does |
|:---:|:---|:---|
| T1 | Micro-XRCE-DDS Agent | PX4 ↔ ROS 2 communication |
| T2 | PX4 + Gazebo | Flight controller + 3D simulation |
| T3 | Keyboard Controller | Manual drone piloting (WASD + T/L) |
| T4 | GZ-ROS2 Bridge | Bridges depth camera + clock to ROS 2 |
| T5 | Point Cloud Filter | Cleans raw depth data in real-time |
| T6 | RViz2 | 3D visualization (point cloud + OctoMap) |
| T7 | TF Broadcaster | Publishes drone-to-map coordinate transforms |
| T8 | OctoMap Server | Builds persistent 3D occupancy map |

### Controls
| Key | Action |
|:---:|:---|
| `T` | Takeoff (arm + fly to 1.8m) |
| `L` | Land at current position |
| `W / S` | Forward / Backward |
| `A / D` | Left / Right |
| `R / F` | Altitude Up / Down |
| `Q` | Quit |

---

## 🔮 Roadmap

- **A* Path Planner** — Convert the OctoMap into a 2D navigation grid and compute the shortest obstacle-free path from start to destination
- **Autonomous Navigation** — Connect the planner output to PX4 offboard control so the drone flies itself through the obstacle course with zero collisions

---

## 👤 Author

**Amit Kumar** — IIIT Delhi  
GitHub: [@ami4go](https://github.com/ami4go)

---

## 📄 License

This project is part of an academic B.Tech Project (BTP) at IIIT Delhi.
