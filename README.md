# 🚁 Autonomous Indoor Drone Navigation & Mapping

> **Sensor-based 3D mapping and autonomous exploration for GPS-denied indoor flight using ROS 2, PX4, and Gazebo.**

This project implements a **fully autonomous indoor drone** that can explore an **unknown environment** with zero prior knowledge, build a **persistent 3D map** using OctoMap, and **navigate collision-free paths** to any user-specified destination — all without GPS.

The drone uses its depth camera as its only sensor. It discovers rooms, doorways, and obstacles through **frontier-based exploration**, compares **6 different path planning algorithms** (A\*, Dijkstra, Bellman Ford, PRM, RRT, Theta\*), detects **new obstacles in real-time** and replans around them, and supports **2.5D altitude-aware planning** to fly over or under ceiling-height hazards.

![Frontier Exploration Complete — Drone autonomously mapped all 3 rooms](Demo_Pic/Frontier1.png)

---

## Table of Contents

- [Project Journey](#-project-journey--from-hardware-to-full-autonomy)
- [System Architecture](#-system-architecture)
- [Frontier-Based Exploration](#-frontier-based-autonomous-exploration)
- [Dynamic Obstacle Detection & Replanning](#-dynamic-obstacle-detection--replanning)
- [Path Planning Algorithms](#-path-planning-algorithm-comparison)
- [2.5D Altitude-Aware Navigation](#-25d-altitude-aware-navigation)
- [Perception Pipeline](#-perception-pipeline)
- [Simulation Environments](#-simulation-environments)
- [Installation & Setup](#-installation--setup)
- [Demo Gallery & Video Demo](#-demo-gallery)
- [Design Decisions](#-key-design-decisions)
- [Future Improvements](#-future-improvements)
- [Authors](#-authors)

---

## 📋 Project Journey — From Hardware to Full Autonomy

This project evolved through **four distinct phases**, each driven by real limitations discovered in the previous one. What started as a hardware-focused BTP became a fully autonomous software-defined navigation system.

### Phase 0 — Legacy BTP (Previous Team)

The original team (Aarehant Jain & Shashank Mishra, supervised by Dr. Anuj Grover) built the **hardware foundation**: ultrasonic radar sensors on STM32 microcontrollers and a memory-efficient CNN for obstacle classification. Their work proved indoor drone navigation was feasible but highlighted the need for a complete software stack.

| Step | What Happened | Key Insight |
|:---:|:---|:---|
| 0.1 | Ultrasonic radar on STM32G071 | Single-point distance (1m range, 4.3cm resolution) — not enough for 3D mapping |
| 0.2 | 4-sensor array on STM32F303RE | Crosstalk issues, still only 4 distance readings vs 307K from a depth camera |
| 0.3 | CNN on microcontroller (32KB RAM) | Proved edge AI is possible but too constrained for real-time SLAM |

---

### Phase 1 — Perception & Mapping Pipeline

We shifted to a **software-defined approach** using a depth camera, ROS 2, and OctoMap for dense 3D reconstruction.

| Step | Milestone | What We Learned |
|:---:|:---|:---|
| 1 | Environment Setup | Ubuntu 22.04, ROS 2 Humble, PX4 SITL, Gazebo Garden — foundation ready |
| 2 | Indoor World Design | Custom SDF environments with realistic obstacles |
| 3 | Blind Waypoint Navigation | **❌ Crashed into walls** → proved sensors are essential |
| 4 | Keyboard Teleoperation | Manual WASD control → proved the PX4 control pipeline works |
| 5 | Depth Camera Integration | `x500_depth` with OakD-Lite (640×480 @ 30fps) — gave the drone "eyes" |
| 6 | Point Cloud Filtering | Voxel Grid + SOR (307K → ~170 pts/frame) — cleaned noisy data |
| 7 | OctoMap 3D Mapping | Persistent voxel map from depth camera — the drone now "remembers" |
| 8 | A\* Path Planning | First successful autonomous navigation on sensor-derived maps |
| 9 | Multi-Room House | 3 rooms with staggered doorways — tested real indoor complexity |

![Early Perception — Point Cloud + OctoMap in single room environment](Demo_Pic/Screenshot%20from%202026-06-08%2002-15-31.png)

---

### Phase 2 — Multi-Algorithm Comparison & Stability

We implemented **6 path planning algorithms** and built infrastructure to compare them objectively.

| Step | Milestone | What We Learned |
|:---:|:---|:---|
| 10 | 4 More Algorithms | PRM, RRT, Theta\*, Dijkstra, Bellman Ford — for comparison |
| 11 | Visual Benchmark | Run all 6 at once, 6 colored paths in RViz — immediate comparison |
| 12 | Drone Stability | Velocity clamping + yaw smoothing — smoother, safer flight |
| 13 | Obstacle Avoidance | Real-time path monitoring + dynamic replanning — safety net |
| 14 | Ceiling Obstacles | Added fans, hanging light, low pipe — realistic indoor hazards |
| 15 | 3D OctoMap Query | Voxel hash set for O(1) 3D occupancy checks at any (x,y,z) |
| 16 | 2.5D Altitude Planning | 3-layer planner (1.2m/1.8m/2.4m) — drone dips under pipes |
| 17 | Semantic Pipeline | YOLOv8n-seg on GPU → colored OctoMap voxels |

![Multi-Algorithm Benchmark — All 6 paths simultaneously](Demo_Pic/Benchmark%201.png)

---

### Phase 3 — Frontier-Based Autonomous Exploration

The hardcoded scan waypoints from Phase 2 worked for known environments but couldn't scale to truly unknown spaces. We replaced them with **frontier-based exploration** — the drone autonomously discovers unexplored regions and maps them without any prior knowledge of the room layout.

| Step | Milestone | What We Learned |
|:---:|:---|:---|
| 18 | Frontier Extraction | Free cells adjacent to unknown → cluster → score → fly to best |
| 19 | Reactive Safety | Raw-grid obstacle distance checks (0.25m threshold) prevent crashes |
| 20 | Stuck Detection | 8s no-movement → blacklist frontier → retreat → try next target |
| 21 | Corner-Cut Fix | Pure Pursuit lookahead reduced to 0.3m to prevent wall clipping on turns |
| 22 | Unknown Space Penalty | Exploration: 3.0 (cross freely) vs Goal Nav: 50.0 (avoid unmapped walls) |
| 23 | Complete Room Coverage | MIN_DISTANCE=0.5m, MIN_CLEARANCE=0.3m — no corner left unscanned |

![Frontier Exploration — Drone autonomously mapping through doorways](Demo_Pic/Frontier1.png)

---

## 🏗️ System Architecture

```
┌───────────────────────────────────────────────────────────────────────┐
│                        GAZEBO GARDEN v7.9.0                          │
│  ┌──────────────────┐  ┌────────────┐  ┌──────────────────────────┐  │
│  │ 3-Room House     │  │ x500_depth │  │ OakD-Lite Depth Camera   │  │
│  │ 18×12×3m         │  │ Drone      │  │ 640×480 @ 30fps          │  │
│  │ 12 furniture pcs │  │ Model      │  │ Range: 0.3–5.0m          │  │
│  └──────────────────┘  └────────────┘  └──────────────────────────┘  │
└──────────────────────────────┬────────────────────────────────────────┘
                               │ Gazebo Transport
                               ▼
                    ┌──────────────────────┐
                    │   ros_gz_bridge      │
                    │   (Topic Bridging)   │
                    └──────────┬───────────┘
                               │ ROS 2 Topics
              ┌────────────────┼──────────────────┐
              ▼                ▼                  ▼
   ┌──────────────┐  ┌────────────────┐  ┌──────────────────┐
   │ PX4 Autopilot│  │ Point Cloud    │  │ TF Broadcaster   │
   │ (SITL)       │  │ Filter Node    │  │                  │
   │              │  │ 307K→170 pts   │  │ map → base_link  │
   │ Odometry ────┼──┤                │  │    → camera      │
   └──────────────┘  └───────┬────────┘  └───────┬──────────┘
                             │                    │
                             ▼                    ▼
                    ┌──────────────────────────────────────┐
                    │          OctoMap Server               │
                    │   Persistent 3D Occupancy Map         │
                    │   Resolution: 10cm | Range: 5m        │
                    └─────────────┬────────────────────────┘
                                  │ /projected_map
                    ┌─────────────┼────────────────┐
                    ▼                              ▼
          ┌──────────────────┐          ┌──────────────────────────┐
          │    RViz2          │          │ Autonomous Navigator     │
          │  3D Visualization │          │ ┌──────────────────────┐ │
          │  (OctoMap + Path) │          │ │ Frontier Explorer    │ │
          └──────────────────┘          │ │ A* Path Planner      │ │
                                        │ │ Obstacle Avoidance   │ │
                                        │ │ 2.5D Altitude Plan   │ │
                                        │ └──────────────────────┘ │
                                        └──────────────────────────┘
```

### Tech Stack

| Component | Version | Purpose |
|:---|:---|:---|
| **Ubuntu** | 22.04 LTS (Jammy) | Operating System |
| **ROS 2** | Humble Hawksbill | Robotics middleware |
| **PX4 Autopilot** | v1.14 (SITL) | Flight controller firmware |
| **Gazebo** | Garden v7.9.0 | 3D physics simulator |
| **Micro-XRCE-DDS** | v2.4.x | PX4 ↔ ROS 2 bridge |
| **OctoMap** | v1.9.8 | 3D occupancy mapping |
| **PCL-ROS** | v2.4.5 | Point cloud processing |
| **Python** | 3.10 + NumPy | ROS 2 node scripting |
| **Path Planners** | A\*, Dijkstra, Bellman Ford, PRM, RRT, Theta\* | 6-algorithm comparison |

---

## 🔍 Frontier-Based Autonomous Exploration

![Frontier Exploration Complete — Drone autonomously mapped all 3 rooms using frontier-based exploration](Demo_Pic/Frontier1.png)

### The Problem

In early phases, the drone visited **hardcoded scan waypoints** — fixed positions manually placed in each room. This worked for known environments but completely fails in the real world where the room layout is unknown. A drone deployed in a new building has no idea where rooms, doorways, or corridors are.

### What is a Frontier?

A **frontier** is the boundary between what the drone has explored and what remains unknown. In the occupancy grid:
- **Free cells** (value = 0): Explored, safe to fly through
- **Unknown cells** (value = -1): Never seen by the depth camera
- **Occupied cells** (value > 50): Confirmed obstacles (walls, furniture)

A **frontier cell** is a free cell that has at least one unknown neighbor. These cells represent the **edge of the drone's knowledge** — the most promising locations to fly to next, because looking outward from them will reveal new space.

### How It Works

```
Step 1: EXTRACT                    Step 2: CLUSTER                  Step 3: SCORE
┌─────────────────┐               ┌─────────────────┐              ┌─────────────────┐
│ ░░░░░░░███░░░░░ │               │ ░░░░░░░███░░░░░ │              │                 │
│ ░░░░░░░█  █░░░░ │   Scan all    │ ░░░░░░░█  █░░░░ │   Group      │  Cluster A: 15  │
│ ░░FFFF░█  █░░░░ │──→free cells──│ ░░AAAA░█  █░░░░ │──→connected──│  cells, 3.2m    │
│ ░░░░░F░█  █FFF░ │   adjacent    │ ░░░░░A░█  █BBB░ │   frontier   │  Score: 8.5     │
│ ░░░░░░░████░░░░ │   to unknown  │ ░░░░░░░████░░░░ │   cells      │                 │
│ ░░░░░░░░░░░░░░░ │               │ ░░░░░░░░░░░░░░░ │              │  Cluster B: 7   │
│ ░ = free █ = wall│               │                 │              │  cells, 5.1m    │
│ F = frontier     │               │ A,B = clusters  │              │  Score: 4.2     │
└─────────────────┘               └─────────────────┘              └─────────────────┘

Step 4: FLY                        Step 5: SCAN                     Step 6: REPEAT
┌─────────────────┐               ┌─────────────────┐              ┌─────────────────┐
│                 │               │     🚁 ←──┐     │              │                 │
│  A* plans safe  │               │    ╱  ╲   │     │   New map    │  Extract new    │
│  path to best   │──→Fly along──→│   ╱ 360°╲  │    │──→data from──│  frontiers from │
│  scoring        │   the path    │  ╱  scan  ╲ │   │   scan       │  updated map    │
│  frontier       │               │ ╱         ╲│    │              │                 │
│                 │               │            │    │              │  If none left:  │
│  🚁────→ · · →⊕ │               │ depth cam   │    │              │  EXPLORATION    │
│                 │               │ captures all│    │              │  COMPLETE! ✅    │
└─────────────────┘               └─────────────────┘              └─────────────────┘
```

**Step-by-step:**

1. **Extract**: Scan the OctoMap's `/projected_map` for all free cells adjacent to unknown cells
2. **Cluster**: Group connected frontier cells into clusters using flood-fill (minimum 3 cells per cluster)
3. **Score**: Rank each cluster: `score = size^1.5 / distance^0.2` — bigger clusters with more unknown space behind them are prioritized
4. **Safety Filter**: Skip clusters too close to walls (< 0.3m clearance) or near previously failed targets
5. **Fly**: Plan an A\* path to the best frontier's safest cell and fly there using Pure Pursuit control
6. **Scan**: Perform a 360° rotation at the frontier so the depth camera captures obstacles from every angle
7. **Repeat**: Go back to Step 1 with the updated map. When no frontiers remain, the room is fully mapped

### Exploration State Machine

```
         ┌──────────┐
         │ TAKEOFF  │
         │ → 1.8m   │
         └────┬─────┘
              ▼
         ┌──────────┐        ┌──────────┐
    ┌───→│ FINDING  │───────→│ FLYING   │
    │    │ Extract  │ Best   │ A* path  │
    │    │ frontiers│ target │ to target│
    │    └────┬─────┘        └────┬─────┘
    │         │ No                │ Arrived
    │         │ frontiers         ▼
    │         │             ┌──────────┐
    │         │             │ SCANNING │
    │         │             │ 360° rot │
    │         │             └────┬─────┘
    │         │                  │
    │         │                  │ Done
    │         │     ┌────────────┘
    │         │     │
    │         ▼     ▼
    │    ┌──────────────┐       ┌──────────────┐
    │    │   COMPLETE   │──────→│    READY      │
    │    │  No frontiers│       │ User clicks   │
    │    │  remaining   │       │ 2D Goal Pose  │
    │    └──────────────┘       └───────┬───────┘
    │                                   │
    │    ┌──────────┐                   ▼
    └────│RETREATING│           ┌──────────────┐
         │ Back away│           │  NAVIGATING  │
         │ from wall│           │  Follow A*   │
         └──────────┘           │  path to goal│
              ▲                 └──────────────┘
              │ Safety
              │ trigger
              └─── (reactive safety / stuck detection)
```

### Key Parameters

| Parameter | Value | Purpose |
|:---|:---:|:---|
| MIN_CLUSTER_SIZE | 3 cells | Catch even small doorway frontiers |
| MIN_DISTANCE | 0.5m | Don't ignore close-by frontiers |
| MIN_OBSTACLE_CLEARANCE | 0.3m | Allow targeting corners near walls |
| SAFETY_MARGIN | 0.40m | Obstacle inflation for physical drone clearance |
| LOOKAHEAD_DIST | 0.3m | Short Pure Pursuit lookahead prevents corner-cutting |
| STUCK_TIME_LIMIT | 8s | Time before declaring drone is stuck |
| YAW_STEP | 0.12 rad | Rotation speed during 360° scan |

---

## 🛡️ Dynamic Obstacle Detection & Replanning

The drone doesn't just plan a path once — it **continuously monitors the environment** and reacts to changes in real-time.

### How It Works

The OctoMap is constantly being updated as the depth camera captures new frames. The drone checks whether any **new obstacles have appeared on its planned path**:

- **During Exploration**: On every control tick (~50ms), the drone checks the next **10 waypoints** against the latest OctoMap data. If any waypoint is now inside an obstacle, it immediately aborts and replans.
- **During Goal Navigation**: Every **2 seconds** (40 ticks at 20Hz), the drone checks the next **5 waypoints** using both 3D OctoMap voxel queries and 2D projected map fallback.

### Decision Logic

```
    Drone is flying along planned path
              │
              ▼
    ┌─────────────────────────┐
    │ Check upcoming waypoints │
    │ against LIVE OctoMap     │
    └────────────┬────────────┘
                 │
          ┌──────┴──────┐
          │             │
    Path is CLEAR   New obstacle ON path
          │             │
          ▼             ▼
    Continue flying   ┌─────────────────────┐
    (no action)       │ HOVER in place      │
                      │ Replan A* path from │
                      │ current position to │
                      │ same destination    │
                      └─────────┬───────────┘
                                │
                          ┌─────┴──────┐
                          │            │
                    New path       No path found
                    found          (too many replans)
                          │            │
                          ▼            ▼
                    Resume flight  Blacklist target,
                    on new path    find new frontier
```

**Key behavior: If a new obstacle appears but is NOT on the drone's planned path, the drone completely ignores it and continues flying.** Only obstacles that directly block the upcoming waypoints trigger a replan. This prevents unnecessary path recalculations and keeps the drone moving efficiently.

### Replan Flood Protection

To prevent infinite replan loops (where the drone keeps finding and aborting paths to the same blocked target), a **replan limiter** tracks how many times replanning occurs:
- If **3 or more replans** happen within **10 seconds**, the current target is **blacklisted** and the drone moves on to the next best frontier

---

## 📊 Path Planning Algorithm Comparison

All 6 algorithms run on the **same OctoMap-derived sensor map**. The drone explores identically; only the path planning strategy differs.

### Individual Algorithm Paths

These screenshots show each algorithm's path on the same 3-room house, navigating through doorways:

![A* — Grid-locked 45°/90° path through doorways](Demo_Pic/astar.png)

![Theta* — Smooth any-angle shortcuts through line-of-sight checks](Demo_Pic/theta.png)

![PRM — Random sample nodes connected by straight-line edges](Demo_Pic/prm.png)

![RRT — Tree grown from start with random exploration](Demo_Pic/rrt.png)

### Algorithm Overview

| Algorithm | Type | How It Works |
|:---|:---|:---|
| **A\*** | Grid-based | `f = g + h` heuristic search on 8-connected grid. Fast, reliable, staircase paths. |
| **Dijkstra** | Grid-based | A\* without heuristic (`f = g` only). Same path, 2–3× more nodes explored. |
| **Bellman Ford** | Edge relaxation | V-1 iterations over all edges. Same path as Dijkstra but 30–50× slower. Designed for negative-weight graphs. |
| **PRM** | Sampling-based | Scatters 600 random points, connects neighbors with collision-free edges, runs Dijkstra on roadmap. |
| **RRT** | Sampling-based | Grows a tree from start by random sampling. Fastest first-path discovery, but jagged results. |
| **Theta\*** | Any-angle | A\* extension with Bresenham line-of-sight checks. Produces the smoothest, shortest true geometric paths. |

### Benchmark Results

| Property | A\* | Dijkstra | Bellman Ford | PRM | RRT | Theta\* |
|:---|:---:|:---:|:---:|:---:|:---:|:---:|
| **Optimal Path?** | ✅ Grid-optimal | ✅ Grid-optimal | ✅ Grid-optimal | ❌ Approximate | ❌ Non-optimal | ✅ True shortest |
| **Path Smoothness** | ⭐⭐ Staircase | ⭐⭐ Staircase | ⭐⭐ Staircase | ⭐⭐⭐ Straight segments | ⭐ Jagged | ⭐⭐⭐⭐ Smoothest |
| **Deterministic?** | ✅ Always same | ✅ Always same | ✅ Always same | ❌ Random each run | ❌ Random each run | ✅ Always same |
| **Narrow Passages** | ✅ Handles well | ✅ Handles well | ✅ Handles well | ⚠️ Needs seeding | ⚠️ Needs goal bias | ✅ Handles well |
| **Planning Speed** | ~60ms | ~100–190ms | ~2500ms | ~80–160ms | ~14ms (fastest) | ~400ms |
| **Nodes Explored** | ~8K | ~13K | ~335K edges | ~600 samples | ~200 tree nodes | ~7K |
| **Best For** | Reliable baseline | Understanding A\* | Negative-weight graphs | Large open spaces | Quick exploration | **Drone flight** |

### Visual Benchmark Mode

The benchmark mode (`--algo benchmark`) runs all 6 algorithms simultaneously when you click a goal, displaying all paths at once with different colors:

🔴 A\* — Red | 🟢 Dijkstra — Green | 🟡 Bellman Ford — Yellow | 🔵 PRM — Blue | 🟣 RRT — Magenta | ⚪ Theta\* — Cyan

![Visual Benchmark — All 6 paths simultaneously](Demo_Pic/Benchmark%201.png)

![Visual Benchmark — Different goal](Demo_Pic/Benchmark%202.png)

### 🏆 Which Algorithm is Best?

| Use Case | Best Choice | Why |
|:---|:---|:---|
| **Autonomous exploration (unknown rooms)** | **A\*** | Evaluates every cell's wall-proximity cost, naturally centers in corridors, no corner-cutting |
| **Goal navigation (mapped environment)** | **Theta\*** | Smoothest and shortest true geometric path once the map is built |
| **Large open spaces (outdoor/warehouse)** | **PRM** | Efficient for large free-space areas with multi-query reuse |
| **Unknown/rapidly changing environment** | **RRT** | Fastest first-path discovery, good for quick escape routes |
| **Academic comparison only** | **Dijkstra / Bellman Ford** | Same paths as A\* but slower — demonstrates algorithmic principles |

---

## 🏔️ 2.5D Altitude-Aware Navigation

### The Problem

Fixed-altitude flight (1.8m) will collide with ceiling-height obstacles. Real indoor environments have fans, hanging lights, and exposed pipes.

### Ceiling Obstacles in the Simulation

| Obstacle | Location | Height | Purpose |
|:---|:---|:---|:---|
| Ceiling Fan (Bedroom) | (0, 0) | z = 2.55m | OctoMap 3D capture test |
| Ceiling Fan (Living Room) | (-6, 1) | z = 2.55m | In cross-room flight path |
| Hanging Light (Door 1) | (-3, 2) | z = 2.05m | Altitude awareness near doorways |
| **Low Pipe (Study Room)** | **(6, 0)** | **z = 1.9m** | **Critical test: exactly at drone altitude** |

### 3 Altitude Layers

Instead of full 3D grid search (1.2M voxels — too slow), we use 3 discrete altitude layers:

```
Layer 2:  z = 2.4m  ─── High (above most furniture, below ceiling)
Layer 1:  z = 1.8m  ─── Default flight altitude
Layer 0:  z = 1.2m  ─── Low (under hanging obstacles)
```

For each layer, a 2D occupancy grid is generated by querying the 3D OctoMap at that altitude range. The planner runs A\* on a **layered graph** where nodes connect horizontally (8 directions, same layer) and vertically (altitude transitions between adjacent layers). Altitude changes carry extra cost to model real-world battery usage.

**Result:** When the drone encounters the low pipe at z=1.9m in the Study Room, the planner automatically routes it through Layer 0 (z=1.2m) to fly **under** the pipe, then back to Layer 1 once past it.

### 3D OctoMap Query (`core/octomap_3d_query.py`)

Subscribes to `/octomap_point_cloud_centers` and builds a **voxel hash set** for O(1) spatial lookups:
- `is_occupied(x, y, z)` — single point occupancy check
- `is_column_clear(x, y, z_min, z_max)` — altitude corridor check
- `safe_altitude(x, y, desired_z)` — finds nearest clear altitude

---

## 👁️ Perception Pipeline

### Depth Camera → Filtered Point Cloud

The OakD-Lite depth camera captures **307,200 raw 3D points per frame** at 30fps. Two filters clean this data:

1. **Voxel Grid Downsampling** — Divides space into 8cm cubes, replacing all points per cube with one average. Reduces volume by ~99%.
2. **Statistical Outlier Removal** — Points unusually far from their neighbors are noise and get removed.

**Result:** 307,200 raw → ~170 clean points per frame.

### 3D Occupancy Mapping (OctoMap)

OctoMap builds a **persistent 3D memory** from the filtered point cloud:
- Space is divided into **10cm voxels**
- Each voxel: **Occupied** (obstacle), **Free** (safe), or **Unknown** (unseen)
- The map **never forgets** — turning away from an obstacle doesn't erase it
- The TF Broadcaster provides the camera's position in the world (`map → base_link → camera_frame`) so OctoMap knows where each depth frame was captured

---

## 🏠 Simulation Environments

### Environment 1 — Single Room (10×8×3m)

The initial test environment: a single room with 3 color-coded obstacles. This is where the perception + mapping pipeline was first proven, and where A\* first successfully navigated around obstacles.

![Single Room — A* path navigating around obstacles](Demo_Pic/Autonomous_AStar_Nav.png)

### Environment 2 — 3-Room House (18×12×3m)

A realistic multi-room house that tests the drone's ability to **explore through doorways** and **plan cross-room paths**.

```
         18m
┌──────────┬──────────┬──────────┐
│          │          │          │
│  LIVING  │ BEDROOM  │  STUDY   │
│  ROOM    │          │  ROOM    │  12m
│          │  🚁 Spawn│          │
│ 🛋️ Sofa  │ 🛏️ Bed   │ 🖥️ Desk  │
│ ☕ Table │ 🗄️ Ward. │ 📚 Books │
│ 📺 TV    │ 🪞 Dress.│ 🗃️ Files │
│ 🪑 Chair │          │ 🪑 Chair │
│          │          │          │
└────┘  └──┴────┘  └──┴─────────┘
   Door 1      Door 2
   (Y=+2)      (Y=-2)
   2.5m wide   2.5m wide
```

**Key features:**
- **12 furniture obstacles** across 3 rooms (sofa, bed, desk, bookshelf, wardrobe, etc.)
- **Staggered doorways** — Door 1 at Y=+2, Door 2 at Y=-2, forcing zig-zag paths
- **Ceiling obstacles** — fans, hanging light, low pipe for 2.5D altitude testing

![3-Room House — Gazebo view with OctoMap overlay](Demo_Pic/House_3Room_Navigation.png)

---

## 🚀 Installation & Setup

> **Full dependency details:** [`requirements.txt`](requirements.txt)

### Prerequisites

```bash
# 1. Ubuntu 22.04 LTS
# 2. ROS 2 Humble
sudo apt update && sudo apt install -y ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc

# 3. ROS 2 Packages
sudo apt install -y \
  ros-humble-ros-gzgarden-bridge \
  ros-humble-octomap ros-humble-octomap-server \
  ros-humble-octomap-msgs ros-humble-octomap-ros \
  ros-humble-octomap-rviz-plugins ros-humble-pcl-ros

# 4. PX4 Autopilot (SITL)
git clone https://github.com/PX4/PX4-Autopilot.git --recursive -b v1.14.0
cd PX4-Autopilot && bash ./Tools/setup/ubuntu.sh
make px4_sitl gz_x500_depth

# 5. Micro-XRCE-DDS Agent
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make && sudo make install && sudo ldconfig

# 6. px4_msgs Workspace
mkdir -p ~/px4_ros_ws/src && cd ~/px4_ros_ws/src
git clone https://github.com/PX4/px4_msgs.git -b release/1.14
cd ~/px4_ros_ws && source /opt/ros/humble/setup.bash && colcon build

# 7. This Repository
cd ~/Desktop
git clone https://github.com/shashank22603/ROS.git Drone_IP
chmod +x ~/Desktop/Drone_IP/launch_sim.sh
```

### Launch

```bash
# Autonomous frontier exploration (recommended):
~/Desktop/Drone_IP/launch_sim.sh --auto

# Manual flight (keyboard control):
~/Desktop/Drone_IP/launch_sim.sh

# Choose a specific algorithm for goal navigation:
~/Desktop/Drone_IP/launch_sim.sh --auto --algo astar      # A* (default)
~/Desktop/Drone_IP/launch_sim.sh --auto --algo dijkstra    # Dijkstra
~/Desktop/Drone_IP/launch_sim.sh --auto --algo bellman     # Bellman Ford
~/Desktop/Drone_IP/launch_sim.sh --auto --algo prm         # PRM
~/Desktop/Drone_IP/launch_sim.sh --auto --algo rrt         # RRT
~/Desktop/Drone_IP/launch_sim.sh --auto --algo theta       # Theta*
~/Desktop/Drone_IP/launch_sim.sh --auto --algo benchmark   # Visual Benchmark (all 6)
```

### What Opens (8 Terminals)

| Terminal | Process | Purpose |
|:---:|:---|:---|
| T1 | Micro-XRCE-DDS Agent | PX4 ↔ ROS 2 communication |
| T2 | PX4 + Gazebo | Flight controller + 3D simulation |
| T3 | Autonomous Navigator | Frontier exploration + path planning |
| T4 | GZ-ROS2 Bridge | Bridges depth camera + pose to ROS 2 |
| T5 | Point Cloud Filter | Cleans raw depth data in real-time |
| T6 | RViz2 | 3D visualization (OctoMap + paths) |
| T7 | TF Broadcaster | Drone-to-map coordinate transforms |
| T8 | OctoMap Server | Builds persistent 3D occupancy map |

### Keyboard Controls (Manual Mode)

| Key | Action |
|:---:|:---|
| `T` | Takeoff (arm + fly to 1.8m) |
| `L` | Land at current position |
| `W / S` | Forward / Backward |
| `A / D` | Left / Right |
| `R / F` | Altitude Up / Down |
| `Q` | Quit |

---

## 🖼️ Demo Gallery

### Full System View

![Full System — Gazebo + RViz + Terminal running together](Demo_Pic/Ref.png)

### Frontier-Based Exploration

![Frontier Exploration — Drone autonomously mapped all 3 rooms](Demo_Pic/Frontier1.png)

### Path Planning Visualization

![Path planning through multi-room house](Demo_Pic/Path%201.png)

![Different goal showing path characteristics](Demo_Pic/Path%202.png)

---

## 💡 Key Design Decisions

### Why Frontier-Based Exploration?

Hardcoded scan waypoints worked for known environments but can't scale to truly unknown spaces. Frontier-based exploration treats every deployment as if it's the first time — the drone discovers room layouts, doorways, and obstacles entirely through its sensors. This is exactly how a real-world autonomous drone would operate.

### Why A\* for Exploration (not Theta\*)?

During evaluation, Theta\*'s line-of-sight shortcuts were **cutting corners too tightly** in narrow indoor spaces. The shortcut calculation checked start and end points but skipped the wall-proximity costs of intermediate cells, causing the drone to graze walls on turns. A\* evaluates every cell individually, so it properly respects the wall-cost map and naturally centers paths in corridors — critical for safe indoor flight.

### Why OctoMap instead of RTAB-Map?

RTAB-Map offers loop closure and dense RGB-D reconstruction, but requires heavy CPU/GPU resources — especially alongside Gazebo, PX4 SITL, and 8 concurrent processes on a laptop. OctoMap provides sufficient 3D mapping fidelity for obstacle avoidance without the overhead of a full visual SLAM pipeline.

### Dual Unknown-Space Penalty

A single penalty value doesn't work for both exploration and navigation:
- **Exploration** (penalty = 3.0): The drone needs to cross unknown space to discover new rooms. Low penalty allows this while still preferring known corridors.
- **Goal Navigation** (penalty = 50.0): When the user clicks a destination, the drone must never route through unmapped walls. High penalty makes unknown space virtually impassable.

---

## 📂 Project Structure

| File | Description |
|:---|:---|
| `launch_sim.sh` | One-command launcher — opens 8 coordinated terminals |
| **planners/** | |
| `planners/navigator_astar.py` | **Main navigator** — Frontier exploration + A\* planning + safety systems |
| `planners/planner_3d.py` | **2.5D Multi-Layer Planner** — Altitude-aware A\* across 3 layers |
| `planners/navigator_dijkstra.py` | **Dijkstra** — A\* without heuristic (uniform cost search) |
| `planners/navigator_bellman_ford.py` | **Bellman Ford** — Edge relaxation (V-1 iterations) |
| `planners/navigator_prm.py` | **PRM** — Probabilistic Roadmap with Dijkstra search |
| `planners/navigator_rrt.py` | **RRT** — Rapidly-exploring Random Tree |
| `planners/navigator_theta_star.py` | **Theta\*** — Any-angle A\* with line-of-sight shortcuts |
| `planners/navigator_benchmark.py` | **Visual Benchmark** — Runs all 6, shows colored paths in RViz |
| **core/** | |
| `core/pointcloud_filter.py` | Voxel Grid + SOR filter node (307K → 170 points) |
| `core/tf_broadcaster.py` | Publishes drone position as TF transforms for OctoMap |
| `core/octomap_3d_query.py` | 3D voxel hash set for O(1) occupancy lookups |
| `core/keyboard_control.py` | Manual WASD drone controller |
| `core/room_scanner.py` | Automated room scanning flight patterns |
| `core/semantic_pointcloud.py` | YOLOv8n-seg semantic segmentation pipeline |
| **benchmark/** | |
| `benchmark/planner_library.py` | All 6 algorithms as pure functions (no ROS dependencies) |
| `benchmark/run_benchmark.py` | Automated benchmark with metrics table + CSV export |
| `benchmark/save_map.py` | Saves OctoMap to disk for offline benchmarking |
| **worlds/** | |
| `worlds/house_3room.sdf` | **Active world** — 3-room house (18×12m) with furniture + ceiling obstacles |
| `worlds/indoor_10x8x3.sdf` | Legacy world — single room (10×8m) with 3 obstacles |
| **config/** | |
| `config/drone_rviz.rviz` | RViz2 config — OctoMap + paths + frontier visualization |
| `config/octomap_params.yaml` | OctoMap server configuration |
| **docs/** | |
| `docs/BTP_report__winter2026.pdf` | Original BTP report (hardware phase) |
| `docs/algorithm_comparison_plan.md` | Algorithm evaluation methodology |

---

## 🔮 Future Improvements

### 1. RTAB-Map Integration for Photorealistic Mapping

The current OctoMap produces a functional but visually sparse voxel grid. Integrating **RTAB-Map** (Real-Time Appearance-Based Mapping) would generate a **dense, textured 3D reconstruction** with loop closure, producing maps that look much more realistic and closer to the actual environment. This would also enable **visual place recognition** — the drone could recognize rooms it has visited before using camera images, not just occupancy data.

### 2. Multi-Drone Collaborative Exploration

Currently a single drone explores the entire environment. Deploying **multiple drones simultaneously** with a shared OctoMap would dramatically reduce exploration time. Each drone would claim different frontier clusters, avoid duplicating work, and merge their individual maps into a single consistent model. This requires solving distributed frontier allocation and map merging.

### 3. Real-World Hardware Deployment

The entire software stack (ROS 2 nodes, OctoMap, path planners) is designed to be **hardware-agnostic**. Replacing Gazebo with a real depth camera (Intel RealSense or OAK-D) and a localization system (Vicon motion capture, Intel T265 tracking camera, or Visual-Inertial Odometry) would enable deployment on a physical drone with minimal code changes. The main challenge would be tuning flight dynamics and sensor noise parameters for real-world conditions.

### 4. Semantic-Aware Navigation

The YOLOv8 semantic segmentation pipeline is implemented but not yet integrated into path planning decisions. Future work could make the drone **avoid fragile objects** (glass tables, electronics), **prefer safe corridors** near walls, or **prioritize exploration of rooms containing specific objects** (e.g., "explore rooms with furniture first").

---

## 👤 Authors

- **Amit Kumar** — Software architecture, autonomous navigation, path planning, frontier exploration

**Advisor:** Dr. Anuj Grover, IIIT Delhi

---

## 📄 License

This project is part of an academic Independent Project (IP) at IIIT Delhi.
