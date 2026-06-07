# Understanding the Previous BTP Report
### *"Design and Development of Electronic Systems – Drone"*
**By:** Aarehant Jain & Shashank Mishra | **Advisor:** Dr. Anuj Grover | **Date:** April 2026

---

## What Is This Project About? (In Simple Words)

Imagine you want a small drone to fly **inside a room** (like a warehouse or tunnel) **all by itself** — no human with a remote control. The drone needs to:

1. **See** what is around it (walls, obstacles, people)
2. **Think** about what it sees (is that a wall? how far is it?)
3. **Decide** where to fly next (avoid the wall, take a safe path)

The previous team built the **brain and eyes** for this drone using cheap, tiny computer chips (microcontrollers). They worked on **three big pieces**:

---

## The Three Big Pieces

### Piece 1: The "Eyes" — Ultrasonic Radar (Chapters 1 & 5)

**What it is:** A small radar system that sends out sound waves (like a bat!) and listens for the echo to measure distance to objects.

**How it works (simple version):**
1. The chip generates a **chirp** — a sound that sweeps from 35 kHz to 45 kHz (too high for humans to hear)
2. The chirp bounces off a wall and comes back
3. The chip measures **how long** the echo took to return
4. Using basic math: `distance = speed x time / 2`, it calculates how far the wall is

**Hardware used:**
- **STM32G071** microcontroller (a tiny chip with only 32 KB of RAM!)
- **Murata MA40S4S** ultrasonic speakers/microphones
- Analog amplifiers and filters on a breadboard

**What they achieved:**
- Detects objects up to **1 meter** away
- Resolution of about **4.3 cm** (can tell apart two objects that are 4.3 cm apart)
- Later upgraded to **4 sensors** (front, back, left, right) running simultaneously using a second chip (**STM32F303RE**)

**Problems they ran into:**
- Sensors interfered with each other (crosstalk) — solved by clever timer-based triggering
- Timer channel conflicts on the chip — solved by reassigning hardware timers

---

### Piece 2: The "Brain" — Memory-Efficient CNN (Chapters 2, 3 & 4)

**What it is:** A way to run a neural network (AI) on a tiny chip that has almost no memory.

**Why it matters:** Normal AI models need gigabytes of RAM. Their chip has only **32 KB**. They figured out how to squeeze a neural network into this tiny space.

**How they did it (simple version):**
1. They modeled the neural network as a **graph** (boxes connected by arrows)
2. Each box (layer) needs some memory to store its temporary results
3. They figured out the **optimal order** to run the layers so that memory usage stays as low as possible
4. Key trick: **reuse memory buffers** — when one layer is done, its memory can be given to the next layer

**Techniques used:**
- **Stripe streaming:** Process the image in small horizontal strips instead of all at once
- **Knapsack optimization:** Choose which layers to recompute vs. store (like a packing problem)
- **Topological sorting:** Run layers in the correct dependency order

**Results:**
- Reduced peak memory usage by **30-50%** compared to naive execution
- Made it possible to run CNNs that would otherwise be impossible on 32 KB RAM

---

### Piece 3: The "Navigation System" — ROS + Gazebo Simulation (Chapters 6, 7 & 8)

**What it is:** The software pipeline that takes sensor data and makes the drone fly autonomously in a simulated room.

**The pipeline (step by step):**

Depth Camera -> Point Cloud -> OctoMap (3D Map) -> Path Planner -> PX4 -> Drone Moves

**Step 1 — Depth Camera (Chapter 6):**
- A simulated camera in Gazebo that gives depth (distance) for every pixel
- Output format: **PointCloud2** (a list of thousands of 3D points representing the room)
- They applied filters to clean up noisy data:
  - **Statistical outlier removal** — removes random wrong points
  - **Voxel grid filtering** — simplifies the point cloud by averaging nearby points into cubes

**Step 2 — 3D Mapping with OctoMap (Chapter 7):**
- Takes the point cloud and builds a **3D map** of the room
- Uses an **octree** data structure (divides space into smaller and smaller cubes)
- Very memory-efficient — only stores cubes that actually contain obstacles
- Uses **ray casting** to determine which space is free vs. occupied

**Step 3 — Path Planning with DWA (Chapter 8):**
- **DWA = Dynamic Window Approach** — a local path planner
- The drone samples many possible paths (different speeds and turn rates)
- Each path is scored by a **cost function**: cost = a x heading + b x distance + c x velocity
  - Heading: is the path pointing toward the goal?
  - Distance: does the path stay far from obstacles?
  - Velocity: does the path maintain good speed?
- The path with the **lowest cost** wins and the drone follows it

**Software stack they used:**
- **ROS 1 Noetic** — robot middleware
- **Gazebo Classic 11** — 3D physics simulator
- **MAVROS** — bridge between ROS and PX4
- **PX4 v1.13** — flight controller firmware

---

## The Overall System Architecture

This is the most important diagram from the report (Figure 8.3, Page 31):

```
SENSORS (Depth Camera, LiDAR, Sonar)
        |
        v
PointCloud Generator
(Convert sensor data to 3D point cloud)
        |
        v
Local Planner Node (Obstacle avoidance logic)
  - Histogram Builder (2D obstacle map)
  - Cost Calculator (Score paths)
        |
        v
STAR Planner (Path search)
        |
        v
Avoidance Setpoint Publisher
(Publishes /trajectory /setpoint)
        |
        v
MAVROS Node (ROS to PX4 Bridge)
        |
        v
PX4 Flight Stack (Flight Controller)
        |
        v
DRONE MOVES
```

---

## What Has Already Been Done vs. What We Need To Do

| **Already Done (by previous team)** | **Our Job (carry forward)** |
|---|---|
| Built FMCW ultrasonic radar on STM32 | Set up the simulation environment (Done in Week 0) |
| Multi-sensor simultaneous sensing (4 directions) | Upgrade the software stack from ROS 1 to ROS 2 (Done) |
| Memory-efficient CNN execution planner | Replace MAVROS with XRCE-DDS bridge (Done) |
| PointCloud processing pipeline in simulation | Integrate real sensor data into the new ROS 2 pipeline |
| OctoMap-based 3D mapping | Re-implement OctoMap in ROS 2 ecosystem |
| DWA path planning with cost function | Test and validate path planning in our indoor world |
| Ran everything in ROS 1 + Gazebo Classic | Migrate the full pipeline to ROS 2 + Gazebo Sim |

---

## What We Have Done So Far (Week 0)

We set up the **foundation** for carrying this project forward:

1. **Installed Ubuntu 22.04** with ROS 2 Humble (replacing ROS 1 Noetic)
2. **Compiled PX4 v1.14** in SITL (Software-In-The-Loop) mode
3. **Built Micro-XRCE-DDS Agent** (replacing MAVROS) — verified the bridge connection
4. **Created a custom 10x8x3m indoor world** in Gazebo Sim with wireframe walls
5. **Verified sensor topics** — IMU running at 248.5 Hz (expected ~250 Hz)
6. **Recorded baseline flight data** — 60-second hover test with 11,807 odometry samples
7. **Measured position hold accuracy** — horizontal drift only 0.04m (very stable)

---

## What We Have Done So Far (Week 1) — Obstacle Navigation

### 1.1 Added Obstacles to the Indoor World

We modified `indoor_10x8x3.sdf` to include **3 obstacles** and **2 floor markers** inside the room:

| Object | Position (X, Y) | Size (W × D × H) | Color | Purpose |
|---|---|---|---|---|
| Pillar 1 | (0, 1) | 0.5 × 0.5 × 2.5m | 🔴 Red | Blocks the direct path between start and destination |
| Low Wall | (-1.5, -1) | 2.0 × 0.3 × 1.5m | 🟠 Orange | Partially blocks the lower flight corridor |
| Pillar 2 | (2, 0) | 0.6 × 0.6 × 2.5m | 🟡 Yellow | Forces the drone to weave between obstacles |
| Start Marker | (-3, -2) | radius 0.4m disc | 🟢 Green | Visual-only — marks the takeoff point |
| Destination Marker | (3, 2) | radius 0.4m disc | 🔵 Blue | Visual-only — marks the landing target |

Each obstacle has both a **collision** component (physics — the drone can't fly through it) and a **visual** component (what you see in Gazebo). The markers are visual-only so the drone can land on them.

**Room layout (top-down view):**
```
        Y = +4 (front wall)
   ┌─────────────────────────────────┐
   │                                 │
   │                  🔵 DEST (3,2)  │
   │           ██ Red (0,1)          │
   │                      ██ Yellow  │
   │      ═══ Orange (-1.5,-1)       │
   │  🟢 START (-3,-2)              │
   │                                 │
   └─────────────────────────────────┘
        Y = -4 (back wall)
   X = -5                       X = +5
```

---

### 1.2 Approach 1: Automated Waypoint Navigation (`offboard_waypoint_nav.py`)

**What this script does:**
- Arms the drone and switches to **offboard mode** (PX4 takes commands from code instead of a remote)
- Takes off to 1.8m altitude
- Follows a pre-planned sequence of **6 waypoints** that route around the obstacles
- At each waypoint, waits until the drone is within 0.4m before advancing to the next
- Lands at the blue destination marker

**Planned flight path:**
```
Spawn(0,0) → Takeoff(0,0,1.8m) → Start(-3,-2,1.8m) →
  WP(-3,0,1.8m) → WP(-1,2,1.8m) → WP(1,2,1.8m) →
    Destination(3,2,1.8m) → LAND
```

**Key concepts in this file:**
| Concept | Explanation |
|---|---|
| **Offboard mode** | PX4 flight mode where the drone takes position/velocity commands from an external computer (our ROS 2 node) instead of a human remote. Requires continuous heartbeat messages at ≥2 Hz. |
| **NED coordinates** | PX4 uses North-East-Down. Z = -1.8 means 1.8 meters UP. Standard in aviation but counterintuitive at first. |
| **OffboardControlMode** | Heartbeat message — tells PX4 "I'm alive and want position control." If PX4 stops receiving this, it switches to failsafe. |
| **TrajectorySetpoint** | The actual "go to this X, Y, Z" command sent to PX4. |
| **VehicleCommand** | One-time commands like ARM (turn on motors, cmd=400) or SET_MODE (switch to offboard, cmd=176). |
| **VehicleOdometry** | PX4 publishes the drone's current position ~30 times/sec. We subscribe to this to check if we've reached a waypoint. |

#### ⚠️ Challenges Encountered with Automated Navigation

The automated waypoint approach had **significant stability issues**:

1. **Wall collisions** — The drone would repeatedly hit the room walls and obstacles, especially during transitions between waypoints. The straight-line path between waypoints passed too close to obstacles.

2. **Falling and recovering** — After collisions, the drone would lose altitude, fall toward the ground, then attempt to climb back up, creating an erratic up-down oscillation.

3. **Lack of real-time awareness** — The script has **no obstacle detection**. It blindly follows pre-programmed coordinates. If the drone drifts due to physics (propwash near walls, collision rebounds), it has no way to correct or avoid the obstacle in its path.

4. **PX4 parameter tuning** — The default PX4 acceleration and velocity limits (`MPC_ACC_HOR_MAX = 5.0 m/s²`) may be too aggressive for a confined indoor space. The previous team used 2.0 m/s² for indoor safety.

**Root cause:** This approach is essentially **open-loop navigation** — it assumes the drone will perfectly follow waypoints without any sensor feedback about nearby obstacles. Real obstacle avoidance requires a **closed-loop** system with sensors.

---

### 1.3 Approach 2: Manual Keyboard Controller (`keyboard_control.py`)

To overcome the issues with automated navigation, we built a **manual keyboard teleop** script that gives direct, smooth control of the drone.

**Controls:**
```
              W (forward / Y+)
              ▲
   A (left) ◄   ► D (right)
              ▼
              S (backward / Y-)

   R = altitude UP       F = altitude DOWN
   T = Takeoff            L = Land
   Q = Quit
```

**How it works:**
- Each key press moves the **target position** by 0.5 meters in the pressed direction
- PX4 handles the smooth flight to the new target (it uses its own PID controllers)
- Room boundaries are **clamped** — you can't accidentally command the drone outside the walls (0.5m safety margin)
- The script uses Python's `termios` module to read individual keypresses without waiting for Enter

**Why this approach works better:**
| Issue | Automated Script | Keyboard Controller |
|---|---|---|
| Obstacle awareness | None — follows blind waypoints | **Human provides the intelligence** — you see obstacles and steer around them |
| Movement granularity | Jumps between distant waypoints | 0.5m incremental steps — smooth and controlled |
| Recovery from drift | None — keeps pushing toward waypoint | You can pause, adjust, correct |
| Learning value | Black box — hard to understand | **Hands-on understanding** of offboard control, NED coordinates, and PX4 behavior |

**Key insight:** The keyboard controller proved that our **offboard control pipeline is working correctly** (arming, mode switching, position commands all work). The problem with the automated script is not the code — it's the **lack of perception**. The drone needs sensors to avoid obstacles, not just pre-programmed waypoints.

---

### 1.4 Files Created/Modified This Week

| File | Type | What It Does |
|---|---|---|
| `indoor_10x8x3.sdf` | Modified | Added 3 obstacles (red pillar, orange wall, yellow pillar) and 2 floor markers (green start, blue destination) |
| `offboard_waypoint_nav.py` | New | Automated waypoint navigation — arms, takes off, follows 6 waypoints, lands. Demonstrates offboard control but lacks obstacle sensing. |
| `keyboard_control.py` | New | Manual keyboard teleop — WASD movement, R/F altitude, boundary-clamped. Smooth and educational. |

---

### 1.5 Key Learnings

1. **Pre-planned waypoints ≠ obstacle avoidance.** Without sensors, the drone is flying blind. It's like walking through a room with your eyes closed, following memorized directions — you'll eventually bump into something.

2. **PX4's offboard mode works reliably.** Both scripts successfully arm the drone, switch to offboard mode, and accept position commands. The communication pipeline (ROS 2 → XRCE-DDS → PX4) is solid.

3. **NED coordinate system matters.** A common mistake: setting Z = +1.8 (which means 1.8m underground in NED) instead of Z = -1.8 (1.8m above ground). Getting this wrong causes the drone to dive into the floor.

4. **Indoor flight is harder than outdoor.** The confined space means small errors compound quickly — a slight drift toward a wall leads to a collision, which causes a bounce, which leads to more collisions. Outdoor drones have much more room for error.

5. **The keyboard controller is a powerful debugging tool.** Before automating anything, being able to manually fly the drone helps you verify the world, understand the drone's behavior, and test the control pipeline.

---

## How We Plan To Carry This Forward (Revised After Week 1)

> After Week 1, we now understand that **pre-planned waypoints are not enough** for indoor obstacle avoidance. The drone needs **sensors** to perceive obstacles in real-time. This directly maps to the previous team's architecture pipeline (BTP Report, Figure 8.3):
>
> ```
> SENSORS → PointCloud → Local Planner → Cost Calculator → Path Search → PX4 → Drone Moves
> ```
>
> We are currently at: `[waypoint commands] → PX4 → Drone Moves`
> We need to build: `SENSORS → PointCloud → Local Planner → [smart commands] → PX4 → Drone Moves`

### What Needs To Happen Next (The Gap)

Our keyboard controller proves the control pipeline works. The automated waypoint script proves blind navigation fails. To bridge this gap, we need:

```
Current state:                     Target state:
                                   
 Hardcoded        PX4              Depth Camera    Obstacle     Smart       PX4
 Waypoints  ───►  Drone            + PointCloud ─► Detection ─► Planner ─►  Drone
                                                                  ▲
 (no eyes,         (crashes)        (the drone      (knows what   │
  no brain)                          can SEE)        is near)   Cost Function
                                                               (from BTP Ch.8)
```

---

### Phase 1: Perception — Give the Drone Eyes (Weeks 2-3)

**Goal:** The drone currently flies blind. We need to add a **simulated depth camera** so it can see obstacles.

- Add a depth camera sensor to the x500 drone model in Gazebo
- Write a ROS 2 node that converts depth images into **PointCloud2** format (a list of thousands of 3D points representing what the camera sees)
- Apply the same **statistical outlier removal** and **voxel grid filtering** from the BTP report (Chapter 6) to clean up noisy data
- Verify the drone can "see" the red/orange/yellow obstacles we placed in the room

**Why this matters:** This is the "eyes" component. Right now our drone is like a person walking blindfolded. The depth camera gives it vision.

### Phase 2: Mapping — Build a Mental Map (Weeks 3-4)

**Goal:** Convert the raw point cloud into a structured **3D occupancy map** using OctoMap.

- Integrate **OctoMap** with ROS 2 — this library uses an octree data structure to efficiently represent which parts of the room are occupied (obstacles) vs. free (flyable space)
- Visualize the occupancy grid in **RViz2** — you'll see a 3D map of the room being built in real-time as the drone looks around
- This is the same approach from BTP Report Chapter 7

**Why this matters:** Raw point clouds are like individual snapshots. OctoMap combines them into a persistent "memory" of where obstacles are, even after the drone has turned away.

### Phase 3: Planning — Smart Path Generation (Weeks 5-6)

**Goal:** Replace our blind waypoints with a **sensor-aware path planner** that computes obstacle-free routes in real-time.

Two options (increasing complexity):

| Approach | How It Works | Pros | Cons |
|---|---|---|---|
| **A\* on Occupancy Grid** | Discretize the room into a 2D/3D grid, search for the shortest obstacle-free path | Simple to implement, guaranteed to find a path if one exists | Doesn't account for drone dynamics |
| **DWA (from BTP Report Ch.8)** | Sample many possible trajectories, score each with a cost function: `J = α·heading + β·velocity + γ·clearance`, select the best | Accounts for drone speed and turning ability, real-time | More complex, needs parameter tuning |

- Start with **A\*** for simplicity (grid-based, well understood)
- Later upgrade to **DWA** using the cost function from the BTP report (Figure 8.2):
  - **Heading cost** — is the path pointing toward the goal?
  - **Velocity cost** — does the path maintain good speed?
  - **Clearance cost** — does the path stay far from obstacles?

**Why this matters:** This replaces the "human intelligence" from the keyboard controller with "algorithmic intelligence." The drone can plan its own safe path.

### Phase 4: Integration & Tuning (Weeks 7-8)

**Goal:** Connect the full pipeline end-to-end and validate with quantitative metrics.

- Connect: **Depth Camera → PointCloud → OctoMap → Path Planner → PX4**
- Tune PX4 parameters for indoor flight:
  - `MPC_ACC_HOR_MAX`: reduce from 5.0 to 2.0 m/s² (gentler acceleration)
  - `MPC_VEL_MANUAL`: reduce maximum velocity for indoor safety
  - `MPC_XY_VEL_MAX`: limit horizontal speed to prevent overshoot
- Run the same obstacle room scenario that failed with blind waypoints — measure improvement
- Run the quantitative validation from Task C (success rate, planning latency, false positive/negative rates)
- Record demonstration videos for the final presentation

**Target metrics:**
| Metric | Blind Waypoints (Week 1) | Target (Week 8) |
|---|---|---|
| Mission success rate | ~10% (crashes frequently) | >90% |
| Obstacle collisions | Multiple per run | 0 |
| Planning latency | N/A (pre-programmed) | <200ms |
| Manual intervention needed | Always | Never |

---

## Key Terms Cheat Sheet (for the meeting)

| Term | What It Means |
|---|---|
| **FMCW Radar** | A type of radar that uses a sweeping frequency chirp to measure distance |
| **STM32** | A family of cheap, low-power microcontroller chips |
| **CNN** | Convolutional Neural Network — an AI model that recognizes patterns in images |
| **SRAM** | The tiny amount of working memory on a microcontroller (32 KB here) |
| **PointCloud2** | A ROS data format — a collection of thousands of 3D points representing objects |
| **OctoMap** | A 3D mapping library that represents space as occupied/free using octrees |
| **Octree** | A tree data structure that recursively divides 3D space into 8 smaller cubes |
| **DWA** | Dynamic Window Approach — a real-time path planning algorithm |
| **PX4** | Open-source flight controller software that runs on drone hardware |
| **SITL** | Software-In-The-Loop — running PX4 in simulation instead of on real hardware |
| **ROS** | Robot Operating System — middleware that connects robot software components |
| **MAVROS** | Old bridge between ROS 1 and PX4 (uses MAVLink protocol) |
| **XRCE-DDS** | New bridge between ROS 2 and PX4 (uses DDS protocol, faster) |
| **Gazebo** | A 3D physics simulator where we test the drone virtually |

---
---

# 🎯 OUR FOCUS: Objective 2 — End-to-End Loop Closure in SITL

> The previous team built the sensor, the AI model, and the simulation — but **never connected them**. The sensor data never actually reached the drone's flight controller. Our job is to **close this loop**.

```
STM32 Sensor → [??? GAP ???] → ROS Topics → Planner → PX4 → Drone Moves
```
**The "???" is our job.**

---

## Our 3 Specific Tasks

### Task A: Sensor-to-ROS Bridge
Stream live STM32 data into ROS topics that PX4 can consume.

**Plan:**
1. STM32 sends data via UART: `{distance: 0.45, class: "glass", confidence: 0.91}`
2. We write a ROS 2 node that reads serial, parses it, and publishes:
   - `/sensor/distance` (Float32)
   - `/sensor/material_class` (String)
   - `/sensor/confidence` (Float32)
3. In SITL, we **mock** this data using a virtual sensor until real hardware is ready

---

### Task B: Classification-Aware Avoidance
Make the drone **react differently** based on what the sensor reports.

| Sensor Input | Drone Reaction |
|---|---|
| Any obstacle < 0.3m | **Emergency stop** — hover immediately |
| Hard surface (metal/concrete) at 0.3-0.8m | **Reroute** — plan alternative path |
| Soft surface (fabric/foam) at 0.3-0.8m | **Slow down** — proceed cautiously |
| Low confidence (< 70%) | **Hover** — wait for more readings |
| No obstacle detected | **Continue** — follow planned trajectory |

**Implementation:** A ROS 2 avoidance node subscribes to sensor topics and publishes modified trajectory setpoints to `/fmu/in/trajectory_setpoint` or emergency commands to `/fmu/in/vehicle_command`.

---

### Task C: Quantitative Mission Validation
Run test scenarios and measure performance with hard numbers.

| Metric | What It Measures | Target |
|---|---|---|
| **Mission Success Rate** | % of runs where drone reaches goal without crashing | > 90% |
| **Planning Latency** | Time from detection to avoidance maneuver | < 200ms |
| **False-Positive Rate** | How often drone avoids a phantom obstacle | < 5% |
| **False-Negative Rate** | How often drone misses a real obstacle | 0% |

**Test scenarios:** (1) Straight flight toward wall, (2) Corridor navigation, (3) Multi-obstacle room, (4) Material-dependent (glass vs. curtain)

---

## Week-by-Week Plan

| Week | Focus | Deliverable |
|---|---|---|
| **Week 0** | Environment setup & baseline | Verified toolchain, hover data ✅ DONE |
| **Week 1** | Indoor obstacles + drone control | SDF obstacles, waypoint nav (challenges noted), keyboard teleop ✅ DONE |
| **Week 2** | Add depth camera to Gazebo drone | Simulated sensor publishing PointCloud2 |
| **Week 3** | PointCloud processing + OctoMap | Noise filtering, 3D occupancy grid in RViz2 |
| **Week 4** | A* path planner on occupancy grid | Drone computes obstacle-free paths automatically |
| **Week 5** | DWA local planner (from BTP report) | Cost-function-based trajectory selection |
| **Week 6** | Sensor-to-ROS bridge for STM32 | Mock/real UART data as ROS 2 topics |
| **Week 7** | Full pipeline integration | Camera → PointCloud → Map → Planner → PX4 |
| **Week 8** | Quantitative validation & report | Success rate, latency, demo video, final documentation |

---

## What To Tell The Tutor

> "In Week 0 we set up the simulation toolchain (ROS 2 Humble, PX4 v1.14, Gazebo Sim with XRCE-DDS bridge) and verified baseline hover stability (0.04m horizontal drift).
>
> In Week 1, we added physical obstacles to the Gazebo world and attempted two approaches to drone navigation:
>
> **Approach 1 (Automated Waypoints)** — We wrote a ROS 2 offboard control node that commands the drone through pre-planned waypoints. This approach failed because the drone has no sensors — it flies blind and repeatedly collides with obstacles. This is **open-loop navigation** and confirmed that the previous team's insight was correct: you need a perception pipeline (sensors → point cloud → occupancy map → planner) for real obstacle avoidance.
>
> **Approach 2 (Keyboard Teleop)** — We built a manual controller that proved the control pipeline is solid. The drone responds accurately to position commands; the missing piece is perception, not control.
>
> **Next step:** Add a simulated depth camera to the drone so it can actually see obstacles, then implement the path planning pipeline from the previous BTP report (OctoMap + DWA cost function)."

