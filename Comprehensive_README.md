# Autonomous Indoor Drone Navigation

## Project Flow & Task Tracker
- [x] **Phase 1: Perception (Giving the Drone Eyes)**
  - Attached a depth camera (`x500_depth`) to the drone in Gazebo.
  - Bridged Gazebo depth topics to ROS 2.
  - Implemented a Point Cloud Filter (Voxel Grid & Statistical Outlier Removal).
- [x] **Phase 2: 3D Mapping (Giving the Drone a Memory)**
  - Configured OctoMap server to build a persistent 3D occupancy grid.
  - Built a TF Broadcaster to link drone odometry with camera frames.
  - Synchronized simulation time (`use_sim_time`) to eliminate map smearing.
  - Ignored ground plane by adjusting Z-axis occupancy limits to keep the map clean.
- [ ] **Phase 3: Path Planning (Giving the Drone a Brain)**
  - (Upcoming) A* Path Planner to navigate the mapped occupancy grid.
  - (Upcoming) Connecting the planner directly to the PX4 control loop for autonomous flight.

## What We Are Doing and Why
* **Why simulate?** To safely test autonomous obstacle avoidance algorithms before deploying them on expensive, real-world hardware.
* **Why filter the point cloud?** Raw depth cameras generate hundreds of thousands of points per second. Filtering downsamples this data and removes noise, allowing real-time mapping without crashing the CPU.
* **Why use OctoMap?** A raw point cloud is just a "snapshot" of what the drone sees *right now*. If the drone turns around, it forgets what was behind it. OctoMap stitches these snapshots together into a permanent 3D memory, allowing the drone to remember obstacles even when they are out of view.
* **Why synchronize time?** ROS 2 defaults to real-world time, but Gazebo runs on simulated time. Synchronizing them (`use_sim_time:=true`) ensures that the drone's recorded position perfectly matches the camera's snapshot time, eliminating "ghost lines" and smearing in the 3D map.

## Technology Stack & Versions
* **OS:** Ubuntu 22.04 LTS (Jammy)
* **Middleware:** ROS 2 Humble
* **Simulator:** Gazebo Garden (v7.9.0)
* **Flight Stack:** PX4 Autopilot (v1.14)
* **DDS Bridge:** Micro-XRCE-DDS Agent
* **Language:** Python 3.10 & NumPy 2.2.6
* **Mapping:** OctoMap Server (v2.3.1), OctoMap RViz Plugins (v2.1.1)

---

## Visual Demonstrations

### 1. The Simulation Environment
The drone operates in a 10x8x3m bounded room containing strategically placed obstacles (pillars and walls) that block the direct path to the destination.
![Simulation Environment](Demo_Pic/Screenshot%20from%202026-06-08%2002-14-03.png)

### 2. Preflight: Raw Point Cloud
Before takeoff, the drone's depth camera (green points) captures a massive amount of data, but it only "sees" what is directly in front of it.
![Preflight Point Cloud](Demo_Pic/Screenshot%20from%202026-06-08%2002-14-56.png)

### 3. Postflight: Persistent 3D OctoMap
As the drone flies, the OctoMap server stitches the point clouds together. The blue cubes represent the persistent 3D map of the pillars and walls. The ground is intentionally ignored to keep the map focused strictly on obstacles.
![Postflight OctoMap](Demo_Pic/Screenshot%20from%202026-06-08%2002-15-31.png)

---
*This entire pipeline is orchestrated via a customized 8-terminal bash script (`launch_sim.sh`) for seamless one-click launch and teardown.*
