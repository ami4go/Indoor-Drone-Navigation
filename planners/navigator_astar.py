#!/usr/bin/env python3
"""
=============================================================================
 AUTONOMOUS A* NAVIGATOR v3 — Sensor-Based Mapping (Real-World Ready)
 File: autonomous_navigator.py
=============================================================================

 This navigator does NOT know the room layout in advance.
 Instead, it uses the depth camera + OctoMap to discover obstacles,
 then plans collision-free paths on the SENSOR-DERIVED map.

 Pipeline:
   1. Drone takes off to 1.8m
   2. EXPLORATION PHASE: Drone flies to multiple scan positions and
      rotates 360° at each, allowing OctoMap to build a complete 3D map
      from depth camera data.
   3. READY PHASE: Map is built. User clicks "2D Goal Pose" in RViz.
   4. A* runs on OctoMap's /projected_map (real sensor data).
   5. Drone flies the obstacle-free path.

 Coordinate System:
   Gazebo ENU:  X = East (+right), Y = North (+forward)
   PX4 NED:     X = North,         Y = East
   Conversion:  PX4_X += Gazebo_delta_Y,  PX4_Y += Gazebo_delta_X

 Why this matters for real-world:
   In real-world deployment, you won't have an SDF file.
   This approach uses only sensor data — swap Gazebo for a real depth
   camera and localization system, and the same code works.

=============================================================================
"""

import math
import time
import heapq
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleOdometry
)


# ─────────────────────────────────────────────────────────────────────────────
#  EXPLORATION WAYPOINTS
# ─────────────────────────────────────────────────────────────────────────────
# These positions are spread across all 3 rooms of the house.
# The drone flies to each one and rotates 360° to give the depth camera
# full coverage. The route goes: Bedroom → Living Room → back → Study.
#
# For real-world: these could be replaced with a frontier-based exploration
# algorithm that automatically decides where to fly next based on unmapped areas.
#
# Format: (x, y) in Gazebo ENU coordinates
# House layout: Living Room (X:-9 to -3) | Bedroom (X:-3 to +3) | Study (X:+3 to +9)
# Doors are STAGGERED: Door 1 at Y=+2, Door 2 at Y=-2
SCAN_WAYPOINTS = [
    # --- Bedroom (drone spawns at 0,0) ---
    ( 0.0,  0.0),   #  1. Spawn point — initial scan
    ( 0.0,  3.0),   #  2. Near bed
    ( 0.0, -3.0),   #  3. Near wardrobe/dresser
    # --- Through Door 1 (at Y=+2) into Living Room ---
    (-3.0,  2.0),   #  4. Doorway 1
    (-6.0,  3.5),   #  5. Near sofa + armchair
    (-6.0, -2.0),   #  6. Near TV stand
    (-7.5,  0.0),   #  7. West wall area
    # --- Back through Door 1, then through Door 2 (at Y=-2) into Study ---
    (-1.0,  2.0),   #  8. Back through bedroom (near door 1)
    ( 1.0, -2.0),   #  9. Approach door 2
    ( 3.0, -2.0),   # 10. Doorway 2
    ( 6.0,  2.5),   # 11. Near desk + chair
    ( 6.0, -2.5),   # 12. Near filing cabinet
    ( 7.5,  0.0),   # 13. Near bookshelf + east wall
]


# ─────────────────────────────────────────────────────────────────────────────
#  A* PLANNER — Plans on OctoMap's /projected_map (sensor-derived)
# ─────────────────────────────────────────────────────────────────────────────
class SensorMapPlanner:
    """
    A* path planner that operates on an OccupancyGrid from OctoMap.
    This is the REAL sensor data, not a hardcoded map.
    """

    def __init__(self, occupancy_grid_msg, safety_margin=0.7):
        info = occupancy_grid_msg.info
        self.resolution = info.resolution
        self.origin_x = info.origin.position.x
        self.origin_y = info.origin.position.y
        self.width = info.width
        self.height = info.height

        # Convert 1D occupancy data → 2D numpy grid
        # OccupancyGrid values: 0=free, 100=occupied, -1=unknown
        raw = np.array(occupancy_grid_msg.data).reshape((self.height, self.width))

        # Only cells with probability > 50 are treated as obstacles.
        # Unknown (-1) and free (0) cells are passable — the drone can
        # explore through unknown space.
        self.grid = np.where(raw > 50, 1, 0).astype(np.uint8)

        # Inflate obstacles for safety
        self._inflate(safety_margin)

    def _inflate(self, margin_m):
        """Grow obstacle cells by margin_m meters in all directions."""
        cells = int(math.ceil(margin_m / self.resolution))
        if cells <= 0:
            return

        inflated = np.copy(self.grid)
        obstacles = np.argwhere(self.grid == 1)

        for r, c in obstacles:
            r_lo = max(0, r - cells)
            r_hi = min(self.height, r + cells + 1)
            c_lo = max(0, c - cells)
            c_hi = min(self.width, c + cells + 1)
            inflated[r_lo:r_hi, c_lo:c_hi] = 1

        self.grid = inflated

    def world_to_grid(self, x, y):
        c = int((x - self.origin_x) / self.resolution)
        r = int((y - self.origin_y) / self.resolution)
        return r, c

    def grid_to_world(self, r, c):
        x = self.origin_x + (c + 0.5) * self.resolution
        y = self.origin_y + (r + 0.5) * self.resolution
        return x, y

    def is_free(self, r, c):
        return (0 <= r < self.height and
                0 <= c < self.width and
                self.grid[r, c] == 0)

    def plan(self, sx, sy, gx, gy):
        """A* from (sx, sy) to (gx, gy) in world coordinates."""
        start = self.world_to_grid(sx, sy)
        goal = self.world_to_grid(gx, gy)

        # Clamp to grid bounds
        goal = (
            max(0, min(self.height - 1, goal[0])),
            max(0, min(self.width - 1, goal[1]))
        )
        start = (
            max(0, min(self.height - 1, start[0])),
            max(0, min(self.width - 1, start[1]))
        )

        # If start or goal lands in an obstacle, nudge to nearest free cell
        if not self.is_free(*start):
            print(f"  ⚠️  Start ({sx:.2f},{sy:.2f}) is in obstacle — finding nearest free cell")
            start = self._nearest_free(start)
            if start is None:
                print("  ❌  Cannot find free start cell!")
                return None

        if not self.is_free(*goal):
            print(f"  ⚠️  Goal ({gx:.2f},{gy:.2f}) is in obstacle — finding nearest free cell")
            goal = self._nearest_free(goal)
            if goal is None:
                print("  ❌  Cannot find free goal cell!")
                return None

        # A* with 8-connectivity
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {start: None}
        g = {start: 0.0}
        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0),
                (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                break
            for dr, dc in dirs:
                nxt = (cur[0] + dr, cur[1] + dc)
                if not self.is_free(*nxt):
                    continue
                step = 1.414 if dr and dc else 1.0
                ng = g[cur] + step
                if nxt not in g or ng < g[nxt]:
                    g[nxt] = ng
                    f = ng + math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    heapq.heappush(open_set, (f, nxt))
                    came_from[nxt] = cur

        if goal not in came_from:
            print("  ❌  No path found!")
            return None

        # Reconstruct path
        path, cur = [], goal
        while cur is not None:
            path.append(self.grid_to_world(*cur))
            cur = came_from[cur]
        path.reverse()
        return self._simplify(path)

    def _nearest_free(self, cell):
        """BFS outward to find the nearest free cell."""
        visited = {cell}
        queue = [cell]
        while queue:
            next_queue = []
            for r, c in queue:
                for dr in range(-1, 2):
                    for dc in range(-1, 2):
                        nb = (r + dr, c + dc)
                        if nb not in visited:
                            if self.is_free(*nb):
                                return nb
                            visited.add(nb)
                            next_queue.append(nb)
            queue = next_queue
        return None

    def _simplify(self, path):
        """Remove collinear intermediate waypoints."""
        if len(path) < 3:
            return path
        result = [path[0]]
        for i in range(1, len(path) - 1):
            dx1 = path[i][0] - path[i - 1][0]
            dy1 = path[i][1] - path[i - 1][1]
            dx2 = path[i + 1][0] - path[i][0]
            dy2 = path[i + 1][1] - path[i][1]
            if abs(dx1 * dy2 - dx2 * dy1) > 1e-4:
                result.append(path[i])
        result.append(path[-1])
        return result


# ─────────────────────────────────────────────────────────────────────────────
#  AUTONOMOUS NAVIGATOR NODE
# ─────────────────────────────────────────────────────────────────────────────
class AutonomousNavigator(Node):
    """
    State Machine:
      INIT       → waiting for PX4 + Gazebo data
      TAKEOFF    → arming and ascending to flight altitude
      EXPLORING  → flying scan waypoints + rotating at each
      READY      → map built, hovering, waiting for user goal
      NAVIGATING → following A* path to goal
    """

    def __init__(self):
        super().__init__('autonomous_navigator')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # ── Subscribers ──
        self.gz_sub = self.create_subscription(
            TFMessage, '/world/house_3room/dynamic_pose/info',
            self.gz_pose_cb, qos)
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_cb, qos)
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose',
            self.goal_cb, 10)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/projected_map',
            self.map_cb, 10)

        # ── Publishers ──
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ── State ──
        self.gz_pos = None       # Gazebo ENU [x, y, z]
        self.px4_pos = None      # PX4 NED  [x, y, z]
        self.occupancy_grid = None  # Latest OccupancyGrid from OctoMap

        self.state = 'INIT'
        self.waypoints = []      # Current A* path waypoints
        self.wp_idx = 0
        self.goal_xy = None      # Store goal for replanning

        # Exploration state
        self.scan_wp_idx = 0     # Which scan waypoint we're heading to
        self.scan_yaw = 0.0      # Current rotation angle during 360° scan
        self.scan_rotating = False

        # Parameters
        self.ALTITUDE = 1.8          # Flight altitude (metres)
        self.WP_REACH_DIST = 0.4     # Distance to consider waypoint reached
        self.SCAN_WP_REACH = 0.5     # Distance to consider scan position reached
        self.YAW_STEP = 0.04         # Radians per control tick during rotation
        self.SAFETY_MARGIN = 0.25     # Obstacle inflation for A* (metres)
        # Doorway clearance: 2.5m - 2*(0.075 wall + 0.25 inflate) = 1.85m gap ✅

        # Stability: velocity clamping + yaw smoothing
        self.MAX_NAV_DELTA = 1.5     # Max target distance from drone (metres)
        self.current_yaw = 0.0       # Smoothed yaw angle (radians)
        self.YAW_SMOOTH = 0.15       # Yaw interpolation factor (0=no turn, 1=snap)

        # Obstacle avoidance: check path every N ticks
        self.AVOIDANCE_CHECK_INTERVAL = 40  # Check every 40 ticks (2 seconds at 20Hz)
        self.avoidance_tick = 0

        # ── Control timer: 20 Hz ──
        self.create_timer(0.05, self.control_loop)

        print("\n╔══════════════════════════════════════════════════════════╗")
        print("║   🧠 AUTONOMOUS A* NAVIGATOR v3 — SENSOR-BASED        ║")
        print("╠══════════════════════════════════════════════════════════╣")
        print("║  Phase 1: Takeoff → 1.8m                               ║")
        print("║  Phase 2: Explore room (7 scan positions, 360° each)    ║")
        print("║  Phase 3: OctoMap builds 3D map from depth camera       ║")
        print("║  Phase 4: Click '2D Goal Pose' in RViz                  ║")
        print("║  Phase 5: A* plans on REAL sensor map → drone flies!    ║")
        print("╚══════════════════════════════════════════════════════════╝\n")

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def gz_pose_cb(self, msg):
        """Get drone's ground-truth position from Gazebo."""
        for tf in msg.transforms:
            if 'x500' in tf.child_frame_id.lower():
                t = tf.transform.translation
                self.gz_pos = np.array([t.x, t.y, t.z])
                return
        if msg.transforms:
            t = msg.transforms[0].transform.translation
            self.gz_pos = np.array([t.x, t.y, t.z])

    def odom_cb(self, msg):
        self.px4_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])

    def map_cb(self, msg):
        """Store the latest projected OccupancyGrid from OctoMap."""
        self.occupancy_grid = msg

    def goal_cb(self, msg):
        """Handle user clicking '2D Goal Pose' in RViz."""
        if self.state != 'READY':
            print("⚠️  Drone is not ready yet! Wait for exploration to complete.")
            return

        if self.occupancy_grid is None:
            print("⚠️  No sensor map available! The OctoMap hasn't built yet.")
            return

        if self.gz_pos is None:
            print("⚠️  No Gazebo position received.")
            return

        gx = msg.pose.position.x
        gy = msg.pose.position.y

        print(f"\n🎯  Goal received: ({gx:.2f}, {gy:.2f})")
        print(f"   Drone is at: ({self.gz_pos[0]:.2f}, {self.gz_pos[1]:.2f})")
        print(f"   Planning on SENSOR MAP (OctoMap projected_map)...")

        # Create planner from the latest OctoMap data
        grid_info = self.occupancy_grid.info
        total_cells = grid_info.width * grid_info.height
        occupied = sum(1 for v in self.occupancy_grid.data if v > 50)
        print(f"   Map: {grid_info.width}x{grid_info.height} cells, "
              f"resolution={grid_info.resolution:.2f}m, "
              f"occupied={occupied}/{total_cells}")

        planner = SensorMapPlanner(self.occupancy_grid, safety_margin=self.SAFETY_MARGIN)
        path = planner.plan(self.gz_pos[0], self.gz_pos[1], gx, gy)

        if path:
            self.waypoints = path
            self.wp_idx = 0
            self.goal_xy = (gx, gy)  # Store for replanning
            self.state = 'NAVIGATING'
            self._publish_rviz_path(path)
            print(f"   ✅ Path found — {len(path)} waypoints. Flying!")
        else:
            print("   ❌ No path found. Try a different target.")

    # ── Control Loop ──────────────────────────────────────────────────────────────────

    def control_loop(self):
        if self.px4_pos is None or self.gz_pos is None:
            return

        if self.state == 'INIT':
            self._takeoff()
            return

        # Always send heartbeat when airborne
        self._heartbeat()

        if self.state == 'EXPLORING':
            self._explore_step()

        elif self.state == 'NAVIGATING':
            self._navigate_step()

        elif self.state == 'READY':
            # Hover in place
            self._send_position(self.px4_pos[0], self.px4_pos[1], -self.ALTITUDE)

    # ── Exploration Phase ─────────────────────────────────────────────────────

    def _explore_step(self):
        """Fly to scan waypoints and rotate 360° at each."""

        if self.scan_wp_idx >= len(SCAN_WAYPOINTS):
            # All scan positions visited — exploration complete!
            self.state = 'READY'
            has_map = self.occupancy_grid is not None
            if has_map:
                info = self.occupancy_grid.info
                occupied = sum(1 for v in self.occupancy_grid.data if v > 50)
                print(f"\n  ✅ EXPLORATION COMPLETE!")
                print(f"     Sensor map: {info.width}x{info.height} cells, "
                      f"{occupied} obstacles detected")
            else:
                print(f"\n  ⚠️  Exploration done but no map received from OctoMap!")
                print(f"     Check that Terminal 8 (OctoMap Server) is running.")
            print(f"     Click '2D Goal Pose' in RViz to set a target.\n")
            return

        target_x, target_y = SCAN_WAYPOINTS[self.scan_wp_idx]

        if not self.scan_rotating:
            # Flying towards scan waypoint
            dx_enu = target_x - self.gz_pos[0]
            dy_enu = target_y - self.gz_pos[1]
            dist = math.hypot(dx_enu, dy_enu)

            # Point drone towards the waypoint while flying
            fly_yaw = math.atan2(dx_enu, dy_enu)  # NED yaw convention

            tgt_px4_x = self.px4_pos[0] + dy_enu
            tgt_px4_y = self.px4_pos[1] + dx_enu
            self._send_position(tgt_px4_x, tgt_px4_y, -self.ALTITUDE, yaw=fly_yaw)

            if dist < self.SCAN_WP_REACH:
                # Arrived at scan position — start rotating
                self.scan_rotating = True
                self.scan_yaw = 0.0
                print(f"  📍 Scan position {self.scan_wp_idx + 1}/{len(SCAN_WAYPOINTS)} "
                      f"reached at ({target_x:.1f}, {target_y:.1f}) — rotating 360°...")
        else:
            # Rotating in place at the scan position
            self.scan_yaw += self.YAW_STEP
            self._send_position(
                self.px4_pos[0], self.px4_pos[1], -self.ALTITUDE,
                yaw=self.scan_yaw
            )

            if self.scan_yaw >= 2 * math.pi:
                # Full rotation done — move to next scan position
                self.scan_rotating = False
                self.scan_wp_idx += 1

    # ── Navigation Phase ──────────────────────────────────────────────────────

    def _navigate_step(self):
        """Follow A* waypoints to the goal with velocity clamping."""

        if self.wp_idx >= len(self.waypoints):
            print("  🎉 DESTINATION REACHED! Set a new goal in RViz.")
            self.waypoints = []
            self.goal_xy = None
            self.state = 'READY'
            return

        # Obstacle avoidance: periodically check if path is still clear
        self.avoidance_tick += 1
        if (self.avoidance_tick % self.AVOIDANCE_CHECK_INTERVAL == 0
                and self.goal_xy is not None
                and self.occupancy_grid is not None):
            if self._path_blocked():
                print("  ⚠️  Obstacle detected on path! Replanning...")
                self._replan()
                return

        wx, wy = self.waypoints[self.wp_idx]

        # Relative delta in Gazebo ENU
        dx_enu = wx - self.gz_pos[0]
        dy_enu = wy - self.gz_pos[1]
        dist = math.hypot(dx_enu, dy_enu)

        # Velocity clamping: limit target distance so drone moves smoothly
        if dist > self.MAX_NAV_DELTA:
            scale = self.MAX_NAV_DELTA / dist
            dx_enu *= scale
            dy_enu *= scale

        # Convert ENU delta → NED target
        tgt_px4_x = self.px4_pos[0] + dy_enu
        tgt_px4_y = self.px4_pos[1] + dx_enu
        tgt_px4_z = -self.ALTITUDE

        # Yaw smoothing: interpolate toward target instead of snapping
        target_yaw = math.atan2(wx - self.gz_pos[0], wy - self.gz_pos[1])
        yaw_diff = target_yaw - self.current_yaw
        # Normalize to [-pi, pi]
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi
        self.current_yaw += self.YAW_SMOOTH * yaw_diff
        self._send_position(tgt_px4_x, tgt_px4_y, tgt_px4_z, yaw=self.current_yaw)

        if dist < self.WP_REACH_DIST:
            self.wp_idx += 1
            if self.wp_idx < len(self.waypoints):
                print(f"  📍 WP {self.wp_idx}/{len(self.waypoints)} — {dist:.2f}m")

    def _path_blocked(self):
        """Check if upcoming waypoints collide with latest OctoMap."""
        if self.occupancy_grid is None:
            return False

        info = self.occupancy_grid.info
        raw = np.array(self.occupancy_grid.data).reshape((info.height, info.width))
        margin_cells = int(math.ceil(self.SAFETY_MARGIN / info.resolution))

        # Check next 3 waypoints (or remaining)
        check_end = min(self.wp_idx + 3, len(self.waypoints))
        for i in range(self.wp_idx, check_end):
            wx, wy = self.waypoints[i]
            c = int((wx - info.origin.position.x) / info.resolution)
            r = int((wy - info.origin.position.y) / info.resolution)

            # Check the cell and its immediate neighbors
            for dr in range(-margin_cells, margin_cells + 1):
                for dc in range(-margin_cells, margin_cells + 1):
                    cr, cc = r + dr, c + dc
                    if (0 <= cr < info.height and 0 <= cc < info.width
                            and raw[cr, cc] > 50):
                        return True
        return False

    def _replan(self):
        """Replan from current position to stored goal."""
        if self.goal_xy is None or self.occupancy_grid is None:
            return

        # Hover while replanning
        self._send_position(self.px4_pos[0], self.px4_pos[1], -self.ALTITUDE)

        planner = SensorMapPlanner(self.occupancy_grid, safety_margin=self.SAFETY_MARGIN)
        path = planner.plan(self.gz_pos[0], self.gz_pos[1],
                            self.goal_xy[0], self.goal_xy[1])

        if path:
            self.waypoints = path
            self.wp_idx = 0
            self._publish_rviz_path(path)
            print(f"  ✅ Replanned — {len(path)} waypoints. Resuming!")
        else:
            print("  ❌ Replan failed — hovering. Try a new goal.")
            self.waypoints = []
            self.goal_xy = None
            self.state = 'READY'

    # ── Takeoff ───────────────────────────────────────────────────────────────

    def _takeoff(self):
        print("  🚀 Takeoff sequence starting...")

        hover_z = -self.ALTITUDE
        print("  ⏳ Sending warmup heartbeats...")
        for _ in range(30):
            self._heartbeat()
            self._send_position(self.px4_pos[0], self.px4_pos[1], hover_z)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        print("  ⚙️  OFFBOARD mode...")
        self._send_command(176, param1=1.0, param2=6.0)
        time.sleep(0.5)

        for _ in range(10):
            self._heartbeat()
            self._send_position(self.px4_pos[0], self.px4_pos[1], hover_z)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

        print("  🔋 Arming motors...")
        self._send_command(400, param1=1.0)
        time.sleep(0.5)

        self.state = 'EXPLORING'
        self.scan_wp_idx = 0
        self.scan_rotating = False
        print(f"\n  ✅ Airborne at {self.ALTITUDE}m!")
        print(f"  🔄 Starting room exploration ({len(SCAN_WAYPOINTS)} scan positions)...\n")

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def _send_position(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _send_command(self, cmd_id, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = cmd_id
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def _publish_rviz_path(self, coords):
        """Draw the planned path as a cyan line in RViz."""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for x, y in coords:
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.position.z = self.ALTITUDE
            msg.poses.append(ps)
        self.path_pub.publish(msg)


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = AutonomousNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n  🛬 Landing...")
        node._send_command(21)  # MAV_CMD_NAV_LAND
        time.sleep(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("  ✅ Navigator stopped.")


if __name__ == '__main__':
    main()
