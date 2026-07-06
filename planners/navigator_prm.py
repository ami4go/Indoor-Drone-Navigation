#!/usr/bin/env python3
"""
=============================================================================
 AUTONOMOUS PRM NAVIGATOR — Probabilistic Roadmap Path Planning
 File: navigator_prm.py
=============================================================================

 Uses the same sensor-based pipeline as autonomous_navigator.py (A*),
 but replaces the A* planner with a Probabilistic Roadmap (PRM) planner.

 PRM Algorithm:
   1. Randomly scatter N sample points in free space on the OctoMap grid
   2. Connect each sample to its K nearest neighbors (if line-of-sight clear)
   3. Connect start and goal to the roadmap
   4. Run Dijkstra on the roadmap graph to find the shortest path

 Advantages over A*:
   - Multi-query: once the roadmap is built, different start/goal pairs
     can be queried instantly without rebuilding
   - Produces smoother paths with straight-line segments (not grid-locked)

 Disadvantages:
   - Not guaranteed to find the shortest path (depends on sample density)
   - Can struggle with narrow passages (doorways) if samples don't land there
   - Roadmap construction takes time upfront

 Launch:
   ~/Desktop/Drone_IP/launch_sim.sh --auto --algo prm

=============================================================================
"""

import math
import time
import heapq
import random
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
#  EXPLORATION WAYPOINTS (same as A* navigator)
# ─────────────────────────────────────────────────────────────────────────────
SCAN_WAYPOINTS = [
    ( 0.0,  0.0),   #  1. Spawn point
    ( 0.0,  3.0),   #  2. Near bed
    ( 0.0, -3.0),   #  3. Near wardrobe/dresser
    (-3.0,  2.0),   #  4. Doorway 1
    (-6.0,  3.5),   #  5. Near sofa + armchair
    (-6.0, -2.0),   #  6. Near TV stand
    (-7.5,  0.0),   #  7. West wall area
    (-1.0,  2.0),   #  8. Back through bedroom
    ( 1.0, -2.0),   #  9. Approach door 2
    ( 3.0, -2.0),   # 10. Doorway 2
    ( 6.0,  2.5),   # 11. Near desk + chair
    ( 6.0, -2.5),   # 12. Near filing cabinet
    ( 7.5,  0.0),   # 13. Near bookshelf + east wall
]


# ─────────────────────────────────────────────────────────────────────────────
#  PRM PLANNER — Probabilistic Roadmap on OctoMap's /projected_map
# ─────────────────────────────────────────────────────────────────────────────
class PRMPlanner:
    """
    Probabilistic Roadmap (PRM) path planner.

    How it works:
      1. Parse the OccupancyGrid into a 2D obstacle grid (same as A*)
      2. Inflate obstacles by safety_margin (same as A*)
      3. Randomly sample N points in free space
      4. For each sample, connect to K nearest neighbors via collision-free lines
      5. Connect start & goal to the roadmap
      6. Run Dijkstra on the resulting graph
    """

    def __init__(self, occupancy_grid_msg, safety_margin=0.25,
                 num_samples=600, k_neighbors=10):
        info = occupancy_grid_msg.info
        self.resolution = info.resolution
        self.origin_x = info.origin.position.x
        self.origin_y = info.origin.position.y
        self.width = info.width
        self.height = info.height
        self.num_samples = num_samples
        self.k_neighbors = k_neighbors

        # Convert 1D occupancy data to 2D numpy grid
        raw = np.array(occupancy_grid_msg.data).reshape((self.height, self.width))
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

    def _is_world_free(self, x, y):
        """Check if a world coordinate is in free space."""
        r, c = self.world_to_grid(x, y)
        return self.is_free(r, c)

    def _collision_free_line(self, x1, y1, x2, y2):
        """
        Check if a straight line between two world points is obstacle-free.
        Uses Bresenham's line algorithm on the grid.
        """
        r1, c1 = self.world_to_grid(x1, y1)
        r2, c2 = self.world_to_grid(x2, y2)

        dr = abs(r2 - r1)
        dc = abs(c2 - c1)
        sr = 1 if r2 > r1 else -1
        sc = 1 if c2 > c1 else -1

        r, c = r1, c1
        err = dr - dc

        while True:
            if not self.is_free(r, c):
                return False
            if r == r2 and c == c2:
                break
            e2 = 2 * err
            if e2 > -dc:
                err -= dc
                r += sr
            if e2 < dr:
                err += dr
                c += sc

        return True

    def _sample_free_points(self):
        """Randomly scatter points in free space on the map."""
        points = []
        # Compute world bounds from grid
        x_min = self.origin_x
        x_max = self.origin_x + self.width * self.resolution
        y_min = self.origin_y
        y_max = self.origin_y + self.height * self.resolution

        attempts = 0
        max_attempts = self.num_samples * 20  # avoid infinite loop

        while len(points) < self.num_samples and attempts < max_attempts:
            x = random.uniform(x_min, x_max)
            y = random.uniform(y_min, y_max)
            if self._is_world_free(x, y):
                points.append((x, y))
            attempts += 1

        # Seed extra samples near doorways to handle narrow passages
        # Door 1: X=-3, Y=+2 (2.5m wide)
        # Door 2: X=+3, Y=-2 (2.5m wide)
        doorway_seeds = [
            (-3.0, 2.0), (-3.0, 1.5), (-3.0, 2.5),
            (-3.5, 2.0), (-2.5, 2.0),
            ( 3.0,-2.0), ( 3.0,-1.5), ( 3.0,-2.5),
            ( 3.5,-2.0), ( 2.5,-2.0),
        ]
        for dx, dy in doorway_seeds:
            if self._is_world_free(dx, dy):
                points.append((dx, dy))

        return points

    def _build_roadmap(self, points):
        """
        Connect each point to its K nearest neighbors if the line is
        collision-free. Returns an adjacency list: graph[i] = [(j, dist), ...]
        """
        n = len(points)
        pts = np.array(points)
        graph = {i: [] for i in range(n)}

        for i in range(n):
            # Compute distances to all other points
            diffs = pts - pts[i]
            dists = np.sqrt(diffs[:, 0]**2 + diffs[:, 1]**2)

            # Get K nearest (skip index 0 which is self with dist=0)
            nearest_indices = np.argsort(dists)[1:self.k_neighbors + 1]

            for j in nearest_indices:
                d = dists[j]
                if d > 3.0:  # max connection radius (metres)
                    continue
                if self._collision_free_line(pts[i][0], pts[i][1],
                                             pts[j][0], pts[j][1]):
                    graph[i].append((j, d))
                    graph[j].append((i, d))  # undirected

        return graph

    def _dijkstra(self, graph, start_idx, goal_idx):
        """Dijkstra's algorithm on the roadmap graph."""
        dist = {start_idx: 0.0}
        came_from = {start_idx: None}
        pq = [(0.0, start_idx)]

        while pq:
            d, u = heapq.heappop(pq)
            if u == goal_idx:
                break
            if d > dist.get(u, float('inf')):
                continue
            for v, w in graph.get(u, []):
                nd = d + w
                if nd < dist.get(v, float('inf')):
                    dist[v] = nd
                    came_from[v] = u
                    heapq.heappush(pq, (nd, v))

        if goal_idx not in came_from:
            return None

        path = []
        cur = goal_idx
        while cur is not None:
            path.append(cur)
            cur = came_from[cur]
        path.reverse()
        return path

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

    def plan(self, sx, sy, gx, gy):
        """
        PRM path planning from (sx, sy) to (gx, gy) in world coordinates.

        Steps:
          1. Sample random points in free space
          2. Build roadmap (connect neighbors with collision-free lines)
          3. Add start and goal to the roadmap
          4. Run Dijkstra to find shortest path on the graph
        """
        t0 = time.time()

        # Validate start and goal are in free space
        sr, sc = self.world_to_grid(sx, sy)
        if not self.is_free(sr, sc):
            print(f"  ⚠️  Start ({sx:.2f},{sy:.2f}) is in obstacle — nudging")
            free = self._nearest_free((sr, sc))
            if free is None:
                print("  ❌  Cannot find free start cell!")
                return None
            sx, sy = self.grid_to_world(*free)

        gr, gc = self.world_to_grid(gx, gy)
        if not self.is_free(gr, gc):
            print(f"  ⚠️  Goal ({gx:.2f},{gy:.2f}) is in obstacle — nudging")
            free = self._nearest_free((gr, gc))
            if free is None:
                print("  ❌  Cannot find free goal cell!")
                return None
            gx, gy = self.grid_to_world(*free)

        # Step 1: Sample free points
        print(f"  📊 PRM: Sampling {self.num_samples} random points...")
        points = self._sample_free_points()
        print(f"     Got {len(points)} free samples")

        # Step 2: Add start and goal as special nodes
        start_idx = len(points)
        points.append((sx, sy))
        goal_idx = len(points)
        points.append((gx, gy))

        # Step 3: Build roadmap
        print(f"  🔗 PRM: Building roadmap (K={self.k_neighbors} neighbors)...")
        graph = self._build_roadmap(points)

        # Step 4: Run Dijkstra
        print(f"  🔍 PRM: Running Dijkstra on roadmap...")
        idx_path = self._dijkstra(graph, start_idx, goal_idx)

        dt = (time.time() - t0) * 1000
        print(f"  ⏱️  PRM planning took {dt:.1f} ms")

        if idx_path is None:
            print("  ❌  PRM: No path found! Try increasing samples.")
            return None

        # Convert index path to world coordinates
        path = [points[i] for i in idx_path]
        return path


# ─────────────────────────────────────────────────────────────────────────────
#  AUTONOMOUS NAVIGATOR NODE (identical to A* version, just uses PRMPlanner)
# ─────────────────────────────────────────────────────────────────────────────
class AutonomousNavigator(Node):
    """
    State Machine:
      INIT       -> waiting for PX4 + Gazebo data
      TAKEOFF    -> arming and ascending to flight altitude
      EXPLORING  -> flying scan waypoints + rotating at each
      READY      -> map built, hovering, waiting for user goal
      NAVIGATING -> following PRM path to goal
    """

    def __init__(self):
        super().__init__('autonomous_navigator')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # Subscribers
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

        # Publishers
        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # State
        self.gz_pos = None
        self.px4_pos = None
        self.occupancy_grid = None

        self.state = 'INIT'
        self.waypoints = []
        self.wp_idx = 0

        # Exploration state
        self.scan_wp_idx = 0
        self.scan_yaw = 0.0
        self.scan_rotating = False

        # Parameters
        self.ALTITUDE = 1.8
        self.WP_REACH_DIST = 0.4
        self.SCAN_WP_REACH = 0.5
        self.YAW_STEP = 0.04
        self.SAFETY_MARGIN = 0.25

        # Stability: velocity clamping + yaw smoothing
        self.MAX_NAV_DELTA = 1.5
        self.current_yaw = 0.0
        self.YAW_SMOOTH = 0.15

        # Obstacle avoidance
        self.AVOIDANCE_CHECK_INTERVAL = 40
        self.avoidance_tick = 0

        # Control timer: 20 Hz
        self.create_timer(0.05, self.control_loop)

        print("\n╔══════════════════════════════════════════════════════════╗")
        print("║   🗺️  AUTONOMOUS PRM NAVIGATOR — SENSOR-BASED         ║")
        print("╠══════════════════════════════════════════════════════════╣")
        print("║  Phase 1: Takeoff -> 1.8m                              ║")
        print("║  Phase 2: Explore rooms (13 scan positions, 360 each)  ║")
        print("║  Phase 3: OctoMap builds 3D map from depth camera      ║")
        print("║  Phase 4: Click '2D Goal Pose' in RViz                 ║")
        print("║  Phase 5: PRM plans on REAL sensor map -> drone flies! ║")
        print("╚══════════════════════════════════════════════════════════╝\n")

    # Callbacks

    def gz_pose_cb(self, msg):
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
        self.occupancy_grid = msg

    def goal_cb(self, msg):
        if self.state != 'READY':
            print("Warning: Drone is not ready yet! Wait for exploration to complete.")
            return

        if self.occupancy_grid is None:
            print("Warning: No sensor map available!")
            return

        if self.gz_pos is None:
            print("Warning: No Gazebo position received.")
            return

        gx = msg.pose.position.x
        gy = msg.pose.position.y

        print(f"\n🎯  Goal received: ({gx:.2f}, {gy:.2f})")
        print(f"   Drone is at: ({self.gz_pos[0]:.2f}, {self.gz_pos[1]:.2f})")
        print(f"   Planning with PRM (Probabilistic Roadmap)...")

        grid_info = self.occupancy_grid.info
        total_cells = grid_info.width * grid_info.height
        occupied = sum(1 for v in self.occupancy_grid.data if v > 50)
        print(f"   Map: {grid_info.width}x{grid_info.height} cells, "
              f"resolution={grid_info.resolution:.2f}m, "
              f"occupied={occupied}/{total_cells}")

        planner = PRMPlanner(self.occupancy_grid, safety_margin=self.SAFETY_MARGIN,
                             num_samples=600, k_neighbors=10)
        path = planner.plan(self.gz_pos[0], self.gz_pos[1], gx, gy)

        if path:
            self.waypoints = path
            self.wp_idx = 0
            self.goal_xy = (gx, gy)
            self.state = 'NAVIGATING'
            self._publish_rviz_path(path)
            print(f"   Path found with {len(path)} waypoints. Flying!")
        else:
            print("   No path found. Try a different target.")

    # Control Loop

    def control_loop(self):
        if self.px4_pos is None or self.gz_pos is None:
            return

        if self.state == 'INIT':
            self._takeoff()
            return

        self._heartbeat()

        if self.state == 'EXPLORING':
            self._explore_step()
        elif self.state == 'NAVIGATING':
            self._navigate_step()
        elif self.state == 'READY':
            self._send_position(self.px4_pos[0], self.px4_pos[1], -self.ALTITUDE)

    # Exploration Phase

    def _explore_step(self):
        if self.scan_wp_idx >= len(SCAN_WAYPOINTS):
            self.state = 'READY'
            has_map = self.occupancy_grid is not None
            if has_map:
                info = self.occupancy_grid.info
                occupied = sum(1 for v in self.occupancy_grid.data if v > 50)
                print(f"\n  EXPLORATION COMPLETE!")
                print(f"     Sensor map: {info.width}x{info.height} cells, "
                      f"{occupied} obstacles detected")
            else:
                print(f"\n  Exploration done but no map received from OctoMap!")
            print(f"     Click '2D Goal Pose' in RViz to set a target.\n")
            return

        target_x, target_y = SCAN_WAYPOINTS[self.scan_wp_idx]

        if not self.scan_rotating:
            dx_enu = target_x - self.gz_pos[0]
            dy_enu = target_y - self.gz_pos[1]
            dist = math.hypot(dx_enu, dy_enu)
            fly_yaw = math.atan2(dx_enu, dy_enu)
            tgt_px4_x = self.px4_pos[0] + dy_enu
            tgt_px4_y = self.px4_pos[1] + dx_enu
            self._send_position(tgt_px4_x, tgt_px4_y, -self.ALTITUDE, yaw=fly_yaw)

            if dist < self.SCAN_WP_REACH:
                self.scan_rotating = True
                self.scan_yaw = 0.0
                print(f"  Scan position {self.scan_wp_idx + 1}/{len(SCAN_WAYPOINTS)} "
                      f"reached at ({target_x:.1f}, {target_y:.1f}) — rotating 360...")
        else:
            self.scan_yaw += self.YAW_STEP
            self._send_position(
                self.px4_pos[0], self.px4_pos[1], -self.ALTITUDE,
                yaw=self.scan_yaw
            )
            if self.scan_yaw >= 2 * math.pi:
                self.scan_rotating = False
                self.scan_wp_idx += 1

    # Navigation Phase

    def _navigate_step(self):
        if self.wp_idx >= len(self.waypoints):
            print("  DESTINATION REACHED! Set a new goal in RViz.")
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
                print("  \u26a0\ufe0f  Obstacle detected! Replanning...")
                self._replan()
                return

        wx, wy = self.waypoints[self.wp_idx]
        dx_enu = wx - self.gz_pos[0]
        dy_enu = wy - self.gz_pos[1]
        dist = math.hypot(dx_enu, dy_enu)

        if dist > self.MAX_NAV_DELTA:
            scale = self.MAX_NAV_DELTA / dist
            dx_enu *= scale
            dy_enu *= scale

        tgt_px4_x = self.px4_pos[0] + dy_enu
        tgt_px4_y = self.px4_pos[1] + dx_enu
        tgt_px4_z = -self.ALTITUDE

        target_yaw = math.atan2(wx - self.gz_pos[0], wy - self.gz_pos[1])
        yaw_diff = target_yaw - self.current_yaw
        yaw_diff = (yaw_diff + math.pi) % (2 * math.pi) - math.pi
        self.current_yaw += self.YAW_SMOOTH * yaw_diff
        self._send_position(tgt_px4_x, tgt_px4_y, tgt_px4_z, yaw=self.current_yaw)

        if dist < self.WP_REACH_DIST:
            self.wp_idx += 1
            if self.wp_idx < len(self.waypoints):
                print(f"  WP {self.wp_idx}/{len(self.waypoints)} \u2014 {dist:.2f}m")

    def _path_blocked(self):
        if self.occupancy_grid is None:
            return False
        info = self.occupancy_grid.info
        raw = np.array(self.occupancy_grid.data).reshape((info.height, info.width))
        margin_cells = int(math.ceil(self.SAFETY_MARGIN / info.resolution))
        check_end = min(self.wp_idx + 3, len(self.waypoints))
        for i in range(self.wp_idx, check_end):
            wx, wy = self.waypoints[i]
            c = int((wx - info.origin.position.x) / info.resolution)
            r = int((wy - info.origin.position.y) / info.resolution)
            for dr in range(-margin_cells, margin_cells + 1):
                for dc in range(-margin_cells, margin_cells + 1):
                    cr, cc = r + dr, c + dc
                    if (0 <= cr < info.height and 0 <= cc < info.width
                            and raw[cr, cc] > 50):
                        return True
        return False

    def _replan(self):
        if self.goal_xy is None or self.occupancy_grid is None:
            return
        self._send_position(self.px4_pos[0], self.px4_pos[1], -self.ALTITUDE)
        planner = PRMPlanner(self.occupancy_grid, safety_margin=self.SAFETY_MARGIN)
        path = planner.plan(self.gz_pos[0], self.gz_pos[1],
                            self.goal_xy[0], self.goal_xy[1])
        if path:
            self.waypoints = path
            self.wp_idx = 0
            self._publish_rviz_path(path)
            print(f"  \u2705 Replanned \u2014 {len(path)} waypoints. Resuming!")
        else:
            print("  \u274c Replan failed. Try a new goal.")
            self.waypoints = []
            self.goal_xy = None
            self.state = 'READY'

    def _takeoff(self):
        print("  Takeoff sequence starting...")
        hover_z = -self.ALTITUDE
        print("  Sending warmup heartbeats...")
        for _ in range(30):
            self._heartbeat()
            self._send_position(self.px4_pos[0], self.px4_pos[1], hover_z)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        print("  OFFBOARD mode...")
        self._send_command(176, param1=1.0, param2=6.0)
        time.sleep(0.5)

        for _ in range(10):
            self._heartbeat()
            self._send_position(self.px4_pos[0], self.px4_pos[1], hover_z)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

        print("  Arming motors...")
        self._send_command(400, param1=1.0)
        time.sleep(0.5)

        self.state = 'EXPLORING'
        self.scan_wp_idx = 0
        self.scan_rotating = False
        print(f"\n  Airborne at {self.ALTITUDE}m!")
        print(f"  Starting room exploration ({len(SCAN_WAYPOINTS)} scan positions)...\n")

    # Helpers

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
        print("\n  Landing...")
        node._send_command(21)  # MAV_CMD_NAV_LAND
        time.sleep(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("  Navigator stopped.")


if __name__ == '__main__':
    main()
