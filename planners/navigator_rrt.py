#!/usr/bin/env python3
"""
=============================================================================
 AUTONOMOUS RRT NAVIGATOR — Rapidly-exploring Random Tree Path Planning
 File: navigator_rrt.py
=============================================================================

 Uses the same sensor-based pipeline as autonomous_navigator.py (A*),
 but replaces the A* planner with an RRT planner.

 RRT Algorithm:
   1. Start a tree from the drone's current position
   2. Randomly sample a point in free space
   3. Find the nearest tree node to that random sample
   4. Extend from that node toward the sample by a fixed step size
   5. If the new point is collision-free, add it to the tree
   6. Repeat until a node reaches close to the goal
   7. Trace back through the tree to reconstruct the path

 Advantages over A*:
   - Very fast to find *a* path (doesn't need to explore the whole grid)
   - Naturally explores large open spaces efficiently
   - Works well in high-dimensional configuration spaces

 Disadvantages:
   - NOT optimal — paths are jagged and usually longer than shortest
   - Randomness means different runs produce different paths
   - Can struggle in narrow passages (doorways) without goal bias

 Launch:
   ~/Desktop/Drone_IP/launch_sim.sh --auto --algo rrt

=============================================================================
"""

import math
import time
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
    ( 0.0,  0.0),
    ( 0.0,  3.0),
    ( 0.0, -3.0),
    (-3.0,  2.0),
    (-6.0,  3.5),
    (-6.0, -2.0),
    (-7.5,  0.0),
    (-1.0,  2.0),
    ( 1.0, -2.0),
    ( 3.0, -2.0),
    ( 6.0,  2.5),
    ( 6.0, -2.5),
    ( 7.5,  0.0),
]


# ─────────────────────────────────────────────────────────────────────────────
#  RRT PLANNER — Rapidly-exploring Random Tree on OctoMap's /projected_map
# ─────────────────────────────────────────────────────────────────────────────
class RRTPlanner:
    """
    Rapidly-exploring Random Tree (RRT) path planner.

    How it works:
      1. Parse the OccupancyGrid into a 2D obstacle grid (same as A*)
      2. Inflate obstacles by safety_margin
      3. Grow a tree from start by sampling random free-space points
      4. Extend the nearest tree node toward each sample by step_size
      5. Stop when a node gets within goal_radius of the goal
      6. Trace back and simplify the path
    """

    def __init__(self, occupancy_grid_msg, safety_margin=0.25,
                 max_iterations=8000, step_size=0.4, goal_radius=0.5,
                 goal_bias=0.10):
        info = occupancy_grid_msg.info
        self.resolution = info.resolution
        self.origin_x = info.origin.position.x
        self.origin_y = info.origin.position.y
        self.width = info.width
        self.height = info.height

        self.max_iterations = max_iterations
        self.step_size = step_size       # metres per tree extension
        self.goal_radius = goal_radius   # close enough to goal
        self.goal_bias = goal_bias       # 10% of samples aim directly at goal

        # Convert 1D occupancy data to 2D numpy grid
        raw = np.array(occupancy_grid_msg.data).reshape((self.height, self.width))
        self.grid = np.where(raw > 50, 1, 0).astype(np.uint8)

        # Inflate obstacles for safety
        self._inflate(safety_margin)

        # World bounds (for random sampling)
        self.x_min = self.origin_x
        self.x_max = self.origin_x + self.width * self.resolution
        self.y_min = self.origin_y
        self.y_max = self.origin_y + self.height * self.resolution

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
        Samples points along the line at half the grid resolution.
        """
        dist = math.hypot(x2 - x1, y2 - y1)
        if dist < 1e-6:
            return self._is_world_free(x1, y1)

        # Check points along the line at fine intervals
        check_step = self.resolution * 0.5
        n_checks = max(2, int(dist / check_step))

        for i in range(n_checks + 1):
            t = i / n_checks
            px = x1 + t * (x2 - x1)
            py = y1 + t * (y2 - y1)
            if not self._is_world_free(px, py):
                return False
        return True

    def _random_sample(self, gx, gy):
        """
        Sample a random point. With probability goal_bias, return the
        goal directly (this speeds up convergence significantly).
        """
        if random.random() < self.goal_bias:
            return gx, gy
        return random.uniform(self.x_min, self.x_max), random.uniform(self.y_min, self.y_max)

    def _nearest_node(self, nodes, x, y):
        """Find the index of the nearest tree node to point (x, y)."""
        pts = np.array([(n[0], n[1]) for n in nodes])
        dists = (pts[:, 0] - x)**2 + (pts[:, 1] - y)**2
        return int(np.argmin(dists))

    def _steer(self, from_x, from_y, to_x, to_y):
        """
        Move from (from_x, from_y) toward (to_x, to_y) by step_size.
        If the target is closer than step_size, go directly there.
        """
        dist = math.hypot(to_x - from_x, to_y - from_y)
        if dist <= self.step_size:
            return to_x, to_y

        ratio = self.step_size / dist
        new_x = from_x + ratio * (to_x - from_x)
        new_y = from_y + ratio * (to_y - from_y)
        return new_x, new_y

    def _nearest_free_world(self, x, y):
        """If a point is in an obstacle, BFS outward to find nearest free cell."""
        r, c = self.world_to_grid(x, y)
        if self.is_free(r, c):
            return x, y
        visited = {(r, c)}
        queue = [(r, c)]
        while queue:
            next_queue = []
            for cr, cc in queue:
                for dr in range(-1, 2):
                    for dc in range(-1, 2):
                        nb = (cr + dr, cc + dc)
                        if nb not in visited:
                            if self.is_free(*nb):
                                return self.grid_to_world(*nb)
                            visited.add(nb)
                            next_queue.append(nb)
            queue = next_queue
        return None

    def _simplify_path(self, path):
        """
        Shortcut the path by removing intermediate nodes when a direct
        collision-free line exists between non-adjacent nodes.
        This is more aggressive than collinear removal — it produces
        much shorter, smoother paths from RRT's jagged output.
        """
        if len(path) <= 2:
            return path

        simplified = [path[0]]
        i = 0

        while i < len(path) - 1:
            # Try to skip as far ahead as possible
            farthest = i + 1
            for j in range(len(path) - 1, i, -1):
                if self._collision_free_line(path[i][0], path[i][1],
                                             path[j][0], path[j][1]):
                    farthest = j
                    break
            simplified.append(path[farthest])
            i = farthest

        return simplified

    def plan(self, sx, sy, gx, gy):
        """
        RRT path planning from (sx, sy) to (gx, gy) in world coordinates.

        Steps:
          1. Validate start/goal are in free space
          2. Grow the RRT tree from start
          3. Each iteration: random sample -> nearest node -> steer -> add
          4. Stop when close to goal
          5. Trace back and simplify
        """
        t0 = time.time()

        # Validate start
        result = self._nearest_free_world(sx, sy)
        if result is None:
            print("  Cannot find free start cell!")
            return None
        sx, sy = result

        # Validate goal
        result = self._nearest_free_world(gx, gy)
        if result is None:
            print("  Cannot find free goal cell!")
            return None
        gx, gy = result

        print(f"  RRT: Growing tree (max {self.max_iterations} iterations, "
              f"step={self.step_size}m, goal_bias={self.goal_bias:.0%})...")

        # Tree: list of (x, y, parent_index)
        # Node 0 = start
        nodes = [(sx, sy, -1)]
        goal_node = None

        for iteration in range(self.max_iterations):
            # Step 1: Random sample (with goal bias)
            rand_x, rand_y = self._random_sample(gx, gy)

            # Step 2: Find nearest tree node
            nearest_idx = self._nearest_node(nodes, rand_x, rand_y)
            nx, ny, _ = nodes[nearest_idx]

            # Step 3: Steer toward the sample
            new_x, new_y = self._steer(nx, ny, rand_x, rand_y)

            # Step 4: Check if the path to the new node is collision-free
            if not self._collision_free_line(nx, ny, new_x, new_y):
                continue

            # Step 5: Add new node to tree
            new_idx = len(nodes)
            nodes.append((new_x, new_y, nearest_idx))

            # Step 6: Check if we reached the goal
            dist_to_goal = math.hypot(new_x - gx, new_y - gy)
            if dist_to_goal < self.goal_radius:
                # Connect directly to goal if possible
                if self._collision_free_line(new_x, new_y, gx, gy):
                    nodes.append((gx, gy, new_idx))
                    goal_node = len(nodes) - 1
                    print(f"  RRT: Goal reached in {iteration + 1} iterations, "
                          f"{len(nodes)} tree nodes")
                    break

            # Progress indicator every 2000 iterations
            if (iteration + 1) % 2000 == 0:
                print(f"     ... {iteration + 1} iterations, {len(nodes)} nodes, "
                      f"nearest to goal: {dist_to_goal:.2f}m")

        dt = (time.time() - t0) * 1000

        if goal_node is None:
            print(f"  RRT: No path found after {self.max_iterations} iterations ({dt:.0f}ms)")
            return None

        # Trace back from goal to start
        path = []
        idx = goal_node
        while idx != -1:
            path.append((nodes[idx][0], nodes[idx][1]))
            idx = nodes[idx][2]
        path.reverse()

        raw_len = len(path)

        # Simplify the jagged RRT path
        path = self._simplify_path(path)

        print(f"  RRT: {raw_len} raw waypoints -> {len(path)} simplified ({dt:.0f}ms)")
        return path


# ─────────────────────────────────────────────────────────────────────────────
#  AUTONOMOUS NAVIGATOR NODE (identical to A* version, just uses RRTPlanner)
# ─────────────────────────────────────────────────────────────────────────────
class AutonomousNavigator(Node):
    """
    State Machine:
      INIT       -> waiting for PX4 + Gazebo data
      TAKEOFF    -> arming and ascending to flight altitude
      EXPLORING  -> flying scan waypoints + rotating at each
      READY      -> map built, hovering, waiting for user goal
      NAVIGATING -> following RRT path to goal
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

        # Control timer: 20 Hz
        self.create_timer(0.05, self.control_loop)

        print("\n╔══════════════════════════════════════════════════════════╗")
        print("║   🌳 AUTONOMOUS RRT NAVIGATOR — SENSOR-BASED          ║")
        print("╠══════════════════════════════════════════════════════════╣")
        print("║  Phase 1: Takeoff -> 1.8m                              ║")
        print("║  Phase 2: Explore rooms (13 scan positions, 360 each)  ║")
        print("║  Phase 3: OctoMap builds 3D map from depth camera      ║")
        print("║  Phase 4: Click '2D Goal Pose' in RViz                 ║")
        print("║  Phase 5: RRT plans on REAL sensor map -> drone flies! ║")
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

        print(f"\n  Goal received: ({gx:.2f}, {gy:.2f})")
        print(f"   Drone is at: ({self.gz_pos[0]:.2f}, {self.gz_pos[1]:.2f})")
        print(f"   Planning with RRT (Rapidly-exploring Random Tree)...")

        grid_info = self.occupancy_grid.info
        total_cells = grid_info.width * grid_info.height
        occupied = sum(1 for v in self.occupancy_grid.data if v > 50)
        print(f"   Map: {grid_info.width}x{grid_info.height} cells, "
              f"resolution={grid_info.resolution:.2f}m, "
              f"occupied={occupied}/{total_cells}")

        planner = RRTPlanner(self.occupancy_grid, safety_margin=self.SAFETY_MARGIN,
                             max_iterations=8000, step_size=0.4,
                             goal_radius=0.5, goal_bias=0.10)
        path = planner.plan(self.gz_pos[0], self.gz_pos[1], gx, gy)

        if path:
            self.waypoints = path
            self.wp_idx = 0
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
            self.state = 'READY'
            return

        wx, wy = self.waypoints[self.wp_idx]
        dx_enu = wx - self.gz_pos[0]
        dy_enu = wy - self.gz_pos[1]
        tgt_px4_x = self.px4_pos[0] + dy_enu
        tgt_px4_y = self.px4_pos[1] + dx_enu
        tgt_px4_z = -self.ALTITUDE
        target_yaw = math.atan2(dx_enu, dy_enu)
        self._send_position(tgt_px4_x, tgt_px4_y, tgt_px4_z, yaw=target_yaw)

        dist = math.hypot(dx_enu, dy_enu)
        if dist < self.WP_REACH_DIST:
            self.wp_idx += 1
            if self.wp_idx < len(self.waypoints):
                print(f"  WP {self.wp_idx}/{len(self.waypoints)} — {dist:.2f}m")

    # Takeoff

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
        node._send_command(21)
        time.sleep(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("  Navigator stopped.")


if __name__ == '__main__':
    main()
