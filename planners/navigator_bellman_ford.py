#!/usr/bin/env python3
"""
=============================================================================
 AUTONOMOUS BELLMAN FORD NAVIGATOR — Edge Relaxation Path Planning
 File: navigator_bellman_ford.py
=============================================================================

 Uses the same sensor-based pipeline as all other navigators,
 but replaces the planner with Bellman Ford algorithm.

 Bellman Ford Algorithm:
   1. Build an edge list from the occupancy grid (each free cell -> 8 neighbors)
   2. Initialize all distances to infinity, except start = 0
   3. Repeat V-1 times: relax every edge (if d[u] + w < d[v], update d[v])
   4. After V-1 iterations, shortest paths are guaranteed

 Key Properties:
   - Can handle NEGATIVE edge weights (unlike Dijkstra/A*)
   - Produces the SAME path as Dijkstra on positive-weight grids
   - MUCH slower: O(V * E) vs O(E * log V) for Dijkstra
   - This makes it a good "why we don't use this" comparison

 Why study it:
   Bellman Ford demonstrates the cost of not using a priority queue.
   By comparing its runtime to Dijkstra and A*, we clearly see the
   value of heuristics and efficient data structures.

 Launch:
   ~/Desktop/Drone_IP/launch_sim.sh --auto --algo bellman

=============================================================================
"""

import math
import time
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
#  EXPLORATION WAYPOINTS (same as all navigators)
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
#  BELLMAN FORD PLANNER — Edge Relaxation on OctoMap's /projected_map
# ─────────────────────────────────────────────────────────────────────────────
class BellmanFordPlanner:
    """
    Bellman Ford path planner — edge relaxation approach.

    Unlike Dijkstra (priority queue) or A* (priority queue + heuristic),
    Bellman Ford simply iterates over ALL edges repeatedly until no
    more improvements can be made. This is simple but very slow.

    Optimization: We use early termination — if an iteration produces
    no updates, the algorithm has converged and we stop early.
    We also limit the search to a bounding box around start/goal
    to keep the edge count manageable.
    """

    def __init__(self, occupancy_grid_msg, safety_margin=0.25):
        info = occupancy_grid_msg.info
        self.resolution = info.resolution
        self.origin_x = info.origin.position.x
        self.origin_y = info.origin.position.y
        self.width = info.width
        self.height = info.height

        raw = np.array(occupancy_grid_msg.data).reshape((self.height, self.width))
        self.grid = np.where(raw > 50, 1, 0).astype(np.uint8)
        self._inflate(safety_margin)

        self.nodes_explored = 0
        self.edges_relaxed = 0

    def _inflate(self, margin_m):
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

    def _build_edges(self, r_lo, r_hi, c_lo, c_hi):
        """
        Build edge list from the grid within the given bounding box.
        Each free cell connects to up to 8 neighbors.
        Edge weight: 1.0 for cardinal, 1.414 for diagonal.
        """
        edges = []
        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0),
                (1, 1), (1, -1), (-1, 1), (-1, -1)]

        for r in range(r_lo, r_hi):
            for c in range(c_lo, c_hi):
                if not self.is_free(r, c):
                    continue
                for dr, dc in dirs:
                    nr, nc = r + dr, c + dc
                    if nr < r_lo or nr >= r_hi or nc < c_lo or nc >= c_hi:
                        continue
                    if not self.is_free(nr, nc):
                        continue
                    w = 1.414 if (dr != 0 and dc != 0) else 1.0
                    edges.append(((r, c), (nr, nc), w))

        return edges

    def plan(self, sx, sy, gx, gy):
        """
        Bellman Ford from (sx, sy) to (gx, gy).

        Uses a bounded search region around start/goal for performance,
        plus early termination when no more relaxations occur.
        """
        t0 = time.time()
        self.nodes_explored = 0
        self.edges_relaxed = 0

        start = self.world_to_grid(sx, sy)
        goal = self.world_to_grid(gx, gy)

        start = (
            max(0, min(self.height - 1, start[0])),
            max(0, min(self.width - 1, start[1]))
        )
        goal = (
            max(0, min(self.height - 1, goal[0])),
            max(0, min(self.width - 1, goal[1]))
        )

        if not self.is_free(*start):
            print(f"  Start ({sx:.2f},{sy:.2f}) is in obstacle — nudging")
            start = self._nearest_free(start)
            if start is None:
                print("  Cannot find free start cell!")
                return None

        if not self.is_free(*goal):
            print(f"  Goal ({gx:.2f},{gy:.2f}) is in obstacle — nudging")
            goal = self._nearest_free(goal)
            if goal is None:
                print("  Cannot find free goal cell!")
                return None

        # Define bounding box with padding around start/goal
        padding = 30  # cells of extra room around the start/goal box
        r_lo = max(0, min(start[0], goal[0]) - padding)
        r_hi = min(self.height, max(start[0], goal[0]) + padding)
        c_lo = max(0, min(start[1], goal[1]) - padding)
        c_hi = min(self.width, max(start[1], goal[1]) + padding)

        print(f"  Bellman Ford: Building edge list in region "
              f"[{r_lo}:{r_hi}, {c_lo}:{c_hi}]...")

        edges = self._build_edges(r_lo, r_hi, c_lo, c_hi)
        print(f"  Bellman Ford: {len(edges)} edges, running relaxation...")

        # Collect all vertices
        vertices = set()
        for u, v, w in edges:
            vertices.add(u)
            vertices.add(v)
        num_v = len(vertices)
        self.nodes_explored = num_v

        # Initialize distances
        dist = {v: float('inf') for v in vertices}
        prev = {v: None for v in vertices}
        dist[start] = 0.0

        # Bellman Ford: relax all edges up to V-1 times
        max_iters = min(num_v - 1, 500)  # cap iterations for sanity
        for iteration in range(max_iters):
            updated = False
            for u, v, w in edges:
                if dist[u] + w < dist[v]:
                    dist[v] = dist[u] + w
                    prev[v] = u
                    updated = True
                    self.edges_relaxed += 1

            # Early termination: if no updates this iteration, we're done
            if not updated:
                print(f"  Bellman Ford: Converged early at iteration "
                      f"{iteration + 1}/{max_iters}")
                break

            if (iteration + 1) % 50 == 0:
                print(f"     ... iteration {iteration + 1}/{max_iters}, "
                      f"{self.edges_relaxed} relaxations")

        dt = (time.time() - t0) * 1000

        if prev[goal] is None and goal != start:
            print(f"  Bellman Ford: No path found! ({dt:.0f}ms)")
            return None

        # Reconstruct path
        path = []
        cur = goal
        while cur is not None:
            path.append(self.grid_to_world(*cur))
            cur = prev[cur]
        path.reverse()
        path = self._simplify(path)

        print(f"  Bellman Ford: Path found — {len(path)} waypoints "
              f"({dt:.0f}ms, {self.edges_relaxed} edge relaxations)")
        return path

    def _nearest_free(self, cell):
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
#  AUTONOMOUS NAVIGATOR NODE (uses BellmanFordPlanner)
# ─────────────────────────────────────────────────────────────────────────────
class AutonomousNavigator(Node):

    def __init__(self):
        super().__init__('autonomous_navigator')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

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

        self.path_pub = self.create_publisher(Path, '/plan', 10)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        self.gz_pos = None
        self.px4_pos = None
        self.occupancy_grid = None

        self.state = 'INIT'
        self.waypoints = []
        self.wp_idx = 0
        self.goal_xy = None

        self.scan_wp_idx = 0
        self.scan_yaw = 0.0
        self.scan_rotating = False

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

        self.create_timer(0.05, self.control_loop)

        print("\n╔══════════════════════════════════════════════════════════╗")
        print("║   🟠 AUTONOMOUS BELLMAN FORD NAVIGATOR                 ║")
        print("╠══════════════════════════════════════════════════════════╣")
        print("║  Edge relaxation — iterates ALL edges V-1 times        ║")
        print("║  Same path as Dijkstra/A*, but significantly slower    ║")
        print("╚══════════════════════════════════════════════════════════╝\n")

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
            print("Warning: Drone is not ready yet!")
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
        print(f"   Planning with Bellman Ford (edge relaxation)...")

        grid_info = self.occupancy_grid.info
        total_cells = grid_info.width * grid_info.height
        occupied = sum(1 for v in self.occupancy_grid.data if v > 50)
        print(f"   Map: {grid_info.width}x{grid_info.height} cells, "
              f"resolution={grid_info.resolution:.2f}m, "
              f"occupied={occupied}/{total_cells}")

        planner = BellmanFordPlanner(self.occupancy_grid,
                                     safety_margin=self.SAFETY_MARGIN)
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
        planner = BellmanFordPlanner(self.occupancy_grid, safety_margin=self.SAFETY_MARGIN)
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
