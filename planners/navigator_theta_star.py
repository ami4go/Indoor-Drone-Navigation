#!/usr/bin/env python3
"""
=============================================================================
 AUTONOMOUS THETA* NAVIGATOR — Any-Angle Path Planning
 File: navigator_theta_star.py
=============================================================================

 Uses the same sensor-based pipeline as autonomous_navigator.py (A*),
 but replaces A* with Theta* — an any-angle path planning algorithm.

 Theta* Algorithm:
   Same as A* but with one key addition: when expanding a neighbor,
   check if there is LINE-OF-SIGHT from the neighbor's GRANDPARENT.
   If yes, connect directly (skip the parent), creating a shorter
   straight-line shortcut at any angle, not just 45/90 degrees.

 Advantages over A*:
   - Produces smoother, shorter paths (true geometric shortest path)
   - Paths can go at any angle, not locked to 8 grid directions
   - Same optimality guarantee as A*

 Advantages over RRT:
   - Deterministic — same input always gives same output
   - Optimal — guaranteed shortest any-angle path
   - Smooth — no jagged turns

 Launch:
   ~/Desktop/Drone_IP/launch_sim.sh --auto --algo theta

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
#  THETA* PLANNER — Any-Angle A* on OctoMap's /projected_map
# ─────────────────────────────────────────────────────────────────────────────
class ThetaStarPlanner:
    """
    Theta* path planner — A* with line-of-sight shortcuts.

    The key difference from A*:
      When expanding a neighbor N of current node C:
        - A*:     always sets parent(N) = C
        - Theta*: checks line-of-sight from parent(C) to N.
                  If clear, sets parent(N) = parent(C) and computes
                  g(N) as the straight-line distance from parent(C).
                  This creates diagonal shortcuts at any angle.
    """

    def __init__(self, occupancy_grid_msg, safety_margin=0.25):
        info = occupancy_grid_msg.info
        self.resolution = info.resolution
        self.origin_x = info.origin.position.x
        self.origin_y = info.origin.position.y
        self.width = info.width
        self.height = info.height

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

    def _line_of_sight(self, r1, c1, r2, c2):
        """
        Bresenham's line algorithm to check if all cells between
        (r1,c1) and (r2,c2) are free. This is the core of Theta*.
        """
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
        Theta* from (sx, sy) to (gx, gy) in world coordinates.

        Like A* but with line-of-sight checks: when a neighbor can
        "see" the grandparent directly, the parent is skipped and
        the path takes a straight-line shortcut at any angle.
        """
        t0 = time.time()

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

        # Nudge if start/goal is inside an obstacle
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

        print(f"  Theta*: Searching with line-of-sight shortcuts...")

        # Theta* search
        open_set = []
        counter = 0  # tie-breaker for heapq
        heapq.heappush(open_set, (0.0, counter, start))
        came_from = {start: start}  # start is its own parent
        g = {start: 0.0}
        closed = set()

        dirs = [(0, 1), (1, 0), (0, -1), (-1, 0),
                (1, 1), (1, -1), (-1, 1), (-1, -1)]

        while open_set:
            _, _, cur = heapq.heappop(open_set)

            if cur == goal:
                break

            if cur in closed:
                continue
            closed.add(cur)

            for dr, dc in dirs:
                nxt = (cur[0] + dr, cur[1] + dc)

                if not self.is_free(*nxt):
                    continue
                if nxt in closed:
                    continue

                # === THE THETA* DIFFERENCE ===
                # Check line-of-sight from parent(cur) to nxt
                parent_cur = came_from[cur]

                if self._line_of_sight(parent_cur[0], parent_cur[1],
                                       nxt[0], nxt[1]):
                    # Path 1: grandparent -> nxt (straight line, any angle)
                    new_g = g[parent_cur] + math.hypot(
                        nxt[0] - parent_cur[0], nxt[1] - parent_cur[1])
                    new_parent = parent_cur
                else:
                    # Path 2: standard A* — cur -> nxt (grid-locked)
                    step = 1.414 if (dr != 0 and dc != 0) else 1.0
                    new_g = g[cur] + step
                    new_parent = cur

                if new_g < g.get(nxt, float('inf')):
                    g[nxt] = new_g
                    came_from[nxt] = new_parent
                    f = new_g + math.hypot(goal[0] - nxt[0], goal[1] - nxt[1])
                    counter += 1
                    heapq.heappush(open_set, (f, counter, nxt))

        dt = (time.time() - t0) * 1000

        if goal not in came_from:
            print(f"  Theta*: No path found! ({dt:.0f}ms)")
            return None

        # Reconstruct path
        path = []
        cur = goal
        while cur != came_from[cur]:  # stop when cur == start (parent of start is itself)
            path.append(self.grid_to_world(*cur))
            cur = came_from[cur]
        path.append(self.grid_to_world(*start))
        path.reverse()

        print(f"  Theta*: Path found — {len(path)} waypoints ({dt:.0f}ms)")
        return path


# ─────────────────────────────────────────────────────────────────────────────
#  AUTONOMOUS NAVIGATOR NODE (uses ThetaStarPlanner)
# ─────────────────────────────────────────────────────────────────────────────
class AutonomousNavigator(Node):
    """
    State Machine:
      INIT       -> waiting for PX4 + Gazebo data
      TAKEOFF    -> arming and ascending to flight altitude
      EXPLORING  -> flying scan waypoints + rotating at each
      READY      -> map built, hovering, waiting for user goal
      NAVIGATING -> following Theta* path to goal
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
        print("║   📐 AUTONOMOUS THETA* NAVIGATOR — ANY-ANGLE          ║")
        print("╠══════════════════════════════════════════════════════════╣")
        print("║  Phase 1: Takeoff -> 1.8m                              ║")
        print("║  Phase 2: Explore rooms (13 scan positions, 360 each)  ║")
        print("║  Phase 3: OctoMap builds 3D map from depth camera      ║")
        print("║  Phase 4: Click '2D Goal Pose' in RViz                 ║")
        print("║  Phase 5: Theta* plans any-angle path -> drone flies!  ║")
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
        print(f"   Planning with Theta* (any-angle A*)...")

        grid_info = self.occupancy_grid.info
        total_cells = grid_info.width * grid_info.height
        occupied = sum(1 for v in self.occupancy_grid.data if v > 50)
        print(f"   Map: {grid_info.width}x{grid_info.height} cells, "
              f"resolution={grid_info.resolution:.2f}m, "
              f"occupied={occupied}/{total_cells}")

        planner = ThetaStarPlanner(self.occupancy_grid,
                                   safety_margin=self.SAFETY_MARGIN)
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
