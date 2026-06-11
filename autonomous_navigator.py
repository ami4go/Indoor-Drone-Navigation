#!/usr/bin/env python3
"""
=============================================================================
 AUTONOMOUS A* NAVIGATOR v2 — Static Room Map (Reliable)
 File: autonomous_navigator.py
=============================================================================

 Instead of depending on an incomplete live OctoMap, this navigator uses
 a precise hardcoded map of the room built from the SDF file geometry.
 This guarantees that A* ALWAYS works, even before the drone has flown.

 Room: 10m x 8m (X: -5 to +5, Y: -4 to +4)
 Obstacles (from indoor_10x8x3.sdf):
   - Pillar Red:     X=0,    Y=1,   size 0.5 x 0.5
   - Wall Orange:    X=-1.5, Y=-1,  size 2.0 x 0.3
   - Pillar Yellow:  X=2,    Y=0,   size 0.5 x 0.5
   - Room walls:     all 4 perimeter walls (0.1m thick)

 Coordinate System:
   Gazebo ENU:  X = East (+right), Y = North (+forward)
   PX4 NED:     X = North,         Y = East
   Drone spawn: approximately Gazebo X=-3, Y=-2

=============================================================================
"""

import math
import time
import heapq
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleOdometry
)


# ─────────────────────────────────────────────────────────────────────────────
#  ROOM MAP (from indoor_10x8x3.sdf) — Units in meters
# ─────────────────────────────────────────────────────────────────────────────
ROOM = {
    'x_min': -5.0,
    'x_max':  5.0,
    'y_min': -4.0,
    'y_max':  4.0,
}

# Each obstacle: (center_x, center_y, half_width_x, half_width_y)
# We read these directly from the SDF poses and sizes.
OBSTACLES_SDF = [
    # name, cx, cy, half_sx, half_sy
    ('pillar_red',    0.0,  1.0,  0.25, 0.25),
    ('wall_orange',  -1.5, -1.0,  1.00, 0.15),
    ('pillar_yellow', 2.0,  0.0,  0.25, 0.25),
    # Room walls (thin, 0.1m thick)
    ('wall_north',   0.0,  4.0,  5.00, 0.05),
    ('wall_south',   0.0, -4.0,  5.00, 0.05),
    ('wall_east',    5.0,  0.0,  0.05, 4.00),
    ('wall_west',   -5.0,  0.0,  0.05, 4.00),
]

# ─────────────────────────────────────────────────────────────────────────────
#  A* PLANNER on the static room map
# ─────────────────────────────────────────────────────────────────────────────
class RoomPlanner:
    def __init__(self, resolution=0.1, safety_margin=0.5):
        self.resolution = resolution
        self.safety_margin = safety_margin

        # Grid covers the full room
        self.origin_x = ROOM['x_min']
        self.origin_y = ROOM['y_min']
        self.width  = int((ROOM['x_max'] - ROOM['x_min']) / resolution) + 1
        self.height = int((ROOM['y_max'] - ROOM['y_min']) / resolution) + 1

        # Build occupancy grid (0=free, 1=obstacle)
        self.grid = np.zeros((self.height, self.width), dtype=np.uint8)
        self._build_grid()

    def _build_grid(self):
        """Mark obstacles on the grid, inflated by safety_margin."""
        for name, cx, cy, hsx, hsy in OBSTACLES_SDF:
            # Inflate
            x_lo = cx - hsx - self.safety_margin
            x_hi = cx + hsx + self.safety_margin
            y_lo = cy - hsy - self.safety_margin
            y_hi = cy + hsy + self.safety_margin

            # Convert to grid cells
            c_lo = max(0, int((x_lo - self.origin_x) / self.resolution))
            c_hi = min(self.width  - 1, int((x_hi - self.origin_x) / self.resolution))
            r_lo = max(0, int((y_lo - self.origin_y) / self.resolution))
            r_hi = min(self.height - 1, int((y_hi - self.origin_y) / self.resolution))

            self.grid[r_lo:r_hi+1, c_lo:c_hi+1] = 1

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
        """Run A* from (sx,sy) to (gx,gy) in world coordinates."""
        start = self.world_to_grid(sx, sy)
        goal  = self.world_to_grid(gx, gy)

        # Clamp goal to room bounds
        goal = (
            max(0, min(self.height - 1, goal[0])),
            max(0, min(self.width  - 1, goal[1]))
        )

        if not self.is_free(*start):
            print(f"  ⚠️  Start ({sx:.2f},{sy:.2f}) is in an obstacle — nudging to nearest free cell")
            start = self._nearest_free(start)
            if start is None:
                print("  ❌  Cannot find free start cell!")
                return None

        if not self.is_free(*goal):
            print(f"  ⚠️  Goal ({gx:.2f},{gy:.2f}) is in an obstacle — nudging to nearest free cell")
            goal = self._nearest_free(goal)
            if goal is None:
                print("  ❌  Cannot find free goal cell!")
                return None

        # A* with 8-connectivity
        open_set = []
        heapq.heappush(open_set, (0.0, start))
        came_from = {start: None}
        g = {start: 0.0}
        dirs = [(0,1),(1,0),(0,-1),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]

        while open_set:
            _, cur = heapq.heappop(open_set)
            if cur == goal:
                break
            for dr, dc in dirs:
                nxt = (cur[0]+dr, cur[1]+dc)
                if not self.is_free(*nxt):
                    continue
                step = 1.414 if dr and dc else 1.0
                ng = g[cur] + step
                if nxt not in g or ng < g[nxt]:
                    g[nxt] = ng
                    f = ng + math.hypot(goal[0]-nxt[0], goal[1]-nxt[1])
                    heapq.heappush(open_set, (f, nxt))
                    came_from[nxt] = cur

        if goal not in came_from:
            print("  ❌  No path found!")
            return None

        # Reconstruct
        path, cur = [], goal
        while cur is not None:
            path.append(self.grid_to_world(*cur))
            cur = came_from[cur]
        path.reverse()
        return self._simplify(path)

    def _nearest_free(self, cell, max_radius=20):
        """BFS outward to find the nearest free cell."""
        visited = {cell}
        queue = [cell]
        while queue:
            next_queue = []
            for r, c in queue:
                for dr in range(-1, 2):
                    for dc in range(-1, 2):
                        nb = (r+dr, c+dc)
                        if nb not in visited and self.is_free(*nb):
                            return nb
                        if nb not in visited:
                            visited.add(nb)
                            next_queue.append(nb)
            queue = next_queue
        return None

    def _simplify(self, path):
        """Remove collinear waypoints to smooth the path."""
        if len(path) < 3:
            return path
        result = [path[0]]
        for i in range(1, len(path) - 1):
            dx1 = path[i][0]   - path[i-1][0]
            dy1 = path[i][1]   - path[i-1][1]
            dx2 = path[i+1][0] - path[i][0]
            dy2 = path[i+1][1] - path[i][1]
            if abs(dx1*dy2 - dx2*dy1) > 1e-4:
                result.append(path[i])
        result.append(path[-1])
        return result


# ─────────────────────────────────────────────────────────────────────────────
#  AUTONOMOUS NAVIGATOR NODE
# ─────────────────────────────────────────────────────────────────────────────
class AutonomousNavigator(Node):
    def __init__(self):
        super().__init__('autonomous_navigator')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        # ── Subscribers ──
        self.gz_sub    = self.create_subscription(
            TFMessage, '/world/indoor_10x8x3/dynamic_pose/info',
            self.gz_pose_cb, qos)
        self.odom_sub  = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_cb, qos)
        self.goal_sub  = self.create_subscription(
            PoseStamped, '/goal_pose',
            self.goal_cb, 10)

        # ── Publishers ──
        self.path_pub     = self.create_publisher(Path, '/plan', 10)
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub  = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ── State ──
        self.gz_pos    = None   # Gazebo ENU [x, y, z]
        self.px4_pos   = None   # PX4 NED  [x, y, z]

        self.waypoints     = []
        self.wp_idx        = 0
        self.is_flying     = False
        self.armed         = False
        self.state         = 'INIT'
        self.scan_yaw      = 0.0

        self.ALTITUDE      = 1.8   # metres
        self.WP_REACH_DIST = 0.20  # metres — when to switch to next waypoint (reduced to prevent corner-cutting)

        # ── Room planner (built once, always ready) ──
        # Increased safety margin to 0.7m to keep drone further away from pillars
        self.planner = RoomPlanner(resolution=0.1, safety_margin=0.7)
        self.get_logger().info("✅ Room map built — A* ready!")

        # ── Control timer: 20 Hz ──
        self.create_timer(0.05, self.control_loop)

        print("\n╔══════════════════════════════════════════════════════╗")
        print("║      🧠 AUTONOMOUS A* NAVIGATOR v2 ACTIVE          ║")
        print("╠══════════════════════════════════════════════════════╣")
        print("║  Static room map loaded — no OctoMap needed!       ║")
        print("║  Drone will take off automatically.                 ║")
        print("║  Set target: RViz → '2D Goal Pose' → click map     ║")
        print("╚══════════════════════════════════════════════════════╝\n")

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def gz_pose_cb(self, msg):
        """Get drone's ground-truth position from Gazebo."""
        # First transform is always the drone model (x500_depth_0)
        for tf in msg.transforms:
            if 'x500' in tf.child_frame_id.lower():
                t = tf.transform.translation
                self.gz_pos = np.array([t.x, t.y, t.z])
                return
        # Fallback: just use first transform
        if msg.transforms:
            t = msg.transforms[0].transform.translation
            self.gz_pos = np.array([t.x, t.y, t.z])

    def odom_cb(self, msg):
        self.px4_pos = np.array([msg.position[0], msg.position[1], msg.position[2]])

    def goal_cb(self, msg):
        if self.state != 'READY':
            print("⚠️  Drone is not ready yet (maybe still scanning or taking off) — please wait!")
            return
        if self.gz_pos is None:
            print("⚠️  No Gazebo position received yet.")
            return

        gx = msg.pose.position.x
        gy = msg.pose.position.y

        # Clamp goal into room
        gx = max(ROOM['x_min'] + 0.6, min(ROOM['x_max'] - 0.6, gx))
        gy = max(ROOM['y_min'] + 0.6, min(ROOM['y_max'] - 0.6, gy))

        print(f"\n🎯  Goal: ({gx:.2f}, {gy:.2f})")
        print(f"   Drone at: ({self.gz_pos[0]:.2f}, {self.gz_pos[1]:.2f})")

        path = self.planner.plan(self.gz_pos[0], self.gz_pos[1], gx, gy)

        if path:
            self.waypoints = path
            self.wp_idx    = 0
            self.state     = 'NAVIGATING'
            self._publish_rviz_path(path)
            print(f"   ✅ Path found — {len(path)} waypoints. Flying!")
        else:
            print("   ❌ No path found. Try a different target.")

    # ── Control Loop ──────────────────────────────────────────────────────────
    def control_loop(self):
        if self.px4_pos is None or self.gz_pos is None:
            return

        if not self.is_flying:
            if not self.armed:
                self._takeoff()
            return

        # Always send heartbeat
        self._heartbeat()

        # State Machine
        if self.state == 'SCANNING':
            self.scan_yaw += 0.05
            self._send_position(self.px4_pos[0], self.px4_pos[1], -self.ALTITUDE, yaw=self.scan_yaw)
            
            if self.scan_yaw >= 2 * math.pi:
                self.state = 'READY'
                print("\n  ✅ SCAN COMPLETE! Map built.")
                print("  ✅ AIRBORNE at 1.8m! Click '2D Goal Pose' in RViz.\n")

        elif self.state == 'NAVIGATING':
            if self.wp_idx < len(self.waypoints):
                wx, wy = self.waypoints[self.wp_idx]

                # Relative delta in Gazebo ENU
                dx_enu = wx - self.gz_pos[0]
                dy_enu = wy - self.gz_pos[1]

                # Convert ENU delta → NED target (relative → absolute PX4)
                # ENU: X=East, Y=North → NED: X=North, Y=East
                tgt_px4_x = self.px4_pos[0] + dy_enu   # North += Gazebo Y delta
                tgt_px4_y = self.px4_pos[1] + dx_enu   # East  += Gazebo X delta
                tgt_px4_z = -self.ALTITUDE             # NED Down = negative

                # We can point the drone towards the next waypoint
                target_yaw = math.atan2(dy_enu, dx_enu)
                self._send_position(tgt_px4_x, tgt_px4_y, tgt_px4_z, yaw=target_yaw)

                dist = math.hypot(dx_enu, dy_enu)
                if dist < self.WP_REACH_DIST:
                    self.wp_idx += 1
                    if self.wp_idx >= len(self.waypoints):
                        print("  🎉 DESTINATION REACHED! Set a new goal in RViz.")
                        self.waypoints = []
                        self.state = 'READY'
                    else:
                        print(f"  📍 WP {self.wp_idx}/{len(self.waypoints)} — {dist:.2f}m")
            else:
                self.state = 'READY'

        elif self.state == 'READY':
            # Hover at current PX4 position
            self._send_position(self.px4_pos[0], self.px4_pos[1], -self.ALTITUDE)

    # ── Takeoff ───────────────────────────────────────────────────────────────
    def _takeoff(self):
        print("  🚀 Takeoff sequence starting...")

        hover_z = -self.ALTITUDE
        print("  ⏳ Sending 30 warmup heartbeats...")
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

        self.armed     = True
        self.is_flying = True
        self.state     = 'SCANNING'
        print("\n  🔄 Rotating 360° to scan the room with OctoMap...")

    # ── Helpers ───────────────────────────────────────────────────────────────
    def _heartbeat(self):
        msg = OffboardControlMode()
        msg.position     = True
        msg.velocity     = False
        msg.acceleration = False
        msg.attitude     = False
        msg.body_rate    = False
        msg.timestamp    = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def _send_position(self, x, y, z, yaw=0.0):
        msg = TrajectorySetpoint()
        msg.position  = [float(x), float(y), float(z)]
        msg.yaw       = float(yaw)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def _send_command(self, cmd_id, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command          = cmd_id
        msg.param1           = param1
        msg.param2           = param2
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)

    def _publish_rviz_path(self, coords):
        """Draw the planned path as a green line in RViz."""
        msg = Path()
        msg.header.stamp    = self.get_clock().now().to_msg()
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
