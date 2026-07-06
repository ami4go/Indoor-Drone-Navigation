#!/usr/bin/env python3
"""
=============================================================================
 VISUAL BENCHMARK NAVIGATOR — Run all 6 algorithms, show paths in RViz
 File: planners/navigator_benchmark.py
=============================================================================

 Explores the environment like any other navigator, but when you click
 "2D Goal Pose" in RViz, it runs ALL 6 path planning algorithms and
 draws all 6 paths simultaneously with different colors:

   🔴 A*           — Red
   🟢 Dijkstra     — Green
   🟡 Bellman Ford — Yellow
   🔵 PRM          — Blue
   🟣 RRT          — Magenta
   ⚪ Theta*       — Cyan

 Also prints a comparison table in the terminal with metrics.

 The drone flies the Theta* path (smoothest), but you can SEE all 6.

 Launch:
   ~/Desktop/Drone_IP/launch_sim.sh --auto --algo benchmark

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
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from px4_msgs.msg import (
    OffboardControlMode, TrajectorySetpoint,
    VehicleCommand, VehicleOdometry
)

# Import all planners as pure functions (no ROS deps)
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'benchmark'))
from planner_library import PLANNERS, inflate_grid, compute_metrics


# ─────────────────────────────────────────────────────────────────────────────
#  EXPLORATION WAYPOINTS
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

# Colors for each algorithm (R, G, B, A)
ALGO_COLORS = {
    'A*':           (1.0, 0.2, 0.2, 1.0),   # Red
    'Dijkstra':     (0.2, 1.0, 0.2, 1.0),   # Green
    'Bellman Ford': (1.0, 1.0, 0.2, 1.0),   # Yellow
    'PRM':          (0.2, 0.4, 1.0, 1.0),   # Blue
    'RRT':          (1.0, 0.2, 1.0, 1.0),   # Magenta
    'Theta*':       (0.2, 1.0, 1.0, 1.0),   # Cyan
}


# ─────────────────────────────────────────────────────────────────────────────
#  VISUAL BENCHMARK NAVIGATOR
# ─────────────────────────────────────────────────────────────────────────────
class AutonomousNavigator(Node):

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
        self.marker_pub = self.create_publisher(MarkerArray, '/benchmark_paths', 10)
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

        self.create_timer(0.05, self.control_loop)

        # Republish markers periodically so they don't disappear
        self.latest_markers = None
        self.create_timer(2.0, self._republish_markers)

        print("\n╔══════════════════════════════════════════════════════════╗")
        print("║   📊 VISUAL BENCHMARK — ALL 6 ALGORITHMS AT ONCE      ║")
        print("╠══════════════════════════════════════════════════════════╣")
        print("║  Click '2D Goal Pose' → see 6 colored paths in RViz   ║")
        print("║                                                        ║")
        print("║  🔴 A*        🟢 Dijkstra    🟡 Bellman Ford          ║")
        print("║  🔵 PRM       🟣 RRT         ⚪ Theta*                ║")
        print("║                                                        ║")
        print("║  Drone flies the Theta* path (smoothest)               ║")
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
            print("Warning: Drone is not ready yet!")
            return
        if self.occupancy_grid is None or self.gz_pos is None:
            print("Warning: No map or position data!")
            return

        gx = msg.pose.position.x
        gy = msg.pose.position.y
        sx = self.gz_pos[0]
        sy = self.gz_pos[1]

        print(f"\n{'='*70}")
        print(f"  BENCHMARK: ({sx:.2f}, {sy:.2f}) → ({gx:.2f}, {gy:.2f})")
        print(f"{'='*70}")

        # Prepare the grid
        info = self.occupancy_grid.info
        raw = np.array(self.occupancy_grid.data, dtype=np.int8)
        grid = np.where(raw.reshape((info.height, info.width)) > 50, 1, 0).astype(np.uint8)
        inflated = inflate_grid(grid, info.resolution, self.SAFETY_MARGIN)

        # Run ALL 6 planners
        results = {}
        all_markers = MarkerArray()
        fly_path = None  # Path the drone will actually fly (Theta*)

        for idx, (algo_name, plan_fn) in enumerate(PLANNERS.items()):
            path, metrics = plan_fn(
                inflated, info.resolution,
                info.origin.position.x, info.origin.position.y,
                (sx, sy), (gx, gy),
                margin=0.0  # already inflated
            )
            results[algo_name] = (path, metrics)

            color = ALGO_COLORS[algo_name]
            status = "✓" if metrics['success'] else "✗"

            if metrics['success']:
                print(f"  {status} {algo_name:<12} │ {metrics['planning_time_ms']:>7.1f}ms │ "
                      f"{metrics['path_length_m']:>6.2f}m │ {metrics['waypoint_count']:>3} wps │ "
                      f"{metrics['nodes_explored']:>7,} nodes │ {metrics['smoothness_deg']:>5.1f}°")

                # Create colored line marker for RViz
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = algo_name
                marker.id = idx
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.08  # line width
                marker.color = ColorRGBA(
                    r=color[0], g=color[1], b=color[2], a=color[3])
                marker.pose.orientation.w = 1.0
                marker.lifetime.sec = 0  # persistent

                for wx, wy in path:
                    from geometry_msgs.msg import Point
                    p = Point()
                    p.x = float(wx)
                    p.y = float(wy)
                    p.z = float(self.ALTITUDE)
                    marker.points.append(p)

                all_markers.markers.append(marker)

                # Add algorithm label at midpoint of path
                mid = len(path) // 2
                label = Marker()
                label.header.frame_id = 'map'
                label.header.stamp = self.get_clock().now().to_msg()
                label.ns = algo_name + '_label'
                label.id = idx + 100
                label.type = Marker.TEXT_VIEW_FACING
                label.action = Marker.ADD
                label.pose.position.x = float(path[mid][0])
                label.pose.position.y = float(path[mid][1])
                label.pose.position.z = float(self.ALTITUDE + 0.3 + idx * 0.15)
                label.pose.orientation.w = 1.0
                label.scale.z = 0.25  # text size
                label.color = ColorRGBA(
                    r=color[0], g=color[1], b=color[2], a=1.0)
                label.text = f"{algo_name} ({metrics['path_length_m']:.1f}m)"
                label.lifetime.sec = 0

                all_markers.markers.append(label)

                # Save Theta* path for flying
                if algo_name == 'Theta*':
                    fly_path = path
            else:
                print(f"  {status} {algo_name:<12} │ {metrics['planning_time_ms']:>7.1f}ms │ FAILED")

        # Print comparison table
        self._print_table(results)

        # Publish all markers at once
        self.marker_pub.publish(all_markers)
        self.latest_markers = all_markers

        # Fly the Theta* path (smoothest)
        if fly_path:
            self.waypoints = fly_path
            self.wp_idx = 0
            self.state = 'NAVIGATING'
            self._publish_rviz_path(fly_path)
            print(f"\n  Drone flying Theta* path ({len(fly_path)} waypoints)...")
        else:
            # Fallback: fly A* if Theta* failed
            for name in ['A*', 'Dijkstra']:
                p, m = results.get(name, (None, None))
                if p:
                    self.waypoints = p
                    self.wp_idx = 0
                    self.state = 'NAVIGATING'
                    self._publish_rviz_path(p)
                    print(f"\n  Drone flying {name} path (Theta* unavailable)")
                    break

    def _print_table(self, results):
        print(f"\n  ┌{'─'*14}┬{'─'*10}┬{'─'*9}┬{'─'*7}┬{'─'*11}┬{'─'*9}┐")
        print(f"  │ {'Algorithm':<12} │ {'Time ms':>8} │ {'Path m':>7} │ {' WPs':>5} │ {'   Nodes':>9} │ {'Smth °':>7} │")
        print(f"  ├{'─'*14}┼{'─'*10}┼{'─'*9}┼{'─'*7}┼{'─'*11}┼{'─'*9}┤")
        for name, (path, m) in results.items():
            if m['success']:
                print(f"  │ {name:<12} │ {m['planning_time_ms']:>8.1f} │ "
                      f"{m['path_length_m']:>7.2f} │ {m['waypoint_count']:>5} │ "
                      f"{m['nodes_explored']:>9,} │ {m['smoothness_deg']:>7.1f} │")
            else:
                print(f"  │ {name:<12} │ {m['planning_time_ms']:>8.1f} │ "
                      f"{'FAIL':>7} │ {'  -':>5} │ {'    -':>9} │ {'   -':>7} │")
        print(f"  └{'─'*14}┴{'─'*10}┴{'─'*9}┴{'─'*7}┴{'─'*11}┴{'─'*9}┘")

    def _republish_markers(self):
        """Keep markers visible in RViz by republishing."""
        if self.latest_markers:
            self.marker_pub.publish(self.latest_markers)

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

    # Exploration

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
                print(f"\n  Exploration done but no map received!")
            print(f"\n  Click '2D Goal Pose' to see ALL 6 paths at once!\n")
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

    # Navigation

    def _navigate_step(self):
        if self.wp_idx >= len(self.waypoints):
            print("  DESTINATION REACHED! Set a new goal in RViz.")
            self.waypoints = []
            self.state = 'READY'
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
                print(f"  WP {self.wp_idx}/{len(self.waypoints)} — {dist:.2f}m")

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
        print(f"  Starting exploration ({len(SCAN_WAYPOINTS)} scan positions)...\n")

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
