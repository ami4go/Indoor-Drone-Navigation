#!/usr/bin/env python3
"""
=============================================================================
 AUTOMATED ROOM SCANNER
 File: room_scanner.py
=============================================================================

 Automatically flies the drone in a systematic lawnmower pattern to map
 the entire room with OctoMap. No manual input needed after launch.

 Flight pattern:
   1. Take off to 1.8m altitude
   2. Fly lawnmower rows covering the room (-3.5 to 3.5 in X, -3.0 to 3.0 in Y)
   3. Hover at end for OctoMap to settle

 Room: 10m × 8m (X: -5 to 5, Y: -4 to 4), ceiling at 3m
 Safe zone: 1.5m margin from walls

=============================================================================
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import numpy as np

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)


class RoomScanner(Node):
    def __init__(self):
        super().__init__('room_scanner')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos)

        # Subscriber
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_callback, qos)

        # State
        self.current_pos = np.array([0.0, 0.0, 0.0])
        self.target_pos = np.array([0.0, 0.0, -1.8])  # NED: negative Z = up
        self.armed = False
        self.is_flying = False
        self.scan_started = False
        self.scan_complete = False

        # ── Scan waypoints (NED coordinates) ──
        # Lawnmower pattern covering the room
        # X: -3.5 to 3.5, Y: -3.0 to 3.0, altitude 1.8m
        ALT = -1.8  # NED
        ROW_SPACING = 1.5  # meters between rows
        self.waypoints = []

        y_values = np.arange(-3.0, 3.5, ROW_SPACING)
        for i, y in enumerate(y_values):
            if i % 2 == 0:
                # Left to right
                self.waypoints.append(np.array([-3.5, y, ALT]))
                self.waypoints.append(np.array([3.5, y, ALT]))
            else:
                # Right to left
                self.waypoints.append(np.array([3.5, y, ALT]))
                self.waypoints.append(np.array([-3.5, y, ALT]))

        # Return to center and hover
        self.waypoints.append(np.array([0.0, 0.0, ALT]))

        self.current_wp_idx = 0
        self.wp_reached_time = None
        self.POSITION_THRESHOLD = 0.5  # meters — close enough to waypoint
        self.DWELL_TIME = 2.0  # seconds to wait at each waypoint

        # Control loop at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

        print("\n╔══════════════════════════════════════════════════╗")
        print("║     🗺️  AUTOMATED ROOM SCANNER  🗺️              ║")
        print("╠══════════════════════════════════════════════════╣")
        print(f"║  Waypoints: {len(self.waypoints)}")
        print(f"║  Altitude:  1.8m")
        print(f"║  Coverage:  X[-3.5, 3.5] × Y[-3.0, 3.0]")
        print("║  Status:    Waiting for PX4 connection...")
        print("╚══════════════════════════════════════════════════╝\n")

    def odom_callback(self, msg):
        self.current_pos = np.array([
            msg.position[0], msg.position[1], msg.position[2]
        ])

    def control_loop(self):
        if not self.is_flying:
            # Try to take off
            if not self.armed:
                self.takeoff_sequence()
            return

        if self.scan_complete:
            # Keep sending heartbeat + hover position
            self.send_heartbeat()
            self.send_position(self.target_pos)
            return

        # ── Waypoint navigation ──
        self.send_heartbeat()
        self.send_position(self.target_pos)

        # Check if we reached current waypoint
        dist = np.linalg.norm(self.current_pos - self.target_pos)

        if dist < self.POSITION_THRESHOLD:
            if self.wp_reached_time is None:
                self.wp_reached_time = time.time()
                wp = self.waypoints[self.current_wp_idx]
                print(f"  ✅ Reached WP {self.current_wp_idx + 1}/{len(self.waypoints)}: "
                      f"({wp[0]:+.1f}, {wp[1]:+.1f}) — dwelling {self.DWELL_TIME}s")

            # Dwell at waypoint for scanning
            if time.time() - self.wp_reached_time >= self.DWELL_TIME:
                self.current_wp_idx += 1
                self.wp_reached_time = None

                if self.current_wp_idx >= len(self.waypoints):
                    self.scan_complete = True
                    print("\n  🎉 SCAN COMPLETE! OctoMap should now show the full room.")
                    print("  📊 Check RViz for the 3D map visualization.")
                    print("  ⏸️  Hovering at center. Press Ctrl+C to land.\n")
                else:
                    wp = self.waypoints[self.current_wp_idx]
                    self.target_pos = wp.copy()
                    print(f"  ➡️  Flying to WP {self.current_wp_idx + 1}/{len(self.waypoints)}: "
                          f"({wp[0]:+.1f}, {wp[1]:+.1f})")

    def takeoff_sequence(self):
        """Arm, switch to offboard, and take off to 1.8m."""
        print("  🚀 Starting takeoff sequence...")

        self.target_pos = np.array([0.0, 0.0, -1.8])

        # Warmup heartbeats
        print("  ⏳ Step 1/3: Sending warmup heartbeats...")
        for i in range(25):
            self.send_heartbeat()
            self.send_position(self.target_pos)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        # Offboard mode
        print("  ⚙️  Step 2/3: Switching to OFFBOARD mode...")
        self.send_command(176, param1=1.0, param2=6.0)
        time.sleep(0.5)

        for i in range(5):
            self.send_heartbeat()
            self.send_position(self.target_pos)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

        # Arm
        print("  🔋 Step 3/3: Arming motors...")
        self.send_command(400, param1=1.0)
        time.sleep(0.5)

        self.armed = True
        self.is_flying = True

        # Set first waypoint after takeoff
        self.target_pos = self.waypoints[0].copy()
        print(f"\n  ✅ AIRBORNE! Starting room scan...")
        print(f"  ➡️  Flying to WP 1/{len(self.waypoints)}: "
              f"({self.waypoints[0][0]:+.1f}, {self.waypoints[0][1]:+.1f})\n")

    def send_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_pub.publish(msg)

    def send_position(self, pos):
        msg = TrajectorySetpoint()
        msg.position = [float(pos[0]), float(pos[1]), float(pos[2])]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_command(self, cmd_id, param1=0.0, param2=0.0):
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


def main():
    rclpy.init()
    node = RoomScanner()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n  🛬 Landing...")
        node.send_command(21)  # NAV_LAND
        time.sleep(2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("  ✅ Scanner stopped.")


if __name__ == '__main__':
    main()
