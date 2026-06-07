#!/usr/bin/env python3
"""
=============================================================================
 OFFBOARD WAYPOINT NAVIGATION
 File: offboard_waypoint_nav.py
=============================================================================

 WHAT THIS SCRIPT DOES:
   Commands a PX4 drone to fly from a START point to a DESTINATION point
   while following pre-planned waypoints that avoid obstacles.

 FLIGHT PLAN (bird's eye view):
                                       
        Y = +4 (front wall)            
   ┌─────────────────────────────┐     
   │                             │     
   │  WP2(-1,2) ──► WP3(1,2) ──►│──► WP4 = DEST (3,2)
   │    ▲        ██ Red(0,1)     │     
   │    │              ██ Yel(2,0)     
   │    │  ═══ Orange(-1.5,-1)   │     
   │ WP1(-3,0)                   │     
   │    ▲                        │     
   │ START(-3,-2)                │     
   └─────────────────────────────┘     
        Y = -4 (back wall)            

 HOW TO RUN:
   Terminal 1:  cd ~/Micro-XRCE-DDS-Agent/build && MicroXRCEAgent udp4 -p 8888
   Terminal 2:  cd ~/PX4-Autopilot && PX4_GZ_WORLD=indoor_10x8x3 make px4_sitl gz_x500
   Terminal 3:  cd ~/px4_ros_ws && source install/setup.bash
                python3 ~/Desktop/Drone_IP/offboard_waypoint_nav.py

 PREREQUISITE:
   Your ROS 2 workspace (~/px4_ros_ws) must have px4_msgs built.

=============================================================================
"""

# ─────────────────────────────────────────────────────────────────────────────
# IMPORTS — What each one does
# ─────────────────────────────────────────────────────────────────────────────

import rclpy                          # ROS 2 Python client library (the "engine" for ROS 2 in Python)
from rclpy.node import Node           # Base class for creating a ROS 2 node
from rclpy.qos import (               # Quality of Service settings — controls how messages are delivered
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
import numpy as np                    # For math (calculating distance between points)

# PX4 message types — these are the "languages" PX4 speaks over ROS 2
from px4_msgs.msg import (
    OffboardControlMode,   # "Hey PX4, I'm controlling you from code, not a remote"
    TrajectorySetpoint,    # "Go to this position (X, Y, Z)"
    VehicleCommand,        # "Arm yourself" / "Switch to offboard mode" / "Land"
    VehicleOdometry,       # PX4 tells US: "I'm currently at position (X, Y, Z)"
)


# ─────────────────────────────────────────────────────────────────────────────
# COORDINATE SYSTEM EXPLANATION (VERY IMPORTANT!)
# ─────────────────────────────────────────────────────────────────────────────
#
# PX4 uses the NED coordinate system (North-East-Down):
#
#   ┌──────────► X (North)
#   │
#   │     In NED, positive Z means DOWN.
#   │     So to fly UP to 1.8 meters, we set Z = -1.8
#   │
#   ▼ Y (East)
#
#   Z points DOWN into the ground.
#   Z = 0    → on the ground
#   Z = -1.8 → 1.8 meters in the air
#   Z = -3.0 → 3 meters in the air (ceiling of our room)
#
# Gazebo uses a different system (Z points UP), but PX4 handles the
# conversion internally. When we send commands, we use NED.
#
# Our room in NED coordinates:
#   X: -5 to +5  (10m)
#   Y: -4 to +4  (8m)
#   Z: 0 (floor) to -3 (ceiling)
#
# The drone spawns at (0, 0, 0) in NED — center of the room, on the floor.
# ─────────────────────────────────────────────────────────────────────────────


class OffboardWaypointNav(Node):
    """
    A ROS 2 node that flies the drone through waypoints using offboard control.

    "Offboard control" means: "I (the computer) am sending position commands
    to PX4, instead of a human with a joystick." PX4 requires you to keep
    sending heartbeat messages, otherwise it assumes you've crashed and
    switches to a safety mode.
    """

    def __init__(self):
        super().__init__('offboard_waypoint_nav')   # Node name — shows up in `ros2 node list`

        # ─────────────────────────────────────────────────────────────────
        # QoS PROFILE
        # ─────────────────────────────────────────────────────────────────
        # PX4 uses specific QoS settings for its topics. If we don't match
        # them, our messages will be silently dropped (very confusing!).
        #
        # Think of QoS like postal service options:
        #   - BEST_EFFORT = like a postcard, fast but might get lost
        #   - KEEP_LAST(10) = keep only the 10 most recent messages
        #   - VOLATILE = don't store messages for late subscribers
        #
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,       # was TRANSIENT_LOCAL
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ─────────────────────────────────────────────────────────────────
        # PUBLISHERS — We SEND these messages TO PX4
        # ─────────────────────────────────────────────────────────────────

        # Publisher 1: Offboard control mode heartbeat
        # This tells PX4: "I'm alive and I want to control your POSITION"
        # We must publish this at ≥2 Hz or PX4 will exit offboard mode
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )

        # Publisher 2: Trajectory setpoint
        # This tells PX4: "Fly to this X, Y, Z position"
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        # Publisher 3: Vehicle command
        # This sends one-time commands like "arm motors" or "switch flight mode"
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )

        # ─────────────────────────────────────────────────────────────────
        # SUBSCRIBER — We RECEIVE this message FROM PX4
        # ─────────────────────────────────────────────────────────────────

        # Subscribe to odometry — PX4 tells us where the drone currently is
        # Every time PX4 publishes a new position, our callback function runs
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,   # ← this function gets called ~30 times/sec
            qos_profile
        )

        # ─────────────────────────────────────────────────────────────────
        # STATE VARIABLES — Tracking what the drone is doing
        # ─────────────────────────────────────────────────────────────────

        # Current position of the drone (updated by odometry callback)
        self.current_position = np.array([0.0, 0.0, 0.0])  # [x, y, z] in NED

        # Counter for how many heartbeats we've sent
        # PX4 needs ~10 heartbeats before it accepts offboard mode
        self.heartbeat_counter = 0

        # Have we already sent the arm + offboard mode commands?
        self.armed = False

        # ─────────────────────────────────────────────────────────────────
        # WAYPOINTS — The flight plan (in NED coordinates!)
        # ─────────────────────────────────────────────────────────────────
        #
        # Remember: Z is NEGATIVE for altitude (NED = Down is positive)
        #
        # The flight goes:
        #   1. Take off at center → climb to 1.8m
        #   2. Fly to start position (-3, -2) at 1.8m altitude
        #   3. Go north to (-3, 0) — clearing the orange wall
        #   4. Go northeast to (-1, 2) — west of the red pillar
        #   5. Go east to (1, 2) — north of the red pillar, north of yellow pillar
        #   6. Arrive at destination (3, 2) at 1.8m altitude
        #
        self.waypoints = [
            # Step 1: Take off — climb straight up from spawn point
            np.array([0.0, 0.0, -1.8]),       # WP0: hover above spawn

            # Step 2: Fly to the start marker
            np.array([-3.0, -2.0, -1.8]),     # WP1: above green start marker

            # Step 3-5: Navigate around obstacles
            np.array([-3.0, 0.0, -1.8]),      # WP2: go north, clear orange wall
            np.array([-1.0, 2.0, -1.8]),      # WP3: northeast, west of red pillar
            np.array([1.0, 2.0, -1.8]),       # WP4: east, north of red & yellow

            # Step 6: Arrive at destination
            np.array([3.0, 2.0, -1.8]),       # WP5: above blue destination marker
        ]

        # Which waypoint we're currently heading toward
        self.current_wp_index = 0

        # How close (in meters) the drone needs to be to a waypoint
        # before we consider it "reached" and move to the next one
        self.waypoint_threshold = 0.4   # 40 cm tolerance

        # Are we done with all waypoints?
        self.mission_complete = False

        # ─────────────────────────────────────────────────────────────────
        # TIMER — The heartbeat loop (runs 10 times per second)
        # ─────────────────────────────────────────────────────────────────
        # This is the main control loop. Every 0.1 seconds (10 Hz), it:
        #   1. Sends the offboard heartbeat
        #   2. Sends the current waypoint as the target position
        #   3. Checks if we've reached the waypoint
        #
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        self.get_logger().info('=' * 60)
        self.get_logger().info('  OFFBOARD WAYPOINT NAVIGATION NODE STARTED')
        self.get_logger().info(f'  Total waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'  Destination: ({self.waypoints[-1][0]}, {self.waypoints[-1][1]})')
        self.get_logger().info('=' * 60)

    # ═════════════════════════════════════════════════════════════════════
    # CALLBACK: Odometry — runs every time PX4 publishes the drone's position
    # ═════════════════════════════════════════════════════════════════════

    def odometry_callback(self, msg):
        """
        PX4 tells us where the drone is. We save it so the timer_callback
        can check "am I close enough to the current waypoint?"

        msg.position is a 3-element array: [x, y, z] in NED frame.
        """
        self.current_position = np.array([
            msg.position[0],   # X (North)
            msg.position[1],   # Y (East)
            msg.position[2],   # Z (Down — negative means up)
        ])

    # ═════════════════════════════════════════════════════════════════════
    # TIMER CALLBACK — The main control loop (runs 10x per second)
    # ═════════════════════════════════════════════════════════════════════

    def timer_callback(self):
        """
        This is the HEARTBEAT. It runs every 0.1 seconds.

        The sequence is:
          1. First ~20 calls: just send heartbeat (PX4 needs this before accepting offboard)
          2. At call #20: send ARM and OFFBOARD MODE commands
          3. All calls after: send heartbeat + current waypoint position
          4. When drone reaches a waypoint: advance to next one
          5. When all waypoints done: send LAND command
        """

        # If the mission is complete, do nothing (we already sent the land command)
        if self.mission_complete:
            return

        # ── ALWAYS: Send the offboard heartbeat ──
        # This message says: "I want to control POSITION" (not velocity, not attitude)
        self.publish_offboard_control_mode()

        # ── ALWAYS: Send the current waypoint as target ──
        if self.current_wp_index < len(self.waypoints):
            target = self.waypoints[self.current_wp_index]
            self.publish_trajectory_setpoint(target)

        # ── Phase 1: Wait for PX4 to accept heartbeats (first 2 seconds) ──
        self.heartbeat_counter += 1

        if self.heartbeat_counter == 20 and not self.armed:
            # After 20 heartbeats (2 seconds), PX4 is ready
            self.get_logger().info('💡 Sending ARM + OFFBOARD mode commands...')
            self.arm()                    # Turn on the motors
            self.set_offboard_mode()      # Switch to offboard control
            self.armed = True
            self.get_logger().info('🛫 Drone armed! Taking off to first waypoint...')

        # ── Phase 2: Check if we've reached the current waypoint ──
        if self.armed and self.current_wp_index < len(self.waypoints):
            target = self.waypoints[self.current_wp_index]
            distance = np.linalg.norm(self.current_position - target)

            # Print status every 20 ticks (~2 seconds) so we can see progress
            if self.heartbeat_counter % 20 == 0:
                self.get_logger().info(
                    f'📍 WP {self.current_wp_index}/{len(self.waypoints)-1} | '
                    f'Target: ({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f}) | '
                    f'Current: ({self.current_position[0]:.1f}, {self.current_position[1]:.1f}, {self.current_position[2]:.1f}) | '
                    f'Distance: {distance:.2f}m'
                )

            # Are we close enough to the waypoint?
            if distance < self.waypoint_threshold:
                self.get_logger().info(
                    f'✅ Reached WP {self.current_wp_index}! '
                    f'({target[0]:.1f}, {target[1]:.1f}, {target[2]:.1f})'
                )
                self.current_wp_index += 1

                # Check if we've completed all waypoints
                if self.current_wp_index >= len(self.waypoints):
                    self.get_logger().info('=' * 60)
                    self.get_logger().info('🎉 ALL WAYPOINTS REACHED — MISSION COMPLETE!')
                    self.get_logger().info('🛬 Sending LAND command...')
                    self.get_logger().info('=' * 60)
                    self.land()
                    self.mission_complete = True
                else:
                    next_wp = self.waypoints[self.current_wp_index]
                    self.get_logger().info(
                        f'➡️  Next target: WP {self.current_wp_index} '
                        f'({next_wp[0]:.1f}, {next_wp[1]:.1f}, {next_wp[2]:.1f})'
                    )

    # ═════════════════════════════════════════════════════════════════════
    # HELPER METHODS — Building and sending specific message types
    # ═════════════════════════════════════════════════════════════════════

    def publish_offboard_control_mode(self):
        """
        Sends the offboard heartbeat message.

        This message has boolean flags for what type of control we want:
          - position = True  → we're sending X, Y, Z targets
          - velocity = False → we're NOT sending velocity commands
          - acceleration, attitude, body_rate = False

        PX4 REQUIRES this message at ≥2 Hz. If it stops receiving it,
        PX4 will exit offboard mode and switch to a failsafe (hover/land).
        That's a safety feature — if your code crashes, the drone doesn't
        fly off uncontrolled.
        """
        msg = OffboardControlMode()
        msg.position = True       # We want position control
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # Microseconds
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, position):
        """
        Tells PX4: "Fly to this position."

        Args:
            position: numpy array [x, y, z] in NED coordinates
                     Remember: z = -1.8 means 1.8m altitude
        """
        msg = TrajectorySetpoint()
        msg.position = [float(position[0]), float(position[1]), float(position[2])]
        msg.yaw = 0.0  # Face north (0 radians). We don't need to turn in this mission.
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        """
        Sends a one-time command to PX4 (arm, change mode, land, etc.)

        This is like pressing a button on a remote control.
        Different 'command' numbers do different things — PX4 defines
        these as constants (like VEHICLE_CMD_COMPONENT_ARM_DISARM = 400).

        Args:
            command: The command ID (integer)
            param1: First parameter (meaning depends on the command)
            param2: Second parameter
        """
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1          # PX4 system ID (always 1 in SITL)
        msg.target_component = 1       # PX4 component ID (always 1)
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True       # This command comes from outside PX4
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        """
        ARM the drone — this enables the motors.

        Like turning the ignition key in a car. The motors start spinning
        but the drone doesn't move yet (it needs a position command too).

        Command 400 = VEHICLE_CMD_COMPONENT_ARM_DISARM
        param1 = 1.0 means ARM (0.0 would mean DISARM)
        """
        self.publish_vehicle_command(400, param1=1.0)
        self.get_logger().info('🔑 ARM command sent')

    def set_offboard_mode(self):
        """
        Switch PX4 to OFFBOARD flight mode.

        PX4 has many flight modes (Manual, Altitude, Position, Mission, etc.)
        OFFBOARD mode means: "take commands from ROS 2 topics instead of
        the remote control."

        Command 176 = VEHICLE_CMD_DO_SET_MODE
        param1 = 1.0 (custom mode flag)
        param2 = 6.0 (offboard mode ID in PX4)
        """
        self.publish_vehicle_command(176, param1=1.0, param2=6.0)
        self.get_logger().info('🎮 OFFBOARD mode command sent')

    def land(self):
        """
        Send a LAND command.

        The drone will descend vertically at the current X, Y position
        and land on the ground, then automatically disarm after a few seconds.

        Command 21 = VEHICLE_CMD_NAV_LAND
        """
        self.publish_vehicle_command(21)
        self.get_logger().info('🛬 LAND command sent')


# ═════════════════════════════════════════════════════════════════════════════
# MAIN — Entry point when you run `python3 offboard_waypoint_nav.py`
# ═════════════════════════════════════════════════════════════════════════════

def main():
    print("\n" + "=" * 60)
    print("  DRONE OBSTACLE NAVIGATION — Starting up...")
    print("=" * 60 + "\n")

    # Step 1: Initialize the ROS 2 system
    # This sets up the internal communication plumbing
    rclpy.init()

    # Step 2: Create our navigation node
    node = OffboardWaypointNav()

    try:
        # Step 3: Start spinning (processing callbacks forever)
        # This keeps the node alive, processing timer callbacks and
        # incoming odometry messages. It runs until you press Ctrl+C.
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n⛔ Interrupted by user (Ctrl+C)")
    finally:
        # Step 4: Cleanup — destroy the node and shut down ROS 2
        node.destroy_node()
        rclpy.shutdown()
        print("Node shut down cleanly.")


if __name__ == '__main__':
    main()
