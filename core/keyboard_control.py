#!/usr/bin/env python3
"""
=============================================================================
 KEYBOARD DRONE CONTROLLER
 File: keyboard_control.py
=============================================================================

 WHAT THIS SCRIPT DOES:
   Lets you fly the drone manually using keyboard keys.
   You control the drone's target position — PX4 handles flying smoothly to it.

 CONTROLS:
   ┌─────────────────────────────────────────────┐
   │                                             │
   │         W = Move Forward (Y+)               │
   │                                             │
   │  A = Move Left (X-)    D = Move Right (X+)  │
   │                                             │
   │         S = Move Backward (Y-)              │
   │                                             │
   │  R = Go Higher (altitude up)                │
   │  F = Go Lower  (altitude down)              │
   │                                             │
   │  T = Takeoff (arm + offboard + climb)       │
   │  L = Land                                   │
   │  Q = Quit                                   │
   │                                             │
   │  Each press moves 0.5m in that direction    │
   └─────────────────────────────────────────────┘

 HOW TO RUN:
   Terminal 1:  cd ~/Micro-XRCE-DDS-Agent/build && MicroXRCEAgent udp4 -p 8888
   Terminal 2:  cd ~/PX4-Autopilot && PX4_GZ_WORLD=indoor_10x8x3 make px4_sitl gz_x500
   Terminal 3:  cd ~/px4_ros_ws && source install/setup.bash
                python3 ~/Desktop/Drone_IP/keyboard_control.py

=============================================================================
"""

import sys
import tty
import termios
import select
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
)
import numpy as np

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)


# ─────────────────────────────────────────────────────────────────────
# HELP TEXT — Printed to the terminal when the script starts
# ─────────────────────────────────────────────────────────────────────

HELP_TEXT = """
╔══════════════════════════════════════════════════╗
║        🎮 DRONE KEYBOARD CONTROLLER 🎮          ║
╠══════════════════════════════════════════════════╣
║                                                  ║
║   Movement (each press = 0.5 meter):             ║
║                                                  ║
║              W (forward / Y+)                    ║
║              ▲                                   ║
║   A (left) ◄   ► D (right)                      ║
║              ▼                                   ║
║              S (backward / Y-)                   ║
║                                                  ║
║   R = altitude UP     F = altitude DOWN          ║
║                                                  ║
║   Commands:                                      ║
║   T = Takeoff (arm + fly to 1.8m)                ║
║   L = Land at current position                   ║
║   Q = Quit                                       ║
║                                                  ║
╚══════════════════════════════════════════════════╝
"""


# ─────────────────────────────────────────────────────────────────────
# KEYBOARD INPUT HELPER
# ─────────────────────────────────────────────────────────────────────

def get_key(timeout=0.1):
    """
    Read a single keypress from the terminal without waiting for Enter.

    How it works:
      Normal terminals wait for you to press Enter before sending input.
      We switch the terminal to "raw mode" which sends each keypress
      immediately. Then we use 'select' to check if a key is available
      within the timeout period (so we don't block forever).

    Returns:
      A string like 'w', 'a', 's', 'd', 'q', etc. or None if no key pressed.
    """
    # Save the original terminal settings so we can restore them later
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        # Switch to raw mode — keys are sent immediately, no echo
        tty.setraw(sys.stdin.fileno())

        # Wait up to 'timeout' seconds for a keypress
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)

        if rlist:
            key = sys.stdin.read(1)
            return key
        return None
    finally:
        # ALWAYS restore normal terminal mode, even if an error occurs
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


class KeyboardDroneController(Node):
    """
    Manual drone controller using keyboard input.

    Instead of following pre-planned waypoints, YOU decide where to go.
    Each key press moves the target position by 0.5m, and PX4 smoothly
    flies the drone to that new target.
    """

    def __init__(self):
        super().__init__('keyboard_drone_controller')

        # ── QoS profile matching PX4's expectations ──
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Publishers (send commands TO PX4) ──
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # ── Subscriber (receive position FROM PX4) ──
        self.odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.odom_callback, qos_profile)

        # ── State ──
        self.current_pos = np.array([0.0, 0.0, 0.0])   # Where the drone IS (NED)
        self.target_pos = np.array([0.0, 0.0, 0.0])     # Where we WANT it to go (NED)
        self.armed = False
        self.heartbeat_count = 0
        self.is_flying = False      # True after takeoff sequence

        # Movement step size (meters per keypress)
        self.STEP = 0.5

        # Room boundaries (NED) — prevent flying outside the room
        # X: -5 to +5, Y: -4 to +4, Z: 0 to -3 (remember NED: negative Z = up)
        self.X_MIN, self.X_MAX = -4.5, 4.5     # Leave 0.5m margin from walls
        self.Y_MIN, self.Y_MAX = -3.5, 3.5
        self.Z_MIN, self.Z_MAX = -2.8, -0.5    # Min altitude 0.5m, max 2.8m

        # ── Main loop timer (10 Hz) ──
        # This handles BOTH the heartbeat AND keyboard reading
        self.timer = self.create_timer(0.1, self.control_loop)

        print(HELP_TEXT)
        print("⏳ Waiting for drone connection... Press T to takeoff when ready.\n")

    def odom_callback(self, msg):
        """Store the drone's current position (updated ~30 times/sec by PX4)."""
        self.current_pos = np.array([
            msg.position[0], msg.position[1], msg.position[2]
        ])

    def control_loop(self):
        """
        Main loop — runs 10 times per second.
        1. Reads keyboard input
        2. Updates target position based on key pressed
        3. Sends heartbeat + position command to PX4
        """

        # ── Step 1: Read keyboard (non-blocking) ──
        key = get_key(timeout=0.05)

        if key is not None:
            self.handle_key(key.lower())

        # ── Step 2: If flying, send heartbeat + position command ──
        if self.is_flying:
            self.send_heartbeat()
            self.send_position(self.target_pos)

    def handle_key(self, key):
        """
        Process a single keypress and update the target position.

        The target position is CLAMPED to room boundaries so you
        can't accidentally fly outside the room.
        """

        if key == 'q':
            # ── QUIT ──
            print("\n🛑 Quitting... drone will hold position.")
            raise SystemExit

        elif key == 't' and not self.is_flying:
            # ── TAKEOFF SEQUENCE ──
            print("\n🛫 TAKEOFF SEQUENCE STARTING...")
            print("   Step 1/3: Sending heartbeats...")
            self.takeoff_sequence()
            return

        elif key == 'l' and self.is_flying:
            # ── LAND ──
            print(f"\n🛬 LANDING at current position ({self.current_pos[0]:.1f}, {self.current_pos[1]:.1f})")
            self.send_command(21)   # VEHICLE_CMD_NAV_LAND
            self.is_flying = False
            self.armed = False
            return

        elif not self.is_flying:
            # Don't process movement keys if we haven't taken off
            return

        # ── MOVEMENT KEYS ──
        moved = False

        if key == 'w':
            # Forward = increase Y (North in NED)
            self.target_pos[1] += self.STEP
            moved = True
        elif key == 's':
            # Backward = decrease Y
            self.target_pos[1] -= self.STEP
            moved = True
        elif key == 'd':
            # Right = increase X (East in NED)
            self.target_pos[0] += self.STEP
            moved = True
        elif key == 'a':
            # Left = decrease X
            self.target_pos[0] -= self.STEP
            moved = True
        elif key == 'r':
            # Up = decrease Z (NED: negative Z = higher altitude)
            self.target_pos[2] -= self.STEP
            moved = True
        elif key == 'f':
            # Down = increase Z (NED: positive Z = lower altitude)
            self.target_pos[2] += self.STEP
            moved = True

        if moved:
            # Clamp to room boundaries
            self.target_pos[0] = np.clip(self.target_pos[0], self.X_MIN, self.X_MAX)
            self.target_pos[1] = np.clip(self.target_pos[1], self.Y_MIN, self.Y_MAX)
            self.target_pos[2] = np.clip(self.target_pos[2], self.Z_MIN, self.Z_MAX)

            # Calculate distance from current to target
            dist = np.linalg.norm(self.current_pos - self.target_pos)

            print(
                f"  🎯 Target: ({self.target_pos[0]:+5.1f}, {self.target_pos[1]:+5.1f}, alt={-self.target_pos[2]:.1f}m) | "
                f"📍 Current: ({self.current_pos[0]:+5.1f}, {self.current_pos[1]:+5.1f}, alt={-self.current_pos[2]:.1f}m) | "
                f"📏 {dist:.1f}m away"
            )

    def takeoff_sequence(self):
        """
        The takeoff sequence:
          1. Send 20 heartbeats (2 seconds) — PX4 needs this warmup
          2. Switch to offboard mode
          3. Arm the motors
          4. Set target to current XY at 1.8m altitude

        Why 20 heartbeats first?
          PX4 has a safety check: it won't accept offboard mode unless
          it has been receiving heartbeats for at least ~1 second. This
          prevents accidental mode switches.
        """
        import time

        # Set the target to hover above current position at 1.8m
        self.target_pos = np.array([0.0, 0.0, -1.8])

        # Send warmup heartbeats
        for i in range(25):
            self.send_heartbeat()
            self.send_position(self.target_pos)

            # Spin once to process any incoming messages
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.05)

        print("   Step 2/3: Switching to OFFBOARD mode...")
        self.send_command(176, param1=1.0, param2=6.0)   # Set offboard mode
        time.sleep(0.5)

        # Keep sending heartbeats during the wait
        for i in range(5):
            self.send_heartbeat()
            self.send_position(self.target_pos)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

        print("   Step 3/3: Arming motors...")
        self.send_command(400, param1=1.0)               # Arm
        time.sleep(0.5)

        self.armed = True
        self.is_flying = True

        print("\n   ✅ AIRBORNE! Use W/A/S/D to move, R/F for altitude, L to land.\n")

    # ─────────────────────────────────────────────────────────────────
    # LOW-LEVEL MESSAGE BUILDERS
    # ─────────────────────────────────────────────────────────────────

    def send_heartbeat(self):
        """Send the offboard control mode heartbeat (position control)."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_pub.publish(msg)

    def send_position(self, pos):
        """Send a target position for the drone to fly to."""
        msg = TrajectorySetpoint()
        msg.position = [float(pos[0]), float(pos[1]), float(pos[2])]
        msg.yaw = 0.0       # Always face north (0 radians)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.setpoint_pub.publish(msg)

    def send_command(self, cmd_id, param1=0.0, param2=0.0):
        """Send a vehicle command (arm, mode switch, land, etc.)."""
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


# ═════════════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════════════

def main():
    # Save terminal settings so we can restore them on exit
    old_settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = KeyboardDroneController()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        print("\n\n👋 Shutting down keyboard controller...")
    finally:
        # CRITICAL: Restore terminal to normal mode!
        # If we don't do this, your terminal will be broken after exit
        # (no echo, no line editing, etc.)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Terminal restored. Done.")


if __name__ == '__main__':
    main()
