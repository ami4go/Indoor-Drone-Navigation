#!/usr/bin/env python3
"""
=============================================================================
 TF BROADCASTER NODE
 File: tf_broadcaster.py
=============================================================================

 WHAT THIS SCRIPT DOES:
   Reads the drone's position from PX4 (via /fmu/out/vehicle_odometry)
   and publishes it as a TF transform so OctoMap knows where the camera
   is in the world.

 WHY WE NEED THIS:
   The depth camera sees points RELATIVE to itself ("2m ahead, 1m left").
   OctoMap needs points in WORLD coordinates ("at position (3, 4) in the room").

   TF (Transform Framework) connects the two:
     map frame (room) ──TF──> camera frame (on drone)

   OctoMap then automatically does the math:
     world_point = drone_position + camera_offset + local_point

 TRANSFORMS PUBLISHED:
   map → base_link          (drone's position and orientation in the world)
   base_link → camera_frame (fixed offset: camera is mounted on the drone)

 HOW TO RUN:
   cd ~/px4_ros_ws && source install/setup.bash
   python3 ~/Desktop/Drone_IP/tf_broadcaster.py

=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import PointCloud2

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

import math


class TFBroadcasterNode(Node):
    """
    Publishes TF transforms that connect the camera frame to the world frame.

    Transform chain:
      map (world) → base_link (drone body) → camera_frame (depth camera)
    """

    def __init__(self):
        super().__init__('drone_tf_broadcaster')

        # ── QoS matching PX4 ──
        qos_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # ── Camera frame name ──
        # This is the frame_id that Gazebo puts in the point cloud header.
        # We detect it automatically from the first point cloud message.
        self.camera_frame = None
        self.camera_frame_detected = False

        # Subscribe to point cloud just to read its frame_id (then unsubscribe)
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.detect_camera_frame,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
        )

        # ── Subscribe to PX4 odometry ──
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos_px4
        )

        # ── TF Broadcasters ──
        # Dynamic: map → base_link (changes every frame as drone moves)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Static: base_link → camera_frame (fixed — camera is bolted to drone)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.static_tf_published = False

        # ── Camera offset on the drone body ──
        # From the x500_depth model SDF: <pose>.12 .03 .242 0 0 0</pose>
        # Camera is 12cm forward, 3cm right, 24.2cm above drone center
        self.CAMERA_X = 0.12   # forward
        self.CAMERA_Y = 0.03   # right
        self.CAMERA_Z = -0.242  # up (negative in NED = up)

        self.get_logger().info('✅ TF Broadcaster started!')
        self.get_logger().info('   Subscribing to: /fmu/out/vehicle_odometry')
        self.get_logger().info('   Waiting to detect camera frame from /depth_camera/points...')

    def detect_camera_frame(self, msg):
        """
        Read the frame_id from the first point cloud message.
        This tells us what Gazebo named the camera frame.
        """
        if not self.camera_frame_detected:
            self.camera_frame = msg.header.frame_id
            self.camera_frame_detected = True
            self.get_logger().info(f'   📷 Detected camera frame: "{self.camera_frame}"')

            # Now publish the static transform: base_link → camera_frame
            self.publish_static_camera_tf()

            # We don't need to keep listening to point cloud
            self.destroy_subscription(self.pc_sub)

    def publish_static_camera_tf(self):
        """
        Publish a STATIC transform from base_link to the camera frame.
        This never changes because the camera is physically bolted to the drone.
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = self.camera_frame

        # Camera position relative to drone center
        t.transform.translation.x = self.CAMERA_X
        t.transform.translation.y = self.CAMERA_Y
        t.transform.translation.z = self.CAMERA_Z

        # Camera orientation relative to drone (no rotation — camera faces forward)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'   📌 Published static TF: base_link → {self.camera_frame}')

    def odom_callback(self, msg):
        """
        Called every time PX4 reports the drone's position (~30 Hz).

        PX4 uses NED (North-East-Down) coordinates:
          position[0] = X (North)
          position[1] = Y (East)
          position[2] = Z (Down — negative means above ground)

          q[0] = w, q[1] = x, q[2] = y, q[3] = z  (quaternion orientation)

        We publish a TF from 'map' to 'base_link' using these values.
        Since both Gazebo and PX4 share the same coordinate system in SITL,
        we use the values directly.
        """
        if not self.camera_frame_detected:
            return  # Wait until we know the camera frame name

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        # ── Position ──
        # PX4 SITL position maps directly to Gazebo world coordinates
        t.transform.translation.x = float(msg.position[0])
        t.transform.translation.y = float(msg.position[1])
        t.transform.translation.z = float(msg.position[2])

        # ── Orientation ──
        # PX4 quaternion: [w, x, y, z]
        t.transform.rotation.w = float(msg.q[0])
        t.transform.rotation.x = float(msg.q[1])
        t.transform.rotation.y = float(msg.q[2])
        t.transform.rotation.z = float(msg.q[3])

        self.tf_broadcaster.sendTransform(t)


# ═════════════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = TFBroadcasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Shutting down TF Broadcaster...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Done.")


if __name__ == '__main__':
    main()
