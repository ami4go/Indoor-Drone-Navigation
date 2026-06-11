#!/usr/bin/env python3
"""
=============================================================================
 TF BROADCASTER NODE (Ground Truth from Gazebo)
 File: tf_broadcaster.py
=============================================================================

 Reads the drone's exact position from Gazebo's dynamic_pose topic
 (bridged to ROS 2 as TFMessage) and re-publishes it with correct
 frame_ids so OctoMap can locate the camera in the world.

 Why Gazebo pose instead of PX4 odometry?
   PX4's EKF has a different coordinate origin than Gazebo, and uses
   NED vs Gazebo's ENU convention. Using Gazebo's ground-truth pose
   eliminates ALL coordinate conversion issues.

 Timestamp synchronization:
   We cache the latest Gazebo pose and publish TF whenever a new
   point cloud arrives, using the point cloud's own timestamp.
   This guarantees OctoMap can always look up TF for any point cloud.

=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster


class TFBroadcasterNode(Node):
    """
    Publishes TF transforms using Gazebo ground-truth pose.

    Transform chain:
      map (world) → base_link (drone body) → camera_frame (depth camera)
    """

    def __init__(self):
        super().__init__('drone_tf_broadcaster')

        # ── Cached Gazebo pose ──
        self.gz_pose = None  # Latest drone position from Gazebo

        # ── Camera frame name ──
        self.camera_frame = None
        self.camera_frame_detected = False

        # ── Subscribe to bridged Gazebo dynamic_pose (TFMessage) ──
        self.gz_sub = self.create_subscription(
            TFMessage,
            '/world/indoor_10x8x3/dynamic_pose/info',
            self.gz_pose_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
            )
        )

        # ── Subscribe to raw point cloud for timestamps + frame detection ──
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.pointcloud_callback,
            QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
            )
        )

        # ── TF Broadcasters ──
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)

        # ── Camera offset (from Gazebo dynamic_pose data) ──
        # OakD-Lite/base_link is at (0.12, 0.03, 0.242) relative to model root
        self.CAMERA_X = 0.12
        self.CAMERA_Y = 0.03
        self.CAMERA_Z = 0.242

        self.get_logger().info('✅ TF Broadcaster started!')
        self.get_logger().info('   Subscribing to: /world/.../dynamic_pose/info (Gazebo ground truth)')
        self.get_logger().info('   Subscribing to: /depth_camera/points (for timestamps)')

    def gz_pose_callback(self, msg):
        """
        Cache the drone's pose from the bridged Gazebo dynamic_pose topic.
        The first transform in the TFMessage is the drone model (x500_depth_0).
        """
        if len(msg.transforms) > 0:
            # First transform = x500_depth_0 model pose in world
            t = msg.transforms[0]
            self.gz_pose = {
                'x': t.transform.translation.x,
                'y': t.transform.translation.y,
                'z': t.transform.translation.z,
                'qx': t.transform.rotation.x,
                'qy': t.transform.rotation.y,
                'qz': t.transform.rotation.z,
                'qw': t.transform.rotation.w,
            }

    def pointcloud_callback(self, msg):
        """
        On each point cloud message:
        1. Detect camera frame name (once)
        2. Publish TF using the point cloud's timestamp + cached Gazebo pose
        """
        # ── Detect camera frame ──
        if not self.camera_frame_detected:
            self.camera_frame = msg.header.frame_id
            self.camera_frame_detected = True
            self.get_logger().info(f'   📷 Detected camera frame: "{self.camera_frame}"')
            self.publish_static_camera_tf(msg.header.stamp)

        # ── Publish dynamic TF ──
        if self.gz_pose is not None:
            self.publish_dynamic_tf(msg.header.stamp)

    def publish_static_camera_tf(self, stamp):
        """Static transform: base_link → camera_frame (camera mounted on drone)."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'base_link'
        t.child_frame_id = self.camera_frame

        t.transform.translation.x = self.CAMERA_X
        t.transform.translation.y = self.CAMERA_Y
        t.transform.translation.z = self.CAMERA_Z
        t.transform.rotation.w = 1.0

        self.static_tf_broadcaster.sendTransform(t)
        self.get_logger().info(f'   📌 Published static TF: base_link → {self.camera_frame}')

    def publish_dynamic_tf(self, stamp):
        """Dynamic transform: map → base_link (drone position in world)."""
        t = TransformStamped()
        t.header.stamp = stamp  # Use point cloud's timestamp for perfect sync
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.gz_pose['x']
        t.transform.translation.y = self.gz_pose['y']
        t.transform.translation.z = self.gz_pose['z']
        t.transform.rotation.x = self.gz_pose['qx']
        t.transform.rotation.y = self.gz_pose['qy']
        t.transform.rotation.z = self.gz_pose['qz']
        t.transform.rotation.w = self.gz_pose['qw']

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
