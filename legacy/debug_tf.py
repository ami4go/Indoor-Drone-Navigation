#!/usr/bin/env python3
"""
Quick diagnostic: prints PX4 position vs what Gazebo expects.
Run alongside the simulation. Take off and fly near the RED PILLAR (Gazebo 0, 1).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry
from sensor_msgs.msg import PointCloud2

class DebugTF(Node):
    def __init__(self):
        super().__init__('debug_tf')
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=5)
        self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.odom_cb, qos)
        self.create_subscription(PointCloud2, '/depth_camera/points', self.pc_cb, qos)
        self.timer = self.create_timer(1.0, self.print_info)
        self.pos = None; self.quat = None; self.pc_frame = None; self.pc_stamp = None

    def odom_cb(self, msg):
        self.pos = msg.position
        self.quat = msg.q

    def pc_cb(self, msg):
        self.pc_frame = msg.header.frame_id
        self.pc_stamp = msg.header.stamp

    def print_info(self):
        if self.pos is not None:
            print(f"\n{'='*60}")
            print(f"PX4 Raw:  X={self.pos[0]:+.3f}  Y={self.pos[1]:+.3f}  Z={self.pos[2]:+.3f}")
            print(f"          Q: w={self.quat[0]:+.4f} x={self.quat[1]:+.4f} y={self.quat[2]:+.4f} z={self.quat[3]:+.4f}")
            print(f"")
            print(f"If NED→ENU (swap X↔Y, negate Z):")
            print(f"  Gazebo:  X={self.pos[1]:+.3f}  Y={self.pos[0]:+.3f}  Z={-self.pos[2]:+.3f}")
            print(f"")
            print(f"If only negate Z:")
            print(f"  Gazebo:  X={self.pos[0]:+.3f}  Y={self.pos[1]:+.3f}  Z={-self.pos[2]:+.3f}")
            print(f"")
            print(f"KNOWN POSITIONS:")
            print(f"  Start marker:  Gazebo (X=-3, Y=-2)")
            print(f"  Red pillar:    Gazebo (X= 0, Y= 1)")
            print(f"  Destination:   Gazebo (X= 3, Y= 2)")
        if self.pc_frame:
            print(f"\nPointCloud frame_id: {self.pc_frame}")
            if self.pc_stamp:
                print(f"PointCloud stamp:    {self.pc_stamp.sec}.{self.pc_stamp.nanosec}")

def main():
    rclpy.init()
    node = DebugTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
