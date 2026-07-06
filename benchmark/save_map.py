#!/usr/bin/env python3
"""
=============================================================================
 MAP SAVER — Save OctoMap's OccupancyGrid to disk for offline benchmarking
 File: benchmark/save_map.py
=============================================================================

 Usage:
   1. Run the simulation with any algo: launch_sim.sh --auto
   2. Wait for exploration to complete
   3. In a NEW terminal:
        cd ~/px4_ros_ws && source install/setup.bash
        python3 ~/Desktop/Drone_IP/benchmark/save_map.py
   4. It saves the map to benchmark/saved_map.npz
   5. Use run_benchmark.py to test all algorithms on this saved map

=============================================================================
"""

import sys
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapSaver(Node):

    def __init__(self):
        super().__init__('map_saver')
        self.map_received = False
        self.sub = self.create_subscription(
            OccupancyGrid, '/projected_map',
            self.map_cb, 10)
        print("\n  Waiting for /projected_map from OctoMap...")
        print("  (Make sure the simulation is running and exploration is done)\n")

    def map_cb(self, msg):
        if self.map_received:
            return
        self.map_received = True

        info = msg.info
        data = np.array(msg.data, dtype=np.int8)
        occupied = np.sum(data > 50)

        print(f"  Map received!")
        print(f"    Size: {info.width} x {info.height} cells")
        print(f"    Resolution: {info.resolution:.2f}m")
        print(f"    Origin: ({info.origin.position.x:.2f}, {info.origin.position.y:.2f})")
        print(f"    Occupied cells: {occupied}")

        # Save as compressed numpy archive
        save_path = '/home/amit/Desktop/Drone_IP/benchmark/saved_map.npz'
        np.savez_compressed(
            save_path,
            data=data,
            width=info.width,
            height=info.height,
            resolution=info.resolution,
            origin_x=info.origin.position.x,
            origin_y=info.origin.position.y
        )

        print(f"\n  Saved to: {save_path}")
        print(f"  Now run: python3 ~/Desktop/Drone_IP/benchmark/run_benchmark.py\n")

        # Shutdown after saving
        rclpy.shutdown()


def main():
    rclpy.init()
    node = MapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
