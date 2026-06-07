#!/usr/bin/env python3
"""
=============================================================================
 POINT CLOUD FILTER NODE
 File: pointcloud_filter.py
=============================================================================

 WHAT THIS SCRIPT DOES:
   Subscribes to the RAW point cloud from the depth camera, cleans it up
   using two filters, and republishes the filtered version.

 WHY WE NEED THIS:
   The raw point cloud from the depth camera contains:
   - Hundreds of thousands of points (too many for real-time planning)
   - Random noisy points (phantom obstacles that don't exist)

   After filtering, we get:
   - ~10x fewer points (fast enough for path planning)
   - Clean data (only real obstacles remain)

 THE TWO FILTERS:

   1. VOXEL GRID FILTER (Downsampling)
      Imagine dividing the room into tiny 5cm cubes (voxels).
      All points inside each cube get replaced by ONE averaged point.
      Result: way fewer points, same shape.

      Before: 300,000 points  →  After: ~30,000 points

   2. STATISTICAL OUTLIER REMOVAL (Noise cleaning)
      For each point, check its K nearest neighbors.
      If a point is unusually far from its neighbors, it's noise → remove it.

      Before: some random floating points  →  After: only real surfaces

 TOPICS:
   Subscribes to:  /depth_camera/points          (raw from Gazebo via bridge)
   Publishes to:   /depth_camera/points_filtered  (cleaned up)

 HOW TO RUN:
   Make sure the simulation + bridge are running (Terminals 1-4), then:
     cd ~/px4_ros_ws && source install/setup.bash
     python3 ~/Desktop/Drone_IP/pointcloud_filter.py

=============================================================================
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
from sensor_msgs_py import point_cloud2


class PointCloudFilter(Node):
    """
    Subscribes to raw point cloud, applies voxel grid + outlier removal,
    and publishes the filtered result.
    """

    def __init__(self):
        super().__init__('pointcloud_filter')

        # ── Filter parameters ──
        # You can tune these to trade off between speed and detail.

        # Voxel size in meters — smaller = more detail but more points
        self.VOXEL_SIZE = 0.08  # 8cm cubes (good for indoor room)

        # Statistical outlier removal parameters
        self.SOR_K_NEIGHBORS = 10   # Check this many nearest neighbors
        self.SOR_STD_RATIO = 1.5    # Remove points > 1.5 std dev away

        # Maximum range — ignore points further than this (meters)
        self.MAX_RANGE = 8.0  # Our room is 10x8m, so 8m is plenty

        # ── QoS profile ──
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,  # Only keep latest — we don't need old frames
        )

        # ── Subscriber: raw point cloud from bridge ──
        self.sub = self.create_subscription(
            PointCloud2,
            '/depth_camera/points',
            self.pointcloud_callback,
            qos
        )

        # ── Publisher: filtered point cloud ──
        self.pub = self.create_publisher(
            PointCloud2,
            '/depth_camera/points_filtered',
            qos
        )

        # Stats tracking
        self.frame_count = 0

        self.get_logger().info('✅ Point Cloud Filter node started!')
        self.get_logger().info(f'   Voxel size: {self.VOXEL_SIZE}m')
        self.get_logger().info(f'   SOR neighbors: {self.SOR_K_NEIGHBORS}, std ratio: {self.SOR_STD_RATIO}')
        self.get_logger().info(f'   Max range: {self.MAX_RANGE}m')
        self.get_logger().info('   Subscribing to: /depth_camera/points')
        self.get_logger().info('   Publishing to:  /depth_camera/points_filtered')

    def pointcloud_callback(self, msg):
        """
        Called every time a new point cloud frame arrives (~30 fps).

        Pipeline:
          raw points → remove NaN/Inf → clip range → voxel grid → outlier removal → publish
        """

        # ── Step 1: Convert ROS message to numpy array ──
        # point_cloud2.read_points() gives us a generator of (x, y, z, ...) tuples
        points_list = []
        for point in point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True):
            points_list.append([point[0], point[1], point[2]])

        if len(points_list) == 0:
            return

        points = np.array(points_list, dtype=np.float32)
        raw_count = len(points)

        # ── Step 2: Remove invalid points ──
        # Remove any remaining NaN or Inf values
        valid_mask = np.isfinite(points).all(axis=1)
        points = points[valid_mask]

        # ── Step 3: Clip range ──
        # Remove points beyond MAX_RANGE (they're too far to matter)
        distances = np.linalg.norm(points, axis=1)
        range_mask = distances < self.MAX_RANGE
        points = points[range_mask]

        if len(points) < 10:
            return  # Not enough points to filter

        # ── Step 4: Voxel Grid Filter ──
        points = self.voxel_grid_filter(points)

        if len(points) < 10:
            return

        # ── Step 5: Statistical Outlier Removal ──
        points = self.statistical_outlier_removal(points)

        if len(points) == 0:
            return

        # ── Step 6: Publish filtered point cloud ──
        filtered_msg = self.numpy_to_pointcloud2(points, msg.header)
        self.pub.publish(filtered_msg)

        # Print stats every 30 frames (~1 second)
        self.frame_count += 1
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'📊 Frame {self.frame_count}: '
                f'{raw_count} raw → {len(points)} filtered '
                f'({100 * len(points) / max(raw_count, 1):.0f}% kept)'
            )

    # ─────────────────────────────────────────────────────────────────────
    # FILTER 1: VOXEL GRID
    # ─────────────────────────────────────────────────────────────────────

    def voxel_grid_filter(self, points):
        """
        Downsample by dividing space into cubes and averaging points per cube.

        How it works:
          1. Divide each (x, y, z) by voxel_size and round to get a grid index
          2. Group all points that fall in the same grid cell
          3. Replace each group with its average (centroid)

        Example with voxel_size = 0.08m:
          A 10m room has 10/0.08 = 125 cells per axis
          Total possible cells: 125³ = ~2 million
          But only cells containing points are kept
        """
        # Convert continuous coordinates to discrete grid indices
        voxel_indices = np.floor(points / self.VOXEL_SIZE).astype(np.int32)

        # Create a unique key for each voxel using a hash-like approach
        # Shift indices to be positive first
        voxel_indices -= voxel_indices.min(axis=0)

        # Create unique integer key for each voxel: x * M*N + y * N + z
        dims = voxel_indices.max(axis=0) + 1
        keys = (voxel_indices[:, 0] * dims[1] * dims[2] +
                voxel_indices[:, 1] * dims[2] +
                voxel_indices[:, 2])

        # Find unique voxels and average the points within each
        unique_keys, inverse = np.unique(keys, return_inverse=True)
        n_voxels = len(unique_keys)

        # Sum all points per voxel, then divide by count to get average
        voxel_sums = np.zeros((n_voxels, 3), dtype=np.float64)
        voxel_counts = np.zeros(n_voxels, dtype=np.int32)

        np.add.at(voxel_sums, inverse, points)
        np.add.at(voxel_counts, inverse, 1)

        centroids = (voxel_sums / voxel_counts[:, np.newaxis]).astype(np.float32)

        return centroids

    # ─────────────────────────────────────────────────────────────────────
    # FILTER 2: STATISTICAL OUTLIER REMOVAL
    # ─────────────────────────────────────────────────────────────────────

    def statistical_outlier_removal(self, points):
        """
        Remove points that are unusually far from their neighbors.

        How it works:
          1. For each point, find its K nearest neighbors
          2. Calculate the mean distance to those neighbors
          3. Compute the overall mean and std of all mean-distances
          4. Remove points where mean_distance > global_mean + std_ratio * global_std

        These "lonely" points are usually noise — real surfaces have
        many points close together.
        """
        n_points = len(points)

        # For very large point clouds, limit to avoid slowness
        # (after voxel filtering this should already be manageable)
        if n_points > 15000:
            # Skip SOR for very large clouds — voxel filter is enough
            return points

        k = min(self.SOR_K_NEIGHBORS, n_points - 1)
        if k < 2:
            return points

        # Calculate distance from each point to all other points
        # For efficiency, we process in batches
        mean_distances = np.zeros(n_points, dtype=np.float32)

        batch_size = 1000
        for i in range(0, n_points, batch_size):
            end = min(i + batch_size, n_points)
            batch = points[i:end]

            # Distance from each point in batch to ALL points
            # Shape: (batch_size, n_points)
            diffs = batch[:, np.newaxis, :] - points[np.newaxis, :, :]
            dists = np.linalg.norm(diffs, axis=2)

            # For each point, sort distances and take the K smallest
            # (excluding distance to self, which is 0)
            sorted_dists = np.sort(dists, axis=1)
            # Skip index 0 (distance to self = 0), take next K
            k_nearest = sorted_dists[:, 1:k+1]

            mean_distances[i:end] = k_nearest.mean(axis=1)

        # Calculate global statistics
        global_mean = mean_distances.mean()
        global_std = mean_distances.std()

        # Keep points where mean distance is within threshold
        threshold = global_mean + self.SOR_STD_RATIO * global_std
        inlier_mask = mean_distances < threshold

        return points[inlier_mask]

    # ─────────────────────────────────────────────────────────────────────
    # HELPER: Convert numpy array back to ROS PointCloud2 message
    # ─────────────────────────────────────────────────────────────────────

    def numpy_to_pointcloud2(self, points, header):
        """
        Convert an Nx3 numpy array of (x, y, z) points into a ROS PointCloud2 msg.
        """
        msg = PointCloud2()
        msg.header = header

        msg.height = 1
        msg.width = len(points)

        # Define the fields: x, y, z — each is a 32-bit float (4 bytes)
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 12        # 3 floats × 4 bytes = 12 bytes per point
        msg.row_step = 12 * len(points)
        msg.data = points.tobytes()
        msg.is_dense = True

        return msg


# ═════════════════════════════════════════════════════════════════════
# MAIN
# ═════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = PointCloudFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n👋 Shutting down Point Cloud Filter...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("✅ Done.")


if __name__ == '__main__':
    main()
