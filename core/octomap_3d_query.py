#!/usr/bin/env python3
"""
=============================================================================
 3D OctoMap Query Interface
 File: core/octomap_3d_query.py
=============================================================================

 Provides fast 3D occupancy queries on the live OctoMap by subscribing to
 /octomap_point_cloud_centers (PointCloud2 of all occupied voxel centers)
 and building a voxel hash set for O(1) spatial lookups.

 Uses a discretized hash approach (no scipy/sklearn dependency).

 Usage (imported by navigators):
   from core.octomap_3d_query import OctoMap3DQuery
   query = OctoMap3DQuery()
   # Feed it PointCloud2 data from the navigator's subscription:
   query.update_from_pointcloud2(pc2_msg)
   # Then query:
   query.is_occupied(x, y, z)
   query.is_column_clear(x, y, z_min, z_max)
   query.safe_altitude(x, y, desired_z)

=============================================================================
"""

import math
import numpy as np


class OctoMap3DQuery:
    """
    Fast 3D occupancy queries using a voxel hash set built from OctoMap's
    occupied voxel centers PointCloud2.

    Uses discretized voxel keys (grid indices) stored in a Python set
    for O(1) occupancy lookups. No external dependencies beyond numpy.
    """

    def __init__(self, resolution=0.10):
        """
        Args:
            resolution: OctoMap voxel resolution in metres (must match
                        the server's config — default 0.10m).
        """
        self.resolution = resolution
        self.voxel_centers = None     # Nx3 numpy array of (x, y, z)
        self.occupied_set = set()     # Set of (ix, iy, iz) voxel keys
        self._last_stamp = None       # Avoid rebuilding on duplicate msgs
        self.REBUILD_INTERVAL = 2.0   # Min seconds (msg stamp) between rebuilds
        self._last_rebuild_time = None

    # ── Data Ingestion ────────────────────────────────────────────────────

    def _world_to_key(self, x, y, z):
        """Convert world coordinates to discrete voxel key."""
        ix = int(math.floor(x / self.resolution))
        iy = int(math.floor(y / self.resolution))
        iz = int(math.floor(z / self.resolution))
        return (ix, iy, iz)

    def update_from_pointcloud2(self, pc2_msg):
        """
        Parse a sensor_msgs/PointCloud2 message from
        /octomap_point_cloud_centers and rebuild the voxel hash set.

        This should be called from the navigator's subscription callback.
        Fully vectorized (numpy) — safe to call from the same executor as
        the 20Hz control loop. Rebuilds are throttled to REBUILD_INTERVAL
        seconds (based on message stamps) since the map changes slowly.
        """
        # Avoid rebuilding if same timestamp
        stamp = (pc2_msg.header.stamp.sec, pc2_msg.header.stamp.nanosec)
        if stamp == self._last_stamp:
            return
        self._last_stamp = stamp

        # Throttle: the navigator only queries every ~2s, so skip
        # intermediate messages instead of rebuilding on every one.
        t = stamp[0] + stamp[1] * 1e-9
        if (self._last_rebuild_time is not None
                and self.occupied_set
                and abs(t - self._last_rebuild_time) < self.REBUILD_INTERVAL):
            return
        self._last_rebuild_time = t

        # Parse the PointCloud2 binary data
        points = self._parse_pc2(pc2_msg)

        if points is None or len(points) == 0:
            self.voxel_centers = None
            self.occupied_set = set()
            return

        self.voxel_centers = points

        # Build the hash set of occupied voxel keys (vectorized)
        keys = np.floor(points / self.resolution).astype(np.int64)
        self.occupied_set = set(map(tuple, keys.tolist()))

    def _parse_pc2(self, msg):
        """
        Extract (x, y, z) points from a PointCloud2 message.
        Handles both structured (with named fields) and simple XYZ layouts.
        Vectorized: reshapes the raw buffer and views the float32 columns.
        """
        # Find field offsets for x, y, z
        offsets = {}
        for field in msg.fields:
            if field.name in ('x', 'y', 'z'):
                offsets[field.name] = field.offset

        if len(offsets) < 3:
            # Fallback: assume tightly packed float32 XYZ
            offsets = {'x': 0, 'y': 4, 'z': 8}

        point_step = msg.point_step
        data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        n_points = len(data) // point_step

        if n_points == 0:
            return None

        # Reshape to (n_points, point_step) and view each 4-byte column
        # as float32 — no per-point Python loop.
        rows = data[:n_points * point_step].reshape(n_points, point_step)
        points = np.empty((n_points, 3), dtype=np.float32)
        for j, name in enumerate(('x', 'y', 'z')):
            off = offsets[name]
            points[:, j] = rows[:, off:off + 4].copy().view(np.float32)[:, 0]

        # Filter out NaN/Inf
        valid = np.isfinite(points).all(axis=1)
        return points[valid]

    @property
    def is_ready(self):
        """True if we have valid 3D data to query."""
        return len(self.occupied_set) > 0

    @property
    def num_voxels(self):
        """Number of occupied voxels in the current map."""
        return len(self.occupied_set)

    # ── Point Queries (O(1) via hash set) ─────────────────────────────────

    def is_occupied(self, x, y, z, check_neighbors=True):
        """
        Check if the voxel at world coordinates (x, y, z) is occupied.

        Args:
            x, y, z: World coordinates (metres).
            check_neighbors: If True, also check the 6 face-adjacent
                             voxels to handle boundary cases.

        Returns:
            True if an occupied voxel is at or adjacent to (x, y, z).
        """
        if not self.is_ready:
            return False

        key = self._world_to_key(x, y, z)

        if key in self.occupied_set:
            return True

        if check_neighbors:
            ix, iy, iz = key
            for dx, dy, dz in [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)]:
                if (ix+dx, iy+dy, iz+dz) in self.occupied_set:
                    return True

        return False

    def nearest_obstacle_distance(self, x, y, z):
        """
        Return the approximate distance (metres) from (x, y, z) to the
        nearest occupied voxel. Uses brute-force numpy (fast for <50K voxels).

        Returns float('inf') if no data.
        """
        if not self.is_ready or self.voxel_centers is None:
            return float('inf')

        diff = self.voxel_centers - np.array([x, y, z], dtype=np.float32)
        dists = np.sqrt(np.sum(diff**2, axis=1))
        return float(dists.min())

    def obstacles_in_radius(self, x, y, z, radius):
        """
        Return all occupied voxel centers within `radius` metres
        of (x, y, z).

        Returns:
            Nx3 numpy array of occupied voxel centers, or empty array.
        """
        if not self.is_ready or self.voxel_centers is None:
            return np.empty((0, 3))

        diff = self.voxel_centers - np.array([x, y, z], dtype=np.float32)
        dists_sq = np.sum(diff**2, axis=1)
        mask = dists_sq <= radius**2
        return self.voxel_centers[mask]

    # ── Column Queries (for altitude planning) ────────────────────────────

    def is_column_clear(self, x, y, z_min, z_max, xy_radius=0.3):
        """
        Check if the vertical column at (x, y) is free of obstacles
        between z_min and z_max.

        Args:
            x, y: Horizontal world coordinates.
            z_min, z_max: Altitude range to check (metres).
            xy_radius: Horizontal search radius around (x, y) to account
                       for drone width (default 0.3m ≈ drone radius).

        Returns:
            True if the column is completely clear.
        """
        if not self.is_ready:
            return True  # No data → assume clear

        res = self.resolution
        # Convert to voxel index ranges
        ix_center = int(math.floor(x / res))
        iy_center = int(math.floor(y / res))
        iz_min = int(math.floor(z_min / res))
        iz_max = int(math.ceil(z_max / res))
        r_cells = int(math.ceil(xy_radius / res))

        # Check all voxels in the column volume
        for ix in range(ix_center - r_cells, ix_center + r_cells + 1):
            for iy in range(iy_center - r_cells, iy_center + r_cells + 1):
                # Check XY distance
                dx = (ix - ix_center) * res
                dy = (iy - iy_center) * res
                if dx*dx + dy*dy > xy_radius * xy_radius:
                    continue
                for iz in range(iz_min, iz_max + 1):
                    if (ix, iy, iz) in self.occupied_set:
                        return False

        return True

    def safe_altitude(self, x, y, desired_z, clearance=0.3,
                      z_min=0.8, z_max=2.5, z_step=0.2):
        """
        Find the safest altitude at (x, y) near desired_z.

        Checks altitudes starting from desired_z, then alternating
        down and up, returning the first altitude where the column
        is clear (with drone clearance above and below).

        Args:
            x, y: Horizontal position.
            desired_z: Preferred altitude (metres).
            clearance: Vertical clearance above and below drone (metres).
            z_min: Minimum allowed altitude.
            z_max: Maximum allowed altitude (below ceiling).
            z_step: Step size for searching alternatives.

        Returns:
            Best safe altitude (float), or desired_z if no data / all clear.
        """
        if not self.is_ready:
            return desired_z

        # Check desired altitude first
        if self.is_column_clear(x, y, desired_z - clearance,
                                desired_z + clearance):
            return desired_z

        # Search alternating down/up from desired altitude
        # Try below first — descending is cheaper for battery
        max_steps = int((z_max - z_min) / z_step) + 1
        for offset_steps in range(1, max_steps):
            for sign in [-1, +1]:
                candidate = desired_z + sign * offset_steps * z_step

                if candidate < z_min or candidate > z_max:
                    continue

                if self.is_column_clear(x, y, candidate - clearance,
                                        candidate + clearance):
                    return candidate

        # No safe altitude found — return desired and let avoidance handle it
        return desired_z

    # ── Height Profile (for visualization/debugging) ──────────────────────

    def height_profile(self, x, y, xy_radius=0.3):
        """
        Get all obstacle heights at position (x, y).

        Returns:
            Sorted list of z-values where obstacles exist near (x, y).
        """
        if not self.is_ready or self.voxel_centers is None:
            return []

        dx = self.voxel_centers[:, 0] - x
        dy = self.voxel_centers[:, 1] - y
        xy_dist_sq = dx**2 + dy**2

        in_xy = xy_dist_sq <= xy_radius**2
        heights = self.voxel_centers[in_xy, 2]

        return sorted(heights.tolist())

    # ── Debug Info ────────────────────────────────────────────────────────

    def debug_summary(self):
        """Print a summary of the current 3D map state."""
        if not self.is_ready:
            print("  OctoMap3DQuery: No data loaded.")
            return

        pts = self.voxel_centers
        print(f"  OctoMap3DQuery: {len(self.occupied_set)} occupied voxels")
        print(f"    X range: [{pts[:, 0].min():.2f}, {pts[:, 0].max():.2f}]")
        print(f"    Y range: [{pts[:, 1].min():.2f}, {pts[:, 1].max():.2f}]")
        print(f"    Z range: [{pts[:, 2].min():.2f}, {pts[:, 2].max():.2f}]")

        # Count voxels at different height bands
        low = np.sum(pts[:, 2] < 1.0)
        mid = np.sum((pts[:, 2] >= 1.0) & (pts[:, 2] < 2.0))
        high = np.sum(pts[:, 2] >= 2.0)
        print(f"    Ground (<1m): {low} | Mid (1-2m): {mid} | Ceiling (>2m): {high}")
