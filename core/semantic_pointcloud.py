#!/usr/bin/env python3
"""
=============================================================================
 Semantic Point Cloud Pipeline
 File: core/semantic_pointcloud.py
=============================================================================

 Combines YOLOv8 semantic segmentation with depth point cloud fusion in a
 single ROS 2 node:

   1. Subscribes to /camera (RGB 640×480) and /depth_camera/points (PointCloud2)
   2. Runs YOLOv8n-seg on the RGB frame (GPU-accelerated, ~5ms on RTX 4050)
   3. Builds a semantic label mask (per-pixel class ID → color)
   4. Projects labels onto the depth point cloud using camera intrinsics
   5. Publishes /semantic/colored_points (XYZRGB PointCloud2) for OctoMap
   6. Publishes /semantic/overlay (annotated RGB image for RViz debugging)

 The OctoMap server subscribes to /semantic/colored_points with
 colored_map:=true, giving colored 3D voxels in RViz.

=============================================================================
"""

import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from sensor_msgs.msg import Image, PointCloud2, PointField, CameraInfo
from std_msgs.msg import Header, String

# YOLOv8 imports (loaded lazily for faster node startup)
_yolo_model = None


def get_yolo_model():
    """Lazy-load YOLOv8n-seg model on first use."""
    global _yolo_model
    if _yolo_model is None:
        from ultralytics import YOLO
        _yolo_model = YOLO('yolov8n-seg.pt')
        # Warm up with a dummy inference
        import torch
        dummy = torch.zeros((1, 3, 480, 640), device='cuda' if torch.cuda.is_available() else 'cpu')
        _yolo_model.predict(source=dummy, verbose=False)
        print("  ✅ YOLOv8n-seg model loaded and warmed up!")
    return _yolo_model


# ─────────────────────────────────────────────────────────────────────────────
#  SEMANTIC COLOR MAP
# ─────────────────────────────────────────────────────────────────────────────
# COCO class IDs → semantic category + display color
# YOLOv8 uses 80 COCO classes. We map them to broad indoor categories.

# Default color for unknown objects
DEFAULT_COLOR = (80, 80, 80)      # Dark grey

# COCO class ID → (R, G, B) color
COCO_COLORS = {
    # Furniture
    56: (139, 90, 43),    # chair → brown
    57: (139, 90, 43),    # couch/sofa → brown
    59: (139, 69, 19),    # bed → dark brown
    60: (160, 82, 45),    # dining table → sienna
    61: (139, 90, 43),    # toilet → brown

    # Electronics
    62: (128, 0, 128),    # tv → purple
    63: (128, 0, 128),    # laptop → purple
    64: (128, 0, 128),    # mouse → purple
    65: (128, 0, 128),    # remote → purple
    66: (128, 0, 128),    # keyboard → purple
    67: (128, 0, 128),    # cell phone → purple
    68: (128, 0, 128),    # microwave → purple
    69: (128, 0, 128),    # oven → purple
    71: (128, 0, 128),    # sink → purple
    72: (128, 0, 128),    # refrigerator → purple

    # Books / shelves
    73: (210, 180, 140),  # book → tan
    74: (210, 180, 140),  # clock → tan
    75: (210, 180, 140),  # vase → tan

    # People / animals (safety-critical → red)
    0:  (255, 50, 50),    # person → red
    15: (255, 80, 80),    # cat → red-ish
    16: (255, 80, 80),    # dog → red-ish

    # Vehicles (unlikely indoors, but just in case)
    2:  (200, 200, 50),   # car → yellow
    3:  (200, 200, 50),   # motorcycle → yellow

    # Containers
    24: (100, 149, 237),  # backpack → cornflower blue
    26: (100, 149, 237),  # handbag → cornflower blue
    28: (100, 149, 237),  # suitcase → cornflower blue

    # Sports / misc
    32: (34, 139, 34),    # sports ball → green
    70: (200, 200, 200),  # toaster → light grey
    76: (200, 200, 200),  # scissors → light grey
}

# Category name lookup for logging
COCO_NAMES = {
    0: 'person', 56: 'chair', 57: 'sofa', 59: 'bed', 60: 'table',
    62: 'tv', 72: 'fridge', 73: 'book', 75: 'vase',
}

# Wall/floor/ceiling detection — these aren't COCO classes, so we handle
# them by looking at unclassified large regions in the depth image:
WALL_COLOR = (50, 100, 220)       # Blue
FLOOR_COLOR = (180, 180, 180)     # Light grey
CEILING_COLOR = (220, 220, 220)   # Near white
OBSTACLE_COLOR = (255, 60, 60)    # Red (pipes, fans, unknown obstacles)


class SemanticPointCloud(Node):
    """
    ROS 2 node that runs YOLOv8 segmentation on the RGB camera feed and
    fuses the semantic labels with the depth point cloud to produce
    colored XYZRGB points for the OctoMap.
    """

    def __init__(self):
        super().__init__('semantic_pointcloud')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        # ── Subscribers ──
        self.rgb_sub = self.create_subscription(
            Image, '/camera', self.rgb_cb, qos)
        self.pc2_sub = self.create_subscription(
            PointCloud2, '/depth_camera/points', self.pc2_cb, qos)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera_info', self.cam_info_cb, qos)
        # Latched (transient local) so we get the current state even if the
        # navigator published it before this node started
        nav_state_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1)
        self.nav_state_sub = self.create_subscription(
            String, '/nav_state', self.nav_state_cb, nav_state_qos)

        # ── Publishers ──
        self.colored_pc_pub = self.create_publisher(
            PointCloud2, '/semantic/colored_points', 10)
        self.overlay_pub = self.create_publisher(
            Image, '/semantic/overlay', 10)

        # ── State ──
        self.latest_rgb = None         # Raw RGB image (numpy HxWx3)
        self.latest_rgb_stamp = None   # Timestamp for sync
        self.latest_pc2 = None         # Raw PointCloud2 message
        self.camera_info = None        # CameraInfo for projection
        self.label_mask = None         # Per-pixel semantic label mask (HxWx3 RGB)

        # Processing control — don't run YOLO on every frame
        self.frame_count = 0
        self.PROCESS_EVERY_N = 5       # Run YOLO every 5th frame (~6 FPS)

        # Exploration gate — pause YOLO while the navigator is mapping so
        # the semantic pipeline doesn't compete with OctoMap for CPU/GPU.
        # Activates when /nav_state reports READY (exploration complete)
        # or NAVIGATE. If no navigator is running (standalone use), start
        # anyway after STANDALONE_TIMEOUT seconds.
        self.nav_state = None
        self.active = False
        self.STANDALONE_TIMEOUT = 60.0
        self._start_time = time.monotonic()
        self._pause_notified = False

        # Stats
        self.total_inferences = 0
        self.objects_detected = {}     # class_name → count
        self._first_publish = True     # Log first successful publish

        print("\n╔══════════════════════════════════════════════════════════╗")
        print("║   🧠 SEMANTIC POINT CLOUD — YOLOv8n-seg + Depth Fusion ║")
        print("╠══════════════════════════════════════════════════════════╣")
        print("║  /camera (RGB) → YOLOv8 → semantic labels              ║")
        print("║  /depth_camera/points → fuse → /semantic/colored_points ║")
        print("║  OctoMap server uses colored_map:=true                  ║")
        print("╚══════════════════════════════════════════════════════════╝\n")
        print("  ⏳ Waiting for camera data...")

    # ── Callbacks ─────────────────────────────────────────────────────────

    def rgb_cb(self, msg):
        """Receive RGB image from Gazebo camera."""
        try:
            # Parse sensor_msgs/Image → numpy array
            h, w = msg.height, msg.width
            if msg.encoding == 'rgb8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
            elif msg.encoding == 'bgr8':
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                img = img[:, :, ::-1].copy()  # BGR → RGB
            elif msg.encoding == 'rgba8':
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                img = raw[:, :, :3].copy()    # Drop alpha
            elif msg.encoding == 'bgra8':
                raw = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                img = raw[:, :, 2::-1].copy() # BGRA → RGB
            else:
                # Try generic 3-channel decode
                img = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, -1))
                if img.shape[2] >= 3:
                    img = img[:, :, :3].copy()
                else:
                    return

            self.latest_rgb = img
            self.latest_rgb_stamp = msg.header.stamp
        except Exception as e:
            self.get_logger().warn(f'RGB parse error: {e}')

        # Process when we have both RGB and depth
        self.frame_count += 1
        if self.frame_count % self.PROCESS_EVERY_N == 0:
            self._process_frame()

    def pc2_cb(self, msg):
        """Store the latest depth point cloud."""
        self.latest_pc2 = msg

    def cam_info_cb(self, msg):
        """Store camera intrinsics for depth→pixel projection."""
        if self.camera_info is None:
            print(f"  📐 Camera intrinsics received: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f} "
                  f"cx={msg.k[2]:.1f} cy={msg.k[5]:.1f} ({msg.width}x{msg.height})")
        self.camera_info = msg

    def nav_state_cb(self, msg):
        """Track navigator state — YOLO stays paused during exploration."""
        self.nav_state = msg.data
        if not self.active and msg.data in ('READY', 'NAVIGATING'):
            self.active = True
            print("  ▶️  Exploration complete — starting semantic pipeline!")

    def _check_active(self):
        """Return True if YOLO processing should run."""
        if self.active:
            return True
        # Standalone fallback: no navigator heard from → start after timeout
        if (self.nav_state is None
                and time.monotonic() - self._start_time > self.STANDALONE_TIMEOUT):
            self.active = True
            print("  ▶️  No navigator detected — starting semantic pipeline (standalone).")
            return True
        if not self._pause_notified:
            self._pause_notified = True
            print("  ⏸️  Semantic pipeline paused until exploration completes"
                  " (waiting for READY on /nav_state)...")
        return False

    # ── Main Processing ───────────────────────────────────────────────────

    def _process_frame(self):
        """Run YOLOv8 segmentation and fuse with depth point cloud."""
        if self.latest_rgb is None or self.latest_pc2 is None:
            return
        if not self._check_active():
            return  # Paused during exploration — keep mapping load low

        # Step 1: Run YOLOv8 segmentation on RGB
        model = get_yolo_model()
        results = model.predict(
            source=self.latest_rgb,
            conf=0.3,
            verbose=False,
            device='cuda',
            imgsz=640,
        )

        # Step 2: Build semantic color mask
        h, w = self.latest_rgb.shape[:2]
        color_mask = np.full((h, w, 3), DEFAULT_COLOR, dtype=np.uint8)

        result = results[0]
        if result.masks is not None and len(result.masks) > 0:
            masks = result.masks.data.cpu().numpy()  # (N, mask_h, mask_w)
            classes = result.boxes.cls.cpu().numpy().astype(int)

            for i, cls_id in enumerate(classes):
                if cls_id in COCO_COLORS:
                    color = COCO_COLORS[cls_id]
                else:
                    # Generate a deterministic distinct color for unmapped classes
                    np.random.seed(int(cls_id))
                    color = tuple(np.random.randint(50, 255, 3).tolist())
                    
                # Resize mask to image size
                mask = masks[i]
                if mask.shape != (h, w):
                    # Simple nearest-neighbor resize
                    import cv2
                    mask = cv2.resize(mask, (w, h), interpolation=cv2.INTER_NEAREST)
                mask_bool = mask > 0.5
                color_mask[mask_bool] = color

                # Track detected objects using YOLO's built-in names dict
                name = result.names.get(cls_id, f'class_{cls_id}')
                self.objects_detected[name] = self.objects_detected.get(name, 0) + 1

        self.label_mask = color_mask
        self.total_inferences += 1

        # Step 3: Publish annotated overlay image
        self._publish_overlay(result)

        # Step 4: Fuse labels with depth point cloud → colored XYZRGB
        self._publish_colored_pointcloud()

        # Log periodically
        if self.total_inferences % 30 == 1:
            det_str = ', '.join(f'{k}:{v}' for k, v in
                                sorted(self.objects_detected.items(),
                                       key=lambda x: -x[1])[:5])
            cam_status = 'OK' if self.camera_info else 'MISSING!'
            pc_status = 'OK' if self.latest_pc2 else 'MISSING!'
            print(f"  🧠 Inference #{self.total_inferences} — "
                  f"detections: {det_str or 'none'}")
            print(f"     cam_info={cam_status}, depth_pc={pc_status}, "
                  f"rgb={self.latest_rgb.shape if self.latest_rgb is not None else 'None'}")

    def _publish_overlay(self, result):
        """Publish annotated RGB image with segmentation overlay."""
        try:
            # Use ultralytics built-in plot
            annotated = result.plot()  # Returns BGR numpy array

            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'base_link'
            msg.height, msg.width = annotated.shape[:2]
            msg.encoding = 'bgr8'
            msg.step = msg.width * 3
            msg.data = annotated.tobytes()
            self.overlay_pub.publish(msg)
        except Exception:
            pass  # Non-critical — overlay is just for debugging

    def _publish_colored_pointcloud(self):
        """Fuse semantic colors with depth point cloud → XYZRGB PointCloud2.

        Fully vectorized with numpy — the previous per-point Python loop
        took seconds per frame on ~300k points and stalled the whole stack.
        """
        if self.label_mask is None or self.latest_pc2 is None:
            return

        pc2 = self.latest_pc2
        mask = self.label_mask

        # Parse XYZ from the depth point cloud (vectorized)
        point_step = pc2.point_step
        raw = np.frombuffer(bytes(pc2.data), dtype=np.uint8)
        n_points = len(raw) // point_step

        if n_points == 0:
            return

        # Find field offsets
        offsets = {}
        for field in pc2.fields:
            offsets[field.name] = field.offset

        x_off = offsets.get('x', 0)
        y_off = offsets.get('y', 4)
        z_off = offsets.get('z', 8)

        rows = raw[:n_points * point_step].reshape(n_points, point_step)
        x = rows[:, x_off:x_off + 4].copy().view(np.float32)[:, 0]
        y = rows[:, y_off:y_off + 4].copy().view(np.float32)[:, 0]
        z = rows[:, z_off:z_off + 4].copy().view(np.float32)[:, 0]

        # Keep finite points beyond min range
        valid = (np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
                 & (z > 0.1))
        x, y, z = x[valid], y[valid], z[valid]
        if len(x) == 0:
            return

        # Get camera intrinsics for 3D→2D projection
        if self.camera_info is not None:
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]
        else:
            # Default OakD-Lite intrinsics (approximate)
            fx, fy = 462.0, 462.0
            cx, cy = 320.0, 240.0

        h, w = mask.shape[:2]

        # Project all points onto the image plane at once
        # Camera coordinate frame: Z forward, X right, Y down
        u = (fx * x / z + cx).astype(np.int32)
        v = (fy * y / z + cy).astype(np.int32)
        in_img = (u >= 0) & (u < w) & (v >= 0) & (v < h)

        # Look up semantic colors (default for out-of-image points)
        colors = np.empty((len(x), 3), dtype=np.uint8)
        colors[:] = DEFAULT_COLOR
        colors[in_img] = mask[v[in_img], u[in_img]]

        # Height-based coloring for unclassified regions:
        # below camera → floor, above camera → ceiling, else wall
        unclassified = np.all(colors == DEFAULT_COLOR, axis=1)
        floor_m = unclassified & (y > 0.3)
        ceil_m = unclassified & (y < -0.5)
        wall_m = unclassified & ~floor_m & ~ceil_m
        colors[floor_m] = FLOOR_COLOR
        colors[ceil_m] = CEILING_COLOR
        colors[wall_m] = WALL_COLOR

        # Pack RGB into a single float32 (PCL convention)
        rgb_uint = ((colors[:, 0].astype(np.uint32) << 16)
                    | (colors[:, 1].astype(np.uint32) << 8)
                    | colors[:, 2].astype(np.uint32))
        rgb_float = rgb_uint.view(np.float32)

        # Assemble XYZRGB buffer: x(4) + y(4) + z(4) + rgb(4) = 16 bytes
        out = np.empty((len(x), 4), dtype=np.float32)
        out[:, 0] = x
        out[:, 1] = y
        out[:, 2] = z
        out[:, 3] = rgb_float
        valid_count = len(x)
        out_step = 16

        # Build PointCloud2 message
        msg = PointCloud2()
        msg.header = pc2.header  # Same frame as depth
        msg.height = 1
        msg.width = valid_count
        msg.is_dense = True
        msg.is_bigendian = False
        msg.point_step = out_step
        msg.row_step = valid_count * out_step
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = out.tobytes()
        self.colored_pc_pub.publish(msg)

        if self._first_publish:
            self._first_publish = False
            print(f"  🎨 First semantic cloud published! {valid_count} colored points, "
                  f"frame='{pc2.header.frame_id}', topic=/semantic/colored_points")
            print(f"     Camera intrinsics: {'from /camera_info' if self.camera_info else 'FALLBACK (no CameraInfo!)'}")


# ─────────────────────────────────────────────────────────────────────────────
#  MAIN
# ─────────────────────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = SemanticPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n  🛑 Semantic pipeline stopped.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
