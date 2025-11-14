import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
import time
import os
import csv
from scipy.spatial import cKDTree
from collections import deque 


def make_colored_cloud(points, color):
    """Adds RGB color to each 3D point."""
    r, g, b = color
    rgb_uint32 = (r << 16) | (g << 8) | b
    rgb_float = np.frombuffer(np.uint32(rgb_uint32).tobytes(), dtype=np.float32)[0]
    rgb_column = np.full((points.shape[0], 1), rgb_float)
    return np.hstack((points, rgb_column))


class TemporalFusionNode(Node):
    def __init__(self):
        super().__init__('temporal_fusion_node')
        self.get_logger().info('Temporal Fusion Node initialized')

        # Parameters
        self.declare_parameter('input_topic', '/mast3r/pointcloud')
        self.declare_parameter('output_topic', '/mast3r/fused_pointcloud')
        self.declare_parameter('alpha', 0.4)
        self.declare_parameter('icp_distance', 0.05)
        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('frame_id', 'map')

        # Load parameters
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.alpha = float(self.get_parameter('alpha').value)
        self.icp_distance = float(self.get_parameter('icp_distance').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.frame_id = self.get_parameter('frame_id').value

        # ROS2 I/O
        self.sub_pointcloud = self.create_subscription(
            PointCloud2, self.input_topic, self.pointcloud_callback, 10
        )
        self.pub_fused = self.create_publisher(PointCloud2, self.output_topic, 10)

        # State
        self.prev_pcd = None
        self.prev_np = None
        self.global_map = o3d.geometry.PointCloud()
        self.last_log_time = 0.0
        self.frame_count = 0
        self.convergence_errors = []

        # 5-frame sliding window buffer (multi-frame prior)
        self.window_min = 3
        self.window_max = 10
        self.window_size = 5
        self.frame_buffer = deque(maxlen=self.window_max)  

        # Folders
        os.makedirs("fused_snapshots", exist_ok=True)
        self.csv_path = os.path.join("fused_snapshots", "validation_metrics.csv")

        # Initialize CSV with headers
        with open(self.csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Frame", "Chamfer_Distance", "Overlap_Ratio", "Delta_Depth", "Convergence_Score"])

        self.get_logger().info(
            f"📡 Listening to {self.input_topic}, publishing fused cloud on {self.output_topic}"
        )

    # ---------------------------------------------
    # Utility: Chamfer distance (Registration Error)
    # ---------------------------------------------
    def compute_chamfer_distance(self, pts1, pts2):
        if len(pts1) == 0 or len(pts2) == 0:
            return None
        tree1 = cKDTree(pts1)
        tree2 = cKDTree(pts2)
        d1, _ = tree1.query(pts2)
        d2, _ = tree2.query(pts1)
        return np.mean(d1) + np.mean(d2)

    # ---------------------------------------------
    # Utility: Overlap ratio
    # ---------------------------------------------
    def compute_overlap_ratio(self, pts1, pts2, threshold=0.05):
        if len(pts1) == 0 or len(pts2) == 0:
            return 0.0
        tree = cKDTree(pts1)
        dists, _ = tree.query(pts2)
        overlap = np.sum(dists < threshold) / len(pts2)
        return overlap

    # ---------------------------------------------
    # Utility: Local confidence estimator (based on local neighbor spread)
    # ---------------------------------------------
    def compute_local_confidence(self, points, k=8, eps=1e-6):
        """
        Estimate per-point confidence using local neighbor distance std.
        Higher local spread -> lower confidence. We return normalized weights in [0,1].
        """
        if len(points) == 0:
            return np.array([], dtype=np.float32)
        tree = cKDTree(points)
        # query k+1 because the point itself is distance 0
        k_q = min(k + 1, len(points))
        dists, _ = tree.query(points, k=k_q)
        # dists shape: (N, k_q)
        # ignore the first column (self-distance zero)
        if dists.ndim == 1:
            # only one neighbor available
            stds = np.array([0.0]) + eps
        else:
            neighbor_dists = dists[:, 1:] if dists.shape[1] > 1 else dists[:, :1]
            stds = np.std(neighbor_dists, axis=1) + eps
        # invert std to get confidence; then normalize to [0.05,1.0] to avoid zero weights
        inv = 1.0 / stds
        # normalize inv to 0..1
        inv_norm = (inv - np.min(inv)) / (np.ptp(inv) + 1e-9)
        weights = 0.05 + 0.95 * inv_norm  # avoid exact zeros
        return weights.astype(np.float32)

    # ---------------------------------------------
    # Main Fusion Logic
    # ---------------------------------------------
    def pointcloud_callback(self, msg):
        """Receive MASt3R cloud, align & temporally fuse using a (now adaptive) multi-frame prior with confidence-weighted fusion and outlier rejection."""
        try:
            cloud_np = np.array(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)))

            if cloud_np.size == 0:
                self.get_logger().warn("Empty input cloud — skipping.")
                return

            if cloud_np.dtype.names is not None:
                cloud_np = np.vstack([cloud_np['x'], cloud_np['y'], cloud_np['z']]).T

            pts = np.asarray(cloud_np, dtype=np.float32)
            if pts.ndim != 2 or pts.shape[1] != 3:
                self.get_logger().warn(f"Invalid point shape: {pts.shape}")
                return

            # Convert to Open3D and downsample
            curr_pcd = o3d.geometry.PointCloud()
            curr_pcd.points = o3d.utility.Vector3dVector(pts)
            curr_pcd = curr_pcd.voxel_down_sample(self.voxel_size)
            curr_np = np.asarray(curr_pcd.points)

            # ------------------------------
            # Multi-frame prior (adaptive window) logic
            # ------------------------------
            # If no frames in buffer yet -> initialize
            if len(self.frame_buffer) == 0:
                self.frame_buffer.append(curr_pcd)
                self.prev_pcd = curr_pcd
                self.prev_np = np.asarray(curr_pcd.points)
                self.global_map += curr_pcd
                self.publish_colored_pcd(curr_pcd, self.global_map)
                self.get_logger().info_once("First frame published as fused cloud (Global map initialized)")
                return

            # Build multi-frame prior by merging frames currently in the buffer (use effective window)
            effective_frames = list(self.frame_buffer)[-self.window_size:]
            multi_prior = o3d.geometry.PointCloud()
            for pcd in effective_frames:
                multi_prior += pcd
            multi_prior = multi_prior.voxel_down_sample(self.voxel_size)
            prior_np = np.asarray(multi_prior.points)

            # If prior empty, fallback to last frame
            if prior_np.size == 0:
                multi_prior = self.frame_buffer[-1]
                prior_np = np.asarray(multi_prior.points)
                if prior_np.size == 0:
                    self.get_logger().warn("Multi-frame prior empty — skipping.")
                    return

            # Align current frame to the multi-frame prior using ICP
            reg = o3d.pipelines.registration.registration_icp(
                curr_pcd, multi_prior, self.icp_distance, np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )

            # Use ICP fitness & inlier_rmse to decide whether to fuse and to adapt window
            fitness = getattr(reg, 'fitness', 0.0)
            inlier_rmse = getattr(reg, 'inlier_rmse', np.inf)

            # If ICP fitness is too low, skip fusion for stability
            if fitness < 0.12:  # heuristic threshold; adjust as needed
                self.get_logger().warn(f"Low ICP fitness ({fitness:.3f}) — skipping fusion for this frame.")
                # still publish raw current cloud for visualization, and append to buffer for prior growth
                self.frame_buffer.append(curr_pcd)
                self.publish_colored_pcd(curr_pcd, self.global_map)
                return

            curr_pcd.transform(reg.transformation)
            curr_np = np.asarray(curr_pcd.points)

            if curr_np.size == 0 or prior_np.size == 0:
                self.get_logger().warn("One of the clouds is empty after alignment — skipping.")
                return

            # --- Adaptive temporal window update based on motion magnitude ---
            # compute translation magnitude and approx rotation angle from transformation
            trans = reg.transformation[:3, 3]
            rot = reg.transformation[:3, :3]
            trans_mag = np.linalg.norm(trans)
            # safe trace->angle computation; clamp numerical errors
            trace = np.clip(np.trace(rot), -1.0, 3.0)
            # rotation angle from trace formula: angle = arccos((trace(R)-1)/2)
            try:
                rot_angle = np.arccos(np.clip((trace - 1.0) / 2.0, -1.0, 1.0))
            except Exception:
                rot_angle = 0.0
            motion_measure = trans_mag + rot_angle  # simple scalar combining translation & rotation

            # Map motion_measure to window size (more motion -> smaller window)
            # normalized mapping heuristics: motion 0 -> max window, big motion -> min window
            # choose some scale factors that should be robust
            motion_clamped = np.clip(motion_measure, 0.0, 1.5)  # 1.5 rad or ~1.5m is considered high
            # invert and scale to window range
            new_window = int(round(self.window_min + (1.0 - (motion_clamped / 1.5)) * (self.window_max - self.window_min)))
            new_window = int(np.clip(new_window, self.window_min, self.window_max))
            # only change if different
            if new_window != self.window_size:
                self.get_logger().info(f"Adaptive window changed: {self.window_size} -> {new_window} (motion={motion_measure:.3f})")
                self.window_size = new_window
                # shrink frame_buffer if necessary (we intentionally don't grow maxlen to keep memory bounded)
                while len(self.frame_buffer) > self.window_size:
                    self.frame_buffer.popleft()

            # Nearest-neighbor mapping: for each prior point find matched current point
            tree = cKDTree(curr_np)
            dists, idx = tree.query(prior_np, k=1)
            matched_curr = curr_np[idx]

            n = min(len(prior_np), len(matched_curr))
            prior_sample = prior_np[:n]
            matched_sample = matched_curr[:n]

            # ------------------------------
            # Confidence-weighted fusion
            # ------------------------------
            # Compute per-point confidences for prior_sample and matched_sample
            prior_weights = self.compute_local_confidence(prior_sample, k=8)
            curr_weights = self.compute_local_confidence(matched_sample, k=8)

            # When sizes are tiny, guard shapes
            if prior_weights.size != n:
                prior_weights = np.ones((n,), dtype=np.float32)
            if curr_weights.size != n:
                curr_weights = np.ones((n,), dtype=np.float32)

            # Blend coefficients: keep EMA alpha but weight per-point by confidence
            alpha = self.alpha
            eps = 1e-8
            # combine weights with alpha: scale confidences with alpha fractions
            wp = prior_weights * (1.0 - alpha)
            wc = curr_weights * (alpha)
            denom = (wp + wc)[:, None] + eps
            fused_np = (wp[:, None] * prior_sample + wc[:, None] * matched_sample) / denom

            fused_pcd = o3d.geometry.PointCloud()
            fused_pcd.points = o3d.utility.Vector3dVector(fused_np)
            fused_pcd = fused_pcd.voxel_down_sample(self.voxel_size)

            # ------------------------------
            # Temporal outlier rejection (statistical filter)
            # ------------------------------
            try:
                filtered_pcd, ind = fused_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
                fused_pcd = filtered_pcd
                fused_np = np.asarray(fused_pcd.points)
            except Exception as e:
                # if filter fails for small clouds, continue with unfiltered
                self.get_logger().warn(f"Statistical outlier removal failed/ skipped: {e}")

            # Update frame buffer: add newly fused frame (sliding window of size self.window_size)
            self.frame_buffer.append(fused_pcd)
            # ensure effective buffer length doesn't exceed window_size (we manage maxlen separately)
            while len(self.frame_buffer) > self.window_size:
                self.frame_buffer.popleft()

            # Update global map
            self.global_map += fused_pcd
            self.global_map = self.global_map.voxel_down_sample(self.voxel_size)

            # For metric computations below, set prev_np to the fused prior (keeps consistency with earlier code)
            prev_np = np.asarray(fused_pcd.points)

            # ------------------------------
            # --- Validation metrics (unchanged section will use curr_np & prev_np) ---
            # ------------------------------

            chamfer_error = self.compute_chamfer_distance(curr_np, prev_np)
            # Guard against None
            if chamfer_error is None:
                chamfer_error = 0.0

            overlap_ratio = self.compute_overlap_ratio(curr_np, prev_np)
            min_len = min(len(curr_np), len(prev_np))
            if min_len == 0:
                self.get_logger().warn(" No overlapping points for delta depth computation — skipping metrics append.")
                return
            delta_depth = np.abs(curr_np[:min_len, 2] - prev_np[:min_len, 2])
            temporal_smoothness = np.mean(delta_depth)

            self.convergence_errors.append(chamfer_error)
            if len(self.convergence_errors) > 50:
                self.convergence_errors.pop(0)
            convergence_score = np.mean(self.convergence_errors)

            self.get_logger().info(
                f" Chamfer: {chamfer_error:.5f} | Overlap: {overlap_ratio:.3f} | Δdepth: {temporal_smoothness:.5f} | Conv: {convergence_score:.5f}"
            )

            # Append metrics to CSV
            with open(self.csv_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([
                    self.frame_count,
                    chamfer_error,
                    overlap_ratio,
                    temporal_smoothness,
                    convergence_score
                ])

            # Save every 10 frames for inspection
            self.frame_count += 1
            if self.frame_count % 10 == 0:
                save_path = f"fused_snapshots/fused_{self.frame_count:04d}.ply"
                o3d.io.write_point_cloud(save_path, self.global_map)
                self.get_logger().info(f"Saved fused map snapshot: {save_path}")

            # Update state & publish
            self.prev_pcd = fused_pcd
            self.prev_np = np.asarray(fused_pcd.points)
            self.publish_colored_pcd(curr_pcd, self.global_map)

            now = time.time()
            if now - self.last_log_time > 2.0:
                self.get_logger().info(
                    f"Published fused cloud with {len(fused_np)} pts | Global map size: {len(self.global_map.points)} pts"
                )
                self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f"Exception in pointcloud_callback: {e}")

    # ---------------------------------------------
    # Publish colored clouds for visualization
    # ---------------------------------------------
    def publish_colored_pcd(self, mast3r_pcd, global_map):
        mast3r_np = np.asarray(mast3r_pcd.points)
        global_np = np.asarray(global_map.points)

        if mast3r_np.size == 0 or global_np.size == 0:
            self.get_logger().warn("One or more empty clouds — skipping publish.")
            return

        mast3r_colored = make_colored_cloud(mast3r_np, (0, 255, 0))  # Green
        global_colored = make_colored_cloud(global_np, (0, 0, 255))  # Blue
        combined_points = np.vstack((mast3r_colored, global_colored))

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        try:
            cloud_msg = point_cloud2.create_cloud(header, fields, combined_points)
            self.pub_fused.publish(cloud_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to publish colored cloud: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TemporalFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Temporal Fusion Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
