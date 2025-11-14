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


class FixedWindowFusionNode(Node):
    def __init__(self):
        super().__init__('fixed_window_fusion_node')
        self.get_logger().info(' Fixed Window Fusion Node initialized')

        # --- Parameters ---
        self.declare_parameter('input_topic', '/mast3r/pointcloud')
        self.declare_parameter('output_topic', '/mast3r/fused_pointcloud')
        self.declare_parameter('alpha', 0.4)
        self.declare_parameter('icp_distance', 0.05)
        self.declare_parameter('voxel_size', 0.02)
        self.declare_parameter('frame_id', 'map')

        # --- Load parameters ---
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.alpha = float(self.get_parameter('alpha').value)
        self.icp_distance = float(self.get_parameter('icp_distance').value)
        self.voxel_size = float(self.get_parameter('voxel_size').value)
        self.frame_id = self.get_parameter('frame_id').value

        # --- ROS2 I/O ---
        self.sub_pointcloud = self.create_subscription(
            PointCloud2, self.input_topic, self.pointcloud_callback, 10
        )
        self.pub_fused = self.create_publisher(PointCloud2, self.output_topic, 10)

        # --- State ---
        self.global_map = o3d.geometry.PointCloud()
        self.frame_buffer = deque(maxlen=5)  # Fixed window = 5 frames
        self.frame_count = 0
        self.convergence_errors = []

        # --- Folders and CSV ---
        os.makedirs("fused_snapshots", exist_ok=True)
        self.csv_path = os.path.join("fused_snapshots", "fixed_metrics.csv")

        # Initialize CSV file with headers
        with open(self.csv_path, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Frame", "Chamfer_Distance", "Overlap_Ratio", "Delta_Depth", "Convergence_Score"])

        self.get_logger().info(
            f"Listening to {self.input_topic}, publishing fused cloud on {self.output_topic}"
        )

    # ---------- Utility Functions ----------
    def compute_chamfer_distance(self, pts1, pts2):
        if len(pts1) == 0 or len(pts2) == 0:
            return None
        tree1, tree2 = cKDTree(pts1), cKDTree(pts2)
        d1, _ = tree1.query(pts2)
        d2, _ = tree2.query(pts1)
        return np.mean(d1) + np.mean(d2)

    def compute_overlap_ratio(self, pts1, pts2, threshold=0.05):
        if len(pts1) == 0 or len(pts2) == 0:
            return 0.0
        tree = cKDTree(pts1)
        dists, _ = tree.query(pts2)
        return np.sum(dists < threshold) / len(pts2)

    def compute_local_confidence(self, points, k=8, eps=1e-6):
        if len(points) == 0:
            return np.array([], dtype=np.float32)
        tree = cKDTree(points)
        k_q = min(k + 1, len(points))
        dists, _ = tree.query(points, k=k_q)
        neighbor_dists = dists[:, 1:] if dists.shape[1] > 1 else dists[:, :1]
        stds = np.std(neighbor_dists, axis=1) + eps
        inv = 1.0 / stds
        inv_norm = (inv - np.min(inv)) / (np.ptp(inv) + 1e-9)
        return (0.05 + 0.95 * inv_norm).astype(np.float32)

    # ---------- Main Fusion ----------
    def pointcloud_callback(self, msg):
        try:
            # Convert ROS PointCloud2 → NumPy
            cloud_np = np.array(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)))
            if cloud_np.size == 0:
                return
            if cloud_np.dtype.names is not None:
                cloud_np = np.vstack([cloud_np['x'], cloud_np['y'], cloud_np['z']]).T

            curr_pcd = o3d.geometry.PointCloud()
            curr_pcd.points = o3d.utility.Vector3dVector(cloud_np)
            curr_pcd = curr_pcd.voxel_down_sample(self.voxel_size)
            curr_np = np.asarray(curr_pcd.points)

            # First frame initialization
            if len(self.frame_buffer) == 0:
                self.frame_buffer.append(curr_pcd)
                self.global_map += curr_pcd
                self.publish_colored_pcd(curr_pcd, self.global_map)
                return

            # Build multi-frame prior (fixed window)
            multi_prior = o3d.geometry.PointCloud()
            for pcd in self.frame_buffer:
                multi_prior += pcd
            multi_prior = multi_prior.voxel_down_sample(self.voxel_size)
            prior_np = np.asarray(multi_prior.points)
            if prior_np.size == 0:
                return

            # ICP alignment
            reg = o3d.pipelines.registration.registration_icp(
                curr_pcd, multi_prior, self.icp_distance, np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
            curr_pcd.transform(reg.transformation)
            curr_np = np.asarray(curr_pcd.points)

            # Confidence-weighted fusion
            tree = cKDTree(curr_np)
            dists, idx = tree.query(prior_np, k=1)
            matched_curr = curr_np[idx]
            n = min(len(prior_np), len(matched_curr))
            prior_sample, matched_sample = prior_np[:n], matched_curr[:n]
            prior_weights = self.compute_local_confidence(prior_sample)
            curr_weights = self.compute_local_confidence(matched_sample)
            wp, wc = prior_weights * (1 - self.alpha), curr_weights * self.alpha
            denom = (wp + wc)[:, None] + 1e-8
            fused_np = (wp[:, None] * prior_sample + wc[:, None] * matched_sample) / denom

            fused_pcd = o3d.geometry.PointCloud()
            fused_pcd.points = o3d.utility.Vector3dVector(fused_np)
            fused_pcd = fused_pcd.voxel_down_sample(self.voxel_size)

            # Outlier removal
            try:
                fused_pcd, _ = fused_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=1.0)
            except Exception:
                pass

            # Update global state
            self.frame_buffer.append(fused_pcd)
            self.global_map += fused_pcd
            self.global_map = self.global_map.voxel_down_sample(self.voxel_size)

            # ---------- Metrics ----------
            chamfer = self.compute_chamfer_distance(curr_np, prior_np) or 0.0
            overlap = self.compute_overlap_ratio(curr_np, prior_np)
            min_len = min(len(curr_np), len(prior_np))
            delta_depth = np.mean(np.abs(curr_np[:min_len, 2] - prior_np[:min_len, 2]))
            self.convergence_errors.append(chamfer)
            if len(self.convergence_errors) > 50:
                self.convergence_errors.pop(0)
            conv = np.mean(self.convergence_errors)

            # Save metrics
            with open(self.csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([self.frame_count, chamfer, overlap, delta_depth, conv])

            # Save PLY snapshots every 10 frames
            self.frame_count += 1
            if self.frame_count % 10 == 0:
                save_path = f"fused_snapshots/fixed_{self.frame_count:04d}.ply"
                o3d.io.write_point_cloud(save_path, self.global_map)
                self.get_logger().info(f"Saved snapshot: {save_path}")

            # Publish fused pointcloud
            self.publish_colored_pcd(curr_pcd, self.global_map)

        except Exception as e:
            self.get_logger().error(f"Error in fixed window callback: {e}")

    def publish_colored_pcd(self, mast3r_pcd, global_map):
        mast3r_np = np.asarray(mast3r_pcd.points)
        global_np = np.asarray(global_map.points)
        if mast3r_np.size == 0 or global_np.size == 0:
            return
        mast3r_colored = make_colored_cloud(mast3r_np, (0, 255, 0))  # Green cloud points
        global_colored = make_colored_cloud(global_np, (0, 0, 255))  # Blue cloud points
        combined = np.vstack((mast3r_colored, global_colored))
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg = point_cloud2.create_cloud(header, fields, combined)
        self.pub_fused.publish(cloud_msg)


def main(args=None):
    rclpy.init(args=args)
    node = FixedWindowFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Fixed Window Fusion Node")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
