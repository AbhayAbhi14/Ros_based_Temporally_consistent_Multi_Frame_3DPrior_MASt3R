import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DepthCompare(Node):
    def __init__(self):
        super().__init__("depth_compare")

        self.bridge = CvBridge()
        self.mast3r_depth = None
        self.realsense_depth = None

        # Params
        self.declare_parameter("mast3r_topic", "/mast3r/depth_image")
        self.declare_parameter("realsense_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("output_topic", "/depth_comparison")

        # Topics
        mast3r_topic = self.get_parameter("mast3r_topic").value
        realsense_topic = self.get_parameter("realsense_topic").value
        output_topic = self.get_parameter("output_topic").value

        # Subscribers
        self.create_subscription(Image, mast3r_topic, self.mast3r_cb, 10)
        self.create_subscription(Image, realsense_topic, self.rs_cb, 10)

        # Publisher
        self.pub = self.create_publisher(Image, output_topic, 10)

    def mast3r_cb(self, msg):
        self.mast3r_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        self.try_compare()

    def rs_cb(self, msg):
        self.realsense_depth = self.bridge.imgmsg_to_cv2(msg, "16UC1") / 1000.0  
        # Convert mm → meters
        self.try_compare()

    def try_compare(self):
        if self.mast3r_depth is None or self.realsense_depth is None:
            return

        # Resize RealSense depth to MASt3R’s resolution
        rs = cv2.resize(self.realsense_depth, (self.mast3r_depth.shape[1], self.mast3r_depth.shape[0]))

        m = self.mast3r_depth
        r = rs

        # Mask valid depth values
        mask = (r > 0) & (m > 0)

        if np.sum(mask) < 1000:
            return

        # Compute metrics
        diff = m[mask] - r[mask]
        rmse = np.sqrt(np.mean(diff**2))
        mae = np.mean(np.abs(diff))
        scale_error = np.mean(m[mask] / r[mask])

        print(f"\n----- DEPTH COMPARISON -----")
        print(f"RMSE: {rmse:.3f} m")
        print(f"MAE:  {mae:.3f} m")
        print(f"Scale error: {scale_error:.3f}")
        print(f"Valid pixels compared: {np.sum(mask)}")

        # Create comparison visualization
        vis = np.hstack([
            cv2.applyColorMap(np.uint8(m / np.max(m) * 255), cv2.COLORMAP_JET),
            cv2.applyColorMap(np.uint8(r / np.max(r) * 255), cv2.COLORMAP_JET)
        ])

        self.pub.publish(self.bridge.cv2_to_imgmsg(vis, "bgr8"))
