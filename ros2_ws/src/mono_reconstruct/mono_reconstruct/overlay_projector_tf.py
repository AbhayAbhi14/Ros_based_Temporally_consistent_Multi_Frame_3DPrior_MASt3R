# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image, PointCloud2
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# import tf2_ros
# import tf2_py as tf2
# import sensor_msgs_py.point_cloud2 as pc2

# class ImageCloudOverlay(Node):
#     def __init__(self):
#         super().__init__("image_cloud_overlay")

#         self.bridge = CvBridge()
#         self.image = None

#         self.image_topic = self.declare_parameter("image_topic", "/camera/image_raw").value
#         self.raw_cloud_topic = self.declare_parameter("raw_cloud_topic", "/mast3r/pointcloud").value
#         self.adaptive_cloud_topic = self.declare_parameter("adaptive_cloud_topic", "/mast3r/fused_pointcloud_adaptive").value
#         self.fixed_cloud_topic = self.declare_parameter("fixed_cloud_topic", "/mast3r/fused_pointcloud_fixed").value
#         self.output_topic = self.declare_parameter("output_image_topic", "/overlay/image").value
#         self.camera_frame = self.declare_parameter("camera_frame", "camera_link").value

#         # TF Buffer
#         self.tf_buffer = tf2_ros.Buffer()
#         self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

#         # Subscribers
#         self.create_subscription(Image, self.image_topic, self.image_callback, 10)
#         self.create_subscription(PointCloud2, self.raw_cloud_topic, self.cloud_callback, 10)

#         # Publisher
#         self.pub = self.create_publisher(Image, self.output_topic, 10)

#     def image_callback(self, msg):
#         self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

#     def cloud_callback(self, cloud_msg):
#         if self.image is None:
#             return

#         try:
#             transform = self.tf_buffer.lookup_transform(
#                 self.camera_frame,
#                 cloud_msg.header.frame_id,
#                 rclpy.time.Time()
#             )
#         except:
#             self.get_logger().warn("No TF available")
#             return

#         img = self.image.copy()
#         points = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))

#         for pt in points[::50]:  # THIN for speed
#             x, y, z = pt
#             if z <= 0:
#                 continue

#             # project to image plane (simple pinhole)
#             u = int(420 * x / z + img.shape[1] / 2)
#             v = int(420 * y / z + img.shape[0] / 2)

#             if 0 <= u < img.shape[1] and 0 <= v < img.shape[0]:
#                 cv2.circle(img, (u, v), 2, (0, 255, 0), -1)

#         self.pub.publish(self.bridge.cv2_to_imgmsg(img, encoding="bgr8"))

# def main():
#     rclpy.init()
#     node = ImageCloudOverlay()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
import tf2_ros
import sensor_msgs_py.point_cloud2 as pc2


class ImageCloudOverlay(Node):
    def __init__(self):
        super().__init__("image_cloud_overlay")

        self.bridge = CvBridge()
        self.image = None

        # ---------------------------
        # PARAMETERS
        # ---------------------------
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/image_raw"
        ).value

        self.selected_cloud = self.declare_parameter(
            "selected_cloud", "raw"  # raw | adaptive | fixed
        ).value

        # three cloud topics
        self.raw_cloud_topic = "/mast3r/pointcloud"
        self.adaptive_cloud_topic = "/mast3r/fused_pointcloud_adaptive"
        self.fixed_cloud_topic = "/mast3r/fused_pointcloud_fixed"

        self.output_topic = self.declare_parameter(
            "output_image_topic", "/overlay/image"
        ).value

        self.camera_frame = self.declare_parameter(
            "camera_frame", "camera_link"
        ).value

        # Camera intrinsics (default values, can override)
        self.fx = self.declare_parameter("fx", 420.0).value
        self.fy = self.declare_parameter("fy", 420.0).value
        self.cx = self.declare_parameter("cx", 640.0).value
        self.cy = self.declare_parameter("cy", 360.0).value

        # ---------------------------
        # SELECT the actual cloud topic
        # ---------------------------
        if self.selected_cloud == "raw":
            cloud_topic = self.raw_cloud_topic
        elif self.selected_cloud == "adaptive":
            cloud_topic = self.adaptive_cloud_topic
        elif self.selected_cloud == "fixed":
            cloud_topic = self.fixed_cloud_topic
        else:
            self.get_logger().error("❌ Invalid selected_cloud parameter! Use raw/adaptive/fixed.")
            raise SystemExit()

        self.get_logger().info(f"📌 Using cloud topic: {cloud_topic}")

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(Image, self.image_topic, self.image_callback, 10)
        self.create_subscription(PointCloud2, cloud_topic, self.cloud_callback, 10)

        # Publisher
        self.pub = self.create_publisher(Image, self.output_topic, 10)

    # ---------------------------------------------------------
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    # ---------------------------------------------------------
    def cloud_callback(self, cloud_msg):
        if self.image is None:
            return

        # Lookup TF transform
        try:
            transform = self.tf_buffer.lookup_transform(
                self.camera_frame,
                cloud_msg.header.frame_id,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"No TF available: {e}")
            return

        # Image to draw on
        img = self.image.copy()

        # Convert point cloud to numpy
        points = pc2.read_points(
            cloud_msg,
            field_names=("x", "y", "z"),
            skip_nans=True
        )

        # Iterate
        for (x, y, z) in points:

            # Ignore points behind camera
            if z <= 0.0:
                continue

            # Apply pinhole projection
            u = int((self.fx * x) / z + self.cx)
            v = int((self.fy * y) / z + self.cy)

            if 0 <= u < img.shape[1] and 0 <= v < img.shape[0]:
                cv2.circle(img, (u, v), 2, (0, 255, 0), -1)

        # Publish result
        out_msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.pub.publish(out_msg)


def main():
    rclpy.init()
    node = ImageCloudOverlay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
