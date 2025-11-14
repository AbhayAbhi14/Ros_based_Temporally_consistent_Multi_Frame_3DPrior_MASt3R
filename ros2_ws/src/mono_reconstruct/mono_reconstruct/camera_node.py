import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import os

class CameraNode(Node):
    def __init__(self, camera_device="/dev/video0"):
        super().__init__('camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # Log which camera we're trying to open
        self.get_logger().info(f"🎥 Trying to open camera: {camera_device}")

        # Try to open the given camera
        self.cap = cv2.VideoCapture(camera_device)

        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open camera at {camera_device}")
        else:
            self.get_logger().info(f"Camera {camera_device} connected successfully")

        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 FPS

    def timer_callback(self):
        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher_.publish(msg)
            else:
                self.get_logger().warn("Failed to read frame from camera")
        else:
            self.get_logger().error("Camera not opened")

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    camera_device = sys.argv[1] if len(sys.argv) > 1 else "/dev/video0"

    if not os.path.exists(camera_device):
        print(f"Device {camera_device} does not exist.")
        return

    node = CameraNode(camera_device)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

