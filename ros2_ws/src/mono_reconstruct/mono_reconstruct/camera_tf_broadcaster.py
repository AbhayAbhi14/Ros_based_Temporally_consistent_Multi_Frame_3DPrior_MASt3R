import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
import time


class CameraTFBroadcaster(Node):
    def __init__(self):
        super().__init__('camera_tf_broadcaster')

        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.broadcast_tf) 
        self.start_time = time.time()

        self.get_logger().info("Camera TF Broadcaster started (map → camera_link)")

    def broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'camera_link'

       
        elapsed = time.time() - self.start_time
        radius = 0.2
        t.transform.translation.x = radius * math.sin(elapsed / 3.0)
        t.transform.translation.y = radius * math.cos(elapsed / 3.0)
        t.transform.translation.z = 0.0

        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = math.sin(elapsed / 10.0) * 0.1
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = CameraTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
