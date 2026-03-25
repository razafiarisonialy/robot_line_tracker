import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from .ImageProcessing import ImageProcessing


KP           = 0.005
MAX_ANGULAR  = 0.8
BASE_SPEED   = 0.15
DEBUG        = True


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.bridge    = CvBridge()
        self.processor = ImageProcessing(
            roi_ratio  = 0.5,
            use_otsu   = True,
            min_area   = 300,
            blur_size  = 7,
            morph_size = 5,
        )

        self.sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )

        self.pub_error  = self.create_publisher(Float32, '/line/error',  10)
        self.pub_debug  = self.create_publisher(Image,   '/line/debug',  10)
        self.pub_binary = self.create_publisher(Image,   '/line/binary', 10)

        self.get_logger().info('camera_node démarré')

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        res   = self.processor.process_frame(frame)

        error_msg = Float32()

        if res['error'] is not None:
            error_msg.data = res['error']
        else:
            error_msg.data = 0.0
            self.get_logger().warn('Ligne non détectée',
                                   throttle_duration_sec=1.0)

        self.pub_error.publish(error_msg)

        if DEBUG:
            if res['debug'] is not None:
                self.pub_debug.publish(
                    self.bridge.cv2_to_imgmsg(res['debug'], encoding='bgr8'))
            self.pub_binary.publish(
                self.bridge.cv2_to_imgmsg(res['binary'], encoding='mono8'))


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
