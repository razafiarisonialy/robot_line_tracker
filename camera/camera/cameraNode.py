import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
from .ImageProcessing import ImageProcessing

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.declare_parameter('roi_ratio',   0.5)
        self.declare_parameter('use_otsu',    True)
        self.declare_parameter('threshold',   80)
        self.declare_parameter('min_area',    300)
        self.declare_parameter('blur_size',   7)
        self.declare_parameter('morph_size',  5)
        self.declare_parameter('debug',       True)

        roi_ratio  = self.get_parameter('roi_ratio').value
        use_otsu   = self.get_parameter('use_otsu').value
        threshold  = self.get_parameter('threshold').value
        min_area   = self.get_parameter('min_area').value
        blur_size  = self.get_parameter('blur_size').value
        morph_size = self.get_parameter('morph_size').value
        self._debug = self.get_parameter('debug').value

        self._bridge    = CvBridge()
        self._processor = ImageProcessing(
            threshold  = threshold,
            roi_ratio  = roi_ratio,
            use_otsu   = use_otsu,
            min_area   = min_area,
            blur_size  = blur_size,
            morph_size = morph_size,
        )

        self._sub = self.create_subscription(
            Image, '/image_raw', self._image_callback, 10
        )
        self._pub_error  = self.create_publisher(Float32, '/line/error',  10)
        self._pub_debug  = self.create_publisher(Image,   '/line/debug',  10)
        self._pub_binary = self.create_publisher(Image,   '/line/binary', 10)

        self.get_logger().info(
            f'camera_node démarré  '
            f'[roi={roi_ratio}, otsu={use_otsu}, min_area={min_area}, debug={self._debug}]'
        )

    def _image_callback(self, msg: Image) -> None:
        self.get_logger().info(
            f"Image reçue: {msg.width}x{msg.height}",
            throttle_duration_sec=2.0,
        )

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        res   = self._processor.process_frame(frame)

        error_msg      = Float32()
        error_msg.data = res['error'] if res['error'] is not None else 0.0

        if res['error'] is not None:
            self.get_logger().info(
                f"Erreur ligne: {res['error']:+.2f} px",
                throttle_duration_sec=1.0,
            )
        else:
            self.get_logger().warn(
                'Ligne non détectée — erreur forcée à 0.0',
                throttle_duration_sec=1.0,
            )

        self._pub_error.publish(error_msg)

        if self._debug:
            if res['debug'] is not None:
                self._pub_debug.publish(
                    self._bridge.cv2_to_imgmsg(res['debug'], encoding='bgr8')
                )
            self._pub_binary.publish(
                self._bridge.cv2_to_imgmsg(res['binary'], encoding='mono8')
            )


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()