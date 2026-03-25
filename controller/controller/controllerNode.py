import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


KP              = 0.005
BASE_SPEED      = 0.15
MAX_ANGULAR     = 0.8
SPEED_REDUCTION = 0.5


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.sub_error = self.create_subscription(
            Float32, 
            '/line/error', 
            self.error_callback, 
            10
        )

        self.pub_cmd = self.create_publisher(
            Twist, 
            '/cmd_vel', 
            10
        )

        self._last_error_time = self.get_clock().now()
        self._watchdog = self.create_timer(0.5, self._watchLog_callback)

        self.get_logger().info('controller_node démarré')

    def error_callback(self, msg: Float32):
        self._last_error_time = self.get_clock().now()

        error     = msg.data
        angular_z = float(np.clip(KP * error, -MAX_ANGULAR, MAX_ANGULAR))

        error_norm = abs(angular_z) / MAX_ANGULAR
        linear_x   = float(np.clip(
            BASE_SPEED * (1.0 - SPEED_REDUCTION * error_norm),
            0.05, BASE_SPEED))

        cmd           = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z
        self.pub_cmd.publish(cmd)

        direction = "← G" if error > 0 else "→ D" if error < 0 else "■"
        self.get_logger().info(
            f'error={error:+.1f}px {direction}  '
            f'ω={angular_z:+.3f}  v={linear_x:.3f}',
            throttle_duration_sec=0.1)

    def _watchLog_callback(self):
        dt = (self.get_clock().now() - self._last_error_time).nanoseconds / 1e9
        if dt > 1.0:
            self.get_logger().warn(
                f'Watchdog : plus d\'erreur reçue ({dt:.1f}s)',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()