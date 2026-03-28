import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller_node')

        self.declare_parameter('kp',               0.003)
        self.declare_parameter('base_speed',       0.15)
        self.declare_parameter('max_angular',      0.8)
        self.declare_parameter('speed_reduction',  0.3)
        self.declare_parameter('min_speed',        0.05)
        self.declare_parameter('watchdog_period',  0.5)
        self.declare_parameter('watchdog_timeout', 1.0)

        self._kp               = self.get_parameter('kp').value
        self._base_speed       = self.get_parameter('base_speed').value
        self._max_angular      = self.get_parameter('max_angular').value
        self._speed_reduction  = self.get_parameter('speed_reduction').value
        self._min_speed        = self.get_parameter('min_speed').value
        watchdog_period        = self.get_parameter('watchdog_period').value
        self._watchdog_timeout = self.get_parameter('watchdog_timeout').value

        self._sub_error = self.create_subscription(
            Float32, '/line/error', self._error_callback, 10
        )
        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)

        self._last_error_time = self.get_clock().now()
        self._watchdog = self.create_timer(watchdog_period, self._watchdog_callback)

        self.get_logger().info(
            f'controller_node démarré  '
            f'[Kp={self._kp}, base_speed={self._base_speed} m/s, '
            f'max_angular={self._max_angular} rad/s]'
        )


    def _error_callback(self, msg: Float32) -> None:
        self._last_error_time = self.get_clock().now()

        error = msg.data

        angular_z = float(np.clip(self._kp * error, -self._max_angular, self._max_angular))

        error_norm = abs(angular_z) / self._max_angular
        linear_x   = float(np.clip(
            self._base_speed * (1.0 - self._speed_reduction * error_norm),
            self._min_speed,
            self._base_speed,
        ))

        self._publish_cmd_vel(linear_x, angular_z)

        direction = "← Gauche" if error > 0 else "→ Droite" if error < 0 else "■ centre"
        self.get_logger().info(
            f'error={error:+.1f}px {direction}  '
            f'ω={angular_z:+.3f} rad/s  v={linear_x:.3f} m/s',
            throttle_duration_sec=0.1,
        )
    
    def _publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self._pub_cmd.publish(cmd)


    def _watchdog_callback(self) -> None:
        dt = (self.get_clock().now() - self._last_error_time).nanoseconds / 1e9
        if dt > self._watchdog_timeout:
            self.get_logger().warn(
                f'Watchdog : aucune erreur reçue depuis {dt:.1f}s',
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()