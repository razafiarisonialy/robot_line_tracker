from __future__ import annotations
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from .GoPiGo3Driver import GoPiGo3Driver

class MotorNode(Node):
    def __init__(self) -> None:
        super().__init__("motor_node")

        self.declare_parameter("max_speed",        400)
        self.declare_parameter("steer_gain",       50.0)
        self.declare_parameter("watchdog_period",  0.5)
        self.declare_parameter("watchdog_timeout", 1.0)

        max_speed              = self.get_parameter("max_speed").value
        steer_gain_raw         = self.get_parameter("steer_gain").value
        watchdog_period        = self.get_parameter("watchdog_period").value
        self._watchdog_timeout = self.get_parameter("watchdog_timeout").value

        steer_gain = steer_gain_raw if steer_gain_raw != 0.0 else None

        self._driver = GoPiGo3Driver(
            max_speed  = max_speed,
            steer_gain = steer_gain,
        )

        self._sub = self.create_subscription(
            Twist, "/cmd_vel", self._cmd_vel_callback, 10
        )

        self._last_msg_time = self.get_clock().now()
        self._watchdog      = self.create_timer(watchdog_period, self._watchdog_callback)

        self.get_logger().info(
            f"motor_node démarré  "
            f"[max={max_speed} DPS, steer_gain={steer_gain}]"
        )


    def _cmd_vel_callback(self, msg: Twist) -> None:
        self._last_msg_time = self.get_clock().now()

        linear_x  = msg.linear.x
        angular_z = msg.angular.z

        if math.isnan(linear_x) or math.isnan(angular_z):
            self.get_logger().error(
                "cmd_vel contient NaN — arrêt de sécurité",
                throttle_duration_sec=1.0,
            )
            self._driver.stop()
            return

        self._driver.apply_twist(linear_x, angular_z)

    def _watchdog_callback(self) -> None:
        dt = (self.get_clock().now() - self._last_msg_time).nanoseconds / 1e9
        if dt > self._watchdog_timeout:
            self._driver.stop()
            self.get_logger().warn(
                f"Watchdog : arrêt d'urgence ({dt:.1f}s sans /cmd_vel)",
                throttle_duration_sec=2.0,
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = MotorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._driver.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
