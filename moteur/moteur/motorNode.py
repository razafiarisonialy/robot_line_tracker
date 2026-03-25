#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# Importer ta classe personnalisée depuis le fichier local
from .GoPiGo3Driver import GoPiGo3Driver


GOPIGO_SPEED = 200
GOPIGO_MAX   = 400
STEER_GAIN   = 50.0


class MotorNode(Node):

    def __init__(self):
        super().__init__('motor_node')

        self.driver = GoPiGo3Driver(
            base_speed = GOPIGO_SPEED,
            max_speed  = GOPIGO_MAX,
            steer_gain = STEER_GAIN
        )

        self.sub = self.create_subscription(
            Twist, 
            '/cmd_vel', 
            self.cmd_vel_callback, 
            10
        )

        self._last_msg_time = self.get_clock().now()
        self._watchdog = self.create_timer(0.5, self._watchLog_callback)

        self.get_logger().info('motor_node démarré — en écoute sur /cmd_vel')

    def cmd_vel_callback(self, msg: Twist):
        self._last_msg_time = self.get_clock().now()

        linear_x  = msg.linear.x
        angular_z = msg.angular.z

        self.driver.apply_twist(linear_x, angular_z)

        direction = "← G" if angular_z > 0 else "→ D" if angular_z < 0 else "■"
        self.get_logger().info(
            f'v={linear_x:.3f} m/s  ω={angular_z:+.3f} rad/s  {direction}',
            throttle_duration_sec=0.1)

    def _watchLog_callback(self):
        dt = (self.get_clock().now() - self._last_msg_time).nanoseconds / 1e9
        if dt > 1.0:
            self.driver.stop()
            self.get_logger().warn(
                f'Watchdog : arrêt ({dt:.1f}s sans /cmd_vel)',
                throttle_duration_sec=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = MotorNode()
    rclpy.spin(node)
    node.driver.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()