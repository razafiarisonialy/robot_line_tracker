from __future__ import annotations
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from line_tracker_interfaces.msg import LineDetection

_STATE_FOLLOWING: int = 0
_STATE_SEARCHING: int = 1


class ControllerNode(Node):

    def __init__(self) -> None:
        super().__init__("controller_node")

        self.declare_parameter("kp",               0.008)
        self.declare_parameter("base_speed",       0.08)
        self.declare_parameter("max_angular",      1.0)
        self.declare_parameter("speed_reduction",  0.6)
        self.declare_parameter("min_speed",        0.03)
        self.declare_parameter("watchdog_period",  0.5)
        self.declare_parameter("watchdog_timeout", 1.0)
        self.declare_parameter("search_angular",   0.55)
        self.declare_parameter("sharp_turn_threshold", 80.0)
        self.declare_parameter("sharp_turn_boost",     2.5)

        self._kp              = self.get_parameter("kp").value
        self._base_speed      = self.get_parameter("base_speed").value
        self._max_angular     = self.get_parameter("max_angular").value
        self._speed_reduction = self.get_parameter("speed_reduction").value
        self._min_speed       = self.get_parameter("min_speed").value
        self._watchdog_timeout = self.get_parameter("watchdog_timeout").value
        self._search_angular  = self.get_parameter("search_angular").value
        watchdog_period       = self.get_parameter("watchdog_period").value
        self._sharp_turn_threshold = self.get_parameter("sharp_turn_threshold").value
        self._sharp_turn_boost     = self.get_parameter("sharp_turn_boost").value

        self._sub = self.create_subscription(
            LineDetection, "/line/detection", self._detection_callback, 10
        )
        self._pub_cmd = self.create_publisher(Twist, "/cmd_vel", 10)

        self._last_msg_time = self.get_clock().now()
        self._watchdog = self.create_timer(watchdog_period, self._watchdog_callback)

        self.get_logger().info(
            f"controller_node démarré  "
            f"[Kp={self._kp}, base={self._base_speed} m/s, "
            f"max_ω={self._max_angular} rad/s]"
        )

    def _detection_callback(self, msg: LineDetection) -> None:
        self._last_msg_time = self.get_clock().now()

        if msg.state == _STATE_FOLLOWING and msg.line_detected:
            self._handle_following(msg.error)

        elif msg.state == _STATE_SEARCHING:
            self._handle_searching(msg.last_direction)

        else:
            self._handle_stop()
            
    def _handle_following(self, error: float) -> None:
        abs_error = abs(error)

        if abs_error > self._sharp_turn_threshold:
            effective_kp = self._kp * self._sharp_turn_boost
        else:
            effective_kp = self._kp

        angular_z = float(
            np.clip(effective_kp * error, -self._max_angular, self._max_angular)
        )
        
        error_norm = abs(angular_z) / self._max_angular

        if abs_error > self._sharp_turn_threshold:
            linear_x = self._min_speed
        else:
            linear_x = float(
                np.clip(
                    self._base_speed * (1.0 - self._speed_reduction * error_norm),
                    self._min_speed,
                    self._base_speed,
                )
            )

        self._publish_cmd_vel(linear_x, angular_z)
    
    def _handle_searching(self, last_direction: float) -> None:
        sign = math.copysign(1.0, last_direction) if last_direction != 0.0 else 1.0
        angular_z = float(sign * self._search_angular)
        self._publish_cmd_vel(self._min_speed * 0.5, angular_z)
    
    def _handle_stop(self) -> None:
        self._publish_cmd_vel(0.0, 0.0)

    def _publish_cmd_vel(self, linear_x: float, angular_z: float) -> None:
        cmd           = Twist()
        cmd.linear.x  = linear_x
        cmd.angular.z = angular_z
        self._pub_cmd.publish(cmd)

    def _watchdog_callback(self) -> None:
        dt = (self.get_clock().now() - self._last_msg_time).nanoseconds / 1e9
        if dt > self._watchdog_timeout:
            self._handle_stop()
            self.get_logger().warn(
                f"[WATCHDOG]  aucun message depuis {dt:.1f}s — arrêt",
                throttle_duration_sec=2.0,
            )


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(ControllerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
