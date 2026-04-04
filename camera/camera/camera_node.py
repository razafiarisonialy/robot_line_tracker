from __future__ import annotations
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .ImageProcessing import (
    ImageProcessing,
    STATE_FOLLOWING,
    STATE_SEARCHING,
    STATE_STOP_LOST,
)
from line_tracker_interfaces.msg import LineDetection

class CameraNode(Node):

    def __init__(self) -> None:
        super().__init__("camera_node")

        self.declare_parameter("roi_ratio",        0.30)
        self.declare_parameter("use_otsu",         True)
        self.declare_parameter("threshold",        80)
        self.declare_parameter("min_area",         300)
        self.declare_parameter("blur_size",        7)
        self.declare_parameter("morph_size",       5)
        self.declare_parameter("debug",            True)
        self.declare_parameter("search_timeout",   1.5)
        self.declare_parameter("stop_timeout",     4.0)

        roi_ratio   = self.get_parameter("roi_ratio").value
        use_otsu    = self.get_parameter("use_otsu").value
        threshold   = self.get_parameter("threshold").value
        min_area    = self.get_parameter("min_area").value
        blur_size   = self.get_parameter("blur_size").value
        morph_size  = self.get_parameter("morph_size").value
        self._debug = self.get_parameter("debug").value
        self._search_timeout = self.get_parameter("search_timeout").value
        self._stop_timeout   = self.get_parameter("stop_timeout").value

        self._bridge    = CvBridge()
        self._processor = ImageProcessing(
            threshold  = threshold,
            roi_ratio  = roi_ratio,
            use_otsu   = use_otsu,
            min_area   = min_area,
            blur_size  = blur_size,
            morph_size = morph_size,
        )

        self._state:          int   = STATE_FOLLOWING
        self._last_seen_time: float = self.get_clock().now().nanoseconds / 1e9
        self._last_direction: float = 0.0

        self._sub = self.create_subscription(
            Image, "/image_raw", self._image_callback, 10
        )
        self._pub_detection = self.create_publisher(
            LineDetection, "/line/detection", 10
        )
        self._pub_debug  = self.create_publisher(Image, "/line/debug",  10)
        self._pub_binary = self.create_publisher(Image, "/line/binary", 10)

        self.get_logger().info(
            f"camera_node démarré  "
            f"[roi={roi_ratio}, otsu={use_otsu}, min_area={min_area}, "
            f"search_timeout={self._search_timeout}s, stop_timeout={self._stop_timeout}s]"
        )


    def _image_callback(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        res = self._processor.process_frame(frame)
        self._update_fsm(
            line_detected = res["line_detected"],
            error         = res["error"],
        )

        det                = LineDetection()
        det.line_detected  = res["line_detected"]
        det.state          = self._state
        det.last_direction = self._last_direction

        if res["line_detected"]:
            det.error = res["error"]
        else:
            det.error = float("nan")

        self._pub_detection.publish(det)

        if self._debug:
            if res["debug"] is not None:
                self._pub_debug.publish(
                    self._bridge.cv2_to_imgmsg(res["debug"], encoding="bgr8")
                )
            self._pub_binary.publish(
                self._bridge.cv2_to_imgmsg(res["binary"], encoding="mono8")
            )


    def _update_fsm(self, *, line_detected: bool, error: float | None) -> None:
        now = self.get_clock().now().nanoseconds / 1e9

        if line_detected:
            self._state          = STATE_FOLLOWING
            self._last_seen_time = now
            if error is not None and abs(error) > 5.0:
                self._last_direction = float(error)
        else:
            elapsed = now - self._last_seen_time
            if elapsed < self._search_timeout:
                self._state = STATE_SEARCHING
            elif elapsed < self._stop_timeout:
                self._state = STATE_SEARCHING
            else:
                self._state = STATE_STOP_LOST



def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
