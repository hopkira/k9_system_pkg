#!/usr/bin/env python3

import cv2
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class EyeCameraNode(Node):
    """
    Capture frames from K9's USB eye camera and publish them as ROS 2 images.

    The camera is expected to be exposed as a standard V4L2/UVC device,
    normally via the persistent udev symlink:

        /dev/k9_eye_camera

    This node deliberately performs no face detection or other vision
    processing. Those workloads belong on the Jetson.
    """

    def __init__(self):
        super().__init__("eye_camera")

        # ------------------------------------------------------------------
        # Parameters
        # ------------------------------------------------------------------

        self.declare_parameter("device", "/dev/k9_eye_camera")
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)
        self.declare_parameter("frame_rate", 30.0)
        self.declare_parameter("frame_id", "k9_eye_camera")
        self.declare_parameter("image_topic", "/k9/camera/eye/image_raw")

        self.device = (
            self.get_parameter("device")
            .get_parameter_value()
            .string_value
        )

        self.width = (
            self.get_parameter("camera_width")
            .get_parameter_value()
            .integer_value
        )

        self.height = (
            self.get_parameter("camera_height")
            .get_parameter_value()
            .integer_value
        )

        self.frame_rate = (
            self.get_parameter("frame_rate")
            .get_parameter_value()
            .double_value
        )

        self.frame_id = (
            self.get_parameter("frame_id")
            .get_parameter_value()
            .string_value
        )

        self.image_topic = (
            self.get_parameter("image_topic")
            .get_parameter_value()
            .string_value
        )

        # ------------------------------------------------------------------
        # ROS image publisher
        #
        # Camera data is transient sensor data. BEST_EFFORT avoids old frames
        # accumulating when a subscriber cannot keep up.
        # ------------------------------------------------------------------

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self.publisher = self.create_publisher(
            Image,
            self.image_topic,
            qos,
        )

        self.bridge = CvBridge()

        # ------------------------------------------------------------------
        # Open the UVC/V4L2 camera
        # ------------------------------------------------------------------

        self.capture = cv2.VideoCapture(
            self.device,
            cv2.CAP_V4L2,
        )

        if not self.capture.isOpened():
            self.get_logger().fatal(
                f"Unable to open eye camera: {self.device}"
            )
            raise RuntimeError(
                f"Unable to open camera {self.device}"
            )

        # Request MJPEG from the Arducam bridge.
        #
        # This is important because the IMX477 USB bridge supports:
        #
        #   1280x720 MJPEG @ up to 100 fps
        #
        # although K9 currently only needs 30 fps.
        fourcc = cv2.VideoWriter_fourcc(*"MJPG")

        self.capture.set(
            cv2.CAP_PROP_FOURCC,
            fourcc,
        )
        self.capture.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self.width,
        )
        self.capture.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self.height,
        )
        self.capture.set(
            cv2.CAP_PROP_FPS,
            self.frame_rate,
        )

        # Keep the capture buffer small so that the ROS stream represents
        # what K9 sees now rather than frames queued several hundred
        # milliseconds ago.
        self.capture.set(
            cv2.CAP_PROP_BUFFERSIZE,
            1,
        )

        # ------------------------------------------------------------------
        # Report the actual mode negotiated with the camera.
        #
        # VideoCapture.set() is a request rather than a guarantee, so this
        # makes configuration errors immediately visible in the log.
        # ------------------------------------------------------------------

        actual_width = int(
            self.capture.get(cv2.CAP_PROP_FRAME_WIDTH)
        )
        actual_height = int(
            self.capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
        )
        actual_fps = self.capture.get(cv2.CAP_PROP_FPS)

        actual_fourcc_int = int(
            self.capture.get(cv2.CAP_PROP_FOURCC)
        )

        actual_fourcc = "".join(
            chr((actual_fourcc_int >> (8 * i)) & 0xFF)
            for i in range(4)
        )

        self.get_logger().info(
            "Eye camera opened: "
            f"{self.device} "
            f"{actual_width}x{actual_height} "
            f"@ {actual_fps:.1f} fps "
            f"format={actual_fourcc}"
        )

        # ------------------------------------------------------------------
        # Frame timer
        # ------------------------------------------------------------------

        timer_period = 1.0 / self.frame_rate

        self.timer = self.create_timer(
            timer_period,
            self.capture_and_publish,
        )

        self.frames_published = 0
        self.frames_failed = 0

    def capture_and_publish(self):
        """
        Capture one frame and publish it.

        OpenCV decodes the MJPEG frame supplied by the UVC camera into BGR.
        cv_bridge then converts it into a sensor_msgs/Image using bgr8.
        """

        success, frame = self.capture.read()

        if not success or frame is None:
            self.frames_failed += 1

            # Avoid flooding the ROS log if the camera temporarily fails.
            if self.frames_failed == 1 or self.frames_failed % 100 == 0:
                self.get_logger().warn(
                    "Failed to capture frame from eye camera "
                    f"(failures={self.frames_failed})"
                )

            return

        # Timestamp as close as practical to acquisition.
        timestamp = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding="bgr8",
        )

        image_msg.header.stamp = timestamp
        image_msg.header.frame_id = self.frame_id

        self.publisher.publish(image_msg)

        self.frames_published += 1

    def destroy_node(self):
        """
        Release the V4L2 device cleanly before shutting down ROS.
        """

        if hasattr(self, "timer") and self.timer is not None:
            self.timer.cancel()

        if hasattr(self, "capture") and self.capture is not None:
            if self.capture.isOpened():
                self.capture.release()

        self.get_logger().info(
            "Eye camera stopped. "
            f"Published {self.frames_published} frames; "
            f"{self.frames_failed} capture failures."
        )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = EyeCameraNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as exc:
        if node is not None:
            node.get_logger().fatal(
                f"Eye camera node failed: {exc!r}"
            )
        else:
            print(f"Eye camera node failed: {exc!r}")

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()