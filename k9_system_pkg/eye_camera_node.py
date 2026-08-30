#!/usr/bin/env python3

import threading
import time

import gi

gi.require_version("Gst", "1.0")
from gi.repository import Gst

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)

from sensor_msgs.msg import CompressedImage


class EyeCameraNode(Node):
    """
    Capture K9's Arducam IMX477 UVC MJPEG stream and publish the
    JPEG frames directly as sensor_msgs/CompressedImage.

    No JPEG decode/re-encode is performed on the Raspberry Pi.

    Camera:
        Arducam IMX477 HQ Camera
        UVC/V4L2
        MJPEG

    Typical operating mode:
        camera capture: 1280x720 @ 100 fps
        ROS publication: 30 fps
    """

    def __init__(self):
        super().__init__("eye_camera")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------

        self.declare_parameter(
            "device",
            "/dev/k9_eye_camera",
        )

        self.declare_parameter(
            "camera_width",
            1280,
        )

        self.declare_parameter(
            "camera_height",
            720,
        )

        # This must correspond to a mode actually advertised by V4L2.
        #
        # The Arducam currently advertises:
        #
        #   1280x720 MJPEG @ 100 fps
        #
        self.declare_parameter(
            "capture_frame_rate",
            100,
        )

        # We do not need to send all 100 camera frames across ROS.
        self.declare_parameter(
            "publish_frame_rate",
            30.0,
        )

        self.declare_parameter(
            "frame_id",
            "k9_eye_camera",
        )

        self.declare_parameter(
            "image_topic",
            "/k9/camera/eye/image/compressed",
        )

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

        self.capture_frame_rate = (
            self.get_parameter("capture_frame_rate")
            .get_parameter_value()
            .integer_value
        )

        self.publish_frame_rate = (
            self.get_parameter("publish_frame_rate")
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

        if self.publish_frame_rate <= 0:
            raise ValueError(
                "publish_frame_rate must be greater than zero"
            )

        # --------------------------------------------------------------
        # ROS publisher
        #
        # BEST_EFFORT is appropriate for live camera data.
        # There is no benefit in retransmitting old frames.
        # --------------------------------------------------------------

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )

        self.publisher = self.create_publisher(
            CompressedImage,
            self.image_topic,
            qos,
        )

        # --------------------------------------------------------------
        # GStreamer
        # --------------------------------------------------------------

        Gst.init(None)

        self.pipeline = None
        self.appsink = None

        self.running = False
        self.capture_thread = None

        # Minimum time between published frames.
        self.publish_interval = 1.0 / self.publish_frame_rate

        self.next_publish_time = time.monotonic()

        # Statistics
        self.frames_received = 0
        self.frames_published = 0
        self.frames_dropped = 0
        self.capture_errors = 0

        self._create_pipeline()

        # --------------------------------------------------------------
        # Start capture thread
        # --------------------------------------------------------------

        self.running = True

        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name="k9-eye-camera",
        )

        self.capture_thread.start()

        # Periodic statistics are useful while tuning the camera.
        self.stats_timer = self.create_timer(
            10.0,
            self._log_statistics,
        )

        self.get_logger().info(
            "Eye camera started: "
            f"{self.device} "
            f"{self.width}x{self.height} "
            f"MJPEG capture={self.capture_frame_rate} fps "
            f"publish={self.publish_frame_rate:.1f} fps "
            f"topic={self.image_topic}"
        )

    def _create_pipeline(self):
        """
        Construct a GStreamer pipeline which leaves the camera's
        JPEG data compressed.

        v4l2src
            ↓
        image/jpeg
            ↓
        appsink

        max-buffers=1 and drop=true are important: perception systems
        generally want the newest image, not an old queued frame.
        """

        pipeline_description = (
            f"v4l2src device={self.device} "
            f"! image/jpeg,"
            f"width={self.width},"
            f"height={self.height},"
            f"framerate={self.capture_frame_rate}/1 "
            f"! appsink "
            f"name=k9_eye_sink "
            f"max-buffers=1 "
            f"drop=true "
            f"sync=false "
            f"emit-signals=false"
        )

        self.get_logger().info(
            f"GStreamer pipeline: {pipeline_description}"
        )

        try:
            self.pipeline = Gst.parse_launch(
                pipeline_description
            )

        except Exception as exc:
            raise RuntimeError(
                f"Unable to create GStreamer pipeline: {exc}"
            ) from exc

        self.appsink = self.pipeline.get_by_name(
            "k9_eye_sink"
        )

        if self.appsink is None:
            raise RuntimeError(
                "Unable to find GStreamer appsink"
            )

        result = self.pipeline.set_state(
            Gst.State.PLAYING
        )

        if result == Gst.StateChangeReturn.FAILURE:
            raise RuntimeError(
                "GStreamer camera pipeline failed to start"
            )

    def _capture_loop(self):
        """
        Continuously retrieve the newest JPEG frame from GStreamer.

        The camera may run at 100 fps, but ROS publication is throttled
        independently to publish_frame_rate.

        Because the appsink buffer is restricted to one frame, frames
        which K9 does not need are discarded rather than accumulating.
        """

        while self.running and rclpy.ok():

            try:
                sample = self.appsink.emit(
                    "try-pull-sample",
                    Gst.SECOND,
                )

                if sample is None:
                    self.capture_errors += 1

                    if (
                        self.capture_errors == 1
                        or self.capture_errors % 10 == 0
                    ):
                        self.get_logger().warn(
                            "Timed out waiting for eye camera frame"
                        )

                    continue

                self.frames_received += 1

                now = time.monotonic()

                # ------------------------------------------------------
                # Throttle ROS publication.
                #
                # We deliberately allow the UVC camera to operate at its
                # supported native frame rate while only forwarding the
                # number of frames K9's perception system requires.
                # ------------------------------------------------------

                if now < self.next_publish_time:
                    self.frames_dropped += 1
                    continue

                # Advance the ideal publication schedule rather than resetting it
                # to the actual arrival time of this camera frame.
                while self.next_publish_time <= now:
                    self.next_publish_time += self.publish_interval

                buffer = sample.get_buffer()

                success, map_info = buffer.map(
                    Gst.MapFlags.READ
                )

                if not success:
                    self.capture_errors += 1
                    continue

                try:
                    # map_info.data contains the JPEG produced by the
                    # Arducam UVC bridge. No decode has occurred.
                    jpeg_data = bytes(map_info.data)

                finally:
                    buffer.unmap(map_info)

                msg = CompressedImage()

                msg.header.stamp = (
                    self.get_clock().now().to_msg()
                )

                msg.header.frame_id = self.frame_id

                # Standard sensor_msgs convention.
                msg.format = "jpeg"

                msg.data = jpeg_data

                self.publisher.publish(msg)

                self.frames_published += 1

            except Exception as exc:
                self.capture_errors += 1

                self.get_logger().error(
                    f"Eye camera capture error: {exc!r}"
                )

                # Prevent a tight error loop.
                time.sleep(0.1)

    def _log_statistics(self):
        """
        Periodically report how the capture/publish pipeline is behaving.
        """

        self.get_logger().info(
            "Eye camera statistics: "
            f"received={self.frames_received}, "
            f"published={self.frames_published}, "
            f"dropped={self.frames_dropped}, "
            f"errors={self.capture_errors}"
        )

    def destroy_node(self):
        """
        Stop capture and release the UVC camera cleanly.
        """

        self.running = False

        if (
            self.capture_thread is not None
            and self.capture_thread.is_alive()
        ):
            self.capture_thread.join(
                timeout=2.0
            )

        if self.pipeline is not None:
            self.pipeline.set_state(
                Gst.State.NULL
            )

        if hasattr(self, "stats_timer"):
            self.stats_timer.cancel()

        self.get_logger().info(
            "Eye camera stopped: "
            f"received={self.frames_received}, "
            f"published={self.frames_published}, "
            f"dropped={self.frames_dropped}, "
            f"errors={self.capture_errors}"
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
            print(
                f"Eye camera node failed: {exc!r}"
            )

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()