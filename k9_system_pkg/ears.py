#!/usr/bin/env python3

# ros2 run k9_system_pkg ears
#
# All services available via Trigger:
#   ears_stop
#   ears_scan
#   ears_fast
#   ears_think
#
# Example:
# ros2 service call /ears_fast std_srvs/srv/Trigger "{}"

import math

import rclpy
import serial

from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger


class SimEars:
    """
    Sim ears backend for gz-sim + ros2_control.

    Publishes to a ForwardCommandController:
      /ears_position_controller/commands
      std_msgs/Float64MultiArray

    Joint order MUST match ear_controllers.yaml:
      [l_ear_joint, r_ear_joint]
    """

    def __init__(
        self,
        node,
        controller_name: str = "ears_position_controller",
    ):
        self.node = node
        self.controller_name = controller_name

        # ForwardCommandController command topic.
        self.cmd_topic = (
            f"/{self.controller_name}/commands"
        )

        self.cmd_pub = node.create_publisher(
            Float64MultiArray,
            self.cmd_topic,
            10,
        )

        # State.
        self.scanning = False
        self.speed = 1.0
        self.phase = 0.0

        # Joint limits from URDF.
        self.l_min = -0.2
        self.l_max = 0.6
        self.r_min = -0.6
        self.r_max = 0.2

        # Centre + amplitude remain within joint limits.
        self.l_center = 0.2
        self.l_amp = 0.4

        self.r_center = -0.2
        self.r_amp = 0.4

        # Timer for sweep motion at 20 Hz.
        self.dt = 0.05

        self.timer = node.create_timer(
            self.dt,
            self._update,
        )

        node.get_logger().info(
            f"SimEars publishing to {self.cmd_topic} "
            "as Float64MultiArray [l,r]"
        )

    def _clamp(
        self,
        value: float,
        minimum: float,
        maximum: float,
    ) -> float:
        return max(
            minimum,
            min(
                maximum,
                value,
            ),
        )

    def _publish(
        self,
        l_angle: float,
        r_angle: float,
    ) -> None:
        msg = Float64MultiArray()

        msg.data = [
            float(l_angle),
            float(r_angle),
        ]

        self.cmd_pub.publish(msg)

    def _update(self) -> None:
        if not self.scanning:
            return

        # Phase advance controls sweep speed.
        self.phase += (
            self.dt
            * self.speed
        )

        # Symmetric scan.
        s = math.sin(
            self.phase
        )

        l_angle = (
            self.l_center
            + self.l_amp * s
        )

        r_angle = (
            self.r_center
            - self.r_amp * s
        )

        # Safety clamps.
        l_angle = self._clamp(
            l_angle,
            self.l_min,
            self.l_max,
        )

        r_angle = self._clamp(
            r_angle,
            self.r_min,
            self.r_max,
        )

        self._publish(
            l_angle,
            r_angle,
        )

    def stop(self) -> None:
        self.scanning = False
        self.speed = 0.0

        # Hold at centre when stopped.
        self._publish(
            self.l_center,
            self.r_center,
        )

    def scan(self) -> None:
        self.scanning = True
        self.speed = 1.0

    def fast(self) -> None:
        self.scanning = True
        self.speed = 2.0

    def think(self) -> None:
        self.scanning = True
        self.speed = 0.4


class Ears:
    """
    Communicate with the Espruino controlling K9's LIDAR ears.
    """

    def __init__(
        self,
        node,
    ) -> None:
        self.node = node

        try:
            self.ser = serial.Serial(
                port="/dev/ears",
                baudrate=115200,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=0.5,
            )

        except serial.SerialException as exc:
            self.node.get_logger().error(
                f"Failed to connect to ears: {exc}"
            )
            self.ser = None

        self.following = False

    def __write(
        self,
        text: str,
    ) -> None:
        self.node.get_logger().debug(
            f"Ears: {text}"
        )

        if not self.ser:
            return

        self.ser.write(
            str.encode(
                text + "()\n"
            )
        )

    def stop(self) -> None:
        self.following = False
        self.__write(
            "stop"
        )

    def scan(self) -> None:
        self.following = False
        self.__write(
            "scan"
        )

    def fast(self) -> None:
        self.following = False
        self.__write(
            "fast"
        )

    def think(self) -> None:
        self.following = False
        self.__write(
            "think"
        )


class EarsServiceNode(Node):

    def __init__(self):
        super().__init__(
            "ears_service_node"
        )

        # ------------------------------------------------------------------
        # Backend
        # ------------------------------------------------------------------

        self.declare_parameter(
            "sim",
            False,
        )

        sim = bool(
            self.get_parameter(
                "sim"
            ).value
        )

        if sim:
            self.get_logger().info(
                "Using SimEars backend"
            )

            self.ears = SimEars(
                self,
                controller_name=(
                    "ears_position_controller"
                ),
            )

        else:
            self.get_logger().info(
                "Using real Ears backend (/dev/ears)"
            )

            self.ears = Ears(
                self
            )

        # ------------------------------------------------------------------
        # Interaction activity
        #
        # PROCESSING is published when STT has detected the end of the
        # user's utterance and begins transcription.
        #
        # The ears therefore enter THINK mode immediately after K9 has
        # finished listening, and remain there throughout STT and LLM
        # processing.
        #
        # SPEAKING stops the ears immediately before TTS playback.
        # ------------------------------------------------------------------

        self._interaction_activity = "IDLE"

        self._activity_subscription = (
            self.create_subscription(
                String,
                "/interaction/activity",
                self._activity_callback,
                10,
            )
        )

        # ------------------------------------------------------------------
        # Services
        # ------------------------------------------------------------------

        self.create_service(
            Trigger,
            "ears_stop",
            self.stop_cb,
        )

        self.create_service(
            Trigger,
            "ears_scan",
            self.scan_cb,
        )

        self.create_service(
            Trigger,
            "ears_fast",
            self.fast_cb,
        )

        self.create_service(
            Trigger,
            "ears_think",
            self.think_cb,
        )

        self.get_logger().info(
            "Ears Node is running."
        )

    # ==================================================================
    # Interaction activity
    # ==================================================================

    def _activity_callback(
        self,
        msg: String,
    ) -> None:
        """
        Animate the ears according to K9's interaction activity.

        PROCESSING:
            VAD has detected the end of the user's utterance.
            Start the thinking animation.

        SPEAKING:
            K9 is about to speak.
            Stop the thinking animation.

        IDLE:
            Safety stop in case processing ends without speech.
        """

        activity = (
            msg.data
            .strip()
            .upper()
        )

        # Ignore repeated publications of the same activity.
        if activity == self._interaction_activity:
            return

        previous_activity = (
            self._interaction_activity
        )

        self._interaction_activity = (
            activity
        )

        if activity == "PROCESSING":
            self.get_logger().debug(
                "Interaction PROCESSING: "
                "ears entering think mode"
            )

            self.ears.think()
            return

        if activity == "SPEAKING":
            self.get_logger().debug(
                "Interaction SPEAKING: "
                "stopping ear think mode"
            )

            self.ears.stop()
            return

        # If processing terminates without entering SPEAKING,
        # ensure that the ears do not remain in think mode.
        if (
            activity == "IDLE"
            and previous_activity == "PROCESSING"
        ):
            self.get_logger().debug(
                "Interaction returned to IDLE: "
                "stopping ear think mode"
            )

            self.ears.stop()

    # ==================================================================
    # Service callbacks
    # ==================================================================

    def stop_cb(
        self,
        request,
        response,
    ):
        self.ears.stop()

        response.success = True
        response.message = (
            "Stopped ears."
        )

        self.get_logger().info(
            response.message
        )

        return response

    def scan_cb(
        self,
        request,
        response,
    ):
        self.ears.scan()

        response.success = True
        response.message = (
            "Started scan."
        )

        self.get_logger().info(
            response.message
        )

        return response

    def fast_cb(
        self,
        request,
        response,
    ):
        self.ears.fast()

        response.success = True
        response.message = (
            "Set to fast mode."
        )

        self.get_logger().info(
            response.message
        )

        return response

    def think_cb(
        self,
        request,
        response,
    ):
        self.ears.think()

        response.success = True
        response.message = (
            "Set to think mode."
        )

        self.get_logger().info(
            response.message
        )

        return response


def main(
    args=None,
):
    rclpy.init(
        args=args
    )

    node = EarsServiceNode()

    try:
        rclpy.spin(
            node
        )

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
