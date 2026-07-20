#!/usr/bin/env python3

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Deque, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger

from k9_interfaces_pkg.srv import GetBrightness, SetBrightness

'''
Start the node and check parameters:
ros2 run k9_system_pkg eyestail
ros2 param dump /eyes_tail_service_node

Test various levels of brightness:
ros2 topic pub --once /audio/effective_mode std_msgs/msg/String "{data: 'NOT_LISTENING'}"
ros2 topic pub --once /audio/effective_mode std_msgs/msg/String "{data: 'WAITING_FOR_HOTWORD'}"
ros2 topic pub --once /audio/effective_mode std_msgs/msg/String "{data: 'LISTENING'}"
ros2 service call /eyes_get_level k9_interfaces_pkg/srv/GetBrightness "{}"
ros2 service call /eyes_set_level k9_interfaces_pkg/srv/SetBrightness "{level: 0.25}"
ros2 service call /eyes_on std_srvs/srv/Trigger "{}"
ros2 service call /eyes_off std_srvs/srv/Trigger "{}"

Test RMS interface:
ros2 topic pub -r 10 /is_talking std_msgs/msg/Bool "{data: true}"
ros2 topic pub -r 30 /voice/rms_level std_msgs/msg/Float32 "{data: 0.02}"
ros2 topic pub -r 30 /voice/rms_level std_msgs/msg/Float32 "{data: 0.08}"
ros2 topic pub -r 30 /voice/rms_level std_msgs/msg/Float32 "{data: 0.50}"
ros2 topic pub --once /is_talking std_msgs/msg/Bool "{data: false}"

Test tail wagging:
ros2 service call /tail_wag_h std_srvs/srv/Trigger "{}"
ros2 service call /tail_wag_v std_srvs/srv/Trigger "{}"
ros2 service call /tail_centre std_srvs/srv/Trigger "{}"
ros2 service call /tail_up std_srvs/srv/Trigger "{}"
ros2 service call /tail_down std_srvs/srv/Trigger "{}"

Check emergency override:
ros2 topic pub --once /safety/emergency_active std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /is_talking std_msgs/msg/Bool "{data: false}"
ros2 topic pub --once /safety/emergency_active std_msgs/msg/Bool "{data: false}"
'''


@dataclass(frozen=True)
class TailStep:
    channel_4: int
    channel_5: int
    duration_sec: float


class EyesTail:
    """Low-level PCA9685 hardware access.
    """

    EYES_CHANNEL = 0
    TAIL_VERTICAL_CHANNEL = 4
    TAIL_HORIZONTAL_CHANNEL = 5

    def __init__(self, node: Node):
        self.node = node
        self.available = False
        self._level = 0.0
        self._last_eye_duty: Optional[int] = None

        try:
            import board
            import busio
            from adafruit_pca9685 import PCA9685

            self.i2c = busio.I2C(board.SCL, board.SDA)
            self.pca = PCA9685(self.i2c, address=0x40)

            # PCA9685 frequency is global to the whole chip. The tail servos
            # require approximately 50-60 Hz, so the eye LED on channel 0
            # must currently use the same carrier frequency.
            self.pca.frequency = 60

            self.available = True
            self.node.get_logger().info(
                "Eyes and tail hardware initialised successfully"
            )
        except Exception as error:
            self.node.get_logger().error(
                f"Failed to initialise PCA9685: {error}"
            )
            return

        self.set_eye_level(0.0)
        self.disable_tail_outputs()

    def set_eye_level(self, level: float) -> None:
        """Set eye LED brightness immediately, without smoothing."""
        level = min(max(float(level), 0.0), 1.0)
        self._level = level

        if not self.available:
            return

        duty = int(round(level * 65535.0))

        # Avoid unnecessary I2C writes when the duty cycle is unchanged.
        if duty == self._last_eye_duty:
            return

        self.pca.channels[self.EYES_CHANNEL].duty_cycle = duty
        self._last_eye_duty = duty

    def get_eye_level(self) -> float:
        return self._level

    def set_tail_outputs(self, channel_4: int, channel_5: int) -> None:
        if not self.available:
            return

        self.pca.channels[self.TAIL_VERTICAL_CHANNEL].duty_cycle = int(channel_4)
        self.pca.channels[self.TAIL_HORIZONTAL_CHANNEL].duty_cycle = int(channel_5)

    def disable_tail_outputs(self) -> None:
        self.set_tail_outputs(0, 0)


class EyesTailServiceNode(Node):
    """Owns eye animation and non-blocking tail motion."""

    MODE_NOT_LISTENING = "NOT_LISTENING"
    MODE_WAITING_FOR_HOTWORD = "WAITING_FOR_HOTWORD"
    MODE_LISTENING = "LISTENING"

    CONTROL_PERIOD_SEC = 0.025  # 40 Hz

    def __init__(self):
        super().__init__("eyes_tail_service_node")

        # Eye-policy parameters.
        self.declare_parameter("not_listening_level", 0.0)
        self.declare_parameter("waiting_for_hotword_level", 0.03)
        self.declare_parameter("listening_level", 0.80)

        # RMS mapping parameters. Assumes /voice/rms_level is normalised 0..1.
        self.declare_parameter("talking_min_level", 0.08)
        self.declare_parameter("talking_max_level", 1.0)
        self.declare_parameter("rms_floor", 0.01)
        self.declare_parameter("rms_ceiling", 0.20)
        self.declare_parameter("rms_gamma", 0.60)
        self.declare_parameter("rms_stale_timeout_sec", 0.20)

        # Eye envelope smoothing.
        self.declare_parameter("eye_attack", 0.55)
        self.declare_parameter("eye_release", 0.18)

        # Old eye services remain available as short diagnostic overrides.
        self.declare_parameter("manual_override_sec", 2.0)

        self.eyestail = EyesTail(self)

        # Authoritative state used by the eye policy.
        self._audio_mode = self.MODE_NOT_LISTENING
        self._is_talking = False
        self._emergency_active = False
        self._latest_rms = 0.0
        self._latest_rms_time = 0.0
        self._current_eye_level = 0.0

        # Optional short-lived manual override for backwards-compatible
        # eyes_set_level / eyes_on / eyes_off service calls.
        self._manual_eye_level: Optional[float] = None
        self._manual_override_until = 0.0

        # Non-blocking tail sequence state.
        self._tail_steps: Deque[TailStep] = deque()
        self._tail_step_deadline = 0.0
        self._tail_step_active = False

        # Input state subscriptions.
        self.create_subscription(
            Float32,
            "/voice/rms_level",
            self.rms_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "is_talking",
            self.talking_cb,
            10,
        )
        self.create_subscription(
            String,
            "/audio/effective_mode",
            self.audio_mode_cb,
            10,
        )
        self.create_subscription(
            Bool,
            "/safety/emergency_active",
            self.emergency_cb,
            10,
        )

        # Existing eye services, retained for diagnostics/backwards compatibility.
        self.create_service(
            SetBrightness,
            "eyes_set_level",
            self.set_level_cb,
        )
        self.create_service(
            GetBrightness,
            "eyes_get_level",
            self.get_level_cb,
        )
        self.create_service(Trigger, "eyes_on", self.on_cb)
        self.create_service(Trigger, "eyes_off", self.off_cb)

        # Tail services. Both spellings are exposed temporarily because the
        # existing codebase has used centre and center.
        self.create_service(Trigger, "tail_wag_h", self.wag_h_cb)
        self.create_service(Trigger, "tail_wag_v", self.wag_v_cb)
        self.create_service(Trigger, "tail_centre", self.centre_cb)
        self.create_service(Trigger, "tail_center", self.centre_cb)
        self.create_service(Trigger, "tail_up", self.up_cb)
        self.create_service(Trigger, "tail_down", self.down_cb)

        # One 40 Hz control loop updates both eyes and tail without blocking.
        self.control_timer = self.create_timer(
            self.CONTROL_PERIOD_SEC,
            self.control_tick,
        )

        self.get_logger().info(
            "Eyes and tail node ready: automatic eye control at 40 Hz"
        )

    # ------------------------------------------------------------------
    # State subscriptions
    # ------------------------------------------------------------------

    def rms_cb(self, msg: Float32) -> None:
        self._latest_rms = max(0.0, float(msg.data))
        self._latest_rms_time = time.monotonic()

    def talking_cb(self, msg: Bool) -> None:
        self._is_talking = bool(msg.data)

        # A new speaking state should take precedence over a diagnostic
        # eye override immediately.
        if self._is_talking:
            self.clear_manual_override()

    def audio_mode_cb(self, msg: String) -> None:
        mode = msg.data.strip().upper()

        aliases = {
            "NOTLISTENING": self.MODE_NOT_LISTENING,
            "NOT_LISTENING": self.MODE_NOT_LISTENING,
            "WAITINGFORHOTWORD": self.MODE_WAITING_FOR_HOTWORD,
            "WAITING_FOR_HOTWORD": self.MODE_WAITING_FOR_HOTWORD,
            "LISTENING": self.MODE_LISTENING,
        }

        resolved = aliases.get(mode)
        if resolved is None:
            self.get_logger().warning(
                f"Ignoring unknown audio mode: {msg.data!r}"
            )
            return

        if resolved != self._audio_mode:
            self._audio_mode = resolved
            self.clear_manual_override()
            self.get_logger().info(
                f"Effective audio mode changed to {resolved}"
            )

    def emergency_cb(self, msg: Bool) -> None:
        was_active = self._emergency_active
        self._emergency_active = bool(msg.data)

        if self._emergency_active:
            self.clear_manual_override()
            self.cancel_tail_motion()
            self._current_eye_level = 0.0
            self.eyestail.set_eye_level(0.0)

            if not was_active:
                self.get_logger().warning(
                    "Emergency active: eyes off and tail motion cancelled"
                )
        elif was_active:
            self.get_logger().info(
                "Emergency released; automatic eye policy resumed"
            )

    # ------------------------------------------------------------------
    # 40 Hz control loop
    # ------------------------------------------------------------------

    def control_tick(self) -> None:
        self.update_tail_motion()

        target = self.calculate_eye_target()
        self.update_eye_envelope(target)

    def calculate_eye_target(self) -> float:
        now = time.monotonic()

        # Highest priority: emergency.
        if self._emergency_active:
            return 0.0

        # Second priority: outgoing speech animation.
        if self._is_talking:
            rms = self._latest_rms
            if now - self._latest_rms_time > self.rms_stale_timeout_sec:
                rms = 0.0
            return self.map_rms_to_eye_level(rms)

        # Optional diagnostic override. Normal BT operation should publish
        # /audio/effective_mode instead of repeatedly calling eye services.
        if (
            self._manual_eye_level is not None
            and now < self._manual_override_until
        ):
            return self._manual_eye_level

        if self._manual_eye_level is not None:
            self.clear_manual_override()

        if self._audio_mode == self.MODE_LISTENING:
            return self.listening_level

        if self._audio_mode == self.MODE_WAITING_FOR_HOTWORD:
            return self.waiting_for_hotword_level

        return self.not_listening_level

    def map_rms_to_eye_level(self, rms: float) -> float:
        floor = self.rms_floor
        ceiling = max(self.rms_ceiling, floor + 1.0e-6)

        normalised = (rms - floor) / (ceiling - floor)
        normalised = min(max(normalised, 0.0), 1.0)

        # Gamma below 1.0 makes normal speech more visibly expressive.
        shaped = math.pow(normalised, self.rms_gamma)

        return (
            self.talking_min_level
            + shaped * (self.talking_max_level - self.talking_min_level)
        )

    def update_eye_envelope(self, target: float) -> None:
        target = min(max(target, 0.0), 1.0)

        alpha = (
            self.eye_attack
            if target > self._current_eye_level
            else self.eye_release
        )
        alpha = min(max(alpha, 0.0), 1.0)

        self._current_eye_level += alpha * (
            target - self._current_eye_level
        )

        # Snap tiny residuals to the target.
        if abs(self._current_eye_level - target) < 0.001:
            self._current_eye_level = target

        self.eyestail.set_eye_level(self._current_eye_level)

    # ------------------------------------------------------------------
    # Non-blocking tail scheduler
    # ------------------------------------------------------------------

    def start_tail_sequence(self, steps: list[TailStep]) -> None:
        if self._emergency_active:
            self.get_logger().warning(
                "Ignoring tail command while emergency is active"
            )
            return

        self._tail_steps = deque(steps)
        self._tail_step_active = False
        self._tail_step_deadline = 0.0

    def update_tail_motion(self) -> None:
        if self._emergency_active:
            self.cancel_tail_motion()
            return

        now = time.monotonic()

        if self._tail_step_active and now < self._tail_step_deadline:
            return

        if self._tail_step_active:
            self._tail_step_active = False

        if not self._tail_steps:
            self.eyestail.disable_tail_outputs()
            return

        step = self._tail_steps.popleft()
        self.eyestail.set_tail_outputs(
            step.channel_4,
            step.channel_5,
        )
        self._tail_step_deadline = now + step.duration_sec
        self._tail_step_active = True

    def cancel_tail_motion(self) -> None:
        self._tail_steps.clear()
        self._tail_step_active = False
        self._tail_step_deadline = 0.0
        self.eyestail.disable_tail_outputs()

    def horizontal_wag_steps(self) -> list[TailStep]:
        steps: list[TailStep] = []
        for _ in range(4):
            steps.append(TailStep(5121, 5201, 0.25))
            steps.append(TailStep(5121, 7042, 0.25))
        steps.append(TailStep(5121, 5601, 0.25))
        return steps

    def vertical_wag_steps(self) -> list[TailStep]:
        steps: list[TailStep] = []
        for _ in range(4):
            steps.append(TailStep(5921, 5601, 0.25))
            steps.append(TailStep(4321, 5601, 0.25))
        steps.append(TailStep(5121, 5601, 0.25))
        return steps

    # ------------------------------------------------------------------
    # Eye services
    # ------------------------------------------------------------------

    def set_manual_override(self, level: float) -> None:
        level = min(max(float(level), 0.0), 1.0)
        duration = max(
            0.0,
            float(self.get_parameter("manual_override_sec").value),
        )

        self._manual_eye_level = level
        self._manual_override_until = time.monotonic() + duration

    def clear_manual_override(self) -> None:
        self._manual_eye_level = None
        self._manual_override_until = 0.0

    def set_level_cb(self, request, response):
        level = min(max(float(request.level), 0.0), 1.0)
        self.set_manual_override(level)
        response.success = True
        response.message = (
            f"Temporary manual eye brightness set to {level:.2f}"
        )
        return response

    def get_level_cb(self, request, response):
        del request
        response.level = self.eyestail.get_eye_level()
        return response

    def on_cb(self, request, response):
        del request
        self.set_manual_override(1.0)
        response.success = True
        response.message = "Eyes temporarily set to full brightness"
        return response

    def off_cb(self, request, response):
        del request
        self.set_manual_override(0.0)
        response.success = True
        response.message = "Eyes temporarily set off"
        return response

    # ------------------------------------------------------------------
    # Tail services
    # ------------------------------------------------------------------

    def wag_h_cb(self, request, response):
        del request
        self.start_tail_sequence(self.horizontal_wag_steps())
        response.success = not self._emergency_active
        response.message = (
            "Horizontal tail wag started"
            if response.success
            else "Tail command rejected: emergency active"
        )
        return response

    def wag_v_cb(self, request, response):
        del request
        self.start_tail_sequence(self.vertical_wag_steps())
        response.success = not self._emergency_active
        response.message = (
            "Vertical tail wag started"
            if response.success
            else "Tail command rejected: emergency active"
        )
        return response

    def centre_cb(self, request, response):
        del request
        self.start_tail_sequence([
            TailStep(5121, 5601, 0.25),
        ])
        response.success = not self._emergency_active
        response.message = (
            "Tail centring started"
            if response.success
            else "Tail command rejected: emergency active"
        )
        return response

    def up_cb(self, request, response):
        del request
        self.start_tail_sequence([
            TailStep(4321, 5601, 0.25),
        ])
        response.success = not self._emergency_active
        response.message = (
            "Tail raise started"
            if response.success
            else "Tail command rejected: emergency active"
        )
        return response

    def down_cb(self, request, response):
        del request
        self.start_tail_sequence([
            TailStep(5921, 5601, 0.25),
        ])
        response.success = not self._emergency_active
        response.message = (
            "Tail lower started"
            if response.success
            else "Tail command rejected: emergency active"
        )
        return response

    # ------------------------------------------------------------------
    # Parameter helpers
    # ------------------------------------------------------------------

    @property
    def not_listening_level(self) -> float:
        return float(self.get_parameter("not_listening_level").value)

    @property
    def waiting_for_hotword_level(self) -> float:
        return float(
            self.get_parameter("waiting_for_hotword_level").value
        )

    @property
    def listening_level(self) -> float:
        return float(self.get_parameter("listening_level").value)

    @property
    def talking_min_level(self) -> float:
        return float(self.get_parameter("talking_min_level").value)

    @property
    def talking_max_level(self) -> float:
        return float(self.get_parameter("talking_max_level").value)

    @property
    def rms_floor(self) -> float:
        return float(self.get_parameter("rms_floor").value)

    @property
    def rms_ceiling(self) -> float:
        return float(self.get_parameter("rms_ceiling").value)

    @property
    def rms_gamma(self) -> float:
        return max(0.05, float(self.get_parameter("rms_gamma").value))

    @property
    def rms_stale_timeout_sec(self) -> float:
        return max(
            0.0,
            float(self.get_parameter("rms_stale_timeout_sec").value),
        )

    @property
    def eye_attack(self) -> float:
        return float(self.get_parameter("eye_attack").value)

    @property
    def eye_release(self) -> float:
        return float(self.get_parameter("eye_release").value)


def main(args=None):
    rclpy.init(args=args)
    node = EyesTailServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cancel_tail_motion()
        node.eyestail.set_eye_level(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()