#!/usr/bin/env python3

import ast
import threading
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_srvs.srv import Trigger

from k9_interfaces_pkg.srv import (
    LightsControl,
    SwitchState,
    StringCommand,
)

import serial


class BackLightsNode(Node):
    """
    Owns the Raspberry Pi Pico back-panel controller.

    The node provides two complementary interfaces:

    1. Existing low-level ROS services
       --------------------------------
       These are retained for diagnostics and for other K9 behaviours that
       need direct control of the panel.

       back_lights_on
       back_lights_off
       back_lights_turn_on
       back_lights_turn_off
       back_lights_toggle
       tv_on
       tv_off
       back_lights_get_switch_state
       back_lights_cmd

    2. Automatic interaction presentation
       -----------------------------------
       /audio/effective_state determines K9's persistent interaction mode:

           NOT_LISTENING        -> blue panel pattern
           WAITING_FOR_HOTWORD  -> red panel pattern
           LISTENING            -> green panel pattern

       /interaction/activity provides temporary visual overlays:

           IDLE        -> show persistent interaction mode
           PROCESSING  -> Pico "two" animation
           SPEAKING    -> Pico "four" animation

       When the temporary activity returns to IDLE, the persistent
       interaction-mode display is restored automatically.

    The Pico already performs switch debouncing. This node polls its
    debounced switch state and detects press edges. Valid mode-changing
    button presses are published as requests on:

        /interaction/mode_request

    The node does NOT directly change K9's audio state. The BT/audio
    coordinator remains authoritative.
    """

    # ------------------------------------------------------------------
    # Persistent interaction modes
    # ------------------------------------------------------------------

    MODE_NOT_LISTENING = "NOT_LISTENING"
    MODE_WAITING_FOR_HOTWORD = "WAITING_FOR_HOTWORD"
    MODE_LISTENING = "LISTENING"

    # ------------------------------------------------------------------
    # Temporary interaction activities
    # ------------------------------------------------------------------

    ACTIVITY_IDLE = "IDLE"
    ACTIVITY_PROCESSING = "PROCESSING"
    ACTIVITY_SPEAKING = "SPEAKING"

    # ------------------------------------------------------------------
    # Static back-panel patterns
    #
    # These reproduce the original K9 interaction-state lighting.
    # ------------------------------------------------------------------

    MODE_LIGHTS = {
        MODE_NOT_LISTENING: [
            1,
            3,
            7,
            10,
            12,
        ],
        MODE_WAITING_FOR_HOTWORD: [
            1,
            3,
            6,
            8,
            9,
            12,
        ],
        MODE_LISTENING: [
            1,
            2,
            5,
            9,
            12,
        ],
    }

    # ------------------------------------------------------------------
    # Physical switch mappings
    #
    # The meaning of an illuminated switch depends on K9's current
    # persistent interaction mode.
    #
    # Original mappings:
    #
    # NOT_LISTENING
    #   switch 2  -> WAITING_FOR_HOTWORD
    #   switch 11 -> LISTENING
    #
    # WAITING_FOR_HOTWORD
    #   switch 0  -> NOT_LISTENING
    #   switch 11 -> LISTENING
    #
    # LISTENING
    #   switch 0  -> NOT_LISTENING
    #   switch 8  -> WAITING_FOR_HOTWORD
    # ------------------------------------------------------------------

    MODE_SWITCHES = {
        MODE_NOT_LISTENING: {
            2: MODE_WAITING_FOR_HOTWORD,
            11: MODE_LISTENING,
        },
        MODE_WAITING_FOR_HOTWORD: {
            0: MODE_NOT_LISTENING,
            11: MODE_LISTENING,
        },
        MODE_LISTENING: {
            0: MODE_NOT_LISTENING,
            8: MODE_WAITING_FOR_HOTWORD,
        },
    }

    def __init__(self):
        super().__init__("back_lights_node")

        # --------------------------------------------------------------
        # Parameters
        # --------------------------------------------------------------

        self.declare_parameter(
            "serial_port",
            "/dev/backpanel",
        )
        self.declare_parameter(
            "baudrate",
            115200,
        )
        self.declare_parameter(
            "serial_timeout_sec",
            0.05,
        )
        self.declare_parameter(
            "switch_poll_hz",
            20.0,
        )

        serial_port = str(
            self.get_parameter(
                "serial_port"
            ).value
        )

        baudrate = int(
            self.get_parameter(
                "baudrate"
            ).value
        )

        serial_timeout = max(
            0.01,
            float(
                self.get_parameter(
                    "serial_timeout_sec"
                ).value
            ),
        )

        # Serial traffic consists of both commands and request/response
        # transactions. Protect the complete transaction so future use of
        # a MultiThreadedExecutor cannot interleave Pico messages.
        self._serial_lock = threading.RLock()

        # --------------------------------------------------------------
        # Open Pico serial interface
        # --------------------------------------------------------------

        try:
            self.ser = serial.Serial(
                port=serial_port,
                baudrate=baudrate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS,
                timeout=serial_timeout,
            )

            self.get_logger().info(
                f"Connected to back panel on {serial_port}"
            )

        except serial.SerialException as error:
            self.get_logger().error(
                "Failed to connect to back panel: "
                f"{error}"
            )
            self.ser = None

        # --------------------------------------------------------------
        # Current state
        # --------------------------------------------------------------

        # The persistent interaction mode remains unchanged while K9 is
        # processing or speaking.
        self._mode = self.MODE_NOT_LISTENING

        # Activity temporarily overrides the presentation of the mode.
        self._activity = self.ACTIVITY_IDLE

        # Previous debounced Pico switch state. This lets us detect only
        # new press edges rather than repeatedly responding to a held key.
        self._last_switch_state: Optional[
            list[bool]
        ] = None

        # --------------------------------------------------------------
        # Existing services
        #
        # These names and interfaces are deliberately preserved.
        # --------------------------------------------------------------

        self.create_service(
            Trigger,
            "back_lights_on",
            self.on_handler,
        )

        self.create_service(
            Trigger,
            "back_lights_off",
            self.off_handler,
        )

        self.create_service(
            LightsControl,
            "back_lights_turn_on",
            self.turn_on_handler,
        )

        self.create_service(
            LightsControl,
            "back_lights_turn_off",
            self.turn_off_handler,
        )

        self.create_service(
            LightsControl,
            "back_lights_toggle",
            self.toggle_handler,
        )

        self.create_service(
            Trigger,
            "tv_on",
            self.tv_on_handler,
        )

        self.create_service(
            Trigger,
            "tv_off",
            self.tv_off_handler,
        )

        self.create_service(
            SwitchState,
            "back_lights_get_switch_state",
            self.get_switch_state_handler,
        )

        self.create_service(
            StringCommand,
            "back_lights_cmd",
            self.cmd_service_handler,
        )

        # --------------------------------------------------------------
        # Automatic presentation subscriptions
        # --------------------------------------------------------------

        self.create_subscription(
            String,
            "/audio/effective_state",
            self.audio_mode_cb,
            10,
        )

        self.create_subscription(
            String,
            "/interaction/activity",
            self.activity_cb,
            10,
        )

        # --------------------------------------------------------------
        # Physical interaction-mode requests
        # --------------------------------------------------------------

        self.mode_request_pub = self.create_publisher(
            String,
            "/interaction/mode_request",
            10,
        )

        # --------------------------------------------------------------
        # Switch polling
        # --------------------------------------------------------------

        poll_hz = max(
            1.0,
            float(
                self.get_parameter(
                    "switch_poll_hz"
                ).value
            ),
        )

        self.switch_timer = self.create_timer(
            1.0 / poll_hz,
            self.poll_switches,
        )

        # Start with a deterministic presentation. Once the authoritative
        # /audio/effective_state arrives, the display will be updated.
        self.render()

        self.get_logger().info(
            "Back Lights Node is running: "
            f"switch polling at {poll_hz:.1f} Hz"
        )

    # ==================================================================
    # Serial helpers
    # ==================================================================

    def __write_unlocked(
        self,
        text: str,
    ) -> bool:
        """
        Write to the Pico.

        Caller must already hold _serial_lock.
        """

        if (
            not self.ser
            or not self.ser.is_open
        ):
            self.get_logger().error(
                "Serial port not available. "
                "Command not sent."
            )
            return False

        try:
            self.ser.write(
                (text + "\n").encode()
            )
            return True

        except serial.SerialException as error:
            self.get_logger().error(
                f"Serial write failed: {error}"
            )
            return False

    def __write(
        self,
        text: str,
    ) -> bool:
        """Safely send one command to the Pico."""

        with self._serial_lock:
            return self.__write_unlocked(text)

    def __sw_light(
        self,
        command: str,
        lights: list,
    ) -> bool:
        """
        Apply an individual light operation to a collection of lights.
        """

        success = True

        # Treat the complete collection as one serial transaction so switch
        # polling cannot be inserted halfway through the sequence.
        with self._serial_lock:
            for light in lights:
                if not self.__write_unlocked(
                    f"light {light} {command}"
                ):
                    success = False

        return success

    def __set_static_lights(
        self,
        lights: list[int],
    ) -> bool:
        """
        Configure an exact static panel pattern.

        This reproduces the original sequence:

            cmd("computer")
            off()
            turn_on([...])

        The complete sequence is kept atomic with respect to serial polling.
        """

        success = True

        with self._serial_lock:
            if not self.__write_unlocked(
                "computer"
            ):
                success = False

            if not self.__write_unlocked(
                "off"
            ):
                success = False

            for light in lights:
                if not self.__write_unlocked(
                    f"light {light} on"
                ):
                    success = False

        return success

    def __read_switch_state(
        self,
    ) -> Optional[list[bool]]:
        """
        Request and parse the Pico's already-debounced switch state.

        None means that no valid state was obtained. This is deliberately
        different from [], because a communication failure must not be
        interpreted as every physical button being released.
        """

        with self._serial_lock:

            if not self.__write_unlocked(
                "switchstate"
            ):
                return None

            try:
                line_bytes = self.ser.readline()

            except serial.SerialException as error:
                self.get_logger().error(
                    "Serial read failed: "
                    f"{error}"
                )
                return None

        if not line_bytes:
            return None

        try:
            line = (
                line_bytes
                .decode()
                .strip()
            )

        except UnicodeDecodeError as error:
            self.get_logger().warning(
                "Unable to decode switch-state "
                f"response: {error}"
            )
            return None

        if not line.startswith(
            "switchstate:"
        ):
            self.get_logger().warning(
                "Unexpected Pico response while "
                f"reading switches: {line!r}"
            )
            return None

        payload = line[
            len("switchstate:"):
        ]

        try:
            switchstate_list = (
                ast.literal_eval(payload)
            )

        except (
            ValueError,
            SyntaxError,
        ) as error:
            self.get_logger().warning(
                "Unable to parse switch state "
                f"{payload!r}: {error}"
            )
            return None

        if not isinstance(
            switchstate_list,
            (list, tuple),
        ):
            self.get_logger().warning(
                "Switch state was not a list: "
                f"{switchstate_list!r}"
            )
            return None

        return [
            bool(value)
            for value in switchstate_list
        ]

    # ==================================================================
    # Automatic interaction presentation
    # ==================================================================

    def audio_mode_cb(
        self,
        msg: String,
    ) -> None:
        """
        Update K9's persistent interaction mode.

        This mode persists while temporary PROCESSING/SPEAKING animations
        are displayed.
        """

        value = (
            msg.data
            .strip()
            .upper()
        )

        aliases = {
            "NOTLISTENING":
                self.MODE_NOT_LISTENING,
            "NOT_LISTENING":
                self.MODE_NOT_LISTENING,

            "WAITINGFORHOTWORD":
                self.MODE_WAITING_FOR_HOTWORD,
            "WAITING_FOR_HOTWORD":
                self.MODE_WAITING_FOR_HOTWORD,

            "LISTENING":
                self.MODE_LISTENING,
        }

        resolved = aliases.get(value)

        if resolved is None:
            self.get_logger().warning(
                "Ignoring unknown audio mode: "
                f"{msg.data!r}"
            )
            return

        if resolved == self._mode:
            return

        self._mode = resolved

        self.get_logger().info(
            "Back-panel interaction mode: "
            f"{resolved}"
        )

        # If an activity override is currently active, render() will leave
        # that animation running. The new persistent mode is nevertheless
        # remembered and will appear when activity returns to IDLE.
        self.render()

    def activity_cb(
        self,
        msg: String,
    ) -> None:
        """
        Update K9's temporary conversational activity.
        """

        value = (
            msg.data
            .strip()
            .upper()
        )

        aliases = {
            "IDLE":
                self.ACTIVITY_IDLE,

            "PROCESSING":
                self.ACTIVITY_PROCESSING,

            # Temporary compatibility if THINKING appears elsewhere.
            "THINKING":
                self.ACTIVITY_PROCESSING,

            "SPEAKING":
                self.ACTIVITY_SPEAKING,

            # Temporary compatibility with older terminology.
            "TALKING":
                self.ACTIVITY_SPEAKING,
        }

        resolved = aliases.get(value)

        if resolved is None:
            self.get_logger().warning(
                "Ignoring unknown interaction "
                f"activity: {msg.data!r}"
            )
            return

        if resolved == self._activity:
            return

        self._activity = resolved

        self.get_logger().info(
            "Back-panel activity: "
            f"{resolved}"
        )

        self.render()

    def render(self) -> None:
        """
        Derive the visible panel state from interaction mode + activity.

        Activity has presentation priority but does not alter the
        persistent interaction mode.
        """

        if (
            self._activity
            == self.ACTIVITY_PROCESSING
        ):
            self.__write("two")
            return

        if (
            self._activity
            == self.ACTIVITY_SPEAKING
        ):
            self.__write("four")
            return

        # IDLE means that the persistent interaction-mode panel should
        # once again be visible.
        lights = self.MODE_LIGHTS.get(
            self._mode
        )

        if lights is None:
            return

        self.__set_static_lights(
            lights
        )

    # ==================================================================
    # Automatic physical switch polling
    # ==================================================================

    def poll_switches(self) -> None:
        """
        Poll the Pico's debounced switch state and detect new presses.
        """

        states = self.__read_switch_state()

        if states is None:
            return

        # The first valid reading establishes the initial state. A button
        # already held while the node starts must not look like a new press.
        if self._last_switch_state is None:
            self._last_switch_state = states
            return

        previous = self._last_switch_state
        self._last_switch_state = states

        switch_count = min(
            len(previous),
            len(states),
        )

        for index in range(
            switch_count
        ):
            # Rising edge only.
            pressed = (
                states[index]
                and not previous[index]
            )

            if pressed:
                self.handle_switch_press(
                    index
                )

    def handle_switch_press(
        self,
        switch_index: int,
    ) -> None:
        """
        Convert an illuminated interaction button into a mode request.

        Invalid/non-mode switches are ignored here; they remain available
        for other future back-panel behaviours.
        """

        mode_mapping = (
            self.MODE_SWITCHES.get(
                self._mode,
                {},
            )
        )

        requested_mode = (
            mode_mapping.get(
                switch_index
            )
        )

        if requested_mode is None:
            return

        msg = String()
        msg.data = requested_mode

        self.mode_request_pub.publish(
            msg
        )

        self.get_logger().info(
            f"Back-panel switch "
            f"{switch_index} pressed: "
            f"requesting {requested_mode}"
        )

    # ==================================================================
    # Existing service handlers
    #
    # These have deliberately been retained.
    # ==================================================================

    def on_handler(
        self,
        request,
        response,
    ):
        del request

        # Preserve the existing semantics:
        # "back_lights_on" selects the Pico's original animation.
        success = self.__write(
            "original"
        )

        response.success = success
        response.message = (
            "Lights turned on"
            if success
            else "Failed to turn on lights"
        )

        self.__log_service_result(
            response
        )

        return response

    def off_handler(
        self,
        request,
        response,
    ):
        del request

        success = self.__write(
            "off"
        )

        response.success = success
        response.message = (
            "Lights turned off"
            if success
            else "Failed to turn off lights"
        )

        self.__log_service_result(
            response
        )

        return response

    def turn_on_handler(
        self,
        request,
        response,
    ):
        success = self.__sw_light(
            "on",
            request.lights,
        )

        response.success = success
        response.message = (
            "Lights turned on"
            if success
            else "Failed to turn on some lights"
        )

        self.__log_service_result(
            response
        )

        return response

    def turn_off_handler(
        self,
        request,
        response,
    ):
        success = self.__sw_light(
            "off",
            request.lights,
        )

        response.success = success
        response.message = (
            "Lights turned off"
            if success
            else "Failed to turn off some lights"
        )

        self.__log_service_result(
            response
        )

        return response

    def toggle_handler(
        self,
        request,
        response,
    ):
        success = self.__sw_light(
            "toggle",
            request.lights,
        )

        response.success = success
        response.message = (
            "Lights toggled"
            if success
            else "Failed to toggle some lights"
        )

        self.__log_service_result(
            response
        )

        return response

    def tv_on_handler(
        self,
        request,
        response,
    ):
        del request

        success = self.__write(
            "tvon"
        )

        response.success = success
        response.message = (
            "Side screen on"
            if success
            else "Failed to turn on side screen"
        )

        self.__log_service_result(
            response
        )

        return response

    def tv_off_handler(
        self,
        request,
        response,
    ):
        del request

        success = self.__write(
            "tvoff"
        )

        response.success = success
        response.message = (
            "Side screen off"
            if success
            else "Failed to turn off side screen"
        )

        self.__log_service_result(
            response
        )

        return response

    def get_switch_state_handler(
        self,
        request,
        response,
    ):
        del request

        states = self.__read_switch_state()

        if states is None:
            response.success = False
            response.message = (
                "No valid switch-state response "
                "received from device"
            )
            response.states = []

            self.get_logger().error(
                response.message
            )

            return response

        response.success = True
        response.message = (
            "Switch states received"
        )
        response.states = states

        self.get_logger().info(
            response.message
        )

        self.get_logger().debug(
            f"Switch states: {states}"
        )

        return response

    def cmd_service_handler(
        self,
        request,
        response,
    ):
        """
        Send an arbitrary existing Pico command.

        This deliberately remains available so patterns such as spiral,
        chase_v, rows, cols, three, six, etc. are not lost.
        """

        success = self.__write(
            request.data
        )

        response.success = success
        response.message = (
            "Command sent"
            if success
            else "Failed to send command"
        )

        self.__log_service_result(
            response
        )

        return response

    # ==================================================================
    # Miscellaneous helpers
    # ==================================================================

    def __log_service_result(
        self,
        response,
    ) -> None:
        if response.success:
            self.get_logger().info(
                response.message
            )
        else:
            self.get_logger().error(
                response.message
            )

    # ==================================================================
    # Shutdown
    # ==================================================================

    def destroy_node(self):
        if (
            self.ser
            and self.ser.is_open
        ):
            try:
                self.ser.close()

                self.get_logger().info(
                    "Serial port closed cleanly."
                )

            except serial.SerialException as error:
                self.get_logger().warning(
                    "Error closing serial port: "
                    f"{error}"
                )

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = BackLightsNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()