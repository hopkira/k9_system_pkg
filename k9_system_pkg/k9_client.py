#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from k9_interfaces_pkg.srv import LightsControl, SwitchState  # Custom interfaces
from std_msgs.msg import String, Bool
from k9_interfaces_pkg.srv import Speak, CancelSpeech  # Updated to k9_voice package
import serial
import ast


class ServiceClientHelper:
    def __init__(self, node: Node):
        self.node = node

    def create_client(self, service_type, service_name):
        """Helper method to create service clients."""
        client = self.node.create_client(service_type, service_name)
        while not client.wait_for_service(timeout_sec=1.0):
            print(f"Waiting for service {service_name} to be available...")
        return client

    def call_service(self, client, request):
        """Helper method to call a service."""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            print(f"Service response: {future.result().message}")
            return future.result()
        else:
            print("Service call failed.")
            return None


class Voice:
    def __init__(self, node: Node, service_helper: ServiceClientHelper):
        self.node = node
        self.service_helper = service_helper
        # Create clients for TTS services
        self.client_speak = self.service_helper.create_client(Speak, 'speak_now')
        self.client_cancel = self.service_helper.create_client(CancelSpeech, 'cancel_speech')

    def speak(self, text: str):
        """Call the Speak service to immediately speak the text."""
        request = Speak.Request()
        request.text = text
        response = self.service_helper.call_service(self.client_speak, request)
        if response and response.success:
            self.node.get_logger().info(f"Speaking: {text}")
        else:
            self.node.get_logger().error(f"Failed to speak: {text}")

    def cancel_speech(self):
        """Call the CancelSpeech service to cancel ongoing speech."""
        request = CancelSpeech.Request()
        response = self.service_helper.call_service(self.client_cancel, request)
        if response and response.success:
            self.node.get_logger().info("Speech canceled.")
        else:
            self.node.get_logger().error("Failed to cancel speech.")


class Eyes:
    def __init__(self, node: Node, service_helper: ServiceClientHelper):
        self.node = node
        self.service_helper = service_helper
        self.client_set_level = self.service_helper.create_client(SetBrightness, 'eyes_set_level')
        self.client_get_level = self.service_helper.create_client(GetBrightness, 'eyes_get_level')
        self.client_on = self.service_helper.create_client(Trigger, 'eyes_on')
        self.client_off = self.service_helper.create_client(Trigger, 'eyes_off')

        self._is_talking = False
        self._stored_level = 0.0  # Saved level before talking began

        # Subscribers
        self.create_subscription(Bool, 'is_talking', self.talking_cb, 10)

    def talking_cb(self, msg: Bool):
        """Handles is_talking state and adjusts brightness."""
        if msg.data and not self._is_talking:
            self._stored_level = self.get_level()
            self.set_level(1.0)  # Eyes on with full brightness
            self._is_talking = True
            self.node.get_logger().info("Talking detected: eyes set to 100%")
        elif not msg.data and self._is_talking:
            # Stop talking: restore previous level
            self.set_level(self._stored_level)
            self._is_talking = False
            self.node.get_logger().info(f"Stopped talking: eyes restored to {self._stored_level:.2f}")

    def set_level(self, level: float):
        """Sets the brightness level of the eyes."""
        request = SetBrightness.Request()
        request.level = level
        self.service_helper.call_service(self.client_set_level, request)

    def get_level(self) -> float:
        """Gets the current brightness level of the eyes."""
        request = GetBrightness.Request()
        result = self.service_helper.call_service(self.client_get_level, request)
        return result.level if result else 0.0

    def on(self):
        """Turns the eyes on."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_on, request)

    def off(self):
        """Turns the eyes off."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_off, request)


class Tail:
    def __init__(self, node: Node, service_helper: ServiceClientHelper):
        self.node = node
        self.service_helper = service_helper
        self.client_wag_h = self.service_helper.create_client(Trigger, 'tail_wag_h')
        self.client_wag_v = self.service_helper.create_client(Trigger, 'tail_wag_v')
        self.client_center = self.service_helper.create_client(Trigger, 'tail_center')
        self.client_up = self.service_helper.create_client(Trigger, 'tail_up')
        self.client_down = self.service_helper.create_client(Trigger, 'tail_down')

    def wag_h(self):
        """Wag the tail horizontally."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_wag_h, request)

    def wag_v(self):
        """Wag the tail vertically."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_wag_v, request)

    def center(self):
        """Center the tail."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_center, request)

    def up(self):
        """Raise the tail."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_up, request)

    def down(self):
        """Lower the tail."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_down, request)


class Ears:
    def __init__(self, node: Node, service_helper: ServiceClientHelper):
        self.node = node
        self.service_helper = service_helper
        self.client_stop = self.service_helper.create_client(Trigger, 'ears_stop')
        self.client_scan = self.service_helper.create_client(Trigger, 'ears_scan')
        self.client_fast = self.service_helper.create_client(Trigger, 'ears_fast')
        self.client_think = self.service_helper.create_client(Trigger, 'ears_think')
        self.client_follow_read = self.service_helper.create_client(Trigger, 'ears_follow_read')
        self.client_safe_rotate = self.service_helper.create_client(Trigger, 'ears_safe_rotate')

    def stop(self):
        """Stop the ears from following."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_stop, request)

    def scan(self):
        """Start the ears scanning."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_scan, request)

    def fast(self):
        """Set ears to fast mode."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_fast, request)

    def think(self):
        """Set ears to think mode."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_think, request)

    def follow_read(self) -> float:
        """Read the distance from the ears' sensors."""
        request = Trigger.Request()
        response = self.service_helper.call_service(self.client_follow_read, request)
        return float(response.message.split(":")[1].strip()) if response else 0.0

    def safe_rotate(self) -> bool:
        """Perform safe rotation check."""
        request = Trigger.Request()
        response = self.service_helper.call_service(self.client_safe_rotate, request)
        return response.success if response else False


class BackLights:
    def __init__(self, node: Node, service_helper: ServiceClientHelper):
        self.node = node
        self.service_helper = service_helper
        self.client_on = self.service_helper.create_client(Trigger, 'back_lights_on')
        self.client_off = self.service_helper.create_client(Trigger, 'back_lights_off')
        self.client_turn_on = self.service_helper.create_client(LightsControl, 'back_lights_turn_on')
        self.client_turn_off = self.service_helper.create_client(LightsControl, 'back_lights_turn_off')
        self.client_toggle = self.service_helper.create_client(LightsControl, 'back_lights_toggle')
        self.client_get_switch_state = self.service_helper.create_client(SwitchState, 'back_lights_get_switch_state')

    def on(self):
        """Turn the back lights on."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_on, request)

    def off(self):
        """Turn the back lights off."""
        request = Trigger.Request()
        self.service_helper.call_service(self.client_off, request)

    def turn_on(self, lights: list):
        """Turn specific lights on."""
        request = LightsControl.Request()
        request.lights = lights
        self.service_helper.call_service(self.client_turn_on, request)

    def turn_off(self, lights: list):
        """Turn specific lights off."""
        request = LightsControl.Request()
        request.lights = lights
        self.service_helper.call_service(self.client_turn_off, request)

    def toggle(self, lights: list):
        """Toggle specific lights."""
        request = LightsControl.Request()
        request.lights = lights
        self.service_helper.call_service(self.client_toggle, request)

    def get_switch_state(self) -> list:
        """Get the switch state of the back lights."""
        request = SwitchState.Request()
        response = self.service_helper.call_service(self.client_get_switch_state, request)
        return response.states if response else []


def main(args=None):
    rclpy.init(args=args)
    node = Node('device_client')
    service_helper = ServiceClientHelper(node)

    # Create the Eyes, Tail, Ears, and BackLights client instances
    eyes = Eyes(node, service_helper)
    tail = Tail(node, service_helper)
    ears = Ears(node, service_helper)
    back_lights = BackLights(node, service_helper)
    voice = Voice(node, service_helper)

    # Example usage of the devices
    eyes.on()  # Turn eyes on
    eyes.set_level(0.5)  # Set eyes brightness to 50%
    tail.wag_h()  # Wag the tail horizontally
    ears.scan()  # Start ears scanning
    back_lights.turn_on(['left', 'right'])  # Turn on specific back lights
    back_lights.turn_off(['right'])  # Turn off the right light
    back_lights.toggle(['left', 'right'])  # Toggle the left and right lights

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
