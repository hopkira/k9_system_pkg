#!/usr/bin/env python3

# ROS 2 Behavior Tree

#import serial
#import ast
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Bool
from k9_interfaces_pkg.srv import LightsControl, SwitchState  # Custom interfaces
from k9_interfaces_pkg.srv import Speak, CancelSpeech  # Updated to k9_voice package
from k9_interfaces_pkg.srv import SetBrightness, GetBrightness

import py_trees
import py_trees_ros
from py_trees_ros.trees import BehaviourTree
from py_trees.blackboard import Blackboard

# Initialize components
#
#k9stt = Listen()
#k9qa = Respond()
#k9history = Backhistory()

blackboard = Blackboard()
blackboard.command = None
blackboard.intent = None
blackboard.speaking = 0.0
blackboard.hotword_detected = False

class NotListening(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="NotListening"):
        super().__init__(name)
        self.node = node

    def initialise(self):
        self.node.eyes.set_level(0.0)
        self.node.back_lights.off()
        self.node.back_lights.cmd('computer')
        self.node.back_lights.turn_on([1, 3, 7, 10, 12])
        self.node.tail.center()
        self.node.ears.stop()

    def update(self):
        return py_trees.common.Status.SUCCESS


class WaitForHotword(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="WaitForHotword"):
        super().__init__(name)
        self.node = node

    def initialise(self):
        if self.node.is_talking:
            self.logger.info("Still speaking, waiting...")
        #if mem.retrieveState("speaking") == 1.0:
        #    self.logger.info("Still speaking, waiting...")
        self.node.back_lights.turn_on([1,3,6,8,9,12])
        self.node.tail.center()
        self.node.eyes.set_level(0.001)
        blackboard.hotword_detected = True

    def update(self):
        if blackboard.hotword_detected:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.RUNNING


class Listening(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="Listening"):
        super().__init__(name)
        self.node = node

    def initialise(self):
        if self.node.is_talking:
            self.logger.info("Still speaking, waiting...")
        self.node.back_lights.cmd('computer')
        self.node.back_lights.off()
        self.node.back_lights.turn_on([1,2,5,9,12])
        self.node.eyes.set_level(0.01)
        blackboard.command = k9stt.listen_for_command()
        self.node.eyes.set_level(0.0)

    def update(self):
        if blackboard.command:
            return py_trees.common.Status.SUCCESS
        return py_trees.common.Status.FAILURE


class Responding(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="Responding"):
        super().__init__(name)
        self.node = node

    def initialise(self):
        command = blackboard.command
        self.node.back_lights.on()
        self.node.eyes.set_level(0.5)
        self.node.ears.think()
        
        if 'thank' in command:
            blackboard.intent = 'PraiseMe'
        elif 'play chess' in command:
            blackboard.intent = 'PlayChess'
        elif 'demo' in command:
            blackboard.intent = 'ShowOff'
        else:
            intent, answer = k9qa.robot_response(command)
            blackboard.intent = intent

        self.node.ears.stop()
        self.node.back_lights.off()

        self.node.publisher.publish(Bool(data=True))  # Signal that speaking has started
        self.node.voice.speak(command)

        # Wait until speaking is finished
        while self.node.is_talking:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.eyes.set_level(0.1)
        time.sleep(0.75)

    def update(self):
        return py_trees.common.Status.SUCCESS


class Demonstration(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="Demonstration"):
        super().__init__(name)
        self.node = node

    def initialise(self):
        self.node.voice.speak("Starting demonstration")
        self.node.publisher.publish(Bool(data=True))  # Signal talking started
        while self.node.is_talking:
            rclpy.spin_once(self.node, timeout_sec=0.1)

    def update(self):
        return py_trees.common.Status.SUCCESS


class PlayChess(py_trees.behaviour.Behaviour):
    def __init__(self, node, name="PlayChess"):
        super().__init__(name)
        self.node = node

    def initialise(self):
        self.game = ChessGame()
        blackboard.intent = None

    def update(self):
        return py_trees.common.Status.SUCCESS


def create_behavior_tree():
    root = py_trees.composites.Selector("K9_Audio")

    demonstration = Demonstration()
    play_chess = PlayChess()
    respond = Responding()
    listen = Listening()
    wait = WaitForHotword()
    idle = NotListening()

    root.add_children([
        demonstration,
        play_chess,
        respond,
        listen,
        wait,
        idle
    ])

    return root


class K9BTNode(Node):
    def __init__(self):
        super().__init__('k9_bt_node')

        self.service_helper = ServiceClientHelper(self)

        # Create the Eyes, Tail, Ears, and BackLights client instances
        self.eyes = Eyes(self, self.service_helper)
        self.tail = Tail(self, self.service_helper)
        self.ears = Ears(self, self.service_helper)
        self.back_lights = BackLights(self, self.service_helper)
        self.voice = Voice(self, self.service_helper)

        self.is_talking = False

        # Publisher for is_talking status
        #self.publisher = self.create_publisher(Bool, 'is_talking', 10)

        # Subscribe to the 'is_talking' topic
        self.subscription = self.create_subscription(
            Bool,
            'is_talking',
            self.is_talking_callback,
            10
        )
        
        tree = create_behavior_tree()
        self.bt = BehaviourTree(tree)
        self.bt.setup(timeout=15)
    
    def is_talking_callback(self, msg):
        self.is_talking = msg.data

    def tick(self):
        self.bt.tick_once()


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
        """Toggle specific back_lights."""
        request = LightsControl.Request()
        request.lights = lights
        self.service_helper.call_service(self.client_toggle, request)

    def get_switch_state(self) -> list:
        """Get the switch state of the back back_lights."""
        request = SwitchState.Request()
        response = self.service_helper.call_service(self.client_get_switch_state, request)
        return response.states if response else []


def main(args=None):
    rclpy.init(args=args)
    node = K9BTNode()
    try:
        while rclpy.ok():
            node.tick()
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()