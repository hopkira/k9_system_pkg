#!/usr/bin/env python3
"""K9 ears: shared services, Gazebo movement, and real serial sensor data.

Real backend publishes /l_ear/scan, /r_ear/scan and ear-only /joint_states.
Do not run alongside another publisher owning the real ear joint positions.
"""

import json
import math

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger


class SimEars:
    """Movement only; Gazebo and its bridge publish simulated sensor data."""

    def __init__(self, node, controller_name="ears_position_controller"):
        self.node = node
        self.cmd_topic = f"/{controller_name}/commands"
        self.cmd_pub = node.create_publisher(Float64MultiArray, self.cmd_topic, 10)
        self.scanning = False
        self.speed = 1.0
        self.phase = 0.0
        self.l_min, self.l_max = -0.2, 0.6
        self.r_min, self.r_max = -0.6, 0.2
        self.l_center, self.l_amp = 0.2, 0.4
        self.r_center, self.r_amp = -0.2, 0.4
        self.dt = 0.05
        self.timer = node.create_timer(self.dt, self._update)
        node.get_logger().info(f"SimEars publishing to {self.cmd_topic} [left,right]")

    def _publish(self, left, right):
        msg = Float64MultiArray()
        msg.data = [float(left), float(right)]
        self.cmd_pub.publish(msg)

    def _update(self):
        if not self.scanning:
            return
        self.phase += self.dt * self.speed
        s = math.sin(self.phase)
        left = max(self.l_min, min(self.l_max, self.l_center + self.l_amp * s))
        right = max(self.r_min, min(self.r_max, self.r_center - self.r_amp * s))
        self._publish(left, right)

    def stop(self):
        self.scanning = False
        self.speed = 0.0
        self._publish(self.l_center, self.r_center)

    def scan(self):
        self.scanning, self.speed = True, 1.0

    def fast(self):
        self.scanning, self.speed = True, 2.0

    def think(self):
        self.scanning, self.speed = True, 0.4

    def close(self):
        self.timer.cancel()


class Ears:
    """Espruino USB commands and newline-delimited LIDAR JSON."""

    def __init__(self, node):
        self.node = node
        defaults = {
            "serial_port": "/dev/ears",
            "serial_baud": 115200,
            "left_sensor_frame": "l_ear_sensor_link",
            "right_sensor_frame": "r_ear_sensor_link",
            "range_min": 0.02,
            "range_max": 2.0,
            "left_angle_scale": 1.0,
            "left_angle_offset": 0.0,
            "right_angle_scale": 1.0,
            "right_angle_offset": 0.0,
        }
        for name, value in defaults.items():
            node.declare_parameter(name, value)
        param = lambda name: node.get_parameter(name).value
        self.range_min = float(param("range_min"))
        self.range_max = float(param("range_max"))
        if not 0.0 <= self.range_min < self.range_max:
            raise ValueError("Require 0 <= range_min < range_max")
        self.frames = {
            "l_ear": str(param("left_sensor_frame")),
            "r_ear": str(param("right_sensor_frame")),
        }
        self.joints = {"l_ear": "l_ear_joint", "r_ear": "r_ear_joint"}
        self.calibration = {
            "l_ear": (float(param("left_angle_scale")), float(param("left_angle_offset"))),
            "r_ear": (float(param("right_angle_scale")), float(param("right_angle_offset"))),
        }
        self.scan_publishers = {
            ear: node.create_publisher(LaserScan, f"/{ear}/scan", qos_profile_sensor_data)
            for ear in self.frames
        }
        self.joint_publisher = node.create_publisher(JointState, "/joint_states", 10)
        self._rx_buffer = bytearray()
        self._last_scan_ns = {}
        self.following = False
        self.ser = None
        try:
            self.ser = serial.Serial(
                port=str(param("serial_port")), baudrate=int(param("serial_baud")),
                timeout=0, write_timeout=1,
            )
            node.get_logger().info(f"Opened ears serial port {self.ser.port}")
        except (serial.SerialException, OSError) as exc:
            node.get_logger().error(f"Failed to connect to ears: {exc}")
        self._serial_read_timer = node.create_timer(0.02, self._read_serial)

    def _write(self, command):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Ears serial port is unavailable; check connection and restart node")
        payload = (command + "();\n").encode("ascii")
        written = self.ser.write(payload)
        if written != len(payload):
            raise RuntimeError("Incomplete ears serial write")
        self.node.get_logger().debug(f"Ears TX: {command}()")

    def stop(self):
        self.following = False
        self._write("stop")

    def scan(self):
        self.following = False
        self._write("scan")

    def fast(self):
        self.following = False
        self._write("fast")

    def think(self):
        self.following = False
        self._write("think")

    def _read_serial(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            waiting = self.ser.in_waiting
            if not waiting:
                return
            self._rx_buffer.extend(self.ser.read(min(waiting, 4096)))
        except (serial.SerialException, OSError) as exc:
            self.node.get_logger().error(f"Ears serial read failed: {exc}")
            self._serial_read_timer.cancel()
            self.ser.close()
            return
        while b"\n" in self._rx_buffer:
            line, _, remaining = self._rx_buffer.partition(b"\n")
            self._rx_buffer = bytearray(remaining)
            self._publish_reading(line)
        if len(self._rx_buffer) > 16384:
            self._rx_buffer.clear()
            self.node.get_logger().warning("Cleared oversized ears serial receive buffer")

    def _publish_reading(self, line):
        text = line.decode("utf-8", errors="replace").strip()
        start = text.find("{")
        if start < 0:
            return  # Console prompts, echoes and startup text.
        try:
            reading = json.loads(text[start:])
            if not isinstance(reading, dict) or reading.get("type") != "LIDAR":
                return
            ear = reading.get("sensor")
            if ear not in self.frames:
                return
            distance = float(reading["distance"])
            angle = float(reading["angle"])
            if not math.isfinite(distance) or not math.isfinite(angle) or distance <= 0:
                return
        except (ValueError, TypeError, KeyError):
            self.node.get_logger().debug(f"Ignored invalid ears serial line: {text!r}")
            return

        scale, offset = self.calibration[ear]
        position = scale * angle + offset
        if not math.isfinite(position):
            return
        now = self.node.get_clock().now()
        stamp = now.to_msg()
        # Publish only this ear: do not stamp the other ear's old position as new.
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = [self.joints[ear]]
        joint.position = [position]
        self.joint_publisher.publish(joint)

        # Invalid/out-of-spec ranges are omitted rather than invented as clear space.
        if not self.range_min <= distance <= self.range_max:
            return
        scan = LaserScan()
        scan.header.stamp = stamp  # Host receipt time; firmware has no timestamp.
        scan.header.frame_id = self.frames[ear]
        # The sensor frame rotates via JointState/TF; ray is along its local +X.
        scan.angle_min = 0.0
        scan.angle_max = 0.0
        scan.angle_increment = 0.0
        scan.time_increment = 0.0
        previous = self._last_scan_ns.get(ear)
        if previous is not None and now.nanoseconds > previous:
            scan.scan_time = (now.nanoseconds - previous) / 1e9
        self._last_scan_ns[ear] = now.nanoseconds
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [distance]
        self.scan_publishers[ear].publish(scan)

    def close(self):
        self._serial_read_timer.cancel()
        if self.ser is not None and self.ser.is_open:
            try:
                self.stop()
            except (serial.SerialException, OSError, RuntimeError):
                pass
            finally:
                self.ser.close()


class EarsServiceNode(Node):
    def __init__(self):
        super().__init__("ears_service_node")
        self.declare_parameter("sim", False)
        sim = bool(self.get_parameter("sim").value)
        self.get_logger().info("Using SimEars backend" if sim else "Using real Ears backend")
        self.ears = SimEars(self) if sim else Ears(self)
        self._interaction_activity = "IDLE"
        self._activity_subscription = self.create_subscription(
            String, "/interaction/activity", self._activity_callback, 10
        )
        self.create_service(Trigger, "ears_stop", self.stop_cb)
        self.create_service(Trigger, "ears_scan", self.scan_cb)
        self.create_service(Trigger, "ears_fast", self.fast_cb)
        self.create_service(Trigger, "ears_think", self.think_cb)
        self.get_logger().info("Ears Node is running.")

    def _activity_callback(self, msg):
        activity = msg.data.strip().upper()
        if activity == self._interaction_activity:
            return
        previous = self._interaction_activity
        self._interaction_activity = activity
        try:
            if activity == "PROCESSING":
                self.ears.think()
            elif activity == "SPEAKING" or (activity == "IDLE" and previous == "PROCESSING"):
                self.ears.stop()
        except (serial.SerialException, OSError, RuntimeError) as exc:
            self.get_logger().error(f"Ears activity command failed: {exc}")

    def _command_cb(self, command, response):
        try:
            getattr(self.ears, command)()
            response.success = True
            response.message = f"Sent {command} command."
            self.get_logger().info(response.message)
        except (serial.SerialException, OSError, RuntimeError) as exc:
            response.success = False
            response.message = str(exc)
            self.get_logger().error(f"Ears command failed: {exc}")
        return response

    def stop_cb(self, request, response):
        return self._command_cb("stop", response)

    def scan_cb(self, request, response):
        return self._command_cb("scan", response)

    def fast_cb(self, request, response):
        return self._command_cb("fast", response)

    def think_cb(self, request, response):
        return self._command_cb("think", response)

    def destroy_node(self):
        self.ears.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EarsServiceNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
