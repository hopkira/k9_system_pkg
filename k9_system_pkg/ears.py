#!/usr/bin/env python3
"""K9 ears: shared services, Gazebo movement, and real serial sensor data.

Real backend polls Espruino for fresh pairs of readings with bounded backlog.
Publishes /l_ear/scan, /r_ear/scan, /l_ear/points, /r_ear/points
and ear-only /joint_states.
Do not run alongside another publisher owning the real ear joint positions.
"""

import json
import math
import struct
import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
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
    """Espruino on-demand paired measurements and ROS sensor publications."""

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
            "poll_period": 0.25,  # 4 complete left/right cycles per second
            "poll_timeout": 0.5,  # Allow a slow request to expire without a backlog
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
        self.point_publishers = {
            ear: node.create_publisher(PointCloud2, f"/{ear}/points", qos_profile_sensor_data)
            for ear in self.frames
        }
        self.joint_publisher = node.create_publisher(JointState, "/joint_states", 10)
        self._rx_buffer = bytearray()
        self._last_scan_ns = {}
        # Latest real potentiometer measurements, not guessed servo commands.
        self._last_positions = {}
        self._last_position_ns = {}
        self._position_timeout_ns = 500_000_000
        self._joint_timer = node.create_timer(0.1, self._publish_cached_positions)
        self.following = False
        self.poll_period = float(param("poll_period"))
        self.poll_timeout = float(param("poll_timeout"))
        if self.poll_period < 0.1 or self.poll_timeout <= self.poll_period:
            raise ValueError("Require poll_period >= 0.1 and poll_timeout > poll_period")
        self._seq = 0
        self._pending_seq = None
        self._pending_since_ns = None
        self._last_poll_ns = 0
        self._cycle_received = set()
        self._last_warning_ns = 0
        self._completed = 0
        self._timeouts = 0
        self._invalid = {"l_ear": 0, "r_ear": 0}
        self._ranges = {"l_ear": 0, "r_ear": 0}
        self._last_stats_ns = time.monotonic_ns()
        self._stats_timer = node.create_timer(10.0, self._log_stats)
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
        self._poll_timer = node.create_timer(0.05, self._poll_if_ready)

    def _write(self, command):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("Ears serial port is unavailable; check connection and restart node")
        payload = (command + "\n").encode("ascii")
        written = self.ser.write(payload)
        if written != len(payload):
            raise RuntimeError("Incomplete ears serial write")
        self.node.get_logger().debug(f"Ears TX: {command}")

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

    def _poll_if_ready(self):
        if self.ser is None or not self.ser.is_open:
            return
        now_ns = time.monotonic_ns()
        if self._pending_seq is not None:
            if now_ns - self._pending_since_ns < int(self.poll_timeout * 1e9):
                return  # Only ONE in-flight request at a time.
            # Drop the stale cycle, rather than accumulate new requests.
            if now_ns - self._last_warning_ns > 5_000_000_000:
                self.node.get_logger().warning(
                    f"Ear poll {self._pending_seq} timed out; discarding late data")
                self._last_warning_ns = now_ns
            self._timeouts += 1
            self._pending_seq = None
        if now_ns - self._last_poll_ns < int(self.poll_period * 1e9):
            return
        self._seq = (self._seq + 1) % 1000000
        payload = f"read:{self._seq}\n".encode("ascii")
        try:
            sent = self.ser.write(payload)
            if sent != len(payload):
                raise serial.SerialTimeoutException("Partial ear poll write")
        except (serial.SerialException, OSError) as exc:
            self.node.get_logger().warning(f"Ear poll write failed: {exc}")
            self._last_poll_ns = now_ns
            return
        self._pending_seq = self._seq
        self._pending_since_ns = now_ns
        self._last_poll_ns = now_ns
        self._cycle_received.clear()

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

    def _publish_cached_positions(self):
        """Continue ear TF updates only while genuine position feedback is fresh."""
        now = self.node.get_clock().now()
        now_mono_ns = time.monotonic_ns()
        names, positions = [], []
        for ear in ("l_ear", "r_ear"):
            seen = self._last_position_ns.get(ear)
            if seen is not None and 0 <= now_mono_ns - seen <= self._position_timeout_ns:
                names.append(self.joints[ear])
                positions.append(self._last_positions[ear])
        if not names:
            return
        joint = JointState()
        joint.header.stamp = now.to_msg()
        joint.name = names
        joint.position = positions
        self.joint_publisher.publish(joint)

    def _publish_reading(self, line):
        text = line.decode("utf-8", errors="replace").strip()
        start = text.find("{")
        if start < 0:
            return
        try:
            reading = json.loads(text[start:])
            if not isinstance(reading, dict):
                return
            kind = reading.get("type")
            # A cycle ends only on its matching DONE. Ignore stray/old replies.
            if kind == "DONE":
                if self._pending_seq is not None and reading.get("seq") == self._pending_seq:
                    self._completed += 1
                    self._pending_seq = None
                return
            if kind not in ("LIDAR", "POSITION"):
                return
            if self._pending_seq is None or reading.get("seq") != self._pending_seq:
                return
            ear = reading.get("sensor")
            if ear not in self.frames or ear in self._cycle_received:
                return
            angle = float(reading["angle"])
            if not math.isfinite(angle):
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
        self._cycle_received.add(ear)
        self._last_positions[ear] = position
        self._last_position_ns[ear] = time.monotonic_ns()

        # Immediately update TF for this actual reading (especially while sweeping).
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = [self.joints[ear]]
        joint.position = [position]
        self.joint_publisher.publish(joint)

        if reading["type"] != "LIDAR":
            self._invalid[ear] += 1
            return

        try:
            distance = float(reading["distance"])
        except (ValueError, TypeError, KeyError):
            return
        if not math.isfinite(distance) or not self.range_min <= distance <= self.range_max:
            self._invalid[ear] += 1
            return
        self._ranges[ear] += 1

        scan = LaserScan()
        scan.header.stamp = stamp  # Host receipt time; firmware has no timestamp.
        scan.header.frame_id = self.frames[ear]
        # One ToF measurement along the ear sensor frame's local +X direction.
        # Retain the three-ray RViz compatibility workaround until the
        # LaserScan consumers can be migrated to the single-point clouds.
        # Do not interpret the duplicates as independent measurements.
        scan.angle_min = -0.001
        scan.angle_max = 0.001
        scan.angle_increment = 0.001
        scan.time_increment = 0.0
        previous = self._last_scan_ns.get(ear)
        if previous is not None and now.nanoseconds > previous:
            scan.scan_time = (now.nanoseconds - previous) / 1e9
        self._last_scan_ns[ear] = now.nanoseconds
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = [distance, distance, distance]
        self.scan_publishers[ear].publish(scan)

        # One actual VL53L0X return: a single XYZ point in the moving
        # sensor's local frame. TF rotates and translates it into base_link.
        cloud = PointCloud2()
        cloud.header.stamp = stamp
        cloud.header.frame_id = self.frames[ear]
        cloud.height = 1
        cloud.width = 1
        cloud.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12
        cloud.is_dense = True
        cloud.data = struct.pack("<fff", distance, 0.0, 0.0)
        self.point_publishers[ear].publish(cloud)

    def _log_stats(self):
        self.node.get_logger().info(
            "Ear polls: completed=%d timed_out=%d valid_ranges(L/R)=%d/%d "
            "position_only_or_invalid(L/R)=%d/%d pending=%s"
            % (self._completed, self._timeouts,
               self._ranges["l_ear"], self._ranges["r_ear"],
               self._invalid["l_ear"], self._invalid["r_ear"],
               str(self._pending_seq))
        )

    def close(self):
        self._stats_timer.cancel()
        self._poll_timer.cancel()
        self._joint_timer.cancel()
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