#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Optional

# ROS 2
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry


# -------------------------
# Shared math helpers
# -------------------------
def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def dist_2d(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(x1 - x0, y1 - y0)


def deg(rad: float) -> float:
    return rad * 180.0 / math.pi


# -------------------------
# Robot constants (from your 2021 script)
# -------------------------
CLICK2METRES = 0.002179
WALKINGSPEED = 1.4
TOPSPEED = int(WALKINGSPEED / CLICK2METRES)
ACCELERATION = int(TOPSPEED / 5)

HALF_WHEEL_GAP = 0.1011
WHEEL_SEPARATION_M = 2.0 * HALF_WHEEL_GAP

TURNING_CIRCLE = 2 * math.pi * HALF_WHEEL_GAP / CLICK2METRES

M1_QPPS = 1987
M2_QPPS = 1837
M1_P = 10.644
M2_P = 9.768
M1_I = 2.206
M2_I = 2.294
M1_D = 0.0
M2_D = 0.0

ROBOCLAW_ADDR = 0x80
ROBOCLAW_PORT = "/dev/roboclaw"
ROBOCLAW_BAUD = 115200


# -------------------------
# Data models
# -------------------------
@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass
class MoveResult:
    expected_linear_m: float
    expected_angular_rad: float

    # "belief" measurement (odom in sim, encoders in real)
    belief_linear_m: float
    belief_angular_rad: float

    # Gazebo ground truth (sim only)
    gz_linear_m: Optional[float]
    gz_angular_rad: Optional[float]

    duration_s: float
    mode: str  # "sim" or "real"


# -------------------------
# RoboClaw driver wrapper (real mode)
# -------------------------
class RoboClawDriver:
    def __init__(self):
        # Import only when needed, so sim mode works on machines without it.
        sys.path.append('/home/pi/k9-chess-angular/python')
        from roboclaw_3 import Roboclaw  # type: ignore

        self.rc = Roboclaw(ROBOCLAW_PORT, ROBOCLAW_BAUD)
        self.rc.Open()

        version = self.rc.ReadVersion(ROBOCLAW_ADDR)
        if not version[0]:
            raise RuntimeError("RoboClaw ReadVersion failed (is it connected / permissions?)")

        # Configure PID etc.
        self.rc.SetM1VelocityPID(ROBOCLAW_ADDR, M1_P, M1_I, M1_D, M1_QPPS)
        self.rc.SetM2VelocityPID(ROBOCLAW_ADDR, M2_P, M2_I, M2_D, M2_QPPS)
        self.rc.SetMainVoltages(ROBOCLAW_ADDR, 240, 292)
        self.rc.SetPinFunctions(ROBOCLAW_ADDR, 2, 0, 0)
        self.rc.ResetEncoders(ROBOCLAW_ADDR)

    def stop(self):
        self.rc.SpeedM1M2(ROBOCLAW_ADDR, 0, 0)

    def read_encoders(self):
        # returns (m1_count, m2_count)
        e1 = self.rc.ReadEncM1(ROBOCLAW_ADDR)
        e2 = self.rc.ReadEncM2(ROBOCLAW_ADDR)
        if not e1[0] or not e2[0]:
            raise RuntimeError("Failed to read encoders from RoboClaw")
        return int(e1[1]), int(e2[1])

    def read_speeds(self):
        s1 = self.rc.ReadSpeedM1(ROBOCLAW_ADDR)
        s2 = self.rc.ReadSpeedM2(ROBOCLAW_ADDR)
        if not s1[0] or not s2[0]:
            raise RuntimeError("Failed to read speeds from RoboClaw")
        return int(s1[1]), int(s2[1])

    def buffer_full(self) -> bool:
        buffers = self.rc.ReadBuffers(ROBOCLAW_ADDR)
        if not buffers[0]:
            raise RuntimeError("Failed to read buffers from RoboClaw")
        return ((buffers[1] != 0x80) or (buffers[2] != 0x80))

    def motors_moving(self) -> bool:
        s1, s2 = self.read_speeds()
        return (s1 != 0) or (s2 != 0)

    def finished_move(self) -> bool:
        # buffer empty AND motors at rest
        return not (self.motors_moving() or self.buffer_full())

    # --- original motion math ---
    def calc_turn_modifier(self, radius: float) -> float:
        radius = abs(radius)
        return 1.0 - (0.9 / (radius + 1.0))

    def calc_click_vel(self, clicks: float, turn_mod: float) -> float:
        sign_modifier = -1.0 if clicks < 0.0 else 1.0
        click_vel = math.sqrt(abs(float(2.0 * clicks * ACCELERATION * turn_mod)))
        if click_vel > TOPSPEED * turn_mod:
            click_vel = TOPSPEED * turn_mod
        if click_vel < 1.0:
            click_vel = 1.0
        return click_vel * sign_modifier

    def calc_accel(self, velocity: float, distance_clicks: float) -> int:
        # constant acceleration to achieve velocity over distance_clicks
        # guard against divide-by-zero
        d = max(abs(distance_clicks), 1.0)
        return int(abs((velocity ** 2.0) / (2.0 * d)))

    # --- motion primitives ---
    def cmd_forward_m(self, distance_m: float):
        clicks = int(round(distance_m / CLICK2METRES))
        click_vel = self.calc_click_vel(clicks=clicks, turn_mod=1.0)
        accel = self.calc_accel(click_vel, clicks / 2.0)

        self.rc.SpeedAccelDistanceM1M2(
            address=ROBOCLAW_ADDR,
            accel=accel,
            speed1=int(round(click_vel)),
            distance1=int(abs(clicks / 2.0)),
            speed2=int(round(click_vel)),
            distance2=int(abs(clicks / 2.0)),
            buffer=1
        )
        self.rc.SpeedAccelDistanceM1M2(
            address=ROBOCLAW_ADDR,
            accel=accel,
            speed1=0,
            distance1=int(abs(clicks / 2.0)),
            speed2=0,
            distance2=int(abs(clicks / 2.0)),
            buffer=0
        )

    def cmd_turn_rad(self, angle_rad: float, fast: bool = False):
        fraction = angle_rad / (2.0 * math.pi)
        clicks = TURNING_CIRCLE * fraction

        turn_modifier = 1.0 if fast else self.calc_turn_modifier(radius=0.0)
        click_vel = self.calc_click_vel(clicks=clicks, turn_mod=turn_modifier)
        accel = ACCELERATION if fast else int(abs(click_vel * click_vel / (2.0 * (clicks / 2.0 if clicks != 0 else 1.0))))

        # Left turn: left wheel backward, right wheel forward
        self.rc.SpeedAccelDistanceM1M2(
            address=ROBOCLAW_ADDR,
            accel=accel,
            speed1=int(round(-click_vel)),
            distance1=abs(int(round(clicks / 2.0))),
            speed2=int(round(click_vel)),
            distance2=abs(int(round(clicks / 2.0))),
            buffer=1
        )
        self.rc.SpeedAccelDistanceM1M2(
            address=ROBOCLAW_ADDR,
            accel=accel,
            speed1=0,
            distance1=abs(int(round(clicks / 2.0))),
            speed2=0,
            distance2=abs(int(round(clicks / 2.0))),
            buffer=0
        )

    def cmd_arc(self, radius_m: float, extent_rad: float):
        if radius_m == 0.0:
            raise ValueError("arc requires non-zero radius")

        if extent_rad > 0.0:
            d1 = int(abs(extent_rad * (radius_m + HALF_WHEEL_GAP) / CLICK2METRES))
            d2 = int(abs(extent_rad * (radius_m - HALF_WHEEL_GAP) / CLICK2METRES))
        else:
            d1 = int(abs(extent_rad * (radius_m - HALF_WHEEL_GAP) / CLICK2METRES))
            d2 = int(abs(extent_rad * (radius_m + HALF_WHEEL_GAP) / CLICK2METRES))

        turn_mod = self.calc_turn_modifier(radius_m)
        v1 = self.calc_click_vel(clicks=d1, turn_mod=turn_mod)
        v2 = self.calc_click_vel(clicks=d2, turn_mod=turn_mod)

        a1 = int(abs(v1 * v1 / (2.0 * (d1 / 2.0 if d1 != 0 else 1.0))))
        a2 = int(abs(v2 * v2 / (2.0 * (d2 / 2.0 if d2 != 0 else 1.0))))
        accel = max(a1, a2)

        self.rc.SpeedAccelDistanceM1M2(
            address=ROBOCLAW_ADDR,
            accel=accel,
            speed1=int(round(v1)),
            distance1=int(round(d1 / 2.0)),
            speed2=int(round(v2)),
            distance2=int(round(d2 / 2.0)),
            buffer=1
        )
        self.rc.SpeedAccelDistanceM1M2(
            address=ROBOCLAW_ADDR,
            accel=accel,
            speed1=0,
            distance1=int(round(d1 / 2.0)),
            speed2=0,
            distance2=int(round(d2 / 2.0)),
            buffer=0
        )


# -------------------------
# Main test node
# -------------------------
class K9MoveTest(Node):
    """
    Unified CLI movement test tool:

    --sim true  (default): drive Gazebo via /cmd_vel, stop-on-/odom, measure Gazebo ground truth
    --sim false          : drive RoboClaw, wait-for-finished_move, measure encoder deltas
    """

    def __init__(self, args):
        super().__init__("k9_move_test")

        self.sim = args.sim
        self.mode = "sim" if self.sim else "real"

        # Sim/ROS interfaces
        self.cmd_vel_topic = args.cmd_vel_topic
        self.odom_topic = args.odom_topic

        self.gt_mode = args.gt_mode
        self.gt_ros_topic = args.gt_ros_topic
        self.gz_pose_topic = args.gz_pose_topic

        self.pub_rate_hz = args.pub_rate
        self.linear_speed = abs(args.linear_speed)
        self.angular_speed = abs(args.angular_speed)
        self.stop_hold_s = args.stop_hold

        self._cmd_pub = None
        self._odom_sub = None
        self._gt_pose_sub = None

        self._latest_odom_pose: Optional[Pose2D] = None
        self._have_odom = False
        self._latest_gt_pose: Optional[Pose2D] = None

        self.roboclaw: Optional[RoboClawDriver] = None

        if self.sim:
            self._cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
            self._odom_sub = self.create_subscription(Odometry, self.odom_topic, self._odom_cb, 10)
            if self.gt_mode == "ros":
                self._gt_pose_sub = self.create_subscription(PoseStamped, self.gt_ros_topic, self._gt_pose_cb, 10)
            self.get_logger().info("Mode=SIM (Gazebo): using /cmd_vel, /odom, and Gazebo ground truth")
        else:
            self.get_logger().info("Mode=REAL (RoboClaw): initialising motor controller…")
            self.roboclaw = RoboClawDriver()
            self.get_logger().info("RoboClaw initialised OK")

    # ---------- SIM callbacks ----------
    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._latest_odom_pose = Pose2D(p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))
        self._have_odom = True

    def _gt_pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        self._latest_gt_pose = Pose2D(p.x, p.y, yaw_from_quat(q.x, q.y, q.z, q.w))

    # ---------- SIM ground truth ----------
    def get_gt_pose_sim(self) -> Pose2D:
        if self.gt_mode == "ros":
            t0 = self.get_clock().now()
            while rclpy.ok() and self._latest_gt_pose is None:
                rclpy.spin_once(self, timeout_sec=0.05)
                if (self.get_clock().now() - t0) > Duration(seconds=2.0):
                    raise RuntimeError(
                        f"No PoseStamped on {self.gt_ros_topic}. "
                        "Bridge a ground-truth pose or use --gt-mode gzcli."
                    )
            return self._latest_gt_pose

        # gzcli fallback
        out = subprocess.check_output(
            ["gz", "topic", "-e", "-n", "1", self.gz_pose_topic],
            stderr=subprocess.STDOUT,
            text=True,
            timeout=2.0,
        )

        # Minimal parse (works for typical Pose output)
        x = y = z = None
        qx = qy = qz = qw = None

        for line in out.splitlines():
            s = line.strip()
            if s.startswith("x:") and x is None:
                try: x = float(s.split(":", 1)[1])
                except: pass
            elif s.startswith("y:") and y is None:
                try: y = float(s.split(":", 1)[1])
                except: pass
            elif s.startswith("z:") and z is None and x is not None and y is not None:
                try: z = float(s.split(":", 1)[1])
                except: pass
            elif s.startswith("x:") and x is not None and y is not None and z is not None and qx is None:
                try: qx = float(s.split(":", 1)[1])
                except: pass
            elif s.startswith("y:") and qx is not None and qy is None:
                try: qy = float(s.split(":", 1)[1])
                except: pass
            elif s.startswith("z:") and qx is not None and qy is not None and qz is None:
                try: qz = float(s.split(":", 1)[1])
                except: pass
            elif s.startswith("w:") and qx is not None and qy is not None and qz is not None and qw is None:
                try: qw = float(s.split(":", 1)[1])
                except: pass

        if None in (x, y, qx, qy, qz, qw):
            raise RuntimeError(f"Could not parse gz pose output from {self.gz_pose_topic}:\n{out}")

        return Pose2D(x, y, yaw_from_quat(qx, qy, qz, qw))

    # ---------- SIM helpers ----------
    def wait_for_odom(self, timeout_s: float = 3.0):
        t0 = self.get_clock().now()
        while rclpy.ok() and not self._have_odom:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (self.get_clock().now() - t0) > Duration(seconds=timeout_s):
                raise RuntimeError(f"No odom received on {self.odom_topic}")

    def publish_twist(self, vx: float, wz: float):
        msg = Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def stop_sim(self):
        self.publish_twist(0.0, 0.0)

    # ---------- REAL measurement from encoders ----------
    def measure_from_encoders(self, e1_start: int, e2_start: int, e1_end: int, e2_end: int) -> tuple[float, float]:
        # RoboClaw encoder signs depend on wiring; use raw deltas
        d1 = (e1_end - e1_start) * CLICK2METRES
        d2 = (e2_end - e2_start) * CLICK2METRES

        linear = (d1 + d2) / 2.0
        angular = (d2 - d1) / WHEEL_SEPARATION_M  # rad (approx, diff-drive kinematics)
        return abs(linear), abs(angular)

    # ---------- Move execution ----------
    def run_move(self, verb: str, param: float, radius: float = 0.0) -> MoveResult:
        # Expected outputs
        expected_linear = 0.0
        expected_angular = 0.0

        if verb == "fd":
            expected_linear = abs(param)
        elif verb == "bk":
            expected_linear = abs(param)
        elif verb == "lt":
            expected_angular = abs(param)
        elif verb == "rt":
            expected_angular = abs(param)
        elif verb == "arc":
            if radius == 0.0:
                raise ValueError("arc requires non-zero radius")
            expected_linear = abs(radius * param)     # arc length
            expected_angular = abs(param)             # extent
        elif verb == "stop":
            # stop immediately
            if self.sim:
                self.stop_sim()
            else:
                self.roboclaw.stop()
            return MoveResult(expected_linear, expected_angular, 0.0, 0.0, None, None, 0.0, self.mode)
        else:
            raise ValueError(f"Unknown command: {verb}")

        t_start = time.time()

        # ---------------- SIM MODE ----------------
        if self.sim:
            self.wait_for_odom()

            odom_start = self._latest_odom_pose
            gz_start = self.get_gt_pose_sim()

            # Choose commanded velocities
            vx = wz = 0.0
            if verb == "fd":
                vx = self.linear_speed if param >= 0 else -self.linear_speed
            elif verb == "bk":
                vx = -self.linear_speed if param >= 0 else self.linear_speed
            elif verb == "lt":
                wz = self.angular_speed if param >= 0 else -self.angular_speed
            elif verb == "rt":
                wz = -self.angular_speed if param >= 0 else self.angular_speed
            elif verb == "arc":
                v = self.linear_speed
                omega = v / abs(radius)
                vx = v
                wz = omega if param >= 0 else -omega

            period = 1.0 / float(self.pub_rate_hz)

            # stop-on-odom
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                odom_now = self._latest_odom_pose
                if odom_now is None:
                    continue

                odom_lin = dist_2d(odom_start.x, odom_start.y, odom_now.x, odom_now.y)
                odom_ang = abs(wrap_to_pi(odom_now.yaw - odom_start.yaw))

                reached = False
                if expected_linear > 0.0:
                    reached = odom_lin >= expected_linear
                elif expected_angular > 0.0:
                    reached = odom_ang >= expected_angular

                if reached:
                    break

                self.publish_twist(vx, wz)
                time.sleep(period)

            # stop + settle
            self.stop_sim()
            time.sleep(self.stop_hold_s)
            rclpy.spin_once(self, timeout_sec=0.05)

            odom_end = self._latest_odom_pose
            gz_end = self.get_gt_pose_sim()
            t_end = time.time()

            belief_linear = dist_2d(odom_start.x, odom_start.y, odom_end.x, odom_end.y)
            belief_angular = abs(wrap_to_pi(odom_end.yaw - odom_start.yaw))

            gz_linear = dist_2d(gz_start.x, gz_start.y, gz_end.x, gz_end.y)
            gz_angular = abs(wrap_to_pi(gz_end.yaw - gz_start.yaw))

            return MoveResult(
                expected_linear_m=expected_linear,
                expected_angular_rad=expected_angular,
                belief_linear_m=belief_linear,
                belief_angular_rad=belief_angular,
                gz_linear_m=gz_linear,
                gz_angular_rad=gz_angular,
                duration_s=(t_end - t_start),
                mode=self.mode
            )

        # ---------------- REAL MODE ----------------
        assert self.roboclaw is not None

        e1_start, e2_start = self.roboclaw.read_encoders()

        # Issue motion command (RoboClaw manages accel / distance)
        if verb == "fd":
            self.roboclaw.cmd_forward_m(param)
        elif verb == "bk":
            self.roboclaw.cmd_forward_m(-param)
        elif verb == "lt":
            self.roboclaw.cmd_turn_rad(param, fast=False)
        elif verb == "rt":
            self.roboclaw.cmd_turn_rad(-param, fast=False)
        elif verb == "arc":
            self.roboclaw.cmd_arc(radius, param)

        # Wait for completion
        while rclpy.ok():
            if self.roboclaw.finished_move():
                break
            time.sleep(0.05)

        # Stop and settle
        self.roboclaw.stop()
        time.sleep(self.stop_hold_s)

        e1_end, e2_end = self.roboclaw.read_encoders()
        t_end = time.time()

        belief_linear, belief_angular = self.measure_from_encoders(e1_start, e2_start, e1_end, e2_end)

        return MoveResult(
            expected_linear_m=expected_linear,
            expected_angular_rad=expected_angular,
            belief_linear_m=belief_linear,
            belief_angular_rad=belief_angular,
            gz_linear_m=None,
            gz_angular_rad=None,
            duration_s=(t_end - t_start),
            mode=self.mode
        )


def print_report(cmd: str, param: float, radius: float, res: MoveResult):
    print("\n=== K9 Move Test Report ===")
    print(f"Mode: {res.mode.upper()}")
    print(f"Command: {cmd}  param={param}  radius={radius}")
    print(f"Duration: {res.duration_s:.2f} s\n")

    print("Expected:")
    print(f"  linear:  {res.expected_linear_m:.4f} m")
    print(f"  angular: {res.expected_angular_rad:.4f} rad  ({deg(res.expected_angular_rad):.2f} deg)\n")

    label = "/odom (robot belief)" if res.mode == "sim" else "encoders (robot belief)"
    print(f"Measured ({label}):")
    print(f"  linear:  {res.belief_linear_m:.4f} m")
    print(f"  angular: {res.belief_angular_rad:.4f} rad  ({deg(res.belief_angular_rad):.2f} deg)\n")

    if res.mode == "sim":
        print("Measured (Gazebo ground truth):")
        print(f"  linear:  {res.gz_linear_m:.4f} m")
        print(f"  angular: {res.gz_angular_rad:.4f} rad  ({deg(res.gz_angular_rad):.2f} deg)\n")
    else:
        print("Measured (Gazebo ground truth):")
        print("  linear:  n/a (real mode)")
        print("  angular: n/a (real mode)\n")

    # Simple scaling hints
    if res.mode == "sim" and res.gz_linear_m and res.gz_linear_m > 1e-6 and res.expected_linear_m > 0.0:
        scale = res.expected_linear_m / res.gz_linear_m
        print(f"Hint (sim linear): expected/actual = {scale:.4f}  (tune wheel_radius)")
    if res.mode == "sim" and res.gz_angular_rad and res.gz_angular_rad > 1e-6 and res.expected_angular_rad > 0.0:
        scale = res.expected_angular_rad / res.gz_angular_rad
        print(f"Hint (sim angular): expected/actual = {scale:.4f}  (tune wheel_separation)")
    if res.mode == "real" and res.belief_linear_m > 1e-6 and res.expected_linear_m > 0.0:
        scale = res.expected_linear_m / res.belief_linear_m
        print(f"Hint (real linear): expected/actual = {scale:.4f}  (tune CLICK2METRES / wheel params)")
    if res.mode == "real" and res.belief_angular_rad > 1e-6 and res.expected_angular_rad > 0.0:
        scale = res.expected_angular_rad / res.belief_angular_rad
        print(f"Hint (real angular): expected/actual = {scale:.4f}  (tune HALF_WHEEL_GAP)")
    print("===========================\n")


def parse_bool(s: str) -> bool:
    s = s.strip().lower()
    if s in ("true", "1", "yes", "y", "on"):
        return True
    if s in ("false", "0", "no", "n", "off"):
        return False
    raise argparse.ArgumentTypeError("Expected true/false")


def parse_args(argv):
    p = argparse.ArgumentParser(
        description="K9 unified move test: Gazebo(sim) or RoboClaw(real). Default is sim=true for safety.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    p.add_argument("command", choices=["arc", "fd", "bk", "lt", "rt", "stop"])
    p.add_argument("parameter", type=float, nargs="?", default=0.0, help="distance (m) or angle (rad)")
    p.add_argument("radius", type=float, nargs="?", default=0.0, help="radius (m) for arc only")

    # Key switch
    p.add_argument("--sim", type=parse_bool, default=True,
                   help="true=Gazebo via ROS topics; false=drive RoboClaw motors")

    # SIM topics / settings
    p.add_argument("--cmd-vel-topic", default="/cmd_vel")
    p.add_argument("--odom-topic", default="/odom")

    p.add_argument("--linear-speed", type=float, default=0.2, help="m/s for sim fd/bk and sim arc")
    p.add_argument("--angular-speed", type=float, default=0.5, help="rad/s for sim lt/rt")
    p.add_argument("--pub-rate", type=float, default=20.0, help="cmd_vel publish rate (Hz)")
    p.add_argument("--stop-hold", type=float, default=0.25, help="seconds to hold stop before measuring end pose")

    # SIM ground truth
    p.add_argument("--gt-mode", choices=["ros", "gzcli"], default="gzcli",
                   help="SIM only: ground truth via PoseStamped topic or gz topic CLI")
    p.add_argument("--gt-ros-topic", default="/ground_truth/pose",
                   help="SIM only: PoseStamped ground truth topic if using --gt-mode ros")

    # Default now matches your Gazebo world naming convention
    # (you can override if your model name differs)
    p.add_argument("--gz-pose-topic", default="/world/empty_world/model/k9_robot/pose",
                   help="SIM only: Gazebo pose topic for gzcli mode")

    return p.parse_args(argv)


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]
    args = parse_args(argv)

    rclpy.init()
    node = K9MoveTest(args)

    try:
        res = node.run_move(args.command, args.parameter, args.radius)
        print_report(args.command, args.parameter, args.radius, res)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted; stopping.")
        try:
            if args.sim:
                node.stop_sim()
            else:
                if node.roboclaw:
                    node.roboclaw.stop()
        except Exception:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
