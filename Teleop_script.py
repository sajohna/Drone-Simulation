#!/usr/bin/env python3
"""
PX4 Offboard keyboard control via ROS2.
Streams OffboardControlMode + TrajectorySetpoint at 10 Hz,
auto-sets EKF origin, arms, and switches to offboard mode.

Keys:
  w/s  = forward / back
  a/d  = left / right
  r/f  = up / down
  q/e  = yaw left / right
  t    = takeoff (sustained climb to 2m)
  l    = land
  x    = exit
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
import sys
import tty
import termios
import threading
import time
import math

# ── QoS matching PX4 uORB DDS bridge ────────────────────────
PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

SPEED     = 2.0   # m/s translation
YAW_RATE  = 0.6   # rad/s
TAKEOFF_Z = -2.0  # NED metres (negative = up)
HOLD_SECS = 0.3   # seconds to hold each velocity command


class OffboardKeyboard(Node):
    def __init__(self):
        super().__init__('offboard_keyboard')

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', PX4_QOS)
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', PX4_QOS)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', PX4_QOS)

        self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self._pos_cb, PX4_QOS)
        self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status',
            self._status_cb, PX4_QOS)

        self.pos        = [0.0, 0.0, 0.0]
        self.armed      = False
        self.nav_state  = -1
        self.counter    = 0          # ticks before arming
        self.ARM_AFTER  = 15         # ~1.5 s of streaming first

        # Current velocity command (set by keyboard, cleared after HOLD_SECS)
        self.vx = self.vy = self.vz = self.vyaw = 0.0
        self._vel_lock  = threading.Lock()
        self._hold_until = 0.0       # time.time() deadline for current vel

        self.create_timer(0.1, self._timer_cb)
        self.get_logger().info('Node started — streaming setpoints...')

    def _pos_cb(self, msg):
        self.pos = [msg.x, msg.y, msg.z]

    def _status_cb(self, msg):
        self.armed     = (msg.arming_state == 2)
        self.nav_state = msg.nav_state

    # ── 10 Hz timer ─────────────────────────────────────────
    def _timer_cb(self):
        now_us = int(self.get_clock().now().nanoseconds / 1000)

        # 1. Always stream OffboardControlMode
        ocm = OffboardControlMode()
        ocm.position     = False
        ocm.velocity     = True
        ocm.acceleration = False
        ocm.attitude     = False
        ocm.body_rate    = False
        ocm.timestamp    = now_us
        self.offboard_pub.publish(ocm)

        # 2. Build velocity setpoint (zero if hold expired)
        with self._vel_lock:
            if time.time() > self._hold_until:
                self.vx = self.vy = self.vz = self.vyaw = 0.0
            vx, vy, vz, vyaw = self.vx, self.vy, self.vz, self.vyaw

        sp = TrajectorySetpoint()
        sp.position  = [float('nan')] * 3
        sp.velocity  = [vx, vy, vz]
        sp.yawspeed  = vyaw
        sp.yaw       = float('nan')
        sp.timestamp = now_us
        self.setpoint_pub.publish(sp)

        # 3. After streaming enough ticks: set EKF origin, arm, offboard
        if self.counter == self.ARM_AFTER:
            self.get_logger().info('Switching to offboard mode...')
            self._set_offboard_mode()
        if self.counter == self.ARM_AFTER + 3:
            self.get_logger().info('Arming...')
            self._arm()

        if self.counter <= self.ARM_AFTER + 10:
            self.counter += 1

    # ── Vehicle commands ─────────────────────────────────────
    def _send_cmd(self, command, p1=0.0, p2=0.0, p3=0.0,
                  p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        msg = VehicleCommand()
        msg.command          = command
        msg.param1           = float(p1)
        msg.param2           = float(p2)
        msg.param3           = float(p3)
        msg.param4           = float(p4)
        msg.param5           = float(p5)
        msg.param6           = float(p6)
        msg.param7           = float(p7)
        msg.target_system    = 1
        msg.target_component = 1
        msg.source_system    = 1
        msg.source_component = 1
        msg.from_external    = True
        msg.timestamp        = int(self.get_clock().now().nanoseconds / 1000)
        self.cmd_pub.publish(msg)

    def _set_ekf_origin(self):
        # MAV_CMD_SET_GPS_HOME_POSITION (242)
        self._send_cmd(242, p5=0.0, p6=0.0, p7=0.0)

    def _set_offboard_mode(self):
        # MAV_CMD_DO_SET_MODE: base_mode=1 (custom), custom_main=6 (offboard)
        self._send_cmd(176, p1=1.0, p2=6.0)

    def _arm(self):
        # MAV_CMD_COMPONENT_ARM_DISARM
        self._send_cmd(400, p1=1.0)

    def _land(self):
        self._send_cmd(21)  # MAV_CMD_NAV_LAND

    # ── Velocity helpers (called from keyboard thread) ───────
    def set_velocity(self, vx=0.0, vy=0.0, vz=0.0, vyaw=0.0, hold=HOLD_SECS):
        with self._vel_lock:
            self.vx, self.vy, self.vz, self.vyaw = vx, vy, vz, vyaw
            self._hold_until = time.time() + hold

    def takeoff(self):
        self.get_logger().info('Takeoff!')
        # Climb until we reach TAKEOFF_Z then hold
        self.set_velocity(vz=-SPEED, hold=2.0)

    def land(self):
        self.get_logger().info('Landing...')
        self._land()


# ── Keyboard loop (main thread) ──────────────────────────────
def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def keyboard_loop(node):
    settings = termios.tcgetattr(sys.stdin)
    print('\n============================================')
    print('  PX4 Offboard Keyboard Control')
    print('  (auto-arming in ~2s, please wait)')
    print('  w/s  = forward / back')
    print('  a/d  = left / right')
    print('  r/f  = up / down')
    print('  q/e  = yaw left / right')
    print('  t    = takeoff')
    print('  l    = land')
    print('  x    = exit')
    print('============================================\n')

    try:
        while rclpy.ok():
            key = get_key(settings)
            if   key == 'w': node.set_velocity(vx= SPEED)
            elif key == 's': node.set_velocity(vx=-SPEED)
            elif key == 'a': node.set_velocity(vy= SPEED)
            elif key == 'd': node.set_velocity(vy=-SPEED)
            elif key == 'r': node.set_velocity(vz=-SPEED)
            elif key == 'f': node.set_velocity(vz= SPEED)
            elif key == 'q': node.set_velocity(vyaw= YAW_RATE)
            elif key == 'e': node.set_velocity(vyaw=-YAW_RATE)
            elif key == 't': node.takeoff()
            elif key == 'l': node.land()
            elif key == 'x':
                print('Exiting.')
                break
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        rclpy.shutdown()


def main():
    rclpy.init()
    node = OffboardKeyboard()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    keyboard_loop(node)


if __name__ == '__main__':
    main()