#!/usr/bin/env python3
"""Live console monitor for tests on the real Worx robot.

One status line per second (GPS, pose, charger/battery, motors, commanded vs
measured wheel speed, PWM and current, mower_logic state, docking progress),
and every WARN/ERROR/FATAL log line of any node as it happens.

    ros2 run open_mower_next worx_monitor.py [--ros-args -p wheel_radius:=0.1]
"""
import json
import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_msgs.action import DockRobot, UndockRobot
from nav_msgs.msg import Odometry
from open_mower_next.msg import WorxStatus
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState, NavSatFix
from std_msgs.msg import String

GPS = {-1: 'NOFIX', 0: 'FIX', 1: 'SBAS', 2: 'RTKfix'}
DOCK_STATES = {0: 'none', 1: 'nav_staging', 2: 'initial_percept', 3: 'controlling', 4: 'wait_charge', 5: 'retry'}
LEVELS = {30: 'WARN', 40: 'ERROR', 50: 'FATAL'}


class Monitor(Node):
    def __init__(self):
        super().__init__('worx_monitor')
        self.r = self.declare_parameter('wheel_radius', 0.1).value
        self.s = {}
        self.last_log = {}  # (node, first words) -> time: repeated warnings every 10 s only
        self.create_subscription(NavSatFix, '/gps/fix', self.gps, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odometry/filtered/map', self.odom, 10)
        self.create_subscription(WorxStatus, '/worx/status', self.status, 10)
        self.create_subscription(TwistStamped, '/diff_drive_base_controller/cmd_vel', self.cmd, 10)
        self.create_subscription(JointState, '/joint_states', self.joints, 10)
        self.create_subscription(Imu, '/imu/data_raw', self.imu, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, '/mower_logic/state', self.logic, latched)
        self.create_subscription(DockRobot.Impl.FeedbackMessage, '/dock_robot/_action/feedback',
                                 self.dock_fb, 10)
        self.create_subscription(UndockRobot.Impl.FeedbackMessage, '/undock_robot/_action/feedback',
                                 lambda m: self.s.update(dock='undocking'), 10)
        self.create_subscription(Log, '/rosout', self.log, QoSProfile(depth=100))
        self.create_timer(1.0, self.print_line)

    def gps(self, m):
        self.s['gps'] = f"{GPS.get(m.status.status, m.status.status)} {math.sqrt(m.position_covariance[0]):.3f}m"
        self.s['gps_t'] = time.monotonic()

    def odom(self, m):
        q = m.pose.pose.orientation
        yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        p = m.pose.pose.position
        self.s['pose'] = f"{p.x:7.2f} {p.y:7.2f} {yaw:5.0f}deg"

    def status(self, m):
        self.s['st'] = m

    def cmd(self, m):
        self.s['cmd'] = f"v{m.twist.linear.x:+.2f} w{m.twist.angular.z:+.2f}"
        self.s['cmd_t'] = time.monotonic()

    def joints(self, m):
        v = dict(zip(m.name, m.velocity))
        self.s['wheels'] = (f"L{v.get('left_wheel_joint', 0.0) * self.r:+.2f} "
                            f"R{v.get('right_wheel_joint', 0.0) * self.r:+.2f}")

    def imu(self, m):
        self.s['gyro'] = f"{math.degrees(m.angular_velocity.z):+5.1f}deg/s"

    def logic(self, m):
        try:
            d = json.loads(m.data)
            self.s['logic'] = f"{d.get('state')}/{d.get('command')}{' GPSBAD' if not d.get('gps_ok', True) else ''}"
        except ValueError:
            self.s['logic'] = m.data

    def dock_fb(self, m):
        self.s['dock'] = f"docking:{DOCK_STATES.get(m.feedback.state, m.feedback.state)}"

    def log(self, m):
        if m.level < 30 or m.name == 'worx_monitor':
            return
        key = (m.name, m.msg[:40])
        now = time.monotonic()
        if now - self.last_log.get(key, -99) < 10:
            return
        self.last_log[key] = now
        print(f"   {LEVELS.get(m.level, m.level)} [{m.name}] {m.msg}", flush=True)

    def print_line(self):
        s = self.s
        now = time.monotonic()
        gps = s.get('gps', '-') if now - s.get('gps_t', -99) < 2 else 'STALE'
        cmd = s.get('cmd', '-') if now - s.get('cmd_t', -99) < 1 else 'v+0.00 w+0.00'
        st = s.get('st')
        if st:
            board = (f"{'CHG' if st.in_charger else 'bat'} {st.battery_voltage:4.1f}V "
                     f"{'LINK' if st.link_ok else 'NOLINK'} {st.motor_state.replace('MOTORREQ_', '')}"
                     f"{' EMERG' if st.emergency else ''}{' BUMP' if st.collision else ''}{' LIFT' if st.lift else ''} "
                     f"pwm {st.left_pwm_cmd}/{st.right_pwm_cmd}/{st.mow_pwm_cmd} "
                     f"cur {st.motor_current[0]}/{st.motor_current[1]}/{st.motor_current[2]}")
        else:
            board = 'no /worx/status'
        print(f"{time.strftime('%H:%M:%S')} GPS {gps} | pose {s.get('pose', '-')} | {board} | "
              f"cmd {cmd} wheel {s.get('wheels', '-')} {s.get('gyro', '')} | "
              f"{s.get('logic', '-')} {s.get('dock', '')}", flush=True)


def main():
    rclpy.init()
    rclpy.spin(Monitor())


if __name__ == '__main__':
    main()
