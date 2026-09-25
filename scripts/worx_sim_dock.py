#!/usr/bin/env python3
"""Simulated charging contacts for worx_sim.launch.py: puts the emulated Worx
board "in the charger" while the robot's charging_port is on a docking station
from /mowing_map (station pose faces out, so the robot faces the opposite way),
via /worx/fake/set_in_charger."""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_srvs.srv import SetBool
import tf2_ros

from open_mower_next.msg import Map


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimDock(Node):
    def __init__(self):
        super().__init__('worx_sim_dock')
        self.tolerance = self.declare_parameter('tolerance', 0.10).value
        self.yaw_tolerance = self.declare_parameter('yaw_tolerance', 0.35).value
        self.docks = []
        self.state = None
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
        self.create_subscription(Map, '/mowing_map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.client = self.create_client(SetBool, '/worx/fake/set_in_charger')
        self.create_timer(0.1, self.tick)

    def on_map(self, m):
        self.docks = [(d.pose.pose.position.x, d.pose.pose.position.y, yaw_of(d.pose.pose.orientation))
                      for d in m.docking_stations]

    def tick(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        try:
            port = self.tf.lookup_transform('map', 'charging_port', rclpy.time.Time())
        except Exception:
            return
        x, y = port.transform.translation.x, port.transform.translation.y
        yaw = yaw_of(t.transform.rotation)
        docked = any(math.hypot(x - dx, y - dy) < self.tolerance and
                     abs(math.remainder(yaw - dyaw - math.pi, 2 * math.pi)) < self.yaw_tolerance
                     for dx, dy, dyaw in self.docks)
        if docked != self.state and self.client.service_is_ready():
            self.client.call_async(SetBool.Request(data=docked))
            self.get_logger().info('in charger' if docked else 'left charger')
            self.state = docked


def main():
    rclpy.init()
    rclpy.spin(SimDock())


if __name__ == '__main__':
    main()
