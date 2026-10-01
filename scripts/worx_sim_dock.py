#!/usr/bin/env python3
"""Simulated docking station for worx_sim.launch.py, for the docking stations in
/mowing_map (station pose faces out, so the robot faces the opposite way).

Like the real one: once the robot's charging_port reaches the contacts the
bumper is pressed (/worx/fake/set_collision: the emulated board latches
BlockForward and stops forward motion), and if the port is on the contacts
the station starts charging after charge_delay (/worx/fake/set_in_charger).
fail_contacts: that many first contacts never charge (to test the retries)."""

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
        self.tolerance = self.declare_parameter('tolerance', 0.05).value  # lateral, port on the contacts
        self.yaw_tolerance = self.declare_parameter('yaw_tolerance', 0.20).value
        self.width = self.declare_parameter('width', 0.25).value  # half width of the station front: bumper
        self.charge_delay = self.declare_parameter('charge_delay', 2.5).value
        self.fail_contacts = self.declare_parameter('fail_contacts', 0).value
        self.docks = []
        self.bumper = None
        self.charging = None
        self.contact_since = None
        self.contacts = 0
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
        self.create_subscription(Map, '/mowing_map', self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.charger = self.create_client(SetBool, '/worx/fake/set_in_charger')
        self.collision = self.create_client(SetBool, '/worx/fake/set_collision')
        self.create_timer(0.02, self.tick)

    def on_map(self, m):
        self.docks = [(d.pose.pose.position.x, d.pose.pose.position.y, yaw_of(d.pose.pose.orientation))
                      for d in m.docking_stations]

    def set(self, client, value, attr, text):
        if getattr(self, attr) != value and client.service_is_ready():
            client.call_async(SetBool.Request(data=value))
            self.get_logger().info(text)
            setattr(self, attr, value)

    def tick(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
            port = self.tf.lookup_transform('map', 'charging_port', rclpy.time.Time())
        except Exception:
            return
        x, y = port.transform.translation.x, port.transform.translation.y
        yaw = yaw_of(t.transform.rotation)
        pressed = on_contacts = False
        for dx, dy, dyaw in self.docks:
            ux, uy = -math.cos(dyaw), -math.sin(dyaw)  # robot's way in
            along = (x - dx) * ux + (y - dy) * uy
            lateral = -(x - dx) * uy + (y - dy) * ux
            if abs(lateral) < self.width and -0.005 <= along < 0.3:
                pressed = True
                on_contacts = (abs(lateral) < self.tolerance and
                               abs(math.remainder(yaw - dyaw - math.pi, 2 * math.pi)) < self.yaw_tolerance)
                self.last = (along, lateral)
        self.set(self.collision, pressed, 'bumper',
                 'bumper pressed at the dock' if pressed else 'bumper released')
        now = self.get_clock().now().nanoseconds * 1e-9
        if on_contacts and self.contact_since is None:
            self.contact_since = now
            self.contacts += 1
            self.get_logger().info('contact %d (along %+.3f, lateral %+.3f)%s' % (
                self.contacts, *self.last, ', will not charge' if self.contacts <= self.fail_contacts else ''))
        elif not on_contacts:
            self.contact_since = None
        charging = (self.contact_since is not None and self.contacts > self.fail_contacts and
                    now - self.contact_since >= self.charge_delay)
        if self.charging and on_contacts:
            charging = True
        self.set(self.charger, charging, 'charging', 'charging' if charging else 'not charging')


def main():
    rclpy.init()
    rclpy.spin(SimDock())


if __name__ == '__main__':
    main()
