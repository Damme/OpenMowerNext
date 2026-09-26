#!/usr/bin/env python3
"""Virtual obstacles for worx_sim.launch.py: presses the emulated Worx board's
collision sensor (/worx/fake/set_collision) while the robot's front edge is
inside one of the obstacle circles. The obstacles are not in the map, so the
robot only finds them by bumping into them.

Parameter `obstacles`: "x,y,r;x,y,r" in the map frame (m); can be changed at
runtime (ros2 param set /worx_sim_obstacles obstacles "...")."""

import math

import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_srvs.srv import SetBool
import tf2_ros


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimObstacles(Node):
    def __init__(self):
        super().__init__('worx_sim_obstacles')
        spec = self.declare_parameter('obstacles', '').value
        self.front = self.declare_parameter('front', 0.47).value       # base_link -> front edge
        self.half_width = self.declare_parameter('half_width', 0.195).value
        self.obstacles = self.parse(spec)
        self.add_on_set_parameters_callback(self.on_params)
        self.state = None
        self.tf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.tf, self)
        self.client = self.create_client(SetBool, '/worx/fake/set_collision')
        self.create_timer(0.02, self.tick)
        self.get_logger().info(f'{len(self.obstacles)} virtual obstacles: {self.obstacles}')

    @staticmethod
    def parse(spec):
        return [tuple(float(v) for v in o.split(',')) for o in spec.split(';') if o.strip()]

    def on_params(self, params):
        for p in params:
            if p.name == 'obstacles':
                try:
                    self.obstacles = self.parse(p.value)
                except ValueError as e:
                    return SetParametersResult(successful=False, reason=str(e))
                self.get_logger().info(f'virtual obstacles: {self.obstacles}')
        return SetParametersResult(successful=True)

    def tick(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        except Exception:
            return
        x, y = t.transform.translation.x, t.transform.translation.y
        yaw = yaw_of(t.transform.rotation)
        c, s = math.cos(yaw), math.sin(yaw)
        # Sample the front edge of the body.
        edge = [(x + self.front * c - o * s, y + self.front * s + o * c)
                for o in (-self.half_width, 0.0, self.half_width)]
        hit = any(math.hypot(px - ox, py - oy) < r for px, py in edge for ox, oy, r in self.obstacles)
        if hit != self.state and self.client.service_is_ready():
            self.client.call_async(SetBool.Request(data=hit))
            self.get_logger().info('bumper pressed' if hit else 'bumper released')
            self.state = hit


def main():
    rclpy.init()
    rclpy.spin(SimObstacles())


if __name__ == '__main__':
    main()
