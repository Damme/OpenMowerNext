#!/usr/bin/env python3
"""map -> odom for worx_sim: the start pose (OM_SIM_X/Y/QZ/QW), published at 20 Hz
so tests can move it. A geometry_msgs/Vector3 on /sim/slide shifts the robot's map
pose by (x, y) metres at once: the robot sliding sideways down a slope while
turning, or an RTK correction, which wheel odometry never sees."""

import rclpy
from geometry_msgs.msg import TransformStamped, Vector3
from rclpy.node import Node
import tf2_ros


class MapOdom(Node):
    def __init__(self):
        super().__init__('worx_sim_map_odom')
        p = lambda name, default: self.declare_parameter(name, default).value
        self.x, self.y = p('x', 0.0), p('y', 0.0)
        self.qz, self.qw = p('qz', 0.0), p('qw', 1.0)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Vector3, '/sim/slide', self.slide, 10)
        self.create_timer(0.05, self.publish)

    def slide(self, v):
        self.x += v.x
        self.y += v.y
        self.get_logger().info(f'slide ({v.x:+.2f}, {v.y:+.2f}) m')

    def publish(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id, t.child_frame_id = 'map', 'odom'
        t.transform.translation.x, t.transform.translation.y = self.x, self.y
        t.transform.rotation.z, t.transform.rotation.w = self.qz, self.qw
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(MapOdom())


if __name__ == '__main__':
    main()
