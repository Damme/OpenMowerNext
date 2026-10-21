#!/usr/bin/env python3
"""Measure the LSM6DSV mount tilt for lsm6dsv_imu's mount_up (180 deg method).

The IMU measures gravity in the body frame standing still (imu/calibrate). On a
slope that is mount tilt + slope; turned 180 deg on the same spot the slope part
flips sign, so the mean of both is the mount alone. Doesn't drive: turn the robot
yourself in between (web UI manual drive), on the same spot. Each measurement
also calibrates the gyro bias, so the robot must stand still (blade off).

    worx_imu_mount.py            # two measurements, prints the mount_up line
    worx_imu_mount.py --once     # one measurement (e.g. on a known level floor)
"""
import argparse
import math
import re
import sys

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


def measure(node, client):
    future = client.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=20.0)
    if not future.done():
        sys.exit('imu/calibrate: no answer within 20 s')
    res = future.result()
    m = re.search(r'up ([-+0-9.]+) ([-+0-9.]+) ([-+0-9.]+)', res.message)
    if not res.success or not m:
        sys.exit(f'imu/calibrate failed: {res.message}')
    print(f'  {res.message}')
    return [float(v) for v in m.groups()]


def tilt(up):
    return math.degrees(math.acos(max(-1.0, min(1.0, up[2] / math.sqrt(sum(v * v for v in up))))))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--once', action='store_true', help='one measurement (level floor)')
    args = ap.parse_args()

    rclpy.init()
    node = Node('worx_imu_mount')
    client = node.create_client(Trigger, '/imu/calibrate')
    if not client.wait_for_service(timeout_sec=10.0):
        sys.exit('/imu/calibrate not available (stack running?)')

    input('Robot standing still, blade off? Enter to measure... ')
    a = measure(node, client)
    print(f'  tilt {tilt(a):.2f} deg')
    if args.once:
        up = a
    else:
        input('Turn the robot 180 deg on the same spot, let it stand still. Enter to measure... ')
        b = measure(node, client)
        print(f'  tilt {tilt(b):.2f} deg')
        up = [(x + y) / 2 for x, y in zip(a, b)]
        slope = [(x - y) / 2 for x, y in zip(a, b)]
        g = math.sqrt(sum(v * v for v in up))
        print(f'ground slope here ~{math.degrees(math.asin(min(1.0, math.hypot(slope[0], slope[1]) / g))):.2f} deg')
    print(f'mount tilt {tilt(up):.2f} deg')
    print('config/launch/worx.yaml, lsm6dsv_imu params:')
    print(f'  - mount_up: [{up[0]:.4f}, {up[1]:.4f}, {up[2]:.4f}]')
    print('  - auto_level: false')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
