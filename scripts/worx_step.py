#!/usr/bin/env python3
"""Step-by-step driving for robot tests: towards the start of an area's first
coverage pass (what a mission drives to first), but only the first N metres of
the planned path. Without --go it only prints the plan.

    worx_step.py --area mow_0 --distance 5        # show where it would stop
    worx_step.py --area mow_0 --distance 5 --go   # drive (same transit as mower_logic)
    worx_step.py --x 3.0 --y 5.0 --distance 2 --go
"""
import argparse
import math
import os
import sys

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, NavigateThroughPoses
from open_mower_next.srv import AreaCoverage
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def pose(x, y, yaw, stamp):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = stamp
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return p


def wait(node, future, timeout):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        sys.exit(f'timeout after {timeout} s')
    return future.result()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--area', help='area id: go towards its first pass start')
    ap.add_argument('--x', type=float)
    ap.add_argument('--y', type=float)
    ap.add_argument('--distance', type=float, required=True, help='metres of the planned path to drive')
    ap.add_argument('--go', action='store_true')
    args = ap.parse_args()
    if not args.area and (args.x is None or args.y is None):
        ap.error('--area or --x/--y')

    rclpy.init()
    node = Node('worx_step')
    tf = Buffer()
    TransformListener(tf, node)

    def robot():
        for _ in range(50):
            rclpy.spin_once(node, timeout_sec=0.1)
            try:
                t = tf.lookup_transform('map', 'base_link', rclpy.time.Time())
                p = pose(t.transform.translation.x, t.transform.translation.y, 0.0, t.header.stamp).pose
                p.orientation = t.transform.rotation
                return p
            except Exception:
                pass
        sys.exit('no map -> base_link transform')

    rp = robot()
    print(f'robot   {rp.position.x:7.2f} {rp.position.y:7.2f} {math.degrees(yaw_of(rp.orientation)):5.0f}deg')

    if args.area:
        cli = node.create_client(AreaCoverage, '/area_coverage')
        cli.wait_for_service(timeout_sec=10)
        req = AreaCoverage.Request()
        req.area_id = args.area
        res = wait(node, cli.call_async(req), 60)
        if res.code != 0 or not res.path.poses:
            sys.exit(f'area_coverage failed: {res.code} {res.message}')
        t = res.path.poses[0].pose.position
        tx, ty = t.x, t.y
        print(f'target  {tx:7.2f} {ty:7.2f}  (first pass start of {args.area})')
    else:
        tx, ty = args.x, args.y

    stamp = node.get_clock().now().to_msg()
    plan = ActionClient(node, ComputePathToPose, 'compute_path_to_pose')
    plan.wait_for_server(timeout_sec=10)
    g = ComputePathToPose.Goal()
    g.goal = pose(tx, ty, math.atan2(ty - rp.position.y, tx - rp.position.x), stamp)
    g.use_start = False
    gh = wait(node, plan.send_goal_async(g), 10)
    r = wait(node, gh.get_result_async(), 60).result
    poses = r.path.poses
    if not poses:
        sys.exit(f'no path (error {r.error_code})')
    total, stop, prev = 0.0, poses[-1], poses[0]
    for p in poses[1:]:
        total += math.hypot(p.pose.position.x - prev.pose.position.x, p.pose.position.y - prev.pose.position.y)
        if total >= args.distance:
            stop = p
            break
        prev = p
    sp = stop.pose.position
    heading = math.atan2(sp.y - prev.pose.position.y, sp.x - prev.pose.position.x)
    print(f'path    {len(poses)} poses; stop after {min(total, args.distance):.2f} m at '
          f'{sp.x:7.2f} {sp.y:7.2f} {math.degrees(heading):5.0f}deg')
    if not args.go:
        print('(dry run: add --go to drive)')
        return

    nav = ActionClient(node, NavigateThroughPoses, 'navigate_through_poses')
    nav.wait_for_server(timeout_sec=10)
    goal = NavigateThroughPoses.Goal()
    goal.poses = [pose(sp.x, sp.y, heading, node.get_clock().now().to_msg())]
    goal.behavior_tree = os.path.join(get_package_share_directory('open_mower_next'),
                                      'config/behavior_trees/transit_to_pass.xml')
    gh = wait(node, nav.send_goal_async(goal), 10)
    if not gh.accepted:
        sys.exit('goal rejected')
    print('driving ...', flush=True)
    res = wait(node, gh.get_result_async(), 300)
    print(f'result status {res.status} error {res.result.error_code} {res.result.error_msg}')
    p = robot()
    print(f'robot   {p.position.x:7.2f} {p.position.y:7.2f} {math.degrees(yaw_of(p.orientation)):5.0f}deg  '
          f'(stop point missed by {math.hypot(p.position.x - sp.x, p.position.y - sp.y):.2f} m)')


if __name__ == '__main__':
    main()
