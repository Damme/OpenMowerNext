#!/usr/bin/env python3
# Usage (with worx_sim.launch.py running): python3 footprint_watch.py <max_seconds> [front rear half_width]
"""Checks the robot's real footprint along the driven track against the
recorded areas + rim (/map_grid: free, rim and blurred cells are allowed;
unknown and lethal are not) and reports every violation: where, how many
footprint points were outside, what the robot was doing (mower_logic state,
blade). A recorded line can turn more sharply than the body: this shows the
manoeuvres whose front/rear leaves line + rim."""
import json, math, sys, time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float64MultiArray, String
import tf2_ros

from open_mower_next.msg import Map

duration = float(sys.argv[1])
front, rear, hw = (float(v) for v in sys.argv[2:5]) if len(sys.argv) >= 5 else (0.47, 0.11, 0.195)
rclpy.init()
n = Node('footprint_watch')
tfb = tf2_ros.Buffer()
tf2_ros.TransformListener(tfb, n)
st = {'grid': None, 'state': '', 'blade': False, 'docks': []}
n.create_subscription(Map, '/mowing_map', lambda m: st.__setitem__(
    'docks', [(d.pose.pose.position.x, d.pose.pose.position.y) for d in m.docking_stations]),
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(OccupancyGrid, '/map_grid', lambda m: st.__setitem__('grid', m),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(String, '/mower_logic/state', lambda m: st.__setitem__('state', json.loads(m.data)['state']),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
n.create_subscription(Float64MultiArray, '/mower_controller/commands',
                      lambda m: st.__setitem__('blade', bool(m.data and m.data[0] > 0.5)), 10)


def cell(g, x, y):
    i = g.info
    cx = int(math.floor((x - i.origin.position.x) / i.resolution))
    cy = int(math.floor((y - i.origin.position.y) / i.resolution))
    if cx < 0 or cy < 0 or cx >= i.width or cy >= i.height:
        return -1
    return g.data[cy * i.width + cx]


def outline():
    pts = []
    for a, b in (((front, hw), (front, -hw)), ((-rear, hw), (-rear, -hw)), ((-rear, hw), (front, hw)), ((-rear, -hw), (front, -hw))):
        k = max(1, int(math.ceil(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05)))
        pts += [(a[0] + (b[0] - a[0]) * t / k, a[1] + (b[1] - a[1]) * t / k) for t in range(k + 1)]
    return pts


fp = outline()
t0 = time.time()
last = 0.0
samples = 0
events = []  # (t, x, y, yaw, n_out, centre_ok, state, blade)
while time.time() - t0 < duration:
    rclpy.spin_once(n, timeout_sec=0.05)
    if st['grid'] is None or time.time() - last < 0.2:
        continue
    last = time.time()
    try:
        t = tfb.lookup_transform('map', 'base_link', rclpy.time.Time())
    except Exception:
        continue
    x, y = t.transform.translation.x, t.transform.translation.y
    q = t.transform.rotation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    if st['state'] in ('IDLE', 'CHARGING', '') or any(math.hypot(x - dx, y - dy) < 2.6 for dx, dy in st['docks']):
        continue  # in/at the dock or its approach: the dock sits outside the areas
    samples += 1
    c, s = math.cos(yaw), math.sin(yaw)
    out = sum(1 for u, v in fp if not 0 <= cell(st['grid'], x + u * c - v * s, y + u * s + v * c) < 100)
    centre = cell(st['grid'], x, y)
    if out or centre != 0:  # centre: free cells only (not the rim)
        events.append((round(time.time() - t0, 1), round(x, 2), round(y, 2), round(math.degrees(yaw)), out,
                       centre == 0, st["state"], st["blade"]))
        print('VIOLATION t=%.1f pos=(%.2f, %.2f) yaw=%d outside=%d/%d centre_ok=%s state=%s blade=%s'
              % (events[-1][0], x, y, events[-1][3], out, len(fp), events[-1][5], st['state'], st['blade']), flush=True)
print('samples %d, violations %d (%.1f%%)' % (samples, len(events), 100.0 * len(events) / max(1, samples)))
