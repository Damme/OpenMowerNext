#!/usr/bin/env python3
# Usage (with worx_sim.launch.py running and mowing started):
#   python3 track_mission.py <area_id> <max_seconds>
"""Record base_link while the blade is on; at the end report cross-track error
against the coverage passes of the area and mission events."""
import math, sys, time, json, rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from open_mower_next.srv import AreaCoverage
import tf2_ros
area = sys.argv[1]; duration = float(sys.argv[2])
rclpy.init(); n = Node('track')
tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n)
blade = {'on': False}; samples = []; states = []
n.create_subscription(Float64MultiArray, '/mower_controller/commands', lambda m: blade.__setitem__('on', bool(m.data and m.data[0] > 0.5)), 10)
n.create_subscription(String, '/mower_logic/state', lambda m: states.append(json.loads(m.data)['state']), 10)
cli = n.create_client(AreaCoverage, '/area_coverage'); cli.wait_for_service()
f = cli.call_async(AreaCoverage.Request(area_id=area)); rclpy.spin_until_future_complete(n, f)
passes = [[(p.pose.position.x, p.pose.position.y) for p in c.path.poses] for c in f.result().paths]
t0 = time.time(); last = 0
while time.time() - t0 < duration:
    rclpy.spin_once(n, timeout_sec=0.05)
    if time.time() - last > 0.1:
        last = time.time()
        if blade['on']:
            try:
                t = tfb.lookup_transform('map', 'base_link', rclpy.time.Time())
                samples.append((t.transform.translation.x, t.transform.translation.y))
            except Exception: pass
    if states and states[-1] == 'IDLE' and time.time() - t0 > 60: break
def dseg(px, py, a, b):
    dx, dy = b[0]-a[0], b[1]-a[1]; L = dx*dx+dy*dy
    t = 0 if L == 0 else max(0, min(1, ((px-a[0])*dx + (py-a[1])*dy)/L))
    return math.hypot(px-a[0]-t*dx, py-a[1]-t*dy)
segs = [(p[i], p[i+1]) for p in passes for i in range(len(p)-1)]
errs = sorted(min(dseg(x, y, a, b) for a, b in segs) for x, y in samples)
if errs:
    print(f'blade-on samples {len(errs)} ({len(errs)/10:.0f} s): cross-track median {errs[len(errs)//2]*100:.1f} cm, p95 {errs[int(len(errs)*.95)]*100:.1f} cm, max {errs[-1]*100:.1f} cm')
print('states:', [s for i, s in enumerate(states) if i == 0 or states[i-1] != s])
