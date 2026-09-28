#!/usr/bin/env python3
# Usage: worx_sim (om_launch worx_sim.yaml) running, then python3 scenario_rejoin.py [slides...]
# (it starts mowing itself). Default slides: 0.3 -0.6 0.9 (m, + = to the robot's left).
"""Rejoin scenario in the kinematic sim: while the robot mows a straight stretch
(blade on, heading steady for 3 s), shift it sideways by /sim/slide (as it slid
down a slope while turning), then follow its distance to the current pass (from
/area_coverage, pass number from /mower_logic/state) for 30 s. Reports per slide:
time until it stayed within 5 cm (nearest point of the pass: approximate on
narrow outlines), seconds nearly standing still, and the FTC rejoin / pass
failure messages from /rosout (the reliable part)."""
import json, math, sys, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from rcl_interfaces.msg import Log
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
from open_mower_next.srv import AreaCoverage
import tf2_ros

slides = [float(a) for a in sys.argv[1:]] or [0.3, -0.6, 0.9]
rclpy.init(); n = Node('scenario_rejoin')
tfb = tf2_ros.Buffer(); tf2_ros.TransformListener(tfb, n)
slide_pub = n.create_publisher(Vector3, '/sim/slide', 10)
st = {'blade': False, 'states': [], 'mission': ''}
n.create_subscription(Float64MultiArray, '/mower_controller/commands',
                      lambda m: st.__setitem__('blade', bool(m.data and m.data[0] > 0.5)), 10)
def on_state(m):
    d = json.loads(m.data); st['mission'] = d['mission']; s = d['state']
    if not st['states'] or st['states'][-1] != s:
        st['states'].append(s); print(f'  state {s}', flush=True)
n.create_subscription(String, '/mower_logic/state', on_state, 10)
events = []  # FTC rejoin log lines and pass failures, from /rosout
KEYS = ('beside the path', 'back on the path', 'while rejoining', 'Failed to make progress', 'Pass failed', 'far away')
n.create_subscription(Log, '/rosout', lambda m: events.append((time.time(), m.msg)) if any(k in m.msg for k in KEYS) else None, 100)
area = __import__('os').environ.get('OM_MOWING_AREAS', 'mow_0').split(',')[0]
cov = n.create_client(AreaCoverage, '/area_coverage'); cov.wait_for_service()
f = cov.call_async(AreaCoverage.Request(area_id=area)); rclpy.spin_until_future_complete(n, f)
passes = [[(q.pose.position.x, q.pose.position.y) for q in c.path.poses] for c in f.result().paths]
start = n.create_client(Trigger, '/mower_logic/start_mowing'); start.wait_for_service()
start.call_async(Trigger.Request())

def to_pass(px, py):
    """Distance to the current pass and the pass heading at the nearest point."""
    import re
    m = re.search(r'pass (\d+)/', st['mission'])
    if not m: return None
    pts = passes[int(m.group(1)) - 1]
    best = (1e9, 0.0)
    for a, b in zip(pts, pts[1:]):
        dx, dy = b[0] - a[0], b[1] - a[1]; L = dx * dx + dy * dy
        if L == 0: continue
        t = max(0.0, min(1.0, ((px - a[0]) * dx + (py - a[1]) * dy) / L))
        d = math.hypot(px - a[0] - t * dx, py - a[1] - t * dy)
        if d < best[0]: best = (d, math.atan2(dy, dx))
    return best

def pose():
    try:
        t = tfb.lookup_transform('map', 'base_link', rclpy.time.Time())
    except Exception:
        return None
    q = t.transform.rotation
    return (t.transform.translation.x, t.transform.translation.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

def spin(seconds):
    end = time.time() + seconds
    while time.time() < end: rclpy.spin_once(n, timeout_sec=0.02)

wrap = lambda a: math.atan2(math.sin(a), math.cos(a))
results = []
for s in slides:
    hist = []  # (t, x, y, yaw) while mowing
    while True:  # a straight, mowing stretch: blade on, yaw steady over 3 s
        spin(0.1); p = pose()
        if not st['blade'] or p is None: hist.clear(); continue
        hist.append((time.time(),) + p); hist = [h for h in hist if h[0] > time.time() - 3.0]
        if len(hist) > 25 and hist[-1][0] - hist[0][0] > 2.8 and \
           max(abs(wrap(h[3] - hist[-1][3])) for h in hist) < math.radians(2) and \
           math.hypot(hist[-1][1] - hist[0][1], hist[-1][2] - hist[0][2]) > 0.3:
            break
    _, x0, y0, h0 = hist[-1]
    nx, ny = -math.sin(h0), math.cos(h0)  # left of the robot = left of the line
    slide_pub.publish(Vector3(x=s * nx, y=s * ny)); t0 = time.time()
    print(f'slide {s:+.1f} m at ({x0:.2f}, {y0:.2f}), heading {math.degrees(h0):.0f} deg', flush=True)
    back = None; still = 0.0; last = None; off = 0.0
    while time.time() - t0 < 30:
        spin(0.1); p = pose()
        if p is None: continue
        tp = to_pass(p[0], p[1])
        if tp is None: continue
        off, path_yaw = tp
        if last and math.hypot(p[0] - last[0], p[1] - last[1]) < 0.005 and off > 0.05: still += 0.1
        last = p
        if off < 0.05 and back is None: back = time.time() - t0
        if off >= 0.05: back = None
    r = dict(slide=s, back_s=None if back is None else round(back, 1), standing_s=round(still, 1),
             end_offset=round(off, 2), log=[f'+{t - t0:.1f}s {m}' for t, m in events if t0 <= t <= t0 + 30])
    print('  ->', r, flush=True); results.append(r)
    spin(5.0)
print('RESULTS', json.dumps(results))
print('states:', st['states'])
