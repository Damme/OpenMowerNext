#!/usr/bin/env python3
# Usage: worx_sim.launch.py ... rain_pause_time:=60.0 rain_resume_dry:=20.0 rain_clear_delay:=60.0,
# then python3 scenario_rain.py (it starts mowing itself).
"""Rain while mowing in the kinematic sim (/worx/fake/rain). Times are seconds
after the first blade ON. A short shower: RAIN_PAUSE on the lawn, mowing on once
dry for rain_resume_dry. Then flickering rain (the real sensor near its
threshold): RAIN_PAUSE, docks after rain_pause_time, WAITING_FOR_RAIN, IDLE once
dry for rain_clear_delay."""
import json, time, rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

SHOWER = (20, 6)           # start, duration
FLICKER = (100, 240)       # start, end: 4 s rain / 6 s dry
END_AFTER = 600

rclpy.init(); n = Node('scenario_rain')
rain = n.create_publisher(Bool, '/worx/fake/rain', 10)
st = {'t0': None, 'blade': None, 'state': None, 'rain': None}
T = lambda: time.time() - st['t0'] if st['t0'] else -1
def log(msg): print(f"[{T():7.1f}] {msg}", flush=True)
def on_blade(m):
    on = bool(m.data and m.data[0] > 0.5)
    if on and st['t0'] is None: st['t0'] = time.time()
    if on != st['blade']: st['blade'] = on; log(f"blade {'ON' if on else 'off'}")
def on_state(m):
    d = json.loads(m.data)
    if d['state'] != st['state']: st['state'] = d['state']; log(f"state {d['state']} ({d['mission']})")
n.create_subscription(Float64MultiArray, '/mower_controller/commands', on_blade, 10)
n.create_subscription(String, '/mower_logic/state', on_state, 10)
def tick():
    t = T()
    wet = SHOWER[0] <= t < SHOWER[0] + SHOWER[1] or (FLICKER[0] <= t < FLICKER[1] and (t - FLICKER[0]) % 10 < 4)
    if wet != st['rain']: st['rain'] = wet; log(f"rain {'ON' if wet else 'off'}")
    rain.publish(Bool(data=wet))
n.create_timer(0.2, tick)
start = n.create_client(Trigger, '/mower_logic/start_mowing'); start.wait_for_service()
start.call_async(Trigger.Request()); print('started', flush=True)
t_begin = time.time()
while rclpy.ok() and time.time() - t_begin < END_AFTER + 300:
    rclpy.spin_once(n, timeout_sec=0.1)
    if st['state'] == 'IDLE' and st['t0'] and T() > FLICKER[1]: log('done'); break
