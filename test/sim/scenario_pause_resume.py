#!/usr/bin/env python3
# Usage: worx_sim.launch.py ... require_gps:=true, then python3 scenario_pause_resume.py
# (it starts mowing itself and ends when the mission is over).
"""Robustness scenario in the kinematic sim. Times are seconds after the first
blade ON: GPS dropout, emergency, low battery. Publishes a fake RTK fix, logs
state changes, blade changes and the mission progress."""
import json, time, rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Float64MultiArray, String, Int32
from std_srvs.srv import SetBool, Trigger

GPS_LOSS = (60, 20)        # start, duration
EMERGENCY = (180, 10)
LOW_BATTERY = 260          # set 22.0 V
END_AFTER = 1500

rclpy.init(); n = Node('scenario')
gps = n.create_publisher(NavSatFix, '/gps/fix', qos_profile_sensor_data)
bat = n.create_publisher(Int32, '/worx/fake/battery_mv', 10)
emg = n.create_client(SetBool, '/worx/emergency')
st = {'t0': None, 'blade': None, 'state': None, 'mission': None, 'done': set()}
T = lambda: time.time() - st['t0'] if st['t0'] else -1
def log(msg): print(f"[{T():7.1f}] {msg}", flush=True)
def on_blade(m):
    on = bool(m.data and m.data[0] > 0.5)
    if on and st['t0'] is None: st['t0'] = time.time()
    if on != st['blade']: st['blade'] = on; log(f"blade {'ON' if on else 'off'}")
def on_state(m):
    d = json.loads(m.data)
    if d['state'] != st['state']: st['state'] = d['state']; log(f"state {d['state']} ({d['mission']}, battery {d['battery']:.2f})")
    st['mission'] = d['mission']
n.create_subscription(Float64MultiArray, '/mower_controller/commands', on_blade, 10)
n.create_subscription(String, '/mower_logic/state', on_state, 10)
def tick():
    t = T()
    lost = GPS_LOSS[0] <= t < GPS_LOSS[0] + GPS_LOSS[1]
    if lost and 'gps_lost' not in st['done']: st['done'].add('gps_lost'); log(f'GPS LOST ({st["mission"]})')
    if t >= GPS_LOSS[0] + GPS_LOSS[1] and 'gps_back' not in st['done'] and 'gps_lost' in st['done']:
        st['done'].add('gps_back'); log('GPS BACK')
    if not lost:
        m = NavSatFix(); m.header.stamp = n.get_clock().now().to_msg(); m.header.frame_id = 'gps'
        m.status.status = NavSatStatus.STATUS_GBAS_FIX; m.latitude, m.longitude = 57.174598, 12.468556
        m.position_covariance = [0.0004, 0, 0, 0, 0.0004, 0, 0, 0, 0.001]
        m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        gps.publish(m)
    if t >= EMERGENCY[0] and 'emg_on' not in st['done']:
        st['done'].add('emg_on'); log(f'EMERGENCY ON ({st["mission"]})'); emg.call_async(SetBool.Request(data=True))
    if t >= EMERGENCY[0] + EMERGENCY[1] and 'emg_off' not in st['done']:
        st['done'].add('emg_off'); log('EMERGENCY OFF'); emg.call_async(SetBool.Request(data=False))
    if t >= LOW_BATTERY and 'bat' not in st['done']:
        st['done'].add('bat'); log(f'BATTERY -> 22.0 V ({st["mission"]})'); bat.publish(Int32(data=22000))
n.create_timer(0.2, tick)
start = n.create_client(Trigger, '/mower_logic/start_mowing'); start.wait_for_service()
start.call_async(Trigger.Request()); print('started', flush=True)
t_begin = time.time()
while rclpy.ok() and time.time() - t_begin < END_AFTER + 600:
    rclpy.spin_once(n, timeout_sec=0.1)
    if st['state'] == 'IDLE' and st['t0'] and T() > 300: log('mission over'); break
