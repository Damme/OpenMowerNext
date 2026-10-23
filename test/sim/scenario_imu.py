#!/usr/bin/env python3
# Usage: worx_sim.launch.py ... imu_calibration_timeout:=30.0, then python3 scenario_imu.py
# (it starts mowing itself and ends when the mower has given up on the IMU).
"""IMU recovery in the kinematic sim, the scenario plays lsm6dsv_imu: it latches
/imu/needs_calibration and serves /imu/calibrate. Times are seconds after the
first blade ON.
  - before: the calibration on the dock before the undock (answered OK)
  - RESET_1: chip reset mid-mission; the first calibration is refused (moving),
    the next one works -> IMU_CALIBRATION, stand still, mow on at the same place
  - RESET_2: the chip stays away (every calibration fails) -> IDLE after
    imu_calibration_timeout
Logs state and blade changes, calibration calls, the robot's speed while
calibrating and the mission progress."""
import json, math, time, rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import Trigger

RESET_1 = 40
RESET_2 = 150
END_AFTER = 600

rclpy.init(); n = Node('scenario_imu')
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
needs = n.create_publisher(Bool, '/imu/needs_calibration', latched)
st = {'t0': None, 'blade': None, 'state': None, 'mission': None, 'done': set(), 'mode': 'ok',
      'calls': 0, 'speed': 0.0, 'max_speed_calib': 0.0}
T = lambda: time.time() - st['t0'] if st['t0'] else -1
def log(msg): print(f"[{T():7.1f}] {msg}", flush=True)
def set_needs(v): needs.publish(Bool(data=v)); log(f'needs_calibration {v}')
def on_blade(m):
    on = bool(m.data and m.data[0] > 0.5)
    if on and st['t0'] is None: st['t0'] = time.time()
    if on != st['blade']: st['blade'] = on; log(f"blade {'ON' if on else 'off'}")
def on_state(m):
    d = json.loads(m.data)
    if d['state'] != st['state']:
        if st['state'] == 'IMU_CALIBRATION': log(f"max speed while calibrating {st['max_speed_calib']:.3f} m/s")
        st['state'] = d['state']; st['max_speed_calib'] = 0.0
        log(f"state {d['state']} ({d['mission']})")
    st['mission'] = d['mission']
def on_odom(m):
    v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y) + abs(m.twist.twist.angular.z) * 0.2
    if st['state'] == 'IMU_CALIBRATION': st['max_speed_calib'] = max(st['max_speed_calib'], v)
def on_calibrate(req, res):
    st['calls'] += 1
    if st['mode'] == 'refuse_once':
        st['mode'] = 'ok'
        res.success, res.message = False, 'Gyro calibration refused: moving (std 0.300 0.200 0.100 dps) [scenario]'
    elif st['mode'] == 'dead':
        res.success, res.message = False, 'Gyro calibration: data-ready timeout at sample 0 [scenario]'
    else:
        set_needs(False)  # as lsm6dsv_imu: cleared before the answer
        res.success, res.message = True, 'bias_dps +0.2000 -0.0050 +0.0500; up -1.5 +1.7 +9.5 [scenario]'
    log(f"imu/calibrate call {st['calls']} -> {'OK' if res.success else 'FAILED'} ({st['state']})")
    return res
n.create_subscription(Float64MultiArray, '/mower_controller/commands', on_blade, 10)
n.create_subscription(String, '/mower_logic/state', on_state, 10)
n.create_subscription(Odometry, '/odometry/filtered/map', on_odom, 10)
n.create_service(Trigger, '/imu/calibrate', on_calibrate)
set_needs(False)
def tick():
    t = T()
    if t >= RESET_1 and 'r1' not in st['done']:
        st['done'].add('r1'); st['mode'] = 'refuse_once'; log(f'IMU RESET 1 ({st["mission"]})'); set_needs(True)
    if t >= RESET_2 and 'r2' not in st['done']:
        st['done'].add('r2'); st['mode'] = 'dead'; log(f'IMU RESET 2, stays away ({st["mission"]})'); set_needs(True)
n.create_timer(0.2, tick)
start = n.create_client(Trigger, '/mower_logic/start_mowing'); start.wait_for_service()
start.call_async(Trigger.Request()); print('started', flush=True)
t_begin = time.time()
while rclpy.ok() and time.time() - t_begin < END_AFTER + 600:
    rclpy.spin_once(n, timeout_sec=0.1)
    if st['state'] == 'IDLE' and 'r2' in st['done']: log('gave up - scenario over'); break
