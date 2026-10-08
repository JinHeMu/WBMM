"""Viewer-on force/MPC integration test using the C++ keyboard publisher.

Start force_mpc.launch.py backend:=sim fake_wrench:=true keyboard_wrench:=true,
then run this script with --all-axes --output /tmp/force_mpc_oscillation.json.
It targets only the WBMM keyboard window and releases every key on exit.
Measurements include posture motion and separate fast velocity ripple so that
smooth whole-body posture adjustments are not counted as repeated shaking.
"""
import argparse, json, time, sys
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from geometry_msgs.msg import Twist, WrenchStamped
from ocs2_msgs.msg import MpcTargetTrajectories, MpcObservation
from rosgraph_msgs.msg import Clock
from tf2_ros import Buffer, TransformListener
from keyboard_test_events import Keyboard
p = argparse.ArgumentParser(description='Drive the C++ keyboard window and check force/MPC oscillation with MuJoCo viewer running.')
p.add_argument('--output', required=True)
p.add_argument('--all-axes', action='store_true')
p.add_argument('--quiet-only', action='store_true')
p.add_argument('--trace', default='')
args = p.parse_args()
rclpy.init()
n = rclpy.create_node('oscillation_integration_check')
tf = Buffer()
listener = TransformListener(tf, n)
state = {'controller': '', 'gate': '', 'q': None, 'cmd': None, 'vel': None, 'force': None, 'ref': None, 'base': [0.0, 0.0], 'sim': 0.0, 'obs': None}
rows = []
phase = 'waiting'
qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)

def js(m):
    if all(('joint_' + str(i) in m.name for i in range(1, 7))):
        ix = [m.name.index('joint_' + str(i)) for i in range(1, 7)]
        state['q'] = [m.position[i] for i in ix]
        state['vel'] = [m.velocity[i] for i in ix]

def ref(m):
    if m.state_trajectory and len(m.state_trajectory[-1].value) == 7:
        state['ref'] = list(m.state_trajectory[-1].value[:3])

def force(m):
    state['force'] = [m.wrench.force.x, m.wrench.force.y, m.wrench.force.z]
subs = [n.create_subscription(JointState, '/joint_states', js, 10), n.create_subscription(Float64MultiArray, '/arm_controller/commands', lambda m: state.update(cmd=list(m.data)), 10), n.create_subscription(String, '/whole_body_force_control/states', lambda m: state.update(controller=m.data), qos), n.create_subscription(String, '/mobile_manipulator_force_execution_state', lambda m: state.update(gate=m.data), qos), n.create_subscription(WrenchStamped, '/whole_body_force_control/processed_wrench', force, qos_profile_sensor_data), n.create_subscription(Twist, '/base_controller/cmd_vel', lambda m: state.update(base=[m.linear.x, m.angular.z]), 10), n.create_subscription(Clock, '/clock', lambda m: state.update(sim=m.clock.sec + m.clock.nanosec * 1e-09), 10), n.create_subscription(MpcTargetTrajectories, '/mobile_manipulator_ee_target', ref, 10), n.create_subscription(MpcObservation, '/mobile_manipulator_mpc_observation', lambda m: state.update(obs=list(m.state.value)), qos_profile_sensor_data)]
start = time.monotonic()
lastsample = 0
faults = set()

def spin(seconds, check=True):
    global lastsample
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.002)
        now = time.monotonic()
        if state['controller'].startswith('FAULT') or state['gate'].startswith('FAULT'):
            faults.add((phase, state['controller'], state['gate']))
        if now - lastsample < 0.01:
            continue
        lastsample = now
        if any((state[k] is None for k in ['q', 'cmd', 'vel', 'ref', 'force', 'obs'])):
            continue
        try:
            trans = tf.lookup_transform('odom', 'tool0', Time()).transform.translation
            actual = [trans.x, trans.y, trans.z]
        except Exception:
            continue
        rows.append(dict(wall=now - start, sim=state['sim'], phase=phase, q=state['q'], cmd=state['cmd'], vel=state['vel'], ref=state['ref'], tcp=actual, force=state['force'], base=state['base'], obs=state['obs']))
        if check and faults:
            raise RuntimeError(str(faults))
end = time.monotonic() + 60
while time.monotonic() < end and (not (state['controller'] == 'ACTIVE' and state['gate'] == 'ACTIVE')):
    spin(0.05, False)
assert state['controller'] == 'ACTIVE' and state['gate'] == 'ACTIVE', state
keyboard = Keyboard()
keyboard.focus()
stages = []

def stage(name, duration, key=None):
    global phase
    phase = name
    t0 = time.monotonic()
    print('START', name, flush=True)
    if key:
        keyboard.key(key, True)
    try:
        spin(duration)
    finally:
        if key:
            keyboard.key(key, False)
    a = [r for r in rows if r['phase'] == name]
    tail = a[int(len(a) * 0.5):]

    def matrix(k, data=tail):
        return np.array([r[k] for r in data])
    report = dict(name=name, samples=len(a), sim_wall_ratio=(a[-1]['sim'] - a[0]['sim']) / (a[-1]['wall'] - a[0]['wall']), q_peak_to_peak_rad=np.ptp(matrix('q'), axis=0).tolist(), command_peak_to_peak_rad=np.ptp(matrix('cmd'), axis=0).tolist(), velocity_rms_rad_s=float(np.sqrt(np.mean(matrix('vel') ** 2))), velocity_max_rad_s=float(np.max(np.abs(matrix('vel')))), joint_tracking_rms_rad=float(np.sqrt(np.mean((matrix('q') - matrix('cmd')) ** 2))), tcp_tracking_rms_m=float(np.sqrt(np.mean(np.sum((matrix('tcp') - matrix('ref')) ** 2, axis=1)))), tcp_peak_to_peak_m=np.ptp(matrix('tcp'), axis=0).tolist(), reference_peak_to_peak_m=np.ptp(matrix('ref'), axis=0).tolist(), force_mean_n=np.mean(matrix('force'), axis=0).tolist(), tcp_net_change_m=(matrix('tcp', a)[-1] - matrix('tcp', a)[0]).tolist())
    v = matrix('vel')
    smooth = np.array([np.convolve(v[:, i], np.ones(31) / 31, 'valid') for i in range(6)]).T
    report['velocity_ripple_rms_rad_s'] = float(np.sqrt(np.mean((v[15:-15] - smooth) ** 2)))
    report['reference_net_change_m'] = (matrix('ref', a)[-1] - matrix('ref', a)[0]).tolist()
    stages.append(report)
    print(json.dumps(report), flush=True)
error = None
try:
    stage('initial_zero', 12)
    if not args.quiet_only:
        keys = 'wsadrf' if args.all_axes else 'ws'
        for key in keys:
            stage('key_' + key, 3, key)
            stage('release_' + key, 8)
except Exception as e:
    error = str(e)
    print('ERROR', error, flush=True)
finally:
    for key in 'wsadrf':
        keyboard.key(key, False)
    failures = []
    if error:
        failures.append(error)
    expected = 1 if args.quiet_only else 13 if args.all_axes else 5
    if len(stages) != expected:
        failures.append('Not all stages completed')
    for stage_result in stages:
        if stage_result['name'].startswith(('initial', 'release')):
            if max(stage_result['tcp_peak_to_peak_m']) > 0.005:
                failures.append(stage_result['name'] + ': TCP oscillation exceeds 5 mm')
            if stage_result['velocity_ripple_rms_rad_s'] > 0.01:
                failures.append(stage_result['name'] + ': joint velocity ripple exceeds 0.01 rad/s')
        else:
            actual = np.array(stage_result['tcp_net_change_m'])
            desired = np.array(stage_result['reference_net_change_m'])
            if np.linalg.norm(actual) < 0.04 or np.dot(actual, desired) <= 0:
                failures.append(stage_result['name'] + ': force direction did not move TCP')
    report = dict(passed=not bool(faults) and (not failures), failures=failures, faults=sorted(faults), stages=stages)
    open(args.output, 'w').write(json.dumps(report, indent=2) + '\n')
    open(args.trace or args.output + '.jsonl', 'w').write('\n'.join((json.dumps(r) for r in rows)) + '\n')
    n.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if report['passed'] else 1)
