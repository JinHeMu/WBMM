#!/usr/bin/env python3
"""Repeatable, simulation-only keyboard force probe for arm cost ablation.

Run against force_mpc.launch.py backend:=sim fake_wrench:=true
keyboard_wrench:=true, in a private ROS_DOMAIN_ID. No hardware commands are
published. Key events target the WBMM Virtual Force window and are released
on every exit. Durations follow /clock, so slow simulation sees the SAME force
history. Metrics/FK are independently recomputed from measured OCS2 states.
Use system Python with -s: the ROS Pinocchio build needs NumPy 1.x.
"""
import argparse
import json
import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import WrenchStamped, Twist
from ocs2_msgs.msg import MpcObservation, MpcTargetTrajectories
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float64MultiArray, String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--name', required=True)
    parser.add_argument('--sequence', default='f:14,zero:4,r:6,zero:4')
    parser.add_argument('--direct-force', action='store_true',
                        help='headless: feed the virtual sensor command instead of X11 keys')
    parser.add_argument('--force-n', type=float, default=5.0)
    args = parser.parse_args()
    if not np.isfinite(args.force_n) or args.force_n <= 0:
        raise ValueError('force-n must be finite and positive')
    phases = [('initial_zero', None, 3.0)]
    for index, item in enumerate(args.sequence.split(',')):
        key, duration = item.split(':')
        if key != 'zero' and key not in 'wsadrf':
            raise ValueError('only zero or configured simulation force keys are allowed')
        phases.append((f'{index}_{key}', None if key == 'zero' else key, float(duration)))

    urdf = Path(get_package_share_directory('tracer_jaka_description')) / 'urdf/tracer_jaka_zu5.urdf'
    model = pin.buildModelFromUrdf(str(urdf))
    data = model.createData()
    frame = model.getFrameId('tool0')
    joints = [model.getJointId(f'joint_{i}') for i in range(1, 7)]
    qindices = [model.joints[j].idx_q for j in joints]
    vindices = [model.joints[j].idx_v for j in joints]
    rclpy.init()
    node = rclpy.create_node('force_mpc_arm_margin_probe')
    state = {'sim': None, 'obs': None, 'ref': None, 'force': None,
             'controller': '', 'gate': '', 'collision_seen': False, 'collision': False,
             'metrics': None, 'prediction': None, 'command': None, 'base_command': [0., 0.]}
    status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def collision(msg):
        state['collision_seen'] = True
        state['collision'] |= bool(msg.data)

    def reference(msg):
        if msg.state_trajectory:
            state['ref'] = list(msg.state_trajectory[0].value)

    subs = [
        node.create_subscription(Clock, '/clock', lambda m: state.update(sim=m.clock.sec + m.clock.nanosec * 1e-9), 10),
        node.create_subscription(MpcObservation, '/mobile_manipulator_mpc_observation', lambda m: state.update(obs=list(m.state.value), obs_time=m.time), qos_profile_sensor_data),
        node.create_subscription(MpcTargetTrajectories, '/mobile_manipulator_ee_target', reference, 10),
        node.create_subscription(WrenchStamped, '/whole_body_force_control/processed_wrench', lambda m: state.update(force=[m.wrench.force.x, m.wrench.force.y, m.wrench.force.z]), qos_profile_sensor_data),
        node.create_subscription(String, '/whole_body_force_control/states', lambda m: state.update(controller=m.data), status_qos),
        node.create_subscription(String, '/mobile_manipulator_force_execution_state', lambda m: state.update(gate=m.data), status_qos),
        node.create_subscription(Bool, '/mujoco/unexpected_collision', collision, 10),
        node.create_subscription(Float64MultiArray, '/mobile_manipulator_arm_kinematic_metrics', lambda m: state.update(metrics=list(m.data)), 10),
        node.create_subscription(Float64MultiArray, '/mobile_manipulator_arm_prediction_metrics', lambda m: state.update(prediction=list(m.data)), 10),
        node.create_subscription(Float64MultiArray, '/arm_controller/commands', lambda m: state.update(command=list(m.data)), 10),
        node.create_subscription(Twist, '/base_controller/cmd_vel', lambda m: state.update(base_command=[m.linear.x, m.angular.z]), 10),
    ]
    rows, stages, faults = [], [], set()
    keyboard = None
    command_pub = None
    force_command = WrenchStamped()
    force_command.header.frame_id = 'tool0'
    last_force_publish = -np.inf
    error = None
    last_sample = -np.inf
    started_wall = time.monotonic()
    def sample(phase):
        nonlocal last_sample
        if state['obs'] is None or len(state['obs']) not in (9, 11) or state['ref'] is None or state['sim'] is None:
            return
        if state['sim'] - last_sample < 0.02:
            return
        last_sample = state['sim']
        obs = np.array(state['obs'])
        q = pin.neutral(model)
        q[qindices] = obs[3:9]
        pin.framesForwardKinematics(model, data, q)
        tcp_local = data.oMf[frame].translation.copy()
        c, s = np.cos(obs[2]), np.sin(obs[2])
        rotation = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        tcp = rotation @ tcp_local + np.array([obs[0], obs[1], 0])
        jacobian = pin.computeFrameJacobian(model, data, q, frame, pin.LOCAL_WORLD_ALIGNED)[:, vindices]
        jacobian[3:] *= 0.3
        sigma = np.linalg.svd(jacobian, compute_uv=False)
        ref = np.array(state['ref'][:3])
        rows.append({'sim': state['sim'], 'wall': time.monotonic() - started_wall,
                     'obs_time': state['obs_time'], 'phase': phase, 'obs': obs.tolist(),
                     'tcp': tcp.tolist(), 'ref': ref.tolist(), 'error_m': float(np.linalg.norm(tcp - ref)),
                     'sigma': float(sigma[-1]), 'yoshikawa': float(np.prod(sigma)),
                     'condition': float(sigma[0] / max(sigma[-1], 1e-9)),
                     'force': state['force'], 'metrics': state['metrics'],
                     'prediction': state['prediction'], 'command': state['command'],
                     'base_command': state['base_command']})

    def spin(phase, check=True):
        nonlocal last_force_publish
        if command_pub is not None and time.monotonic() - last_force_publish >= .02:
            command_pub.publish(force_command)
            last_force_publish = time.monotonic()
        rclpy.spin_once(node, timeout_sec=0.002)
        if state['controller'].startswith('FAULT') or state['gate'].startswith('FAULT'):
            faults.add((phase, state['controller'], state['gate']))
        if check and (faults or state['collision']):
            raise RuntimeError(f'fault/collision: {sorted(faults)}, collision={state["collision"]}')
        sample(phase)

    try:
        deadline = time.monotonic() + 120
        while time.monotonic() < deadline:
            spin('startup', False)
            if (state['controller'] == state['gate'] == 'ACTIVE' and state['metrics'] is not None and
                    state['prediction'] is not None and state['collision_seen']):
                break
        else:
            raise RuntimeError(f'startup timeout: {state}')
        if args.direct_force:
            command_pub = node.create_publisher(
                WrenchStamped, '/whole_body_force_control/virtual_wrench_command', 1)
        else:
            from keyboard_test_events import Keyboard
            keyboard = Keyboard()
            keyboard.focus()
        for name, key, duration in phases:
            t0, wall0 = state['sim'], time.monotonic()
            print(f'START {args.name} {name} {duration} simulation seconds', flush=True)
            if key:
                if keyboard:
                    keyboard.key(key, True)
                else:
                    axis, sign = {'w': ('x', 1), 's': ('x', -1), 'a': ('y', 1),
                                  'd': ('y', -1), 'r': ('z', 1), 'f': ('z', -1)}[key]
                    setattr(force_command.wrench.force, axis, sign * args.force_n)
            try:
                while state['sim'] - t0 < duration:
                    spin(name)
                    if time.monotonic() - wall0 > max(60, 10 * duration):
                        raise RuntimeError('simulation clock stalled')
            finally:
                if key:
                    if keyboard:
                        keyboard.key(key, False)
                    else:
                        force_command.wrench.force.x = 0.0
                        force_command.wrench.force.y = 0.0
                        force_command.wrench.force.z = 0.0
                        command_pub.publish(force_command)
            selected = [r for r in rows if r['phase'] == name]
            if len(selected) < 10:
                raise RuntimeError('insufficient measured samples')
            stages.append({'name': name, 'key': key, 'duration_s': duration,
                           'samples': len(selected), 'min_sigma': min(r['sigma'] for r in selected),
                           'min_yoshikawa': min(r['yoshikawa'] for r in selected),
                           'tracking_rms_m': float(np.sqrt(np.mean([r['error_m'] ** 2 for r in selected]))),
                           'tcp_change_m': (np.array(selected[-1]['tcp']) - selected[0]['tcp']).tolist(),
                           'end_q': selected[-1]['obs'][3:9]})
            forces = [r['force'] for r in selected if r['force'] is not None]
            mean_force = np.mean(forces, axis=0) if forces else np.zeros(3)
            stages[-1]['mean_force_tcp_n'] = mean_force.tolist()
            if key:
                axis, sign = {'w': (0, 1), 's': (0, -1), 'a': (1, 1),
                              'd': (1, -1), 'r': (2, 1), 'f': (2, -1)}[key]
                if sign * mean_force[axis] < 1.0 or np.linalg.norm(stages[-1]['tcp_change_m']) < 0.01:
                    raise RuntimeError('virtual force or measured TCP motion was not observed')
            print(json.dumps(stages[-1]), flush=True)
    except Exception as exc:
        error = str(exc)
    finally:
        if keyboard:
            for key in 'wsadrf':
                keyboard.key(key, False)
        if command_pub is not None:
            command_pub.publish(WrenchStamped(header=force_command.header))
        valid = [r for r in rows if r['phase'] != 'startup']
        prediction = [r['prediction'] for r in valid if r['prediction'] and r['prediction'][7] == 1]
        report = {'name': args.name, 'passed': error is None and len(stages) == len(phases),
                  'error': error, 'faults': sorted(faults), 'unexpected_collision': state['collision'],
                  'collision_monitor_seen': state['collision_seen'], 'stages': stages,
                  'jacobian': {'scope': 'arm', 'task': 'pose', 'scaling': 'characteristic_length', 'ell_m': 0.3},
                  'sequence': args.sequence,
                  'metrics_are_diagnostic_only': all(r['metrics'] is not None and len(r['metrics']) == 6 for r in valid)}
        if valid:
            report.update(min_sigma=min(r['sigma'] for r in valid),
                          min_yoshikawa=min(r['yoshikawa'] for r in valid),
                          tracking_rms_m=float(np.sqrt(np.mean([r['error_m'] ** 2 for r in valid]))),
                          maximum_tracking_error_m=max(r['error_m'] for r in valid),
                          end_state=valid[-1]['obs'],
                          predicted_horizon_s=prediction[-1][1] if prediction else None,
                          solve_ms_p95=float(np.percentile([p[8] for p in prediction], 95)) if prediction else None)
        path = Path(args.output)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(json.dumps(report, indent=2) + '\n')
        path.with_suffix('.jsonl').write_text(''.join(json.dumps(r) + '\n' for r in rows))
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
