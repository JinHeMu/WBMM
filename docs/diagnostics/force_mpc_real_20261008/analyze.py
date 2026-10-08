#!/usr/bin/env python3
"""Offline calculations only; no ROS, SDK, network, or hardware commands.

The scalar model illustrates a mechanism. It is not an OCS2 replay, a fitted
JAKA transfer function, or proof of real-robot stability.
"""
import csv
import hashlib
import json
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
URDF = ROOT / 'src/robotics/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf'
CALIBRATION = ROOT / 'src/bringup/config/real/force_mpc_calibration.yaml'


def axis_rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    skew = np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * skew @ skew


def rpy_rotation(angles):
    rx, ry, rz = angles
    return (axis_rotation([0, 0, 1], rz) @ axis_rotation([0, 1, 0], ry)
            @ axis_rotation([1, 0, 0], rx))


def forward(q):
    tree = ET.parse(URDF)
    joints = {j.find('child').get('link'): j for j in tree.findall('joint')}
    chain, link = [], 'tool0'
    while link != 'jaka_base_link':
        joint = joints[link]
        chain.append(joint)
        link = joint.find('parent').get('link')
    rotation, position, origins, axes = np.eye(3), np.zeros(3), [], []
    for joint in reversed(chain):
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz', '0 0 0'), sep=' ')
        angles = np.fromstring(origin.get('rpy', '0 0 0'), sep=' ')
        position += rotation @ xyz
        rotation = rotation @ rpy_rotation(angles)
        if joint.get('type') in ('revolute', 'continuous'):
            axis = np.fromstring(joint.find('axis').get('xyz'), sep=' ')
            axis /= np.linalg.norm(axis)
            origins.append(position.copy())
            axes.append(rotation @ axis)
            index = int(joint.get('name').split('_')[-1]) - 1
            rotation = rotation @ axis_rotation(axis, q[index])
    jacobian = np.column_stack([
        np.r_[np.cross(axis, position - origin), axis]
        for origin, axis in zip(origins, axes)])
    return position, rotation, jacobian


def scalar_model(gain, delay, mode):
    dt, tau, horizon = .008, 1 / (2 * math.pi * 2), .1
    delay_steps = round(delay / dt)
    measured, command = .001, .001
    history, samples = [measured] * (delay_steps + 1), []
    for _ in range(2500):
        feedback = history[-1 - delay_steps]
        if mode == 'velocity_integral':
            velocity = np.clip(-gain * feedback, -.2, .2)
            command = np.clip(command + dt * velocity, feedback - .05, feedback + .05)
        else:
            # Ideal scalar integrator/LQR future position, not actual MPC.
            target = feedback * math.exp(-gain * horizon)
            lower = max(command - dt * .2, feedback - .05)
            upper = min(command + dt * .2, feedback + .05)
            if lower > upper:
                raise ValueError('Disjoint rate/feedback command envelopes')
            command = np.clip(target, lower, upper)
        measured += -math.expm1(-dt / tau) * (command - measured)
        history.append(measured)
        samples.append(measured)
    return {'mode': mode, 'gain_s_inv': gain, 'assumed_delay_s': delay,
            'first_1s_peak_to_peak_rad': float(np.ptp(samples[:125])),
            'last_1s_peak_to_peak_rad': float(np.ptp(samples[-125:]))}


def collision_distances(q):
    # Optional independent geometry check using the same URDF and link pairs.
    try:
        import pinocchio as pin
    except ImportError:
        return {'available': False}
    model = pin.buildModelFromUrdf(str(URDF))
    geometry = pin.buildGeomFromUrdf(
        model, str(URDF), pin.GeometryType.COLLISION,
        package_dirs=[str(ROOT / 'install/tracer_jaka_description/share')])
    configuration = pin.neutral(model)
    for i in range(6):
        joint = model.joints[model.getJointId(f'joint_{i + 1}')]
        configuration[joint.idx_q] = q[i]
    task = (ROOT / 'src/bringup/config/real/task.info').read_text()
    start = task.index('collisionLinkPairs')
    section = task[start:task.index('minimumDistance', start)]
    for first, second in re.findall(r'"([^",]+),\s*([^"\n]+)"', section):
        first_id, second_id = model.getFrameId(first), model.getFrameId(second)
        for i, obj in enumerate(geometry.geometryObjects):
            for j, other in enumerate(geometry.geometryObjects):
                if obj.parentFrame == first_id and other.parentFrame == second_id:
                    geometry.addCollisionPair(pin.CollisionPair(i, j))
    data, geometry_data = model.createData(), geometry.createData()
    pin.updateGeometryPlacements(model, data, geometry, geometry_data, configuration)
    pin.computeDistances(geometry, geometry_data)
    distances = sorted([
        {'distance_m': float(result.min_distance),
         'first': geometry.geometryObjects[pair.first].name,
         'second': geometry.geometryObjects[pair.second].name}
        for pair, result in zip(geometry.collisionPairs, geometry_data.distanceResults)
    ], key=lambda item: item['distance_m'])
    return {'available': True, 'pairs': len(distances), 'nearest': distances[:8],
            'within_joint_limits': bool(np.all(configuration >= model.lowerPositionLimit)
                                        and np.all(configuration <= model.upperPositionLimit))}


def main():
    with (HERE / 'status.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    q = np.array([[float(row[f'q{i}']) for i in range(1, 7)] for row in rows])
    poses = [forward(configuration) for configuration in q]
    calibration = yaml.safe_load(CALIBRATION.read_text())[
        'force_sensor_processor']['ros__parameters']['force_sensor']
    rotation = np.array(calibration['sensor_to_tcp_rotation']).reshape(3, 3)
    load = calibration['load_compensation']
    gravity_direction = np.array(load['gravity_direction_base'])
    gravity_direction /= np.linalg.norm(gravity_direction)
    gravity_base = load['mass_kg'] * load['gravity_m_s2'] * gravity_direction
    gravity_sensor = np.array([rotation.T @ pose[1].T @ gravity_base for pose in poses])
    bias = np.array(load['force_bias_sensor_n'])
    result = {
        'hardware_write_performed': False,
        'rows': len(rows), 'sample_span_s': float(rows[-1]['t']) - float(rows[0]['t']),
        'max_joint_span_rad': np.ptp(q, axis=0).tolist(),
        'tcp_position_in_jaka_base_m': poses[0][0].tolist(),
        'predicted_bias_plus_gravity_sensor_n': (bias + gravity_sensor.mean(axis=0)).tolist(),
        'jacobian_singular_values_mixed_units': np.linalg.svd(poses[0][2], compute_uv=False).tolist(),
        'edg_channel_correspondence_confirmed': False,
        'channels': {},
        'file_sha256': {str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
                        for path in [URDF, CALIBRATION, HERE / 'status.csv']},
    }
    for prefix, name in [('raw', 'sdk_torque_original'), ('act', 'sdk_actTorque')]:
        force = np.array([[float(row[f'{prefix}_{axis}']) for axis in ['fx', 'fy', 'fz']]
                          for row in rows])
        residual = (force - bias - gravity_sensor) @ rotation.T
        mean = residual.mean(axis=0)
        result['channels'][name] = {
            'raw_mean_n': force.mean(axis=0).tolist(), 'raw_std_n': force.std(axis=0).tolist(),
            'residual_mean_tool0_n': mean.tolist(), 'residual_norm_n': float(np.linalg.norm(mean)),
            'conditional_velocity_at_damping200_m_s': (np.where(abs(mean) < 1., 0., mean) / 200).tolist(),
        }
    jac = poses[0][2]
    eigenvalues = np.linalg.eigvalsh(jac.T @ np.diag([100, 100, 100, 25, 25, 25]) @ jac / .01)
    result['approximate_fixed_base_LQR_modal_gains_s_inv'] = np.sqrt(eigenvalues).tolist()
    result['illustrative_model_not_real_mpc_replay'] = [
        scalar_model(gain, delay, mode)
        for delay in [0., .024, .04] for gain in [10, 20, 40, 60]
        for mode in ['velocity_integral', 'predicted_position']]
    result['static_model_collision'] = collision_distances(q[0])
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
