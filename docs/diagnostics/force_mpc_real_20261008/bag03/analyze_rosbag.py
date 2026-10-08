#!/usr/bin/env python3
"""Read sqlite/CDR offline. Never initializes ROS, publishes, or plays a bag.

Usage (after sourcing ROS/workspace): python3 analyze_rosbag.py BAG_DIR OUTPUT_DIR
Recorded-policy position commands are previews, not a closed-loop validation.
"""
import collections
import csv
import hashlib
import importlib.util
import json
from pathlib import Path
import sqlite3
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import find_peaks
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('offline_fk', HERE.parent / 'analyze.py')
fk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fk)


def main(bag, out):
    out.mkdir(parents=True, exist_ok=True)
    db = next(bag.glob('*.db3'))
    connection = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    topics = {i: (name, get_message(typ)) for i, name, typ in
              connection.execute('select id,name,type from topics')}
    start, end = connection.execute('select min(timestamp),max(timestamp) from messages').fetchone()
    data = collections.defaultdict(list)
    for tid, stamp, blob in connection.execute('select topic_id,timestamp,data from messages order by timestamp'):
        name, typ = topics[tid]
        data[name].append(((stamp - start) * 1e-9, deserialize_message(blob, typ)))
    connection.close()

    def series(topic, extract):
        rows = data[topic]
        return np.array([t for t, _ in rows]), np.array([extract(m) for _, m in rows])

    def interpolate(times, source_times, values):
        return np.column_stack([np.interp(times, source_times, values[:, i])
                                for i in range(values.shape[1])])

    tq, q = series('/joint_states', lambda m: [m.position[m.name.index(f'joint_{i}')]
                                              for i in range(1, 7)])
    _, velocity = series('/joint_states', lambda m: [m.velocity[m.name.index(f'joint_{i}')]
                                                    for i in range(1, 7)])
    tc, command = series('/arm_controller/commands', lambda m: m.data)
    tw, wrench = series('/whole_body_force_control/processed_wrench',
                        lambda m: [m.wrench.force.x, m.wrench.force.y, m.wrench.force.z])
    tr, correction = series('/whole_body_force_control/correction', lambda m: m.data)
    tt, target = series('/mobile_manipulator_ee_target', lambda m: m.state_trajectory[0].value)
    to, observation = series('/mobile_manipulator_mpc_observation', lambda m: m.state.value)
    control_time = np.array([m.time for _, m in data['/mobile_manipulator_mpc_observation']])
    tb, base_command = series('/cmd_vel', lambda m: [m.linear.x, m.angular.z])
    _, odom = series('/wheel/odometry', lambda m: [m.pose.pose.position.x, m.pose.pose.position.y,
                                                m.pose.pose.orientation.z, m.pose.pose.orientation.w])
    # Same URDF fixed base-to-JAKA transforms as the recorded model.
    world_position = []
    for state in observation:
        local_position, _, _ = fk.forward(state[3:])
        world_R_jaka = fk.rpy_rotation([0, 0, state[2] - 1.57])
        world_position.append(world_R_jaka @ local_position + [state[0], state[1], .147 + .221])
    world_position = np.array(world_position)
    target_at = interpolate(to, tt, target[:, :3])
    first_force = float(tw[np.any(wrench != 0, axis=1)][0])
    first_correction = float(tr[np.any(correction[:, 15:21] != 0, axis=1)][0])

    windows = []
    for lower, upper in [(.08, 1.08), (1.08, 2.08), (2.08, 2.65), (2.4, 3.4)]:
        windows.append({'start_s': lower, 'end_s': upper,
                        'measured_joint_peak_to_peak_rad': np.ptp(q[(tq >= lower) & (tq < upper)], axis=0).tolist(),
                        'command_peak_to_peak_rad': np.ptp(command[(tc >= lower) & (tc < upper)], axis=0).tolist()})
    fits = []
    for joint in [1, 2, 3]:
        mask = (tq > .18) & (tq < 2.65)
        times, measured, measured_velocity = tq[mask], q[mask, joint], velocity[mask, joint]
        best = None
        for delay in np.arange(0, .101, .001):
            lead = np.interp(times - delay, tc, command[:, joint]) - measured
            rate = np.dot(lead, measured_velocity) / np.dot(lead, lead)
            if rate <= 0:
                continue
            predicted = rate * lead
            rmse = float(np.sqrt(np.mean((measured_velocity - predicted) ** 2)))
            if best is None or rmse < best['velocity_rmse_rad_s']:
                best = {'joint': joint + 1, 'assumed_model': 'qdot=(qcmd(t-delay)-q)/tau',
                        'delay_s': float(delay), 'tau_s': float(1 / rate),
                        'velocity_rmse_rad_s': rmse, 'velocity_correlation': float(np.corrcoef(predicted, measured_velocity)[0, 1])}
        fits.append(best)
    peaks, _ = find_peaks(q[:, 1], prominence=.0003, distance=8)
    periods = np.diff(tq[peaks][tq[peaks] < 2.65])
    period = float(np.median(periods))

    # Approximate which wire policy was available at each command timestamp.
    policies = data['/mobile_manipulator_mpc_policy']
    policy_arrival = np.array([t for t, _ in policies])
    times, queries, future, policy_input, age, observed_command = [], [], [], [], [], []
    for t, message in data['/arm_controller/commands']:
        index = np.searchsorted(policy_arrival, t, side='right') - 1
        if index < 0:
            continue
        policy = policies[index][1]
        query = control_time[np.argmin(abs(to - t))]
        trajectory_time = np.array(policy.time_trajectory)
        inputs = np.array([v.value for v in policy.input_trajectory])
        states = np.array([v.value for v in policy.state_trajectory])
        horizon = min(query + .1, trajectory_time[-1] - .001)
        future.append([np.interp(horizon, trajectory_time, states[:, i]) for i in range(3, 9)])
        policy_input.append([np.interp(query, trajectory_time, inputs[:, i]) for i in range(8)])
        times.append(t); queries.append(query); age.append(query - policy.init_observation.time)
        observed_command.append(message.data)
    times, queries, future, policy_input = map(np.array, [times, queries, future, policy_input])
    measured_at = interpolate(times, tq, q)
    preview, previous, previous_time = [], measured_at[0], queries[0]
    for measured, desired, query in zip(measured_at, future, queries):
        dt = np.clip(query - previous_time, 0, .05)
        lower = np.maximum(previous - .2 * dt, measured - .05)
        upper = np.minimum(previous + .2 * dt, measured + .05)
        if np.any(lower > upper):
            raise ValueError('Preview rate/feedback envelopes became disjoint')
        previous = np.clip(desired, lower, upper)
        preview.append(previous.copy()); previous_time = query
    preview = np.array(preview)
    summary = {
        'bag': str(bag), 'sqlite_sha256': hashlib.sha256(db.read_bytes()).hexdigest(),
        'recording_duration_s': (end - start) * 1e-9,
        'topic_counts': {name: len(rows) for name, rows in data.items()},
        'states': {name: dict(collections.Counter(m.data for _, m in data[name])) for name in
                   ['/whole_body_force_control/states', '/whole_body_force_control/force_sensor_states',
                    '/mobile_manipulator_force_execution_state']},
        'first_nonzero_processed_force_s': first_force,
        'first_nonzero_admittance_correction_s': first_correction,
        'target_before_first_correction_peak_to_peak': np.ptp(target[tt < first_correction], axis=0).tolist(),
        'target_full_position_peak_to_peak_mm': (np.ptp(target[:, :3], axis=0) * 1000).tolist(),
        'measured_tcp_peak_to_peak_mm': (np.ptp(world_position, axis=0) * 1000).tolist(),
        'max_tcp_tracking_error_mm': float(np.max(np.linalg.norm(world_position - target_at, axis=1)) * 1000),
        'joint_windows': windows,
        'dominant_joint2_period_s': period, 'dominant_joint2_frequency_hz': 1 / period,
        'first_order_fits_from_ros_only': fits,
        'max_joint_command_lead_rad': np.max(abs(command - interpolate(tc, tq, q)), axis=0).tolist(),
        'policy_input_reconstruction_is_approximate': True,
        'estimated_policy_age_mean_s': float(np.mean(age)), 'estimated_policy_age_max_s': float(np.max(age)),
        'estimated_policy_arm_velocity_max_rad_s': np.max(abs(policy_input[:, 2:]), axis=0).tolist(),
        'estimated_velocity_saturation_fraction': np.mean(abs(policy_input[:, 2:]) > .2, axis=0).tolist(),
        'first_estimated_arm_velocity_saturation_s': float(times[np.any(abs(policy_input[:, 2:]) > .2, axis=1)][0]),
        'recorded_command_peak_to_peak_rad': np.ptp(command, axis=0).tolist(),
        'recorded_policy_position_preview_peak_to_peak_rad': np.ptp(preview, axis=0).tolist(),
        'position_preview_is_not_closed_loop_validation': True,
        'base_command_max_abs': np.max(abs(base_command), axis=0).tolist(), 'wheel_odom_peak_to_peak': np.ptp(odom, axis=0).tolist(),
        'nominal_capture_log_original_stamp_relative_s': [
            (m.stamp.sec * 10**9 + m.stamp.nanosec - start) * 1e-9
            for _, m in data['/rosout'] if 'Captured nominal TCP' in m.msg],
    }
    (out / 'summary.json').write_text(json.dumps(summary, indent=2))
    with (out / 'aligned_joint3.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['bag_time_s', 'joint3_command_rad', 'joint3_measured_rad',
                         'joint3_recorded_policy_position_preview_rad', 'estimated_mpc_joint3_velocity_rad_s'])
        writer.writerows(zip(times, np.array(observed_command)[:, 2], measured_at[:, 2], preview[:, 2], policy_input[:, 4]))

    fig, axes = plt.subplots(4, 1, figsize=(11, 10), sharex=True, constrained_layout=True)
    for ax in axes:
        ax.axvspan(0, first_correction, color='#dce8e3', alpha=.35)
        ax.axvline(first_correction, color='#7755aa', ls='--', lw=1)
        ax.grid(alpha=.22)
    axes[0].plot(to, (world_position[:, 2] - target[0, 2]) * 1000, label='Measured TCP Z')
    axes[0].plot(tt, (target[:, 2] - target[0, 2]) * 1000, label='Admittance target Z', color='#dc752a')
    axes[0].set_ylabel('Z deviation (mm)'); axes[0].legend(loc='upper left')
    axes[0].set_title('Real ROS bag 03: motion grows while admittance target is fixed (tare=true)')
    axes[1].plot(tc, (command[:, 2] - q[0, 2]) * 1000, label='Recorded joint 3 command')
    axes[1].plot(tq, (q[:, 2] - q[0, 2]) * 1000, label='Measured joint 3')
    axes[1].plot(times, (preview[:, 2] - q[0, 2]) * 1000, label='Position command preview (recorded policies)', color='#31916a')
    axes[1].set_ylabel('Joint 3 (mrad)'); axes[1].legend(loc='upper left', fontsize=8)
    axes[2].plot(tw, wrench[:, 2], label='Processed force Z (tool0)')
    axes[2].plot(tr, correction[:, 11], label='Admittance force Z (fixed axes)', alpha=.7)
    axes[2].set_ylabel('Force (N)'); axes[2].legend(loc='upper left')
    axes[3].plot(times, policy_input[:, 4], label='Estimated current MPC qdot3')
    axes[3].plot(tq, velocity[:, 2], label='Measured joint 3 velocity')
    axes[3].axhline(.2, color='#ba5555', ls=':', label='Command velocity limits')
    axes[3].axhline(-.2, color='#ba5555', ls=':')
    axes[3].set_ylabel('Velocity (rad/s)'); axes[3].set_xlabel('Seconds since bag started')
    axes[3].legend(loc='upper left', fontsize=8)
    fig.savefig(out / 'ros_chain.png', dpi=170)
    plt.close(fig)
    print(json.dumps({key: summary[key] for key in ['recording_duration_s', 'first_nonzero_processed_force_s',
          'dominant_joint2_frequency_hz', 'measured_tcp_peak_to_peak_mm', 'max_tcp_tracking_error_mm',
          'recorded_policy_position_preview_peak_to_peak_rad']}, indent=2))


if __name__ == '__main__':
    main(Path(sys.argv[1]), Path(sys.argv[2]))
