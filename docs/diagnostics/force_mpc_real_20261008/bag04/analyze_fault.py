#!/usr/bin/env python3
"""Offline sqlite/CDR fault analysis. No ROS init, playback or publishing."""
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
import yaml
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('offline_fk', HERE.parent / 'analyze.py')
fk = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fk)


def main(bag, out):
    out.mkdir(parents=True, exist_ok=True)
    db = next(bag.glob('*.db3'))
    conn = sqlite3.connect(f'file:{db}?mode=ro', uri=True)
    topics = {i: (name, get_message(typ)) for i, name, typ in
              conn.execute('select id,name,type from topics')}
    start, end = conn.execute('select min(timestamp),max(timestamp) from messages').fetchone()
    data = collections.defaultdict(list)
    for tid, stamp, blob in conn.execute('select topic_id,timestamp,data from messages order by timestamp'):
        name, typ = topics[tid]
        data[name].append(((stamp - start) * 1e-9, deserialize_message(blob, typ)))
    conn.close()

    def series(topic, extract):
        return (np.array([t for t, _ in data[topic]]),
                np.array([extract(m) for _, m in data[topic]]))

    def interp(times, source_times, values):
        return np.column_stack([np.interp(times, source_times, values[:, i])
                                for i in range(values.shape[1])])

    def wrench(m):
        return [m.wrench.force.x, m.wrench.force.y, m.wrench.force.z,
                m.wrench.torque.x, m.wrench.torque.y, m.wrench.torque.z]

    tq, q = series('/joint_states', lambda m: [m.position[m.name.index(f'joint_{i}')]
                                              for i in range(1, 7)])
    _, velocity = series('/joint_states', lambda m: [m.velocity[m.name.index(f'joint_{i}')]
                                                    for i in range(1, 7)])
    jq_stamp = np.array([(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec - start) * 1e-9
                         for _, m in data['/joint_states']])
    tc, command = series('/arm_controller/commands', lambda m: m.data)
    tw, processed = series('/whole_body_force_control/processed_wrench', wrench)
    tw_stamp = np.array([(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec - start) * 1e-9
                         for _, m in data['/whole_body_force_control/processed_wrench']])
    tx, raw = series('/fts_broadcaster/wrench', wrench)
    tx_stamp = np.array([(m.header.stamp.sec * 10**9 + m.header.stamp.nanosec - start) * 1e-9
                         for _, m in data['/fts_broadcaster/wrench']])
    tr, correction = series('/whole_body_force_control/correction', lambda m: m.data)
    tt, target = series('/mobile_manipulator_ee_target', lambda m: m.state_trajectory[0].value)
    to, observation = series('/mobile_manipulator_mpc_observation', lambda m: m.state.value)
    control_time = np.array([m.time for _, m in data['/mobile_manipulator_mpc_observation']])
    command_control_time = np.array([control_time[np.argmin(abs(to - t))] for t in tc])
    tb, basecmd = series('/cmd_vel', lambda m: [m.linear.x, m.angular.z])
    odomt, odom = series('/wheel/odometry', lambda m: [m.pose.pose.position.x, m.pose.pose.position.y,
                                                    m.pose.pose.orientation.z, m.pose.pose.orientation.w])
    world_position = []
    world_rotations = []
    for state in observation:
        local_position, local_rotation, _ = fk.forward(state[3:])
        world_R_jaka = fk.rpy_rotation([0, 0, state[2] - 1.57])
        world_position.append(world_R_jaka @ local_position + [state[0], state[1], .147 + .221])
        world_rotations.append(world_R_jaka @ local_rotation)
    world_position, world_rotations = np.array(world_position), np.array(world_rotations)

    states = {}
    for name in ['/whole_body_force_control/states', '/whole_body_force_control/force_sensor_states',
                 '/mobile_manipulator_force_execution_state']:
        transitions = []
        for t, m in data[name]:
            if not transitions or transitions[-1]['state'] != m.data:
                transitions.append({'receipt_s': t, 'state': m.data})
        states[name] = transitions
    logs = [{'receipt_s': t, 'event_s': (m.stamp.sec * 10**9 + m.stamp.nanosec - start) * 1e-9,
             'level': m.level, 'node': m.name, 'message': m.msg} for t, m in data['/rosout']
            if m.name != 'rosbag2_recorder']
    sensor_fault = next(s['receipt_s'] for s in states['/whole_body_force_control/force_sensor_states']
                        if s['state'].startswith('FAULT'))
    force_fault = next(s['receipt_s'] for s in states['/whole_body_force_control/states']
                       if s['state'].startswith('FAULT'))
    gate_fault = next(s['receipt_s'] for s in states['/mobile_manipulator_force_execution_state']
                      if s['state'].startswith('FAULT'))
    ci = int(np.searchsorted(tc, gate_fault)) - 1  # Command is published before execution state.
    faultcommand = command[ci]
    jump = command[ci] - command[ci-1]
    nominal_cycle_speed = jump / .008
    velocity_fits = []
    for joint in [1, 2, 3]:
        mask = (tq > tc[ci] + .008) & (tq < tc[ci] + .220)
        times, values = tq[mask], velocity[mask, joint]
        initial = float(np.median(velocity[(tq > tc[ci]) & (tq < tc[ci] + .020), joint]))
        def predict(params):
            onset, tau, final = params
            return np.where(times < onset, initial, final + (initial - final) *
                            np.exp(-np.maximum(0, times - onset) / tau))
        result = least_squares(lambda params: predict(params) - values,
                               [tc[ci] + .024, .08, nominal_cycle_speed[joint]],
                               bounds=([tc[ci], .02, -3], [tc[ci] + .048, .2, 3]))
        velocity_fits.append({'joint': joint+1, 'model': 'v=v_inf+(v0-v_inf)*exp(-(t-onset)/tau)',
                              'onset_s': float(result.x[0]), 'tau_s': float(result.x[1]),
                              'asymptotic_velocity_rad_s': float(result.x[2]),
                              'command_jump_divided_by_8ms_rad_s': float(nominal_cycle_speed[joint]),
                              'rmse_rad_s': float(np.sqrt(np.mean((predict(result.x)-values)**2)))})
    q_at_fault = interp([tc[ci]], tq, q)[0]
    post = tc >= tc[ci]
    step = np.diff(command, axis=0)
    receipt_speed = step / np.diff(tc)[:, None]
    active = (tc[1:] > 2.01) & (tc[1:] < tc[ci])
    tcp_at_fault = interp([gate_fault], to, world_position)[0]
    windows = []
    for lower, upper in [(2.1, 5), (28, 30), (30, sensor_fault),
                         (gate_fault, gate_fault + .1), (gate_fault, gate_fault + .5),
                         (gate_fault, to[-1])]:
        qm = (tq >= lower) & (tq <= upper)
        pm = (to >= lower) & (to <= upper)
        windows.append({'start_s': lower, 'end_s': upper,
                        'q_peak_to_peak_rad': np.ptp(q[qm], axis=0).tolist(),
                        'max_abs_joint_velocity_rad_s': np.max(abs(velocity[qm]), axis=0).tolist(),
                        'max_tcp_distance_from_gate_fault_mm': float(np.max(np.linalg.norm(
                            world_position[pm] - tcp_at_fault, axis=1)) * 1000)})
    snapshots = []
    for t in [28., 29., 30., 30.5, 30.9, 31., sensor_fault, gate_fault,
              gate_fault + .04, gate_fault + .1, gate_fault + .2, gate_fault + .5,
              32., 33., 34., 35.]:
        snapshots.append({'time_s': t, 'q_rad': interp([t], tq, q)[0].tolist(),
                          'qcmd_rad': interp([t], tc, command)[0].tolist(),
                          'qdot_rad_s': interp([t], tq, velocity)[0].tolist(),
                          'tcp_m': interp([t], to, world_position)[0].tolist(),
                          'target_m': interp([t], tt, target[:, :3])[0].tolist(),
                          'raw_sensor_wrench': interp([t], tx, raw)[0].tolist()})
    summary = {'bag': str(bag), 'sqlite_sha256': hashlib.sha256(db.read_bytes()).hexdigest(),
               'start_epoch_ns': start, 'duration_s': (end - start) * 1e-9,
               'topic_counts': {name: len(rows) for name, rows in data.items()},
               'state_transitions': states, 'logs': logs,
               'sensor_fault_receipt_s': sensor_fault, 'force_fault_receipt_s': force_fault,
               'gate_fault_receipt_s': gate_fault, 'first_hold_command_receipt_s': float(tc[ci]),
               'hold_command_rad': faultcommand.tolist(),
               'last_active_command_rad': command[ci-1].tolist(),
               'hold_transition_command_jump_rad': (command[ci]-command[ci-1]).tolist(),
               'velocity_step_fits': velocity_fits,
               'hold_command_minus_interpolated_feedback_rad': (faultcommand-q_at_fault).tolist(),
               'post_gate_command_peak_to_peak_rad': np.ptp(command[post], axis=0).tolist(),
               'post_gate_cmd_vel_max_abs': np.max(abs(basecmd[tb >= gate_fault]), axis=0).tolist(),
               'max_active_command_step_rad': np.max(abs(step[active]), axis=0).tolist(),
               'max_active_receipt_based_command_speed_rad_s': np.max(abs(receipt_speed[active]), axis=0).tolist(),
               'max_active_command_feedback_lead_rad': np.max(abs(command[tc < tc[ci]] -
                    interp(tc[tc < tc[ci]], tq, q)), axis=0).tolist(),
               'last_processed_receipt_s': float(tw[-1]), 'last_processed_header_s': float(tw_stamp[-1]),
               'last_processed_wrench': processed[-1].tolist(),
               'post_fault_max_joint_deviation_from_hold_rad': np.max(abs(q[tq >= gate_fault]-faultcommand), axis=0).tolist(),
               'final_feedback_minus_hold_rad': (q[-1] - faultcommand).tolist(),
               'joint_state_max_header_gap_s': float(np.max(np.diff(jq_stamp))),
               'joint_state_max_receipt_gap_s': float(np.max(np.diff(tq))),
               'wheel_odom_peak_to_peak': np.ptp(odom, axis=0).tolist(),
               'windows': windows, 'snapshots': snapshots}
    (out/'summary.json').write_text(json.dumps(summary, indent=2))
    with (out/'command_rows.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['receipt_s', 'control_time_s', 'fault_hold'] +
                        [f'qcmd{i}' for i in range(1,7)] + [f'q{i}' for i in range(1,7)] +
                        [f'qdot{i}' for i in range(1,7)])
        # Prior feedback sample, not future interpolation, approximates what
        # the independently clocked hardware writer can know at each command.
        for index, t in enumerate(tc):
            feedback_index = max(0, np.searchsorted(tq, t, side='right')-1)
            writer.writerow([t, command_control_time[index], int(index >= ci),
                             *command[index], *q[feedback_index], *velocity[feedback_index]])
    np.savez_compressed(out/'series.npz', tq=tq, q=q, qdot=velocity, jq_stamp=jq_stamp,
                        tc=tc, command=command, tw=tw, processed=processed, tw_stamp=tw_stamp,
                        tx=tx, raw=raw, tx_stamp=tx_stamp, tr=tr, correction=correction,
                        tt=tt, target=target, to=to, observation=observation,
                        tcp=world_position, tcp_rotation=world_rotations, tb=tb, basecmd=basecmd)
    with (out/'last_seconds.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['receipt_s'] + [f'q{i}_rad' for i in range(1,7)] +
                        [f'qcmd{i}_rad' for i in range(1,7)] + [f'qdot{i}_rad_s' for i in range(1,7)] +
                        ['tcp_x_m','tcp_y_m','tcp_z_m'])
        mask = tq > 28
        for t, joint, cmd, vel, tcp in zip(tq[mask], q[mask], interp(tq[mask], tc, command),
                                          velocity[mask], interp(tq[mask], to, world_position)):
            writer.writerow([t, *joint, *cmd, *vel, *tcp])
    for lower, upper, name in [(28, 35.2, 'last_seconds.png'), (30.9, 31.7, 'fault_transition.png')]:
        fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True, constrained_layout=True)
        for ax in axes:
            ax.axvline(sensor_fault, color='#a33', ls='--', label='Sensor WRENCH_LIMIT')
            ax.axvline(gate_fault, color='#557', ls=':', label='MRT hold')
            ax.grid(alpha=.2)
        axes[0].plot(tx_stamp, np.linalg.norm(raw[:, :3], axis=1), label='Raw force norm (sensor, uncompensated)')
        axes[0].plot(tw_stamp, np.linalg.norm(processed[:, :3], axis=1), label='Processed force norm (TCP, filtered)')
        axes[0].set_ylabel('Force (N)'); axes[0].legend(fontsize=8)
        for i, label in enumerate(['X','Y','Z']):
            axes[1].plot(to, (world_position[:,i]-tcp_at_fault[i])*1000, label=f'Measured {label}')
            axes[1].plot(tt, (target[:,i]-tcp_at_fault[i])*1000, ls='--', label=f'Target {label}')
        axes[1].set_ylabel('TCP offset (mm)'); axes[1].legend(fontsize=8, ncol=3)
        for i in [1, 2, 3]:
            axes[2].plot(tq, (q[:,i]-faultcommand[i])*1000, label=f'Measured J{i+1}')
            axes[2].plot(tc, (command[:,i]-faultcommand[i])*1000, ls='--', label=f'Command J{i+1}')
        axes[2].set_ylabel('Joint offset (mrad)'); axes[2].legend(fontsize=8, ncol=3)
        for i in range(6):
            axes[3].plot(tq, velocity[:,i], label=f'J{i+1}')
        axes[3].set_ylabel('Measured qdot (rad/s)'); axes[3].legend(fontsize=8, ncol=6)
        for i in [1,2,3]:
            axes[4].plot(tc[1:], receipt_speed[:,i], label=f'J{i+1} command finite difference')
        axes[4].axhline(.2, color='#777', ls=':'); axes[4].axhline(-.2, color='#777', ls=':')
        axes[4].set_ylabel('Command step/dt (rad/s)'); axes[4].legend(fontsize=8)
        axes[4].set_xlabel('Seconds since bag start (receipt time; wrench uses header time)')
        axes[-1].set_xlim(lower, upper)
        for ax in axes:
            # Autoscale to the displayed interval, not the full recording.
            vals = []
            for line in ax.lines:
                x,y = np.asarray(line.get_xdata()),np.asarray(line.get_ydata())
                if x.shape == y.shape and len(x)>2:
                    vals.extend(y[(x>=lower)&(x<=upper)])
            if vals:
                lo,hi=min(vals),max(vals); pad=max((hi-lo)*.08,.01)
                ax.set_ylim(lo-pad,hi+pad)
        fig.suptitle('Real bag 04: wrench limit and command-to-hold transition')
        fig.savefig(out/name, dpi=150); plt.close(fig)
    print(json.dumps({key: summary[key] for key in ['sensor_fault_receipt_s','gate_fault_receipt_s',
          'hold_transition_command_jump_rad','post_gate_command_peak_to_peak_rad',
          'post_fault_max_joint_deviation_from_hold_rad','final_feedback_minus_hold_rad','windows']}, indent=2))


if __name__ == '__main__':
    main(Path(sys.argv[1]), Path(sys.argv[2]))
