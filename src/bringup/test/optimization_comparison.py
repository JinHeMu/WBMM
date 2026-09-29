#!/usr/bin/env python3
"""Compare optimization off/on using fresh, isolated MuJoCo tracking launches."""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String
from wbmm_planner_ros.msg import WholeBodyTrajectory


def run_case(args, enabled):
    label = 'optimized' if enabled else 'baseline'
    node = rclpy.create_node('optimization_comparison_' + label)
    state = {'odom': None, 'status': '', 'trajectory': None}
    subscriptions = []
    subscriptions.append(node.create_subscription(
        Odometry, '/wheel/odometry', lambda msg: state.update(odom=msg), qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        String, '/wbmm/planning/status', lambda msg: state.update(status=msg.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    subscriptions.append(node.create_subscription(
        WholeBodyTrajectory, '/wbmm/whole_body_trajectory',
        lambda msg: state.update(trajectory=msg), 1))
    publisher = node.create_publisher(PoseStamped, '/goal_pose', 1)
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    log_path = output / (label + '.log')
    stream = log_path.open('w')
    command = ['ros2', 'launch', 'tracer_jaka_bringup', 'wbmm_tracking.launch.py',
               'esdf_file:=' + args.esdf_file, 'viewer:=false', 'use_rviz:=false',
               'planner_enable_optimization:=' + str(enabled).lower()]
    process = subprocess.Popen(command, cwd=args.workspace, stdout=stream,
                               stderr=stream, start_new_session=True)
    result = {'enabled': enabled, 'command': command, 'success': False}

    def wait(predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError('Tracking launch exited')
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError('Timed out: ' + state['status'])

    try:
        wait(lambda: state['odom'] is not None and publisher.get_subscription_count() >= 2, 180)
        settled = time.monotonic()
        wait(lambda: time.monotonic() - settled > 4.0, 6)
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = state['odom'].header.stamp
        goal.pose.position.x, goal.pose.position.y = args.goal
        goal.pose.orientation.w = 1.0
        start = time.monotonic()
        publisher.publish(goal)
        wait(lambda: state['trajectory'] is not None or state['status'].startswith(('FAILED', 'FAULT')), 90)
        result['request_to_trajectory_s'] = time.monotonic() - start
        if state['trajectory'] is None:
            raise RuntimeError(state['status'])
        msg = state['trajectory']
        t = np.asarray(msg.time_from_start)
        q = np.asarray(msg.joint_positions).reshape(len(t), -1)
        u = np.column_stack([msg.base_linear_velocity, msg.base_yaw_rate,
                             np.asarray(msg.joint_velocities).reshape(len(t), -1)])
        dt = np.diff(t)
        acceleration = np.diff(u, axis=0) / dt[:, None]
        xy = np.column_stack([msg.base_x, msg.base_y])
        result.update(duration_s=float(t[-1]), samples=len(t),
                      base_path_length_m=float(np.linalg.norm(np.diff(xy, axis=0), axis=1).sum()),
                      max_base_speed=float(np.max(np.abs(u[:, 0]))),
                      max_yaw_rate=float(np.max(np.abs(u[:, 1]))),
                      max_joint_speed=float(np.max(np.abs(u[:, 2:]))),
                      base_acceleration_rms=float(np.sqrt(np.mean(acceleration[:, 0]**2))),
                      arm_acceleration_rms=float(np.sqrt(np.mean(acceleration[:, 2:]**2))))
        np.savez_compressed(output / (label + '_trajectory.npz'), t=t, xy=xy,
                            yaw=msg.base_yaw, q=q, u=u)
        wait(lambda: state['status'] == 'SUCCEEDED' or state['status'].startswith(('FAILED', 'FAULT')),
             max(30, float(t[-1]) + 25))
        result['status'] = state['status']
        odom = state['odom'].pose.pose
        result['goal_error_m'] = math.hypot(odom.position.x - args.goal[0], odom.position.y - args.goal[1])
        result['success'] = state['status'] == 'SUCCEEDED'
        metrics = re.findall(r'PLAN_METRICS ([^\n]+)', log_path.read_text(errors='replace'))
        if metrics:
            result['planner_metrics'] = metrics[-1]
    except Exception as error:
        result['error'] = str(error)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        stream.close()
        node.destroy_node()
        (output / (label + '.json')).write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workspace', default='/home/a/WBMM')
    parser.add_argument('--esdf-file', default='maps/map1/site_remani.npz')
    parser.add_argument('--goal', type=float, nargs=2, default=[0.8, 0.0])
    parser.add_argument('--output', default='/tmp/wbmm_optimization_comparison')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        parser.error('Use ROS_LOCALHOST_ONLY=1 and a dedicated nonzero ROS_DOMAIN_ID')
    os.environ['CYCLONEDDS_URI'] = ('<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex>'
                                  '<MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>')
    rclpy.init()
    try:
        results = [run_case(args, False), run_case(args, True)]
        (Path(args.output) / 'comparison.json').write_text(json.dumps(results, indent=2))
        if not all(r['success'] for r in results):
            raise SystemExit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
