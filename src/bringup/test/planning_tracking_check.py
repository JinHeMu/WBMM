#!/usr/bin/env python3
"""Headless MuJoCo + OCS2 check on map1; never starts hardware.

Run after sourcing install/setup.bash in an unused ROS_DOMAIN_ID with
ROS_LOCALHOST_ONLY=1. Logs/report go to --output (default /tmp/wbmm_tracking_check).
This is a small navigation/rotation/EE regression, not general map acceptance.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ocs2_msgs.msg import MpcObservation, MpcTargetTrajectories, MpcState, MpcInput
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from wbmm_ocs2_ros.msg import TaskPhaseState


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='/tmp/wbmm_tracking_check')
    parser.add_argument('--entrypoint', choices=['wbmm_tracking.launch.py', 'remani_tracking.launch.py'], default='wbmm_tracking.launch.py')
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0:
        raise RuntimeError('Use ROS_LOCALHOST_ONLY=1 and a dedicated nonzero ROS_DOMAIN_ID')
    # This composition has more than ten DDS participants. Give this isolated
    # test process and its children a sufficient local discovery budget.
    os.environ['CYCLONEDDS_URI'] = (
        '<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex>'
        '<MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>'
        '</Discovery></Domain></CycloneDDS>')
    output = Path(args.output); output.mkdir(parents=True, exist_ok=True)
    report = {'passed': False, 'entrypoint': args.entrypoint, 'stages': []}
    log = (output / 'launch.log').open('w')
    process = subprocess.Popen([
        'ros2', 'launch', 'tracer_jaka_bringup', args.entrypoint,
        'viewer:=false', 'use_rviz:=false',
        'lib_folder:=' + str(output / 'ocs2_cache'),
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node('wbmm_tracking_check', parameter_overrides=[Parameter('use_sim_time', value=True)])
    state = {'odom': None, 'obs': None, 'phase': None, 'status': None}
    def record(key, message):
        state[key] = message
        if key == 'status': print(message.data, flush=True)
    node.create_subscription(Odometry, '/wheel/odometry', lambda msg: record('odom', msg), qos_profile_sensor_data)
    node.create_subscription(MpcObservation, '/mobile_manipulator_mpc_observation', lambda msg: record('obs', msg), qos_profile_sensor_data)
    durable = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(TaskPhaseState, '/mobile_manipulator_task_phase_state', lambda msg: record('phase', msg), durable)
    node.create_subscription(String, '/wbmm/planning/status', lambda msg: record('status', msg), durable)
    goal_pub = node.create_publisher(PoseStamped, '/goal_pose', 10)
    ee_pub = node.create_publisher(MpcTargetTrajectories, '/mobile_manipulator_ee_target', 10)
    buffer = Buffer(); listener = TransformListener(buffer, node)

    def wait(predicate, timeout, stage):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None: raise RuntimeError('launch exited during ' + stage)
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate(): return
        raise RuntimeError('timeout: ' + stage)

    def pose():
        return buffer.lookup_transform('map', 'tool0', rclpy.time.Time()).transform

    def navigation(x, y, yaw, stage):
        state['status'] = None
        goal = PoseStamped(); goal.header.frame_id = 'map'
        goal.header.stamp = node.get_clock().now().to_msg()
        goal.pose.position.x = float(x); goal.pose.position.y = float(y)
        goal.pose.orientation.z = math.sin(yaw / 2); goal.pose.orientation.w = math.cos(yaw / 2)
        goal_pub.publish(goal)
        wait(lambda: state['status'] is not None and
             (state['status'].data == 'SUCCEEDED' or
              state['status'].data.startswith(('FAILED', 'FAULT'))), 100, stage)
        if state['status'].data != 'SUCCEEDED': raise RuntimeError(state['status'].data)
        odom = state['odom'].pose.pose
        error = math.hypot(odom.position.x - x, odom.position.y - y)
        q = odom.orientation
        measured_yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        yaw_error = abs(math.remainder(measured_yaw-yaw, 2*math.pi))
        assert error <= 0.20 and yaw_error <= 0.30, (error, yaw_error)
        report['stages'].append({'stage': stage, 'position_error': error, 'yaw_error': yaw_error})
        print('PASS', stage, error, yaw_error, flush=True)

    try:
        wait(lambda: all(state.values()) and buffer.can_transform('map', 'tool0', rclpy.time.Time()), 240, 'startup')
        # Allow feedback to settle before handing the first goal to the planner.
        navigation(0.8, 0.0, 0.0, 'navigation')
        odom = state['odom'].pose.pose
        navigation(odom.position.x, odom.position.y, 0.6, 'rotation')
        current = pose(); target = [current.translation.x + 0.04, current.translation.y, current.translation.z]
        q = current.rotation
        ee = MpcTargetTrajectories(); ee.time_trajectory = [state['obs'].time]
        ee.state_trajectory = [MpcState(value=target + [q.x, q.y, q.z, q.w])]
        ee.input_trajectory = [MpcInput(value=[0.] * 8)]
        ee_pub.publish(ee)
        wait(lambda: state['phase'].active_phase == 2, 20, 'Execution phase')
        reached_since = None
        def ee_reached():
            nonlocal reached_since
            transform = pose()
            measured = transform.translation
            rotation = transform.rotation
            dot = abs(q.x*rotation.x+q.y*rotation.y+q.z*rotation.z+q.w*rotation.w)
            angular_error = 2*math.acos(min(1.0, dot))
            within = math.dist(target, [measured.x, measured.y, measured.z]) < 0.02 and angular_error < 0.1
            if not within:
                reached_since = None
                return False
            if reached_since is None:
                reached_since = time.monotonic()
            return time.monotonic()-reached_since >= 1.0
        wait(ee_reached, 40, 'EE translation')
        measured = pose().translation
        error = math.dist(target, [measured.x, measured.y, measured.z])
        report['stages'].append({'stage': 'EE tracking', 'active_phase': 2, 'position_error': error})
        print('PASS EE tracking', error, flush=True)
        odom = state['odom'].pose.pose
        rotation = odom.orientation
        yaw = math.atan2(2*(rotation.w*rotation.z+rotation.x*rotation.y),
                         1-2*(rotation.y*rotation.y+rotation.z*rotation.z))
        navigation(odom.position.x, odom.position.y, yaw, 'return to navigation')
        wait(lambda: state['phase'].active_phase == 0, 10, 'Navigation phase')
        report['passed'] = True
    except Exception as error:
        report['error'] = str(error)
        if state['odom'] is not None:
            odom = state['odom']
            report['last_base'] = [odom.pose.pose.position.x, odom.pose.pose.position.y,
                                   odom.twist.twist.linear.x, odom.twist.twist.angular.z]
        if state['obs'] is not None:
            report['last_observation'] = list(state['obs'].state.value)
            report['last_input'] = list(state['obs'].input.value)
        raise
    finally:
        (output / 'report.json').write_text(json.dumps(report, indent=2))
        if process.poll() is None: os.killpg(process.pid, signal.SIGINT)
        try: process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL); process.wait()
        log.close(); node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
