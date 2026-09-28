#!/usr/bin/env python3
"""Isolated display integration check; never starts a controller or robot driver."""
import argparse
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from ocs2_msgs.msg import MpcFlattenedController, MpcState
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray
from wbmm_planning_msgs.msg import WholeBodyTrajectory


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf', required=True)
    parser.add_argument('--demo-seconds', type=float, default=0,
                        help='After checks, publish both layers in map for an RViz preview.')
    args = parser.parse_args()
    # Prevent the test publisher from joining a normal control domain by accident.
    if int(os.environ.get('ROS_DOMAIN_ID', '0')) == 0 or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('Use a dedicated nonzero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1.')
    urdf = Path(args.urdf).resolve()
    tree = ET.parse(urdf)
    visual_count = len(tree.findall('.//link/visual'))
    assert visual_count > 0
    for mesh in tree.findall('.//visual/geometry/mesh'):
        uri = mesh.attrib['filename']
        if uri.startswith('package://'):
            package, relative = uri[10:].split('/', 1)
            resource = Path(get_package_share_directory(package)) / relative
        elif uri.startswith('file://'):
            resource = Path(uri[7:])
        else:
            resource = urdf.parent / uri
        assert resource.is_file(), f'Missing RViz mesh: {resource}'

    rclpy.init()
    node = rclpy.create_node('visualization_display_check')
    retained = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
    messages = {'planner': [], 'mpc': []}
    subscriptions = []
    for layer in messages:
        subscriptions.append(node.create_subscription(
            MarkerArray, '/wbmm/visualization/' + layer,
            lambda msg, key=layer: messages[key].append(msg), retained))
    planner = node.create_publisher(WholeBodyTrajectory, '/wbmm/whole_body_trajectory', 1)
    mpc = node.create_publisher(MpcFlattenedController, '/mobile_manipulator_mpc_policy', 1)
    binary = Path(get_package_prefix('wbmm_visualization')) / 'lib/wbmm_visualization/trajectory_visualizer'
    config = Path(get_package_share_directory('wbmm_visualization')) / 'config/visualization.yaml'
    log = tempfile.NamedTemporaryFile(prefix='wbmm_visualization_check_', suffix='.log', delete=False)
    process = subprocess.Popen([
        str(binary), '--ros-args', '--params-file', str(config),
        '-p', f'urdf_file:={urdf}', '-p', 'mpc_frame:=map' if args.demo_seconds else 'mpc_frame:=odom',
        '-p', 'planner.max_robot_poses:=4', '-p', 'mpc.max_robot_poses:=3',
        '-p', 'publish_rate:=10.0', '-p', 'mpc.timeout:=0.8'], stdout=log, stderr=log)

    def wait(predicate, seconds=8.0):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise AssertionError(f'Display node exited: {Path(log.name).read_text()}')
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
        raise AssertionError(f'Timed out; node log: {log.name}')

    def send(publisher, msg, layer):
        old = len(messages[layer])
        publisher.publish(msg)
        wait(lambda: len(messages[layer]) > old)
        return messages[layer][-1]

    def additions(msg):
        return [m for m in msg.markers if m.action == Marker.ADD]

    def planned(n):
        msg = WholeBodyTrajectory()
        msg.header.frame_id = 'map'
        msg.joint_names = [f'joint_{i}' for i in range(1, 7)]
        for i in range(n):
            msg.time_from_start.append(i * 0.5)
            msg.base_x.append(i * 0.3)
            msg.base_y.append(0.0)
            msg.base_yaw.append(i * 0.05)
            msg.joint_positions.extend([i * 0.08, 0.4, -0.8, 0.0, 0.4, 0.0])
        return msg

    def policy():
        msg = MpcFlattenedController()
        for i in range(7):
            msg.time_trajectory.append(100.0 + i * 0.2)
            msg.state_trajectory.append(MpcState(value=[i * 0.3, 1.2, 0.0, i * 0.1, 0.4, -0.8, 0.0, 0.4, 0.0]))
        # Deliberately unrelated reference: displayed models must follow predictions.
        msg.plan_target_trajectories.state_trajectory = [MpcState(value=[999.0] * 9)]
        return msg

    try:
        wait(lambda: planner.get_subscription_count() > 0 and mpc.get_subscription_count() > 0)
        first = send(planner, planned(9), 'planner')
        assert len(additions(first)) == 4 * visual_count
        assert all(m.header.frame_id == 'map' for m in first.markers)
        assert any(m.type == Marker.MESH_RESOURCE for m in first.markers)
        assert all(math.isfinite(m.pose.position.x) and m.color.a > 0 for m in first.markers)
        # Snapshot durability lets RViz join after the plan was published.
        late = []
        subscriptions.append(node.create_subscription(MarkerArray, '/wbmm/visualization/planner', late.append, retained))
        wait(lambda: len(late) > 0)
        assert len(additions(late[-1])) == 4 * visual_count
        predicted = send(mpc, policy(), 'mpc')
        assert len(additions(predicted)) == 3 * visual_count
        frame = 'map' if args.demo_seconds else 'odom'
        assert all(m.header.frame_id == frame and abs(m.pose.position.x) < 10 for m in predicted.markers)
        assert all(m.color.r > m.color.g for m in additions(predicted))
        smaller = send(planner, planned(2), 'planner')
        assert len(additions(smaller)) == 2 * visual_count
        assert sum(m.action == Marker.DELETE for m in smaller.markers) == 2 * visual_count
        # Display has exactly two application publishers, both MarkerArray.
        graph = dict(node.get_publisher_names_and_types_by_node('wbmm_visualization', '/'))
        app_outputs = {k: v for k, v in graph.items() if k not in ('/rosout', '/parameter_events')}
        assert app_outputs == {f'/wbmm/visualization/{k}': ['visualization_msgs/msg/MarkerArray'] for k in messages}, app_outputs
        wait(lambda: messages['mpc'][-1].markers and
             all(m.action == Marker.DELETE for m in messages['mpc'][-1].markers))
        assert len(additions(messages['planner'][-1])) == 2 * visual_count
        invalid = planned(3)
        invalid.base_x[1] = float('nan')
        cleared = send(planner, invalid, 'planner')
        assert len(cleared.markers) == 2 * visual_count and not additions(cleared)
        send(planner, planned(2), 'planner')
        cleared = send(planner, WholeBodyTrajectory(), 'planner')
        assert len(cleared.markers) == 2 * visual_count and not additions(cleared)
        print(f'PASS: {visual_count} URDF visuals; mesh resources; planner/MPC ghosts; frames; '
              'late subscriber; shrink/invalid/empty deletes; MPC expiry; display-only publishers.', flush=True)
        deadline = time.monotonic() + args.demo_seconds
        while time.monotonic() < deadline:
            planner.publish(planned(9))
            mpc.publish(policy())
            rclpy.spin_once(node, timeout_sec=0.15)
            time.sleep(0.15)
    finally:
        process.send_signal(signal.SIGINT) if process.poll() is None else None
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        print(f'Display node log: {log.name}', flush=True)


if __name__ == '__main__':
    main()
