#!/usr/bin/env python3
"""Read-only runtime audit for reference, command, and TF ownership."""

import sys
from collections import defaultdict

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class ReadinessCheck(Node):

    def __init__(self):
        super().__init__('wbmm_readiness_check')
        self.declare_parameter('audit_duration', 5.0)
        self.declare_parameter('expected_target_publishers', 1)
        self.declare_parameter('command_output_enabled', False)
        self.declare_parameter(
            'mpc_target_topic', '/mobile_manipulator_mpc_target')
        self.declare_parameter('base_command_topic', '/cmd_vel')
        self.declare_parameter(
            'arm_command_topic', '/arm_controller/commands')
        self._tf_owners = defaultdict(set)
        self._result = None
        self.create_subscription(TFMessage, '/tf', self._tf_callback, 100)
        self.create_subscription(
            TFMessage, '/tf_static', self._tf_callback, 100)
        self._timer = self.create_timer(
            max(0.5, self.get_parameter('audit_duration').value),
            self._finish)

    def _tf_callback(self, message, info):
        owner = bytes(info.publisher_gid).hex()
        for transform in message.transforms:
            child = transform.child_frame_id.lstrip('/')
            if child:
                self._tf_owners[child].add(owner)

    def _publisher_count(self, topic):
        return len(self.get_publishers_info_by_topic(topic))

    def _finish(self):
        failures = []
        target_topic = self.get_parameter('mpc_target_topic').value
        expected = self.get_parameter('expected_target_publishers').value
        actual = self._publisher_count(target_topic)
        if actual != expected:
            failures.append(
                f'{target_topic}: expected {expected} publisher(s), found {actual}')

        if not self.get_parameter('command_output_enabled').value:
            for parameter in ('base_command_topic', 'arm_command_topic'):
                topic = self.get_parameter(parameter).value
                count = self._publisher_count(topic)
                if count != 0:
                    failures.append(
                        f'{topic}: dry-run expected 0 publishers, found {count}')

        duplicate_frames = sorted(
            child for child, owners in self._tf_owners.items()
            if len(owners) > 1)
        if duplicate_frames:
            failures.append(
                'duplicate TF child owners: ' + ', '.join(duplicate_frames))

        if failures:
            for failure in failures:
                self.get_logger().error('FAIL %s', failure)
            self._result = 1
        else:
            self.get_logger().info(
                'PASS target owner count, command-output gate, and TF child ownership')
            self._result = 0
        self._timer.cancel()


def main():
    rclpy.init()
    node = ReadinessCheck()
    while rclpy.ok() and node._result is None:
        rclpy.spin_once(node, timeout_sec=0.1)
    result = 1 if node._result is None else node._result
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(result)


if __name__ == '__main__':
    main()
