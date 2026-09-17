#!/usr/bin/env python3
"""Virtual 6D wrench source for simulation.

Manual mode (default):
  ros2 param set /virtual_force_publisher force "[5.0, 0.0, 0.0]"
  ros2 param set /virtual_force_publisher torque "[0.0, 0.0, 1.0]"
  ros2 param set /virtual_force_publisher publish_enabled false

Sequence mode:
  ros2 run whole_body_force_control fake_wrench_sequence.py \
    --ros-args -p sequence_enabled:=true
"""

import math
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node


class VirtualForcePublisher(Node):
    def __init__(self):
        super().__init__('virtual_force_publisher')

        # Interface.
        self.declare_parameter('topic', '/whole_body_force_control/fake_wrench')
        self.declare_parameter('frame_id', 'jk_se_vi_200_link')
        self.declare_parameter('rate', 50.0)

        # Live manual control.
        self.declare_parameter('force', [0.0, 0.0, 0.0])
        self.declare_parameter('torque', [0.0, 0.0, 0.0])
        self.declare_parameter('publish_enabled', True)

        # Optional automatic sequence: zero -> force -> release -> disconnect -> zero.
        self.declare_parameter('sequence_enabled', False)
        self.declare_parameter('zero_duration', 3.0)
        self.declare_parameter('test_duration', 5.0)
        self.declare_parameter('disconnect_duration', 3.0)
        self.declare_parameter('test_force', [5.0, 5.0, 5.0])
        self.declare_parameter('test_torque', [0.0, 0.0, 0.0])

        rate = float(self.get_parameter('rate').value)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError('rate must be finite and positive')
        self.period = 1.0 / rate

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.force = self._vector3('force')
        self.torque = self._vector3('torque')
        self.publish_enabled = bool(self.get_parameter('publish_enabled').value)
        self.sequence_enabled = bool(self.get_parameter('sequence_enabled').value)
        self.zero_duration = self._duration('zero_duration')
        self.test_duration = self._duration('test_duration')
        self.disconnect_duration = self._duration('disconnect_duration')
        self.test_force = self._vector3('test_force')
        self.test_torque = self._vector3('test_torque')

        topic = str(self.get_parameter('topic').value)
        self.publisher = self.create_publisher(WrenchStamped, topic, 10)
        self.add_on_set_parameters_callback(self._parameter_callback)
        self.timer = self.create_timer(self.period, self._tick)

        self.stages = [
            ('zero_for_tare', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), self.zero_duration),
            ('constant_force', self.test_force, self.test_torque, self.test_duration),
            ('release', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), self.zero_duration),
            ('disconnect', None, None, self.disconnect_duration),
            ('recover_data', (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), self.zero_duration),
        ]
        self.stage_index = 0
        self.stage_started = time.monotonic()
        self._log_mode()

    @staticmethod
    def _validate_vector3(value, name):
        if len(value) != 3 or not all(math.isfinite(float(v)) for v in value):
            raise ValueError(f'{name} must contain three finite numbers')
        return tuple(float(v) for v in value)

    def _vector3(self, name):
        return self._validate_vector3(self.get_parameter(name).value, name)

    def _duration(self, name):
        value = float(self.get_parameter(name).value)
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f'{name} must be finite and non-negative')
        return value

    def _log_mode(self):
        if self.sequence_enabled:
            self.get_logger().info(
                'Virtual wrench in SEQUENCE mode: zero -> force -> release -> disconnect -> recover')
        else:
            self.get_logger().info(
                'Virtual wrench in MANUAL mode. Use ros2 param set '
                f'/{self.get_name()} force "[x, y, z]" or torque "[x, y, z]".')

    def _parameter_callback(self, parameters):
        next_force = self.force
        next_torque = self.torque
        next_publish_enabled = self.publish_enabled
        try:
            for parameter in parameters:
                if parameter.name == 'force':
                    next_force = self._validate_vector3(parameter.value, 'force')
                elif parameter.name == 'torque':
                    next_torque = self._validate_vector3(parameter.value, 'torque')
                elif parameter.name == 'publish_enabled':
                    next_publish_enabled = bool(parameter.value)
                else:
                    return SetParametersResult(
                        successful=False,
                        reason='Only force, torque and publish_enabled may change while running')
        except (TypeError, ValueError) as error:
            return SetParametersResult(successful=False, reason=str(error))

        self.force = next_force
        self.torque = next_torque
        self.publish_enabled = next_publish_enabled
        return SetParametersResult(successful=True)

    def _publish(self, force, torque):
        message = WrenchStamped()
        # Zero stamp intentionally requests latest TF for manual simulation.
        message.header.frame_id = self.frame_id
        message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = force
        message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z = torque
        self.publisher.publish(message)

    def _advance_stage(self):
        self.stage_index += 1
        if self.stage_index >= len(self.stages):
            self.stage_index = len(self.stages) - 1
        self.stage_started = time.monotonic()
        name, force, torque, duration = self.stages[self.stage_index]
        if force is None:
            self.get_logger().info(
                f'stage={name} duration={duration:.2f}s PUBLISH DISABLED')
        else:
            self.get_logger().info(
                f'stage={name} duration={duration:.2f}s '
                f'force={list(force)} torque={list(torque)}')

    def _tick(self):
        if not self.sequence_enabled:
            if self.publish_enabled:
                self._publish(self.force, self.torque)
            return

        name, force, torque, duration = self.stages[self.stage_index]
        if time.monotonic() - self.stage_started >= duration:
            self._advance_stage()
            name, force, torque, duration = self.stages[self.stage_index]
        if force is None:
            return
        self._publish(force, torque)


def main():
    rclpy.init()
    node = VirtualForcePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
