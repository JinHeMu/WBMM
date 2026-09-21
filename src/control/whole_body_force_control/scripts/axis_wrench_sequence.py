#!/usr/bin/env python3
"""Publish a deterministic single-axis virtual force sequence.

Sequence (default timings):
  0-2 s   : zero
  2-4 s   : +X 5 N
  4-6 s   : zero
  6-8 s   : +Y 5 N
  8-10 s  : zero
  10-12 s : +Z 5 N
  12-14 s : zero

After the sequence the node keeps publishing zero so the force-control node
does not enter a wrench-timeout fault.  Ctrl-C stops it.
"""

import math
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node


class AxisWrenchSequence(Node):
    def __init__(self):
        super().__init__("axis_wrench_sequence")

        self.declare_parameter(
            "topic", "/whole_body_force_control/fake_wrench")
        self.declare_parameter("frame_id", "jk_se_vi_200_link")
        self.declare_parameter("rate", 50.0)
        self.declare_parameter("force_magnitude", 5.0)
        self.declare_parameter("zero_duration", 2.0)
        self.declare_parameter("hold_duration", 2.0)
        self.declare_parameter("final_zero_duration", 2.0)
        self.declare_parameter("loop", False)

        self.topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.rate = float(self.get_parameter("rate").value)
        self.magnitude = float(self.get_parameter("force_magnitude").value)
        self.zero_duration = float(self.get_parameter("zero_duration").value)
        self.hold_duration = float(self.get_parameter("hold_duration").value)
        self.final_zero_duration = float(
            self.get_parameter("final_zero_duration").value)
        self.loop = bool(self.get_parameter("loop").value)

        if not math.isfinite(self.rate) or self.rate <= 0.0:
            raise ValueError("rate must be finite and positive")
        if not math.isfinite(self.magnitude):
            raise ValueError("force_magnitude must be finite")
        for name, value in (
                ("zero_duration", self.zero_duration),
                ("hold_duration", self.hold_duration),
                ("final_zero_duration", self.final_zero_duration)):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")

        z = (0.0, 0.0, 0.0)
        f = self.magnitude
        self.stages = [
            ("zero_initial", z, self.zero_duration),
            ("x_positive", (f, 0.0, 0.0), self.hold_duration),
            ("zero_after_x", z, self.zero_duration),
            ("y_positive", (0.0, f, 0.0), self.hold_duration),
            ("zero_after_y", z, self.zero_duration),
            ("z_positive", (0.0, 0.0, f), self.hold_duration),
            ("zero_final", z, self.final_zero_duration),
        ]
        self.stage_index = 0
        self.stage_started = time.monotonic()
        self.completed = False

        self.publisher = self.create_publisher(WrenchStamped, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

        self._log_stage_start()
        self.get_logger().info(
            f"Publishing virtual force on {self.topic}, "
            f"frame_id={self.frame_id}, magnitude={self.magnitude} N")

    def _publish(self, force):
        message = WrenchStamped()
        # Zero stamp asks the force-control node to use the latest TF.
        message.header.frame_id = self.frame_id
        message.wrench.force.x = float(force[0])
        message.wrench.force.y = float(force[1])
        message.wrench.force.z = float(force[2])
        message.wrench.torque.x = 0.0
        message.wrench.torque.y = 0.0
        message.wrench.torque.z = 0.0
        self.publisher.publish(message)

    def _log_stage_start(self):
        if self.stage_index >= len(self.stages):
            return
        name, force, duration = self.stages[self.stage_index]
        self.get_logger().info(
            f"stage={name} duration={duration:.2f}s force={list(force)}")

    def _advance(self):
        self.stage_index += 1
        self.stage_started = time.monotonic()
        if self.stage_index >= len(self.stages):
            self.completed = True
            if self.loop:
                self.stage_index = 0
                self.completed = False
            else:
                self.get_logger().info(
                    "sequence complete; continuing to publish zero")
            self._log_stage_start()
            return
        self._log_stage_start()

    def _tick(self):
        if self.completed:
            self._publish((0.0, 0.0, 0.0))
            return

        name, force, duration = self.stages[self.stage_index]
        if time.monotonic() - self.stage_started >= duration:
            self._advance()
            if self.completed:
                self._publish((0.0, 0.0, 0.0))
                return
            name, force, duration = self.stages[self.stage_index]
        self._publish(force)


def main():
    rclpy.init()
    node = AxisWrenchSequence()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
