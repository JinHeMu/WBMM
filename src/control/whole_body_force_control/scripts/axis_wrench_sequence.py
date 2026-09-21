#!/usr/bin/env python3
"""Publish a deterministic multi-axis virtual force/torque sequence.

Default sequence (each active stage is held for ``hold_duration`` seconds,
normally 2 s; by default the axis changes every 2 s after an initial 2 s
zero/tare window):
  2 s  zero
  2 s  +X 5 N
  2 s  +Y 5 N
  2 s  +Z 5 N
  2 s  +Tx 1 Nm
  2 s  +Ty 1 Nm
  2 s  +Tz 1 Nm
  2 s  zero

Set ``zero_duration`` > 0 to insert zero pauses between active axes.

The torque magnitude defaults to 1 Nm because the current simulation
force-control profile has a 4 Nm hard wrench limit.  Use
``torque_magnitude:=...`` explicitly if a different limit is configured.

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
        self.declare_parameter("torque_magnitude", 1.0)
        self.declare_parameter("initial_zero_duration", 2.0)
        self.declare_parameter("zero_duration", 0.0)
        self.declare_parameter("hold_duration", 2.0)
        self.declare_parameter("final_zero_duration", 2.0)
        self.declare_parameter("include_force", True)
        self.declare_parameter("include_torque", True)
        self.declare_parameter("loop", False)

        self.topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.rate = float(self.get_parameter("rate").value)
        self.force_magnitude = float(
            self.get_parameter("force_magnitude").value)
        self.torque_magnitude = float(
            self.get_parameter("torque_magnitude").value)
        self.initial_zero_duration = float(
            self.get_parameter("initial_zero_duration").value)
        self.zero_duration = float(self.get_parameter("zero_duration").value)
        self.hold_duration = float(self.get_parameter("hold_duration").value)
        self.final_zero_duration = float(
            self.get_parameter("final_zero_duration").value)
        self.include_force = bool(
            self.get_parameter("include_force").value)
        self.include_torque = bool(
            self.get_parameter("include_torque").value)
        self.loop = bool(self.get_parameter("loop").value)

        if not math.isfinite(self.rate) or self.rate <= 0.0:
            raise ValueError("rate must be finite and positive")
        if not math.isfinite(self.force_magnitude):
            raise ValueError("force_magnitude must be finite")
        if not math.isfinite(self.torque_magnitude):
            raise ValueError("torque_magnitude must be finite")
        for name, value in (
                ("initial_zero_duration", self.initial_zero_duration),
                ("zero_duration", self.zero_duration),
                ("hold_duration", self.hold_duration),
                ("final_zero_duration", self.final_zero_duration)):
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if not self.include_force and not self.include_torque:
            raise ValueError(
                "at least one of include_force/include_torque must be true")

        z = (0.0, 0.0, 0.0)
        f = self.force_magnitude
        t = self.torque_magnitude
        active_axes = []
        if self.include_force:
            active_axes.extend([
                ("x_positive", (f, 0.0, 0.0), z),
                ("y_positive", (0.0, f, 0.0), z),
                ("z_positive", (0.0, 0.0, f), z),
            ])
        if self.include_torque:
            active_axes.extend([
                ("tx_positive", z, (t, 0.0, 0.0)),
                ("ty_positive", z, (0.0, t, 0.0)),
                ("tz_positive", z, (0.0, 0.0, t)),
            ])

        self.stages = [("zero_initial", z, z, self.initial_zero_duration)]
        for name, force, torque in active_axes:
            self.stages.append((name, force, torque, self.hold_duration))
            self.stages.append(
                (f"zero_after_{name}", z, z, self.zero_duration))
        self.stages.append(("zero_final", z, z, self.final_zero_duration))
        self.stage_index = 0
        self.stage_started = time.monotonic()
        self.completed = False

        self.publisher = self.create_publisher(WrenchStamped, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

        self._log_stage_start()
        self.get_logger().info(
            f"Publishing virtual force on {self.topic}, "
            f"frame_id={self.frame_id}, force={self.force_magnitude} N, "
            f"torque={self.torque_magnitude} Nm, "
            f"include_force={self.include_force}, "
            f"include_torque={self.include_torque}")

    def _publish(self, force, torque):
        message = WrenchStamped()
        # Zero stamp asks the force-control node to use the latest TF.
        message.header.frame_id = self.frame_id
        message.wrench.force.x = float(force[0])
        message.wrench.force.y = float(force[1])
        message.wrench.force.z = float(force[2])
        message.wrench.torque.x = float(torque[0])
        message.wrench.torque.y = float(torque[1])
        message.wrench.torque.z = float(torque[2])
        self.publisher.publish(message)

    def _log_stage_start(self):
        if self.stage_index >= len(self.stages):
            return
        name, force, torque, duration = self.stages[self.stage_index]
        self.get_logger().info(
            f"stage={name} duration={duration:.2f}s "
            f"force={list(force)} torque={list(torque)}")

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
        z = (0.0, 0.0, 0.0)
        if self.completed:
            self._publish(z, z)
            return

        name, force, torque, duration = self.stages[self.stage_index]
        if time.monotonic() - self.stage_started >= duration:
            self._advance()
            if self.completed:
                self._publish(z, z)
                return
            name, force, torque, duration = self.stages[self.stage_index]
        self._publish(force, torque)


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
