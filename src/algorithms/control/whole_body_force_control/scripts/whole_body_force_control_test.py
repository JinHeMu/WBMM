#!/usr/bin/env python3
"""Inject force stages and verify measured MuJoCo whole-body compliance."""

import json
import math
import sys
import threading
import time
from collections import deque
from pathlib import Path

import rclpy
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String


class WholeBodyForceControlTest(Node):
    def __init__(self):
        super().__init__("whole_body_force_control_test")
        self.report_file = Path(
            self.declare_parameter(
                "report_file", "/tmp/whole_body_force_control_test_report.json"
            ).value
        )
        self.ready_timeout = float(
            self.declare_parameter("ready_timeout", 25.0).value
        )
        self.wrench_topic = str(self.declare_parameter(
            "wrench_topic", "/whole_body_force_control/fake_wrench").value)
        self.wrench_frame = str(self.declare_parameter(
            "wrench_frame", "tool0").value)
        self.test_profile = str(self.declare_parameter(
            "test_profile", "signed_regression").value)
        low_force = float(self.declare_parameter("low_force", 5.0).value)
        high_force = float(self.declare_parameter("high_force", 12.0).value)
        pull_force = float(self.declare_parameter("pull_force", -8.0).value)
        baseline_duration = float(
            self.declare_parameter("baseline_duration", 4.0).value)
        if self.test_profile in ("continuous_20s", "continuous_infinite"):
            continuous_force = float(
                self.declare_parameter("continuous_force", 7.0).value)
            continuous_duration = float(
                self.declare_parameter("continuous_duration", 20.0).value)
            self.stages = (
                ("baseline", 0.0, baseline_duration),
                ("continuous_push", continuous_force, continuous_duration),
            )
            if continuous_force <= 0.0:
                raise ValueError("continuous_force must be positive")
            if continuous_duration < 20.0:
                raise ValueError("continuous_duration must be at least 20 seconds")
        elif self.test_profile == "signed_regression":
            self.stages = (
                ("baseline", 0.0, baseline_duration),
                ("low_push", low_force, float(
                    self.declare_parameter("low_force_duration", 10.0).value)),
                ("high_push", high_force, float(
                    self.declare_parameter("high_force_duration", 15.0).value)),
                ("release", 0.0, float(
                    self.declare_parameter("release_duration", 10.0).value)),
                ("pull", pull_force, float(
                    self.declare_parameter("pull_duration", 12.0).value)),
                ("final_release", 0.0, float(
                    self.declare_parameter("final_release_duration", 10.0).value)),
            )
            if low_force <= 0.0 or high_force <= low_force or pull_force >= 0.0:
                raise ValueError(
                    "high_force must be greater than low_force > 0 and pull_force < 0")
        else:
            raise ValueError(
                "test_profile must be signed_regression, continuous_20s, "
                "or continuous_infinite")
        if any(duration <= 1.0 for _, _, duration in self.stages):
            raise ValueError("every force stage must last longer than one second")

        self.publisher = self.create_publisher(
            WrenchStamped, self.wrench_topic, 10)
        self.create_subscription(
            Float64MultiArray, "/whole_body_force_control/status",
            self.status_callback, 10)
        self.create_subscription(
            Bool, "/mujoco/unexpected_collision", self.collision_callback, 10)
        self.create_subscription(
            String, "/whole_body_force_control/control_state",
            self.control_state_callback, 10)
        self.timer = self.create_timer(0.02, self.tick)
        self.created_at = time.monotonic()
        self.stage_started_at = None
        self.stage_index = 0
        self.latest_status = None
        self.samples = {name: deque(maxlen=100) for name, _, _ in self.stages}
        self.trajectory_samples = {
            name: [] for name, _, _ in self.stages
        }
        self.unexpected_collision = False
        self.control_state = "UNKNOWN"
        self.control_fault = None
        self.finished = False

    def status_callback(self, message):
        if len(message.data) >= 9 and all(math.isfinite(x) for x in message.data):
            self.latest_status = list(message.data[:9])

    def collision_callback(self, message):
        self.unexpected_collision = self.unexpected_collision or bool(message.data)

    def control_state_callback(self, message):
        self.control_state = message.data
        if message.data.startswith("FAULT_"):
            self.control_fault = message.data

    def publish_force(self, force):
        message = WrenchStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.wrench_frame
        message.wrench.force.x = float(force)
        self.publisher.publish(message)

    def tick(self):
        if self.finished:
            return
        if self.latest_status is None:
            self.publish_force(0.0)
            if time.monotonic() - self.created_at > self.ready_timeout:
                self.finish(False, ["timed out waiting for force-control status"])
            return
        now = time.monotonic()
        if self.stage_started_at is None:
            self.stage_started_at = now
            self.get_logger().info("Starting generic whole-body force validation")
        name, force, duration = self.stages[self.stage_index]
        self.publish_force(force)
        elapsed = now - self.stage_started_at
        self.trajectory_samples[name].append((elapsed, list(self.latest_status)))
        if elapsed >= max(0.0, duration - 1.0):
            self.samples[name].append(self.latest_status)
        if elapsed < duration:
            return
        summary = self.mean_sample(name)
        self.get_logger().info(
            f"{name}: force={summary[0]:.2f} N, ref={summary[1]:.4f} m, "
            f"base={summary[4]:.4f} m, arm={summary[6]:.4f} m, "
            f"ee={summary[5]:.4f} m")
        self.stage_index += 1
        self.stage_started_at = now
        if self.stage_index >= len(self.stages):
            if self.test_profile in ("continuous_20s", "continuous_infinite"):
                self.evaluate_continuous_follow()
            else:
                self.evaluate_signed_regression()

    def mean_sample(self, stage):
        samples = list(self.samples[stage])
        if not samples:
            return [float("nan")] * 9
        return [sum(row[i] for row in samples) / len(samples) for i in range(9)]

    def evaluate_signed_regression(self):
        means = {name: self.mean_sample(name) for name, _, _ in self.stages}
        low = means["low_push"]
        high = means["high_push"]
        release = means["release"]
        pull = means["pull"]
        final_release = means["final_release"]
        failures = []
        if not high[1] > low[1] + 0.020:
            failures.append("high-force offset did not exceed low-force by 20 mm")
        if not high[4] > low[4] + 0.005:
            failures.append("base displacement did not increase with force")
        if not high[5] > low[5] + 0.010:
            failures.append("end-effector displacement did not increase with force")
        if not high[4] > 0.010:
            failures.append("base did not participate by at least 10 mm")
        if not high[6] > 0.010:
            failures.append("arm did not participate by at least 10 mm")
        if abs(high[8]) > 0.010:
            failures.append("base lateral slip exceeded 10 mm")
        if not abs(release[5]) < max(0.010, abs(high[5]) * 0.60):
            failures.append("end effector did not return after force release")
        if not pull[1] < -0.020:
            failures.append("negative pull did not produce a negative reference")
        if not pull[4] < -0.005:
            failures.append("base did not participate in negative pull")
        if not pull[6] < -0.005:
            failures.append("arm did not participate in negative pull")
        if not abs(final_release[5]) < max(0.010, abs(pull[5]) * 0.60):
            failures.append("end effector did not return after pull release")
        if self.unexpected_collision:
            failures.append("MuJoCo reported an unexpected collision")
        if self.control_fault is not None:
            failures.append(f"force controller latched {self.control_fault}")
        report = {
            "passed": not failures,
            "package": "whole_body_force_control",
            "test": "MuJoCo signed whole-body force-follow tracking",
            "force_profile": [
                {"stage": name, "force_N": force, "duration_s": duration}
                for name, force, duration in self.stages
            ],
            "status_layout": [
                "filtered_force_N", "control_offset_m", "base_reference_m",
                "arm_reference_m", "base_measured_m", "ee_measured_m",
                "arm_measured_relative_to_base_m", "max_joint_motion_rad",
                "base_lateral_motion_m",
            ],
            "stage_means": means,
            "unexpected_collision": self.unexpected_collision,
            "final_control_state": self.control_state,
            "control_fault": self.control_fault,
            "failures": failures,
        }
        self.write_report(report, failures)

    def evaluate_continuous_follow(self):
        push_samples = self.trajectory_samples["continuous_push"]
        force_duration = self.stages[1][2]
        window_duration = force_duration / 4.0
        windows = []
        for index in range(4):
            start = index * window_duration + min(1.0, 0.2 * window_duration)
            end = (index + 1) * window_duration
            rows = [
                row for elapsed, row in push_samples
                if start <= elapsed <= end
            ]
            if rows:
                windows.append([
                    sum(row[field] for row in rows) / len(rows)
                    for field in range(9)
                ])
            else:
                windows.append([float("nan")] * 9)

        failures = []
        minimum_base_window_progress = 0.50
        minimum_arm_window_progress = 0.005
        for index in range(1, len(windows)):
            base_progress = windows[index][4] - windows[index - 1][4]
            arm_progress = windows[index][6] - windows[index - 1][6]
            if not base_progress > minimum_base_window_progress:
                failures.append(
                    f"base stopped following between 5 s windows {index} and {index + 1}")
            if not arm_progress > minimum_arm_window_progress:
                failures.append(
                    f"arm stopped following between 5 s windows {index} and {index + 1}")
        final_mean = self.mean_sample("continuous_push")
        if not final_mean[4] > 4.50:
            failures.append("base final displacement was not greater than 4.50 m")
        if not final_mean[6] > 0.070:
            failures.append("arm final displacement was not greater than 70 mm")
        if not final_mean[5] > 4.60:
            failures.append("end-effector final displacement was not greater than 4.60 m")
        if any(abs(window[8]) > 0.010 for window in windows):
            failures.append("base lateral slip exceeded 10 mm")
        if self.unexpected_collision:
            failures.append("MuJoCo reported an unexpected collision")
        if self.control_fault is not None:
            failures.append(f"force controller latched {self.control_fault}")

        report = {
            "passed": not failures,
            "package": "whole_body_force_control",
            "test": "MuJoCo " + (
                "infinite sustained whole-body force following"
                if self.test_profile == "continuous_infinite" else
                "20 s sustained whole-body force following"),
            "force_profile": [
                {"stage": name, "force_N": force, "duration_s": duration}
                for name, force, duration in self.stages
            ],
            "status_layout": [
                "filtered_force_N", "control_offset_m", "base_reference_m",
                "arm_reference_m", "base_measured_m", "ee_measured_m",
                "arm_measured_relative_to_base_m", "max_joint_motion_rad",
                "base_lateral_motion_m",
            ],
            "five_second_window_means": windows,
            "final_second_mean": final_mean,
            "minimum_base_progress_per_window_m": minimum_base_window_progress,
            "minimum_arm_progress_per_window_m": minimum_arm_window_progress,
            "unexpected_collision": self.unexpected_collision,
            "final_control_state": self.control_state,
            "control_fault": self.control_fault,
            "failures": failures,
        }
        self.write_report(report, failures)

    def write_report(self, report, failures):
        self.report_file.parent.mkdir(parents=True, exist_ok=True)
        self.report_file.write_text(
            json.dumps(report, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8")
        self.finish(not failures, failures)

    def finish(self, passed, failures):
        self.finished = True
        self.publish_force(0.0)
        self.exit_code = 0 if passed else 1
        if passed:
            self.get_logger().info(f"RESULT: PASS ({self.report_file})")
        else:
            self.get_logger().error(f"RESULT: FAIL: {'; '.join(failures)}")
        self.destroy_timer(self.timer)
        self.shutdown_timer = threading.Timer(0.2, rclpy.shutdown)
        self.shutdown_timer.daemon = True
        self.shutdown_timer.start()


def main():
    rclpy.init()
    node = WholeBodyForceControlTest()
    try:
        rclpy.spin(node)
    finally:
        code = getattr(node, "exit_code", 1)
        node.destroy_node()
    sys.exit(code)


if __name__ == "__main__":
    main()
