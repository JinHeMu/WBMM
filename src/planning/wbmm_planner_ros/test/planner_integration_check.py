#!/usr/bin/env python3
"""End-to-end check for the WBMM planner node.

Feeds the node a robot state and a goal, then asserts that a contract-valid
WholeBodyTrajectory comes back. Run the node first:

    ros2 run wbmm_planner_ros wbmm_planner_ros_node --ros-args \\
      -p urdf_file:=... -p esdf_file:=... -p world_frame:=map

Exits non-zero on failure so it can be used as a smoke gate.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from wbmm_planner_ros.msg import WholeBodyTrajectory

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
TRAJECTORY_TOPIC = "/wbmm/whole_body_trajectory"


def yaw_to_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class Harness(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__("planner_harness")
        sensor = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.odom_pub = self.create_publisher(Odometry, "/wheel/odometry", sensor)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", sensor)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 1)
        self.subscription = self.create_subscription(
            WholeBodyTrajectory, TRAJECTORY_TOPIC, self._on_trajectory, 1
        )
        self.trajectory = None
        self.goal_x = goal_x
        self.goal_y = goal_y

    def _on_trajectory(self, message):
        self.trajectory = message

    def publish_state(self):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        joints = JointState()
        joints.header.stamp = odom.header.stamp
        joints.name = JOINT_NAMES
        joints.position = [0.0] * len(JOINT_NAMES)
        joints.velocity = [0.0] * len(JOINT_NAMES)
        self.joint_pub.publish(joints)

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        qx, qy, qz, qw = yaw_to_quaternion(0.0)
        goal.pose.orientation.x = qx
        goal.pose.orientation.y = qy
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        self.goal_pub.publish(goal)


def main():
    goal_x = float(sys.argv[1]) if len(sys.argv) > 1 else 1.5
    goal_y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

    rclpy.init()
    harness = Harness(goal_x, goal_y)

    # Let discovery settle, then feed the state so the node has odom and joints
    # before the goal arrives.
    deadline = time.time() + 5.0
    while time.time() < deadline and harness.trajectory is None:
        harness.publish_state()
        rclpy.spin_once(harness, timeout_sec=0.1)

    harness.publish_state()
    harness.publish_goal()

    deadline = time.time() + 30.0
    while time.time() < deadline and harness.trajectory is None:
        rclpy.spin_once(harness, timeout_sec=0.1)

    failures = []
    trajectory = harness.trajectory
    if trajectory is None:
        failures.append("no trajectory published within 30 s")
    else:
        if trajectory.header.frame_id != "map":
            failures.append(f"frame_id={trajectory.header.frame_id!r}, expected 'map'")
        samples = len(trajectory.time_from_start)
        if samples < 2:
            failures.append(f"only {samples} samples")
        if len(trajectory.joint_names) != len(JOINT_NAMES):
            failures.append(f"joint_names={list(trajectory.joint_names)}")
        for name in ("base_x", "base_y", "base_yaw", "base_linear_velocity",
                     "base_yaw_rate", "phase"):
            if len(getattr(trajectory, name)) != samples:
                failures.append(f"{name} length != {samples}")
        expected_flat = samples * len(JOINT_NAMES)
        for name in ("joint_positions", "joint_velocities"):
            if len(getattr(trajectory, name)) != expected_flat:
                failures.append(f"{name} length != {expected_flat}")
        for i in range(1, samples):
            if trajectory.time_from_start[i] <= trajectory.time_from_start[i - 1]:
                failures.append(f"times not increasing at {i}")
                break
        if samples >= 2:
            if abs(trajectory.base_linear_velocity[0]) > 1e-6:
                failures.append("trajectory does not start at rest")
            if abs(trajectory.base_linear_velocity[-1]) > 1e-6:
                failures.append("trajectory does not end at rest")
            reached = trajectory.base_x[-1]
            if reached < goal_x - 0.30:
                failures.append(f"endpoint x={reached:.3f} is short of goal {goal_x}")
        print(
            f"trajectory: {samples} samples, {trajectory.time_from_start[-1]:.2f} s, "
            f"end x={trajectory.base_x[-1]:.3f}, "
            f"v_max={max(abs(v) for v in trajectory.base_linear_velocity):.3f} m/s, "
            f"w_max={max(abs(w) for w in trajectory.base_yaw_rate):.3f} rad/s"
        )

    harness.destroy_node()
    rclpy.shutdown()

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1
    print("PASS: planner produced a contract-valid whole-body trajectory")
    return 0


if __name__ == "__main__":
    sys.exit(main())
