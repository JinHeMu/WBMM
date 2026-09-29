#!/usr/bin/env python3
"""End-to-end check of the WBMM planning pipeline.

Validates the two new nodes together, with no REMANI involvement:

    /goal_pose + odom + /joint_states
        -> wbmm_planner_node    -> /wbmm/whole_body_trajectory
        -> wbmm_reference_bridge -> <robot_name>_whole_body_target

Start both first, for example:

    ros2 launch tracer_jaka_bringup wbmm_planning.launch.py \\
      urdf_file:=... esdf_file:=... world_frame:=map bridge_world_frame:=map

Exits non-zero on failure.
"""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ocs2_msgs.msg import MpcObservation, MpcTargetTrajectories
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from wbmm_planner_ros.msg import WholeBodyTrajectory

JOINT_NAMES = [f"joint_{i}" for i in range(1, 7)]
OBSERVATION_TOPIC = "mobile_manipulator_mpc_observation"
TARGET_TOPIC = "mobile_manipulator_whole_body_target"
TRAJECTORY_TOPIC = "/wbmm/whole_body_trajectory"

STATE_DIM = 9
INPUT_DIM = 8


def sensor_qos():
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
    )


class PipelineHarness(Node):
    def __init__(self, goal_x, goal_y):
        super().__init__("planning_pipeline_harness")
        self.odom_pub = self.create_publisher(
            Odometry, "/wheel/odometry", sensor_qos())
        self.joint_pub = self.create_publisher(
            JointState, "/joint_states", sensor_qos())
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 1)
        # The bridge learns the ROS -> OCS2 time mapping from this topic, so a
        # stand-in MPC observation is required to exercise it.
        self.observation_pub = self.create_publisher(
            MpcObservation, OBSERVATION_TOPIC, sensor_qos())

        self.trajectory = None
        self.target = None
        self.create_subscription(
            WholeBodyTrajectory, TRAJECTORY_TOPIC, self._on_trajectory, 1)
        self.create_subscription(
            MpcTargetTrajectories, TARGET_TOPIC, self._on_target, 1)

        self.goal_x = goal_x
        self.goal_y = goal_y

    def _on_trajectory(self, message):
        self.trajectory = message

    def _on_target(self, message):
        self.target = message

    def publish_state(self, sim_time):
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

        joints = JointState()
        joints.header.stamp = stamp
        joints.name = JOINT_NAMES
        joints.position = [0.0] * len(JOINT_NAMES)
        joints.velocity = [0.0] * len(JOINT_NAMES)
        self.joint_pub.publish(joints)

        observation = MpcObservation()
        observation.time = sim_time
        observation.state.value = [0.0] * STATE_DIM
        observation.input.value = [0.0] * INPUT_DIM
        observation.mode = 0
        self.observation_pub.publish(observation)

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = "map"
        goal.pose.position.x = self.goal_x
        goal.pose.position.y = self.goal_y
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)


def main():
    goal_x = float(sys.argv[1]) if len(sys.argv) > 1 else 1.0
    goal_y = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0

    rclpy.init()
    harness = PipelineHarness(goal_x, goal_y)

    start = time.time()
    # Discovery plus a few observation samples so the bridge has a time mapping.
    while time.time() - start < 6.0:
        harness.publish_state(time.time() - start)
        rclpy.spin_once(harness, timeout_sec=0.1)

    harness.publish_state(time.time() - start)
    harness.publish_goal()

    deadline = time.time() + 40.0
    while time.time() < deadline and harness.target is None:
        harness.publish_state(time.time() - start)
        rclpy.spin_once(harness, timeout_sec=0.1)

    failures = []
    if harness.trajectory is None:
        failures.append("wbmm_planner_node published no WholeBodyTrajectory")
    if harness.target is None:
        failures.append(
            "wbmm_reference_bridge published no MpcTargetTrajectories")
    else:
        target = harness.target
        times = list(target.time_trajectory)
        states = list(target.state_trajectory)
        inputs = list(target.input_trajectory)
        if not (len(times) == len(states) == len(inputs)):
            failures.append(
                f"target arrays differ: {len(times)}/{len(states)}/{len(inputs)}")
        if len(times) < 2:
            failures.append(f"only {len(times)} reference samples")
        for i in range(1, len(times)):
            if times[i] <= times[i - 1]:
                failures.append(f"reference times not increasing at {i}")
                break
        for i, state in enumerate(states):
            if len(state.value) != STATE_DIM:
                failures.append(f"state {i} has {len(state.value)} entries")
                break
        for i, control in enumerate(inputs):
            if len(control.value) != INPUT_DIM:
                failures.append(f"input {i} has {len(control.value)} entries")
                break
        print(
            f"reference: {len(times)} samples, "
            f"t=[{times[0]:.3f}, {times[-1]:.3f}], "
            f"start x={states[0].value[0]:.3f}, yaw={states[0].value[2]:.3f}, "
            f"v={inputs[0].value[0]:.3f}, w={inputs[0].value[1]:.3f}"
        )

    harness.destroy_node()
    rclpy.shutdown()

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}")
        return 1
    print("PASS: planner -> bridge produced a valid OCS2 reference window")
    return 0


if __name__ == "__main__":
    sys.exit(main())
