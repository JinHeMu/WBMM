#!/usr/bin/env python3
"""Standalone REMANI whole-body-goal test.

Sends a single traj_utils/WholeBodyGoal to /remani_planner/whole_body_goal and
watches the FSM on /remani_planner/fsm_state and the completion flag on
/planning/finish. This isolates REMANI's whole-body planning from WipePlanner's
automatic goal injection, so you can see whether the planner itself is healthy.

Two modes
---------
sanity   (default)  Goal = current base moved by --dx/--dy/--dyaw, arm joints
                    left at their *measured* values (no arm displacement).
                    If REMANI planning is healthy, this reaches TASK/finish
                    quickly and cleanly.
goal                Explicit base (--x/--y/--yaw) and all six arm joints
                    (--joints "q1 q2 q3 q4 q5 q6"). Use this to reproduce the
                    wall-wipe first-contact frame and observe the
                    "mani acc ... not feasible" rejection.

Prerequisite: the MuJoCo + REMANI stack must be running (odom + joint_states
live), otherwise REMANI silently ignores the goal until odom/joints are ready.

Example
-------
  source install/setup.bash
  python3 test_whole_body_goal.py --mode sanity
  python3 test_whole_body_goal.py --mode goal --x 0.5 --y -0.3 --yaw 1.57 \
      --joints "0.2 -0.4 0.1 0.5 -0.3 0.0"
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from traj_utils.msg import WholeBodyGoal

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def quat_from_yaw(yaw):
    q = Quaternion()
    q.z = math.sin(0.5 * yaw)
    q.w = math.cos(0.5 * yaw)
    return q


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class WholeBodyGoalTester(Node):
    def __init__(self, args):
        super().__init__("test_whole_body_goal")
        self.args = args

        self.goal_pub = self.create_publisher(
            WholeBodyGoal, "/remani_planner/whole_body_goal", 10)

        self.fsm_state = None
        self.finished = False
        self.odom = None
        self.joints = None

        transient = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/remani_planner/fsm_state",
            self._fsm_cb, transient)
        self.create_subscription(
            Bool, "/planning/finish", self._finish_cb, 10)
        self.create_subscription(
            Odometry, "/diff_drive_controller/odom", self._odom_cb, 10)
        self.create_subscription(
            JointState, "/joint_states", self._js_cb, 10)

        self._log("Waiting for live odom + joint_states ...")

    def _fsm_cb(self, msg):
        if msg.data != self.fsm_state:
            self._log("FSM -> %s" % msg.data)
            self.fsm_state = msg.data

    def _finish_cb(self, msg):
        if msg.data and not self.finished:
            self._log("planning/finish = True  (navigation complete)")
            self.finished = True

    def _odom_cb(self, msg):
        self.odom = msg

    def _js_cb(self, msg):
        self.joints = msg

    def _log(self, text):
        self.get_logger().info(text)

    def _current_joints(self):
        if self.joints is None:
            return None
        by_name = dict(zip(self.joints.name, self.joints.position))
        try:
            return [by_name[n] for n in JOINT_NAMES]
        except KeyError:
            return None

    def _current_base(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        q = self.odom.pose.pose.orientation
        return p.x, p.y, yaw_from_quat(q)

    def wait_for_state(self, timeout=15.0):
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.odom is not None and self._current_joints() is not None:
                return True
        return False

    def build_goal(self):
        msg = WholeBodyGoal()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.joint_names = list(JOINT_NAMES)

        if self.args.mode == "sanity":
            base = self._current_base()
            joints = self._current_joints()
            if base is None or joints is None:
                return None
            x, y, yaw = base
            goal_x = x + self.args.dx
            goal_y = y + self.args.dy
            goal_yaw = yaw + self.args.dyaw
            msg.base_pose = Pose(position=_pos(goal_x, goal_y),
                                 orientation=quat_from_yaw(goal_yaw))
            msg.joint_positions = [float(v) for v in joints]
            self._log(
                "sanity goal: base=(%.3f, %.3f, %.3f) arm=measured (zero arm "
                "displacement)" % (goal_x, goal_y, goal_yaw))
        else:  # goal
            joints = [float(v) for v in self.args.joints]
            if len(joints) != 6:
                raise SystemExit("--joints must provide exactly 6 values")
            msg.base_pose = Pose(position=_pos(self.args.x, self.args.y),
                                 orientation=quat_from_yaw(self.args.yaw))
            msg.joint_positions = joints
            self._log(
                "goal: base=(%.3f, %.3f, %.3f) joints=%s" %
                (self.args.x, self.args.y, self.args.yaw, joints))

        return msg

    def run(self):
        if not self.wait_for_state():
            self._log("Timed out waiting for odom/joint_states. Is the sim "
                      "running?")
            return 2

        goal = self.build_goal()
        if goal is None:
            self._log("Failed to build goal (missing odom/joints).")
            return 2

        self.goal_pub.publish(goal)
        self._log("Published whole-body goal. Watching FSM for %.0f s ..." %
                  self.args.watch)

        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < self.args.watch:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.finished:
            self._log("RESULT: REMANI reached the whole-body goal.")
            return 0
        self._log(
            "RESULT: not finished in %.0f s (last FSM=%s). Check the REMANI "
            "log for 'opt failed' / 'not feasible' warnings." %
            (self.args.watch, self.fsm_state))
        return 1


def _pos(x, y, z=0.0):
    from geometry_msgs.msg import Point
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def parse_args(argv):
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["sanity", "goal"], default="sanity")
    ap.add_argument("--dx", type=float, default=0.5)
    ap.add_argument("--dy", type=float, default=0.0)
    ap.add_argument("--dyaw", type=float, default=0.0)
    ap.add_argument("--x", type=float, default=0.0)
    ap.add_argument("--y", type=float, default=0.0)
    ap.add_argument("--yaw", type=float, default=0.0)
    ap.add_argument("--joints", nargs="+", default=["0.0"] * 6,
                    help="6 space-separated joint positions for --mode goal")
    ap.add_argument("--watch", type=float, default=20.0)
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(sys.argv[1:] if argv is None else argv)
    rclpy.init()
    node = WholeBodyGoalTester(args)
    try:
        return node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
