#!/usr/bin/env python3
"""Bridge REMANI navigation events to the OCS2 dual-reference task phase.

Events:
  - A new /goal_pose request       -> Navigation (0)
  - REMANI /planning/finish true   -> stay Navigation, wait for EE target
  - Base odometry reaches the last 2D goal -> stay Navigation, wait for EE target
  - An EE target arrives           -> Execution (2) directly

The odometry fallback keeps the demo usable on REMANI versions that do not
publish /planning/finish for manual 2D goals. The actual phase change is
always performed through the WbmmMpcNode SetTaskPhase service.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from ocs2_msgs.msg import MpcTargetTrajectories
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from wbmm_ocs2_ros.srv import SetTaskPhase


def _yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z +
               quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y +
                     quaternion.z * quaternion.z),
    )


def _wrap_to_pi(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class RemaniPhaseBridge(Node):
    def __init__(self):
        super().__init__("remani_phase_bridge")

        self.declare_parameter("phase_service", "/mobile_manipulator_set_task_phase")
        self.declare_parameter("goal_topic", "/goal_pose")
        self.declare_parameter("finish_topic", "/planning/finish")
        self.declare_parameter("odom_topic", "/wheel/odometry")
        self.declare_parameter("ee_target_topic", "/mobile_manipulator_ee_target")
        self.declare_parameter("navigation_phase", 0)
        self.declare_parameter("execution_phase", 2)
        self.declare_parameter("goal_position_tolerance", 0.15)
        self.declare_parameter("goal_yaw_tolerance", 0.30)
        self.declare_parameter("goal_hold_time", 1.0)
        self.declare_parameter("enable_odom_fallback", True)

        self.phase_service = str(self.get_parameter("phase_service").value)
        goal_topic = str(self.get_parameter("goal_topic").value)
        finish_topic = str(self.get_parameter("finish_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)
        ee_target_topic = str(self.get_parameter("ee_target_topic").value)
        self.navigation_phase = int(self.get_parameter("navigation_phase").value)
        self.execution_phase = int(self.get_parameter("execution_phase").value)
        self.position_tolerance = float(
            self.get_parameter("goal_position_tolerance").value)
        self.yaw_tolerance = float(
            self.get_parameter("goal_yaw_tolerance").value)
        self.goal_hold_time = float(self.get_parameter("goal_hold_time").value)
        self.enable_odom_fallback = bool(
            self.get_parameter("enable_odom_fallback").value)

        self.client = self.create_client(SetTaskPhase, self.phase_service)
        qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(PoseStamped, goal_topic, self.on_goal, qos)
        self.create_subscription(Bool, finish_topic, self.on_finish, qos)
        self.create_subscription(Odometry, odom_topic, self.on_odom, qos)
        self.create_subscription(
            MpcTargetTrajectories, ee_target_topic, self.on_ee_target, qos)

        self._goal = None
        self._goal_reached_since = None
        self._navigation_done = False
        self._execution_requested = False
        self.get_logger().info(
            f"REMANI phase bridge: goal={goal_topic} -> phase "
            f"{self.navigation_phase}; navigation finish/odom arrival "
            f"keeps Navigation until an EE target arrives -> phase "
            f"{self.execution_phase}; service={self.phase_service}")

    def on_goal(self, message):
        position = message.pose.position
        yaw = _yaw_from_quaternion(message.pose.orientation)
        self._goal = (float(position.x), float(position.y), float(yaw))
        self._goal_reached_since = None
        self._navigation_done = False
        self._execution_requested = False
        self.request_phase(self.navigation_phase, "new navigation goal")

    def on_finish(self, message):
        if not message.data:
            self._navigation_done = False
            return
        if self._navigation_done:
            return
        self._navigation_done = True
        self._goal_reached_since = None
        self.get_logger().info(
            "REMANI navigation finished; staying in Navigation and waiting "
            "for a fresh end-effector target.")

    def on_ee_target(self, message):
        if len(message.state_trajectory) == 0:
            return
        if self._execution_requested:
            return
        self._execution_requested = True
        self.request_phase(
            self.execution_phase, "end-effector target received")

    def on_odom(self, message):
        if (not self.enable_odom_fallback or self._goal is None or
                self._navigation_done):
            return

        goal_x, goal_y, goal_yaw = self._goal
        dx = float(message.pose.pose.position.x) - goal_x
        dy = float(message.pose.pose.position.y) - goal_y
        position_error = math.hypot(dx, dy)
        yaw_error = abs(_wrap_to_pi(
            _yaw_from_quaternion(message.pose.pose.orientation) - goal_yaw))

        within_tolerance = (
            position_error <= self.position_tolerance and
            yaw_error <= self.yaw_tolerance)
        if not within_tolerance:
            self._goal_reached_since = None
            return

        now_sec = self._now_seconds()
        if self._goal_reached_since is None:
            self._goal_reached_since = now_sec
            return
        if now_sec - self._goal_reached_since < self.goal_hold_time:
            return

        self._navigation_done = True
        self._goal_reached_since = None
        self.get_logger().info(
            f"Base reached goal ({position_error:.3f} m, "
            f"{yaw_error:.3f} rad) and held for {self.goal_hold_time:.2f} s; "
            "staying in Navigation until a fresh EE target arrives.")

    def _now_seconds(self):
        return self.get_clock().now().nanoseconds * 1.0e-9

    def request_phase(self, phase, reason):
        if not self.client.service_is_ready():
            self.get_logger().warning(
                f"Cannot switch to phase {phase} ({reason}): "
                f"service {self.phase_service} is not ready yet.")
            return

        request = SetTaskPhase.Request()
        request.phase = int(phase)
        future = self.client.call_async(request)
        future.add_done_callback(
            lambda done, p=phase, r=reason: self.on_phase_response(done, p, r))

    def on_phase_response(self, future, phase, reason):
        try:
            response = future.result()
        except Exception as error:  # noqa: BLE001 - service exceptions vary
            self.get_logger().error(
                f"Task phase {phase} request failed ({reason}): {error}")
            if phase == self.execution_phase:
                self._execution_requested = False
            return
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            self.get_logger().error(
                f"Task phase {phase} rejected ({reason}): {message}")
            if phase == self.execution_phase:
                self._execution_requested = False
            return
        self.get_logger().info(
            f"Task phase -> {phase} ({reason}); requested="
            f"{response.requested_phase}, active={response.active_phase}")


def main(args=None):
    rclpy.init(args=args)
    node = RemaniPhaseBridge()
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
