#!/usr/bin/env python3
# =============================================================================
#  set_task_phase.py
#
#  通过 MPC 节点的 SetTaskPhase service 切换双参考模式:
#    0 = Navigation, 1 = Transition, 2 = Execution, 3 = Retract
#
#  示例:
#    ros2 run wbmm_ocs2_ros set_task_phase.py 2
# =============================================================================

import sys

import rclpy
from rclpy.node import Node

from wbmm_ocs2_ros.srv import SetTaskPhase


PHASE_NAMES = {
    0: "Navigation",
    1: "Transition",
    2: "Execution",
    3: "Retract",
}


def main():
    if len(sys.argv) != 2:
        print("Usage: set_task_phase.py <0|1|2|3>")
        print("  0=Navigation, 1=Transition, 2=Execution, 3=Retract")
        sys.exit(1)

    try:
        phase = int(sys.argv[1])
    except ValueError:
        print(f"Invalid phase: {sys.argv[1]!r}")
        sys.exit(1)

    if phase not in PHASE_NAMES:
        print("phase must be in [0, 3]")
        sys.exit(1)

    rclpy.init()
    node = Node("set_task_phase_client")
    client = node.create_client(SetTaskPhase, "/mobile_manipulator_set_task_phase")

    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().error(
            "Service /mobile_manipulator_set_task_phase is not available.")
        rclpy.shutdown()
        sys.exit(1)

    request = SetTaskPhase.Request()
    request.phase = phase
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)

    if not future.done() or future.result() is None:
        node.get_logger().error("Task phase service call failed.")
        rclpy.shutdown()
        sys.exit(1)

    response = future.result()
    if response.success:
        node.get_logger().info(
            f"Requested phase {phase} ({PHASE_NAMES[phase]}), "
            f"requested={response.requested_phase}, "
            f"active={response.active_phase}.")
    else:
        node.get_logger().error(f"Task phase rejected: {response.message}")
        rclpy.shutdown()
        sys.exit(1)

    rclpy.shutdown()


if __name__ == "__main__":
    main()
