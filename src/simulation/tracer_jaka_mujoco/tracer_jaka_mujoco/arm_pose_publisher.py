#!/usr/bin/env python3
"""Publish a fixed "arm-up" joint state so the full TF tree is available."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# Default parking pose — arm folded up, close to the body.
DEFAULT_POSE = {
    "joint_1": 0.0,
    "joint_2": 1.5707,
    "joint_3": 0.0,
    "joint_4": 1.5707,
    "joint_5": 3.14159,
    "joint_6": 0.785398,
}


class ArmPosePublisher(Node):
    def __init__(self):
        super().__init__("arm_pose_publisher")
        self._pub = self.create_publisher(JointState, "/joint_states", 10)
        self._timer = self.create_timer(0.05, self._publish)  # 20 Hz
        self.get_logger().info(
            "Publishing default arm-up joint states at 20 Hz"
        )

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(DEFAULT_POSE.keys())
        msg.position = list(DEFAULT_POSE.values())
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ArmPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
