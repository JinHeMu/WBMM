#!/usr/bin/env python3
"""Publish one deterministic REMANI 2D goal when the planner is ready."""

import math

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


class DemoGoalPublisher(Node):
    def __init__(self):
        super().__init__('demo_goal_publisher')
        self.declare_parameter('goal_x', 3.8)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_yaw', 0.0)
        self.declare_parameter('frame_id', 'odom')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.published = False
        self.create_timer(0.5, self._publish_when_ready)

    def _publish_when_ready(self):
        if self.published or self.publisher.get_subscription_count() == 0:
            return
        yaw = float(self.get_parameter('goal_yaw').value)
        message = PoseStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = str(self.get_parameter('frame_id').value)
        message.pose.position.x = float(self.get_parameter('goal_x').value)
        message.pose.position.y = float(self.get_parameter('goal_y').value)
        message.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.orientation.w = math.cos(0.5 * yaw)
        self.publisher.publish(message)
        self.published = True
        self.get_logger().info(
            'Published REMANI demo goal: '
            f'({message.pose.position.x:.2f}, '
            f'{message.pose.position.y:.2f}, yaw={yaw:.2f}) in '
            f'{message.header.frame_id}')


def main(args=None):
    rclpy.init(args=args)
    node = DemoGoalPublisher()
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
