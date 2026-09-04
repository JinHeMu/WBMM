#!/usr/bin/env python3
"""Relay /odometry/filtered (odom frame) as /odometry/filtered_map (map frame).

REMANI plans in the persistent map frame while OCS2/MRT continues to control in
the continuous odom frame. This node transforms only the pose part of the
odometry into map; twist is kept in the body frame, matching REMANI's
odom_twist_in_body_frame:=true convention.
"""

import math
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener, TransformException


class OdomToMapRelay(Node):
    def __init__(self):
        super().__init__('odom_to_map_relay')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('output_topic', '/odometry/filtered_map')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('child_frame', 'base_footprint')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub = self.create_publisher(
            Odometry,
            self.get_parameter('output_topic').value,
            10)
        self.sub = self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self.odom_callback,
            10)

    def odom_callback(self, msg):
        map_frame = self.get_parameter('map_frame').value
        odom_frame = self.get_parameter('odom_frame').value
        child_frame = self.get_parameter('child_frame').value
        try:
            # AMCL may stop refreshing map->odom while the robot is still,
            # while the EKF keeps publishing newer odometry timestamps.  An
            # exact-time lookup then repeatedly extrapolates past the newest
            # AMCL transform and the relay publishes nothing.  For the current
            # EKF pose, compose with the latest global localization correction.
            transform = self.tf_buffer.lookup_transform(
                map_frame, odom_frame, Time(),
                timeout=Duration(seconds=0.1))
        except TransformException as exc:
            self.get_logger().warning(
                f'Cannot transform {odom_frame} -> {map_frame}: {exc}',
                throttle_duration_sec=2.0)
            return

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = map_frame
        out.child_frame_id = child_frame

        # Position: p_map = R_map_odom * p_odom + t_map_odom
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        t = transform.transform.translation
        q = transform.transform.rotation
        # Apply the quaternion rotation (only yaw is meaningful in 2D).
        # Use tf2 geometry for correctness; here we implement 2D rotation.
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        c, s = math.cos(yaw), math.sin(yaw)
        out.pose.pose.position.x = t.x + c * x - s * y
        out.pose.pose.position.y = t.y + s * x + c * y
        out.pose.pose.position.z = t.z + z

        # Orientation: compose R_map_odom * R_odom_base
        msg_yaw = math.atan2(
            2.0 * (msg.pose.pose.orientation.w * msg.pose.pose.orientation.z +
                   msg.pose.pose.orientation.x * msg.pose.pose.orientation.y),
            1.0 - 2.0 * (msg.pose.pose.orientation.y * msg.pose.pose.orientation.y +
                         msg.pose.pose.orientation.z * msg.pose.pose.orientation.z))
        total_yaw = yaw + msg_yaw
        out.pose.pose.orientation.z = math.sin(total_yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(total_yaw / 2.0)
        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0

        # Keep twist in body frame. REMANI's odom_twist_in_body_frame:=true
        # will rotate it by the map-frame yaw internally.
        out.twist = msg.twist
        for i in range(36):
            out.pose.covariance[i] = msg.pose.covariance[i]
        for i in range(36):
            out.twist.covariance[i] = msg.twist.covariance[i]
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToMapRelay()
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
