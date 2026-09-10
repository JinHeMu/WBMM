#!/usr/bin/env python3
"""Drive a deterministic sensor-coverage route for the nvblox demo scene."""

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class EsdfMappingScan(Node):
    """Rotate once, then follow a known-free coverage route through the scene."""

    def __init__(self):
        super().__init__('esdf_mapping_scan')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('linear_speed', 0.35)
        self.declare_parameter('angular_speed', 0.55)
        self.declare_parameter('start_delay', 6.0)

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)
        self.start_delay = float(self.get_parameter('start_delay').value)
        self.cmd_pub = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10)
        done_qos = QoSProfile(depth=1)
        done_qos.reliability = ReliabilityPolicy.RELIABLE
        done_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.done_pub = self.create_publisher(
            Bool, '/esdf_mapping_scan_done', done_qos)
        self.create_subscription(
            Odometry,
            self.get_parameter('odom_topic').value,
            self._odom_callback,
            20)
        self.create_timer(0.05, self._control)

        # Odom coordinates; MuJoCo world and odometry share the same origin.
        self.waypoints = [
            (0.0, -1.20),
            (1.25, -1.20),
            (2.25, -0.75),
            (2.55, 0.0),
            (3.35, 0.0),
            (3.75, 0.75),
        ]
        self.pose = None
        self.start_time = None
        self.phase = 'delay'
        self.spin_accumulated = 0.0
        self.last_spin_yaw = None
        self.waypoint_index = 0
        self.final_spin_accumulated = 0.0
        self.last_final_yaw = None
        self.finished = False
        self.get_logger().info(
            'Coverage scan ready: initial 360-degree scan, 6 waypoints, '
            'then a final 360-degree scan.')

    def _odom_callback(self, message):
        q = message.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        p = message.pose.pose.position
        self.pose = (float(p.x), float(p.y), yaw)
        if self.start_time is None:
            self.start_time = self.get_clock().now()

    def _publish(self, linear=0.0, angular=0.0):
        command = Twist()
        command.linear.x = float(linear)
        command.angular.z = float(angular)
        self.cmd_pub.publish(command)

    def _spin_control(self, final=False):
        yaw = self.pose[2]
        if final:
            if self.last_final_yaw is None:
                self.last_final_yaw = yaw
            self.final_spin_accumulated += abs(
                wrap_angle(yaw - self.last_final_yaw))
            self.last_final_yaw = yaw
            complete = self.final_spin_accumulated >= 2.0 * math.pi
        else:
            if self.last_spin_yaw is None:
                self.last_spin_yaw = yaw
            self.spin_accumulated += abs(wrap_angle(yaw - self.last_spin_yaw))
            self.last_spin_yaw = yaw
            complete = self.spin_accumulated >= 2.0 * math.pi
        if not complete:
            self._publish(angular=self.angular_speed)
        return complete

    def _control(self):
        if self.pose is None or self.finished:
            return
        now = self.get_clock().now()
        if self.phase == 'delay':
            elapsed = (now - self.start_time).nanoseconds * 1e-9
            self._publish()
            if elapsed >= self.start_delay:
                self.phase = 'initial_spin'
                self.get_logger().info('Starting initial 360-degree RGB-D scan')
            return

        if self.phase == 'initial_spin':
            if self._spin_control():
                self.phase = 'waypoints'
                self._publish()
                self.get_logger().info('Initial scan complete; following coverage route')
            return

        if self.phase == 'waypoints':
            target_x, target_y = self.waypoints[self.waypoint_index]
            x, y, yaw = self.pose
            dx = target_x - x
            dy = target_y - y
            distance = math.hypot(dx, dy)
            if distance < 0.12:
                self.waypoint_index += 1
                self._publish()
                if self.waypoint_index >= len(self.waypoints):
                    self.phase = 'final_spin'
                    self.get_logger().info(
                        'Coverage route complete; starting final 360-degree scan')
                else:
                    self.get_logger().info(
                        f'Reached mapping waypoint {self.waypoint_index}/'
                        f'{len(self.waypoints)}')
                return
            heading_error = wrap_angle(math.atan2(dy, dx) - yaw)
            angular = max(
                -self.angular_speed,
                min(self.angular_speed, 1.5 * heading_error))
            linear = 0.0
            if abs(heading_error) < 0.55:
                linear = min(self.linear_speed, 0.6 * distance)
                linear *= max(0.2, 1.0 - abs(heading_error) / 0.7)
            self._publish(linear=linear, angular=angular)
            return

        if self.phase == 'final_spin':
            if self._spin_control(final=True):
                self._publish()
                self.finished = True
                self.done_pub.publish(Bool(data=True))
                self.get_logger().info(
                    'ESDF mapping coverage scan complete. The map can now be exported.')


def main(args=None):
    rclpy.init(args=args)
    node = EsdfMappingScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
