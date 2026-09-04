#!/usr/bin/env python3
"""Small ROS 2 simulator used by the REMANI examples.

It replaces the ROS 1-only fake_mm/controller/local_sensing chain: a static
point cloud is published as the global map, while PolynomialTraj messages are
evaluated directly to provide ideal odometry and joint-state feedback.
"""

import math
import random
from dataclasses import dataclass
from typing import List

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from quadrotor_msgs.msg import PolynomialTraj
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header


@dataclass
class Segment:
    trajectory_id: int
    singul: int
    start_time: float
    pieces: list

    @property
    def duration(self) -> float:
        return sum(piece.duration for piece in self.pieces)


class RemaniSimulator(Node):
    def __init__(self) -> None:
        super().__init__('remani_simulator')
        self.declare_parameter('map.map_type', 0)
        self.declare_parameter('map.x_size', 8.0)
        self.declare_parameter('map.y_size', 8.0)
        self.declare_parameter('map.z_size', 3.0)
        self.declare_parameter('map.resolution', 0.05)
        self.declare_parameter('map.seed', 30)
        self.declare_parameter('fsm.init_state', [-3.0, 0.0, 0.0, -70.0, 110.0, 0.0, -110.0, 0.0])
        self.declare_parameter('fsm.init_yaw', 0.0)
        self.declare_parameter('fsm.init_gripper_close', False)
        self.declare_parameter('mm.manipulator_dof', 6)
        self.declare_parameter('auto_start', False)

        init_state = list(self.get_parameter('fsm.init_state').value)
        self.manipulator_dof = int(self.get_parameter('mm.manipulator_dof').value)
        self.position = [float(init_state[0]), float(init_state[1])]
        self.joints = [
            math.radians(float(value))
            for value in init_state[2:2 + self.manipulator_dof]
        ]
        self.velocity = [0.0, 0.0]
        self.joint_velocity = [0.0] * self.manipulator_dof
        self.yaw = math.radians(float(self.get_parameter('fsm.init_yaw').value))
        self.gripper = bool(self.get_parameter('fsm.init_gripper_close').value)
        self.segments: List[Segment] = []
        self.auto_start_sent = False
        self.startup_time = self.now_seconds()

        self.odom_pub = self.create_publisher(Odometry, '/mm/car/odom', 10)
        self.joint_pub = self.create_publisher(JointState, '/mm/mani/joint_state', 10)
        self.gripper_pub = self.create_publisher(Bool, '/mm/mani/gripper_state', 10)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(
            point_cloud2.PointCloud2, '/map_generator/global_cloud', map_qos)
        self.goal_pub = self.create_publisher(PoseStamped, '/move_base_simple/goal', 1)
        self.create_subscription(
            PolynomialTraj, '/planning/trajectory', self.trajectory_callback, 10)
        self.create_subscription(Bool, '/mm_controller_node/gripper_cmd',
                                 self.gripper_callback, 10)

        self.cloud = self.make_map()
        self.create_timer(0.01, self.update)
        self.create_timer(1.0, self.publish_map)
        self.publish_map()
        self.get_logger().info(
            f'ROS 2 example simulator ready ({len(self.cloud)} map points)')

    def now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1.0e-9

    def add_box(self, points, center, size, step) -> None:
        x0, y0, z0 = (center[i] - size[i] * 0.5 for i in range(3))
        nx, ny, nz = (max(1, math.ceil(size[i] / step)) for i in range(3))
        for ix in range(nx + 1):
            for iy in range(ny + 1):
                for iz in range(nz + 1):
                    if ix in (0, nx) or iy in (0, ny) or iz in (0, nz):
                        points.append((x0 + ix * step, y0 + iy * step,
                                       z0 + iz * step))

    def make_map(self):
        map_type = int(self.get_parameter('map.map_type').value)
        x_size = float(self.get_parameter('map.x_size').value)
        y_size = float(self.get_parameter('map.y_size').value)
        resolution = float(self.get_parameter('map.resolution').value)
        step = max(resolution, 0.05)
        points = []

        # Low boundary walls keep the search inside the configured map.
        self.add_box(points, (0.0, -y_size / 2.0, 0.1),
                     (x_size, step, 0.2), step)
        self.add_box(points, (0.0, y_size / 2.0, 0.1),
                     (x_size, step, 0.2), step)
        self.add_box(points, (-x_size / 2.0, 0.0, 0.1),
                     (step, y_size, 0.2), step)
        self.add_box(points, (x_size / 2.0, 0.0, 0.1),
                     (step, y_size, 0.2), step)

        if map_type == 1:
            # The bridge experiment: a raised board with two supporting walls.
            self.add_box(points, (0.0, 0.0, 0.75), (0.6, 1.5, 0.1), step)
            self.add_box(points, (0.0, -0.70, 0.35), (0.6, 0.1, 0.7), step)
            self.add_box(points, (0.0, 0.70, 0.35), (0.6, 0.1, 0.7), step)
            self.add_box(points, (0.0, -2.35, 0.1), (0.1, 3.3, 0.2), step)
            self.add_box(points, (0.0, 2.35, 0.1), (0.1, 3.3, 0.2), step)
        else:
            # Deterministic forest, with clearance around preset start/goals.
            rng = random.Random(int(self.get_parameter('map.seed').value))
            protected = [(-3.0, 0.0), (3.0, 1.5), (-0.5, -3.0)]
            centers = []
            attempts = 0
            while len(centers) < 14 and attempts < 1000:
                attempts += 1
                x = rng.uniform(-2.7, 2.7)
                y = rng.uniform(-3.2, 3.2)
                if any(math.hypot(x - px, y - py) < 1.0
                       for px, py in protected + centers):
                    continue
                centers.append((x, y))
                self.add_box(
                    points, (x, y, rng.uniform(0.45, 0.9)),
                    (rng.uniform(0.25, 0.5), rng.uniform(0.25, 0.5),
                     rng.uniform(0.9, 1.8)), step)
        return points

    def publish_map(self) -> None:
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'world'
        self.map_pub.publish(point_cloud2.create_cloud_xyz32(header, self.cloud))

    def trajectory_callback(self, msg: PolynomialTraj) -> None:
        if msg.action in (PolynomialTraj.ACTION_ABORT,
                          PolynomialTraj.ACTION_WARN_IMPOSSIBLE):
            self.segments.clear()
            self.velocity = [0.0, 0.0]
            self.joint_velocity = [0.0] * self.manipulator_dof
            return
        if msg.action != PolynomialTraj.ACTION_ADD or not msg.trajectory:
            return
        if msg.trajectory_id == 1:
            self.segments.clear()
        base_start = msg.header.stamp.sec + msg.header.stamp.nanosec * 1.0e-9
        offset = sum(segment.duration for segment in self.segments)
        segment = Segment(int(msg.trajectory_id), int(msg.singul),
                          base_start + offset, list(msg.trajectory))
        self.segments = [
            old for old in self.segments
            if old.trajectory_id != segment.trajectory_id
        ]
        self.segments.append(segment)
        self.segments.sort(key=lambda item: item.trajectory_id)
        self.get_logger().info(
            f'Executing trajectory segment {segment.trajectory_id} '
            f'({segment.duration:.2f} s, singul={segment.singul})')

    def gripper_callback(self, msg: Bool) -> None:
        self.gripper = bool(msg.data)

    @staticmethod
    def evaluate_piece(piece, t):
        dim = int(piece.num_dim)
        degree = int(piece.num_order)
        pos = [0.0] * dim
        vel = [0.0] * dim
        for d in range(dim):
            for column in range(degree + 1):
                power = degree - column
                coefficient = piece.data[column * dim + d]
                pos[d] += coefficient * (t ** power)
                if power:
                    vel[d] += power * coefficient * (t ** (power - 1))
        return pos, vel

    def sample_trajectory(self, now):
        for segment in self.segments:
            elapsed = now - segment.start_time
            if elapsed < 0.0:
                return None
            for piece in segment.pieces:
                if elapsed <= piece.duration:
                    pos, vel = self.evaluate_piece(
                        piece, max(0.0, min(elapsed, piece.duration)))
                    return pos, vel, segment.singul
                elapsed -= piece.duration
        if self.segments:
            last = self.segments[-1]
            piece = last.pieces[-1]
            pos, _ = self.evaluate_piece(piece, piece.duration)
            return pos, [0.0] * len(pos), last.singul
        return None

    def update(self) -> None:
        now = self.now_seconds()
        sampled = self.sample_trajectory(now)
        if sampled is not None:
            pos, vel, singul = sampled
            self.position = pos[:2]
            self.velocity = vel[:2]
            self.joints = pos[2:2 + self.manipulator_dof]
            self.joint_velocity = vel[2:2 + self.manipulator_dof]
            if math.hypot(*self.velocity) > 1.0e-4:
                self.yaw = math.atan2(self.velocity[1], self.velocity[0])
                if singul < 0:
                    self.yaw += math.pi

        stamp = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.position[0]
        odom.pose.pose.position.y = self.position[1]
        odom.pose.pose.orientation.z = math.sin(self.yaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(self.yaw * 0.5)
        odom.twist.twist.linear.x = self.velocity[0]
        odom.twist.twist.linear.y = self.velocity[1]
        self.odom_pub.publish(odom)

        joints = JointState()
        joints.header.stamp = stamp
        joints.header.frame_id = 'world'
        joints.name = [f'joint_{index + 1}' for index in range(self.manipulator_dof)]
        joints.position = self.joints
        joints.velocity = self.joint_velocity
        joints.effort = [0.0] * self.manipulator_dof
        self.joint_pub.publish(joints)
        self.gripper_pub.publish(Bool(data=self.gripper))

        if (bool(self.get_parameter('auto_start').value)
                and not self.auto_start_sent
                and now - self.startup_time > 2.0):
            goal = PoseStamped()
            goal.header.stamp = stamp
            goal.header.frame_id = 'world'
            goal.pose.orientation.w = 1.0
            self.goal_pub.publish(goal)
            self.auto_start_sent = True
            self.get_logger().info('Automatically triggered planner goal input')


def main(args=None):
    rclpy.init(args=args)
    node = RemaniSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == '__main__':
    main()
