#!/usr/bin/env python3
"""Publish the wipe_task.yaml board region as a blue marker in RViz.

Reads the ``surface`` block of wipe_task.yaml and publishes the board region
as a translucent blue CUBE (MarkerArray, transient-local). The file is
re-read every second, so editing wipe_task.yaml updates the RViz display
live. Run it next to any launch that has RViz:

  ros2 run grid_map board_marker --frame odom

Then in RViz add a MarkerArray display on topic ``/wipe_task/board_marker``.
"""

import argparse
import sys

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


def strip_ros_args(argv):
    out = []
    i = 0
    while i < len(argv):
        if argv[i] == "--ros-args":
            i += 1
            while i < len(argv) and argv[i] != "--":
                i += 1
            if i < len(argv):
                i += 1
        else:
            out.append(argv[i])
            i += 1
    return out


def parse_args(argv):
    ap = argparse.ArgumentParser(
        description="Publish wipe_task.yaml board region as blue marker")
    ap.add_argument(
        "--task-file", type=str,
        required=True,
        help="wipe_task.yaml path (re-read every second)")
    ap.add_argument(
        "--frame", type=str, default="odom",
        help="marker frame; wipe_task.yaml is expressed in odom, and in the "
             "validation launch odom coincides with map at zero offset")
    ap.add_argument(
        "--topic", type=str, default="/wipe_task/board_marker",
        help="MarkerArray topic")
    return ap.parse_args(argv)


def board_marker_from_yaml(args, stamp):
    """Build the MarkerArray from the current wipe_task.yaml contents."""
    markers = MarkerArray()

    clear = Marker()
    clear.header.frame_id = args.frame
    clear.header.stamp = stamp
    clear.ns = "wipe_board"
    clear.action = Marker.DELETEALL
    markers.markers.append(clear)

    try:
        with open(args.task_file, "r", encoding="utf-8") as handle:
            surface = yaml.safe_load(handle)["wipe_task"]["surface"]
    except (OSError, KeyError, yaml.YAMLError) as exc:
        raise RuntimeError(f"cannot read task file {args.task_file}: {exc}")

    def vec(key):
        value = surface[key]
        return [float(v) for v in value]

    surface_type = surface.get("type", "horizontal")
    center = vec("center")
    x_limits = vec("x_limits")
    z_limits = vec("z_limits")
    y_limits = vec("y_limits") if "y_limits" in surface \
        else [center[1], center[1]]

    marker = Marker()
    marker.header.frame_id = args.frame
    marker.header.stamp = stamp
    marker.ns = "wipe_board"
    marker.id = 0
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    if surface_type == "horizontal":
        marker.pose.position.x = 0.5 * (x_limits[0] + x_limits[1])
        marker.pose.position.y = 0.5 * (y_limits[0] + y_limits[1])
        marker.pose.position.z = center[2] - 0.025
        marker.pose.orientation.w = 1.0
        marker.scale.x = x_limits[1] - x_limits[0]
        marker.scale.y = y_limits[1] - y_limits[0]
        marker.scale.z = 0.05
    else:
        # Vertical: the wall direction is cross(Z, normal_into_room), with the
        # sign chosen so x_limits are natural coordinates along the wall (a
        # normal Y-plane wall reduces exactly to odom X). Mirrors the C++
        # Planner::wallDirection().
        import math
        nx, ny, _ = vec("normal_into_room")
        wx, wy = -ny, nx
        if math.hypot(wx, wy) < 1e-6:
            wx, wy = 1.0, 0.0
        norm = math.hypot(wx, wy)
        wx, wy = wx / norm, wy / norm
        limits_mid = 0.5 * (x_limits[0] + x_limits[1])
        if limits_mid > center[0] * wx + center[1] * wy:
            wx, wy = -wx, -wy
        wall_offset = limits_mid - (center[0] * wx + center[1] * wy)
        marker.pose.position.x = center[0] + wx * wall_offset
        marker.pose.position.y = center[1] + wy * wall_offset
        marker.pose.position.z = 0.5 * (z_limits[0] + z_limits[1])
        yaw = math.atan2(wy, wx)
        marker.pose.orientation.z = math.sin(0.5 * yaw)
        marker.pose.orientation.w = math.cos(0.5 * yaw)
        marker.scale.x = x_limits[1] - x_limits[0]
        marker.scale.y = 0.05
        marker.scale.z = z_limits[1] - z_limits[0]
    marker.color.r = 0.15
    marker.color.g = 0.45
    marker.color.b = 0.95
    marker.color.a = 0.65
    markers.markers.append(marker)
    return markers


class BoardMarkerNode(Node):
    def __init__(self, args):
        super().__init__("board_marker")
        self.args = args
        qos = QoSProfile(depth=1,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.publisher = self.create_publisher(MarkerArray, args.topic, qos)
        self.timer = self.create_timer(1.0, self.publish_once)
        self.get_logger().info(
            f"publishing {args.task_file} board region to {args.topic} "
            f"(frame {args.frame}), every second")

    def publish_once(self):
        try:
            markers = board_marker_from_yaml(
                self.args, self.get_clock().now().to_msg())
        except RuntimeError as exc:
            self.get_logger().error(str(exc))
            return
        self.publisher.publish(markers)


def main(argv=None):
    args = parse_args(strip_ros_args(sys.argv[1:] if argv is None else argv))
    rclpy.init()
    node = BoardMarkerNode(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
