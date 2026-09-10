#!/usr/bin/env python3
"""One-click whiteboard annotation for wipe_task.yaml.

Click ONE point in RViz (Publish Point toolbar button) on the physical
whiteboard; this node derives the whole wipe rectangle from the known board
size and prints a ready-to-paste ``wipe_task`` fragment. Every click
re-prints the result; the last click wins.

Prerequisite: the board must be axis-aligned with the map X axis (this is what
"one click is enough" relies on). After pasting the output, verify the overlay
with ``ros2 launch wipe_planner wipe_plan_preview.launch.py``.

Usage:
  ros2 run grid_map annotate_board --layout horizontal --anchor center
  ros2 run grid_map annotate_board --layout vertical --anchor corner --corner ll

Run ``python3 annotate_board.py --help`` for all options.
"""

import argparse
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped


def f3(v):
    return f"{v:+.3f}"


def strip_ros_args(argv):
    """Remove rclpy --ros-args ... before argparse sees them."""
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
        description="Annotate a board with one RViz click -> wipe_task yaml")
    ap.add_argument(
        "--layout", choices=["horizontal", "vertical"], default="horizontal",
        help="horizontal: board lies flat, wipe on its top face (X-Y plane); "
             "vertical: board stands on the wall (X-Z plane)")
    ap.add_argument(
        "--anchor", choices=["center", "corner"], default="center",
        help="center: the clicked point is the board center (easiest to aim); "
             "corner: it is one corner of the board")
    ap.add_argument(
        "--corner", choices=["ll", "lr", "ul", "ur"], default="ll",
        help="which corner the clicked point is (lower/upper x left/right); "
             "for vertical layout 'lower' means smaller Z")
    ap.add_argument(
        "--x-extent", type=float, default=0.90,
        help="board dimension along map X (m)")
    ap.add_argument(
        "--height", type=float, default=0.60,
        help="board dimension perpendicular to X, in-plane (m): along Y for "
             "horizontal layout, along Z for vertical layout")
    ap.add_argument(
        "--margin", type=float, default=0.03,
        help="inset from the board edges, so the wipe stays on the board (m)")
    return ap.parse_args(argv)


class BoardAnnotator(Node):
    def __init__(self, args):
        super().__init__("board_annotator")
        self.args = args
        self.last = None
        if args.margin >= 0.5 * min(args.x_extent, args.height):
            raise ValueError("margin must be smaller than half the board size")
        self.create_subscription(
            PointStamped, "/clicked_point", self.on_click, 10)
        self.get_logger().info(
            "Click ONE point in RViz (Publish Point button) on the board.\n"
            f"  layout={args.layout} anchor={args.anchor} "
            f"corner={args.corner} size={args.x_extent:.2f}x{args.height:.2f} m "
            f"margin={args.margin:.2f} m\n"
            "  Every click re-prints the wipe_task block; the last click wins.")

    def on_click(self, msg):
        p = np.array([msg.point.x, msg.point.y, msg.point.z])
        if self.last is not None and np.linalg.norm(p - self.last) < 0.05:
            return  # near-duplicate click
        self.last = p
        self.get_logger().info(
            f"clicked [{msg.header.frame_id}]: {f3(p[0])} {f3(p[1])} {f3(p[2])}")
        if msg.header.frame_id != "map":
            self.get_logger().warn(
                f"frame is '{msg.header.frame_id}', expected 'map'")
        print(self.build_yaml(p), flush=True)
        print("---", flush=True)

    def build_yaml(self, p):
        a = self.args
        w, h, m = a.x_extent, a.height, a.margin
        # Board center in the plane, derived from the clicked point.
        if a.anchor == "corner":
            x_sign = 1.0 if a.corner[0] == "l" else -1.0
            perp_sign = 1.0 if a.corner[1] == "l" else -1.0
            perp = p[2] if a.layout == "vertical" else p[1]
            cx = p[0] + x_sign * 0.5 * w
            cperp = perp + perp_sign * 0.5 * h
        else:
            cx = p[0]
            cperp = p[2] if a.layout == "vertical" else p[1]

        hw, hh = 0.5 * w - m, 0.5 * h - m
        lines = ["  surface:"]
        if a.layout == "vertical":
            plane_y = p[1]
            lines += [
                f'    type: "vertical"',
                f"    # Board stands on the wall at plane Y = {f3(plane_y)}",
                f"    center: [{f3(cx)}, {f3(plane_y)}, {f3(cperp)}]",
                "    normal_into_room: [0.0, ±1.0, 0.0]  "
                "# pick +Y or -Y: from board INTO the room (robot side)",
                f"    x_limits: [{f3(cx - hw)}, {f3(cx + hw)}]",
                f"    y_limits: [{f3(plane_y - 0.01)}, {f3(plane_y + 0.01)}]",
                f"    z_limits: [{f3(cperp - hh)}, {f3(cperp + hh)}]",
            ]
            standoff = ("[0.0, ±1.0, 0.0]  # same sign as normal_into_room")
        else:
            plane_z = p[2]
            lines += [
                f'    type: "horizontal"',
                f"    # Board lies flat at plane Z = {f3(plane_z)}",
                f"    center: [{f3(cx)}, {f3(cperp)}, {f3(plane_z)}]",
                "    normal_into_room: [0.0, 0.0, 1.0]",
                f"    x_limits: [{f3(cx - hw)}, {f3(cx + hw)}]",
                f"    y_limits: [{f3(cperp - hh)}, {f3(cperp + hh)}]",
                f"    z_limits: [{f3(plane_z)}, {f3(plane_z)}]",
            ]
            standoff = ("[0.0, ±1.0, 0.0]  # pick +Y or -Y: the side the base "
                        "drives on")
        lines += [
            "  whole_body:  # keys to update in the existing block",
            "    cleaning_yaw: 0.0  # valid only if the board is X-aligned",
            f"    base_standoff_direction: {standoff}",
            "  # NOTE: one-click mode assumes the board is axis-aligned with",
            "  # map X. Verify with wipe_plan_preview.launch.py that the",
            "  # green rectangle overlays the board before running on the robot.",
        ]
        return "\n".join(lines)


def main(argv=None):
    args = parse_args(strip_ros_args(sys.argv[1:] if argv is None else argv))
    rclpy.init()
    node = BoardAnnotator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
