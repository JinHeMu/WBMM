#!/usr/bin/env python3
"""Continuously save the newest bag-replayed OccupancyGrid as PGM/YAML."""

import math
import os
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default


class MapSnapshotSaver(Node):
    """Keep a disk snapshot of the most recently received 2D map."""

    def __init__(self):
        super().__init__('bag_map_snapshot_saver')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter(
            'output_yaml',
            'nvblox_output/bag_map.yaml')
        self._output_yaml = Path(
            self.get_parameter('output_yaml').value).expanduser().resolve()
        self._received = 0
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('map_topic').value,
            self._map_callback,
            qos_profile_system_default)
        self.get_logger().info(
            f'Saving the latest 2D map to {self._output_yaml}')

    @staticmethod
    def _yaw(quaternion):
        siny = 2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y)
        cosy = 1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z)
        return math.atan2(siny, cosy)

    def _map_callback(self, message):
        width = int(message.info.width)
        height = int(message.info.height)
        values = np.asarray(message.data, dtype=np.int16)
        if values.size != width * height or width == 0 or height == 0:
            self.get_logger().warning('Ignoring an invalid OccupancyGrid')
            return
        values = values.reshape((height, width))
        image = np.full((height, width), 205, dtype=np.uint8)
        image[values == 0] = 254
        image[values >= 65] = 0
        # PGM rows run top-to-bottom while OccupancyGrid starts at bottom-left.
        image = np.flipud(image)

        self._output_yaml.parent.mkdir(parents=True, exist_ok=True)
        output_pgm = self._output_yaml.with_suffix('.pgm')
        pgm_tmp = output_pgm.with_suffix('.pgm.tmp')
        yaml_tmp = self._output_yaml.with_suffix('.yaml.tmp')
        with pgm_tmp.open('wb') as stream:
            stream.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
            stream.write(image.tobytes())

        origin = message.info.origin.position
        yaw = self._yaw(message.info.origin.orientation)
        yaml_text = (
            f'image: {output_pgm.name}\n'
            'mode: trinary\n'
            f'resolution: {float(message.info.resolution):.9f}\n'
            f'origin: [{origin.x:.9f}, {origin.y:.9f}, {yaw:.9f}]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
        yaml_tmp.write_text(yaml_text, encoding='utf-8')
        os.replace(pgm_tmp, output_pgm)
        os.replace(yaml_tmp, self._output_yaml)
        self._received += 1
        if self._received == 1 or self._received % 20 == 0:
            self.get_logger().info(
                f'Saved 2D map snapshot #{self._received}: '
                f'{width}x{height}')


def main(args=None):
    rclpy.init(args=args)
    node = MapSnapshotSaver()
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
