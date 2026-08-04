#!/usr/bin/env python3
"""Save an nvblox map and export its 3D ESDF for REMANI."""

from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

from nvblox_msgs.srv import EsdfAndGradients, FilePath


class NvbloxMapExporter(Node):
    """One-shot map and ESDF exporter."""

    def __init__(self):
        super().__init__('nvblox_map_exporter')
        self.declare_parameter(
            'save_map_service', '/nvblox_node/save_map')
        self.declare_parameter(
            'esdf_service', '/nvblox_node/get_esdf_and_gradient')
        self.declare_parameter('map_output', 'nvblox_map.nvblx')
        self.declare_parameter('esdf_output', 'remani_esdf.npz')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('esdf_min_x', -6.0)
        self.declare_parameter('esdf_min_y', -6.0)
        self.declare_parameter('esdf_min_z', -0.2)
        self.declare_parameter('esdf_size_x', 12.0)
        self.declare_parameter('esdf_size_y', 12.0)
        self.declare_parameter('esdf_size_z', 3.0)
        self.declare_parameter('unknown_value_threshold', -999.0)
        self.declare_parameter('unknown_is_occupied', True)
        self.declare_parameter('trigger_topic', '')
        self._triggered = False
        self._trigger_subscription = None

    def wait_for_trigger(self):
        trigger_topic = str(self.get_parameter('trigger_topic').value)
        if not trigger_topic:
            return
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        def callback(message):
            if message.data:
                self._triggered = True

        self._trigger_subscription = self.create_subscription(
            Bool, trigger_topic, callback, qos)
        self.get_logger().info(
            f'Waiting for mapping completion trigger on {trigger_topic}')
        while rclpy.ok() and not self._triggered:
            rclpy.spin_once(self, timeout_sec=1.0)
        if not rclpy.ok():
            raise RuntimeError('Interrupted while waiting for mapping trigger')
        self.get_logger().info('Mapping completion received; exporting nvblox map')

    def _wait_for_service(self, client, label):
        while rclpy.ok() and not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'Waiting for {label} service')
        if not rclpy.ok():
            raise RuntimeError(f'Interrupted while waiting for {label}')

    def _call(self, client, request, label):
        self._wait_for_service(client, label)
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.exception() is not None:
            raise RuntimeError(f'{label} service failed: {future.exception()}')
        return future.result()

    def export(self):
        map_output = Path(
            self.get_parameter('map_output').value).expanduser().resolve()
        esdf_output = Path(
            self.get_parameter('esdf_output').value).expanduser().resolve()
        map_output.parent.mkdir(parents=True, exist_ok=True)
        esdf_output.parent.mkdir(parents=True, exist_ok=True)

        # nvblox stores native maps in SQLite. Reusing an existing database can
        # leave stale schemas/tables and produce a partially written map, so a
        # requested one-shot export has explicit replace semantics.
        if map_output.exists():
            map_output.unlink()
            self.get_logger().info(
                f'Removed previous native map before export: {map_output}')

        save_client = self.create_client(
            FilePath, self.get_parameter('save_map_service').value)
        save_request = FilePath.Request()
        save_request.file_path = str(map_output)
        save_response = self._call(save_client, save_request, 'save_map')
        if save_response is None or not save_response.success:
            raise RuntimeError('nvblox returned success=false while saving map')
        self.get_logger().info(f'Saved native nvblox map: {map_output}')

        esdf_client = self.create_client(
            EsdfAndGradients, self.get_parameter('esdf_service').value)
        esdf_request = EsdfAndGradients.Request()
        esdf_request.update_esdf = True
        esdf_request.visualize_esdf = False
        esdf_request.use_aabb = True
        esdf_request.frame_id = self.get_parameter('frame_id').value
        aabb_min = [
            self.get_parameter('esdf_min_x').value,
            self.get_parameter('esdf_min_y').value,
            self.get_parameter('esdf_min_z').value,
        ]
        aabb_size = [
            self.get_parameter('esdf_size_x').value,
            self.get_parameter('esdf_size_y').value,
            self.get_parameter('esdf_size_z').value,
        ]
        esdf_request.aabb_min_m.x = float(aabb_min[0])
        esdf_request.aabb_min_m.y = float(aabb_min[1])
        esdf_request.aabb_min_m.z = float(aabb_min[2])
        esdf_request.aabb_size_m.x = float(aabb_size[0])
        esdf_request.aabb_size_m.y = float(aabb_size[1])
        esdf_request.aabb_size_m.z = float(aabb_size[2])
        esdf_response = self._call(esdf_client, esdf_request, '3D ESDF')
        if esdf_response is None or not esdf_response.success:
            raise RuntimeError('nvblox returned success=false for the ESDF query')

        dimensions = esdf_response.esdf_and_gradients.layout.dim
        labels = [dimension.label for dimension in dimensions]
        shape = tuple(int(dimension.size) for dimension in dimensions)
        if labels != ['x', 'y', 'z'] or len(shape) != 3:
            raise RuntimeError(
                f'Expected nvblox ESDF layout [x,y,z], got {labels}')
        raw_esdf = np.asarray(
            esdf_response.esdf_and_gradients.data, dtype=np.float32)
        if raw_esdf.size != int(np.prod(shape)):
            raise RuntimeError(
                f'ESDF payload has {raw_esdf.size} values for shape {shape}')
        raw_esdf = raw_esdf.reshape(shape)

        voxel_size = np.float32(esdf_response.voxel_size_m)
        origin = np.asarray([
            esdf_response.origin_m.x,
            esdf_response.origin_m.y,
            esdf_response.origin_m.z,
        ], dtype=np.float32)
        bounds_max = origin + voxel_size * np.asarray(shape, dtype=np.float32)
        unknown_threshold = float(
            self.get_parameter('unknown_value_threshold').value)
        observed = raw_esdf > unknown_threshold
        unknown_is_occupied = bool(
            self.get_parameter('unknown_is_occupied').value)

        esdf = raw_esdf.copy()
        if unknown_is_occupied:
            # A finite negative value keeps unknown space conservative without
            # injecting nvblox's -1000 sentinel into the optimizer.
            esdf[~observed] = -voxel_size
            occupancy = (~observed) | (esdf <= 0.0)
        else:
            known = raw_esdf[observed]
            free_distance = max(
                float(np.max(known)) if known.size else 2.0,
                float(voxel_size))
            esdf[~observed] = free_distance
            occupancy = observed & (esdf <= 0.0)

        np.savez_compressed(
            esdf_output,
            esdf=np.ascontiguousarray(esdf, dtype=np.float32),
            occupancy=np.ascontiguousarray(occupancy, dtype=np.bool_),
            observed=np.ascontiguousarray(observed, dtype=np.bool_),
            origin=origin,
            voxel_size=voxel_size,
            bounds_max=bounds_max.astype(np.float32),
            frame_id=np.asarray(esdf_request.frame_id),
            source=np.asarray('nvblox/EsdfAndGradients'),
            unknown_is_occupied=np.asarray(unknown_is_occupied),
        )
        observed_count = int(np.count_nonzero(observed))
        occupied_count = int(np.count_nonzero(occupancy))
        self.get_logger().info(
            f'Saved REMANI ESDF: {esdf_output}; shape={shape}, '
            f'voxel={float(voxel_size):.3f} m, observed={observed_count}, '
            f'occupied={occupied_count}, origin={origin.tolist()}')


def main(args=None):
    rclpy.init(args=args)
    node = NvbloxMapExporter()
    exit_code = 0
    try:
        node.wait_for_trigger()
        node.export()
    except Exception as error:  # noqa: B902 - ROS service exceptions vary
        node.get_logger().error(str(error))
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
