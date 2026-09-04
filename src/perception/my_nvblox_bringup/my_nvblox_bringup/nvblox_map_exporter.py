#!/usr/bin/env python3
"""Save an nvblox map and export its 3D ESDF for REMANI."""

import re
import time
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
            'save_ply_service', '/nvblox_node/save_ply')
        self.declare_parameter(
            'save_timings_service', '/nvblox_node/save_timings')
        self.declare_parameter(
            'esdf_service', '/nvblox_node/get_esdf_and_gradient')
        self.declare_parameter('map_output', 'nvblox_map.nvblx')
        self.declare_parameter('ply_output', 'nvblox_mesh.ply')
        self.declare_parameter('timings_output', 'nvblox_timings.txt')
        self.declare_parameter('esdf_output', 'remani_esdf.npz')
        self.declare_parameter('bag_path', '')
        self.declare_parameter(
            'depth_topic', '/camera/d455/depth/image_rect_raw')
        self.declare_parameter('drain_timeout_sec', 120.0)
        self.declare_parameter('drain_poll_period_sec', 1.0)
        self.declare_parameter('drain_stable_polls', 3)
        self.declare_parameter('drain_max_pending_frames', 3)
        self.declare_parameter('require_all_depth_integrated', False)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('esdf_min_x', -6.0)
        self.declare_parameter('esdf_min_y', -6.0)
        self.declare_parameter('esdf_min_z', -0.2)
        self.declare_parameter('esdf_size_x', 12.0)
        self.declare_parameter('esdf_size_y', 12.0)
        self.declare_parameter('esdf_size_z', 3.0)
        self.declare_parameter('unknown_value_threshold', -999.0)
        self.declare_parameter('unknown_is_occupied', True)
        self.declare_parameter('esdf_use_aabb', False)
        self.declare_parameter('trigger_topic', '')
        self.declare_parameter('service_call_timeout_sec', 300.0)
        self._triggered = False
        self._trigger_subscription = None
        self._service_clients = {}

    def _prepare_service_clients(self):
        """Create all nvblox clients as early as possible.

        rclpy service discovery is asynchronous.  Creating the clients while
        the bag is still playing (the offline launch does exactly that) makes
        the post-playback save_timings / save_ply / save_map / ESDF calls
        independent of a fresh DDS discovery round-trip at the very end of the
        replay.
        """
        self._get_client(FilePath, 'save_timings_service')
        self._get_client(FilePath, 'save_ply_service')
        self._get_client(FilePath, 'save_map_service')
        self._get_client(EsdfAndGradients, 'esdf_service')

    def _get_client(self, srv_type, service_parameter):
        service_name = str(self.get_parameter(service_parameter).value)
        key = (srv_type, service_name)
        if key not in self._service_clients:
            self._service_clients[key] = self.create_client(srv_type, service_name)
        return self._service_clients[key]

    def wait_for_trigger(self):
        trigger_topic = str(self.get_parameter('trigger_topic').value)
        self._prepare_service_clients()
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
            f'Waiting for mapping completion trigger on {trigger_topic}; '
            'nvblox service clients are already being discovered')
        while rclpy.ok() and not self._triggered:
            rclpy.spin_once(self, timeout_sec=1.0)
        if not rclpy.ok():
            raise RuntimeError('Interrupted while waiting for mapping trigger')
        self.get_logger().info(
            'Mapping completion received; exporting nvblox map')

    def _wait_for_service(self, client, label, timeout_sec):
        deadline = None if timeout_sec is None else (
            time.monotonic() + float(timeout_sec))
        last_log = 0.0
        while rclpy.ok():
            if client.service_is_ready():
                return
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                raise TimeoutError(
                    f'Timed out after {float(timeout_sec):.1f}s waiting for '
                    f'{label} service ({client.srv_name})')
            if now - last_log >= 2.0:
                self.get_logger().info(
                    f'Waiting for {label} service ({client.srv_name})',
                    throttle_duration_sec=2.0)
                last_log = now
            rclpy.spin_once(self, timeout_sec=0.25)
        raise RuntimeError(f'Interrupted while waiting for {label} service')

    def _call(self, client, request, label, timeout_sec):
        deadline = None if timeout_sec is None else (
            time.monotonic() + float(timeout_sec))
        self._wait_for_service(
            client, label,
            None if deadline is None else max(deadline - time.monotonic(), 0.1))
        future = client.call_async(request)
        if deadline is None:
            rclpy.spin_until_future_complete(self, future)
        else:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(f'{label} service call timed out')
            rclpy.spin_until_future_complete(self, future, timeout_sec=remaining)
        if not future.done():
            future.cancel()
            raise TimeoutError(f'{label} service call timed out')
        if future.exception() is not None:
            raise RuntimeError(f'{label} service failed: {future.exception()}')
        return future.result()

    @staticmethod
    def _timing_count(text, label):
        match = re.search(
            rf'^{re.escape(label)}\s+(\d+)\s', text, flags=re.MULTILINE)
        if match is None:
            raise RuntimeError(
                f'Cannot find {label!r} in nvblox timing statistics')
        return int(match.group(1))

    def wait_until_input_drained(self, timings_output):
        """Require every synchronized depth callback to leave the queue."""
        timings_client = self._get_client(FilePath, 'save_timings_service')
        timeout_sec = float(self.get_parameter('drain_timeout_sec').value)
        poll_sec = float(
            self.get_parameter('drain_poll_period_sec').value)
        required_stable = int(
            self.get_parameter('drain_stable_polls').value)
        deadline = time.monotonic() + timeout_sec
        stable_polls = 0
        previous_counts = None
        expected_callbacks = self._expected_depth_callbacks()
        require_all_integrated = bool(
            self.get_parameter('require_all_depth_integrated').value)
        max_pending = int(
            self.get_parameter('drain_max_pending_frames').value)
        if max_pending < 0:
            max_pending = 0

        while rclpy.ok() and time.monotonic() < deadline:
            request = FilePath.Request()
            request.file_path = str(timings_output)
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(
                    'Timed out waiting for nvblox input to drain')
            response = self._call(
                timings_client, request, 'save_timings', remaining)
            if response is None or not response.success:
                raise RuntimeError(
                    'nvblox returned success=false while saving timings')
            statistics = timings_output.read_text(encoding='utf-8')
            callback_count = self._timing_count(
                statistics, 'ros/depth_image_callback')
            processed_count = self._timing_count(statistics, 'ros/depth')
            integrated_count = self._timing_count(
                statistics, 'ros/depth/integrate')
            counts = (callback_count, processed_count, integrated_count)
            received_all = (
                expected_callbacks is None or
                callback_count >= expected_callbacks - max_pending)
            # A few depth frames can be missing from the synchronized callback
            # stream (exact-time camera_info/depth pairing or transport loss),
            # and the bag's final depth frame can wait forever for TF at its
            # own stamp. max_pending covers both missing and stuck frames.
            processed_all = processed_count >= callback_count - max_pending
            integrated_all = (
                not require_all_integrated or
                integrated_count >= callback_count - max_pending)
            drained = (
                callback_count > 0 and
                received_all and
                processed_all and
                integrated_all)
            if drained and counts == previous_counts:
                stable_polls += 1
            elif drained:
                stable_polls = 1
            else:
                stable_polls = 0
            self.get_logger().info(
                'Waiting for complete nvblox input drain: '
                f'callbacks={callback_count}, processed={processed_count}, '
                f'integrated={integrated_count}, '
                f'expected={expected_callbacks}, '
                f'max_pending={max_pending}, '
                f'require_all_integrated={require_all_integrated}, '
                f'stable={stable_polls}/{required_stable}')
            if stable_polls >= required_stable:
                self.get_logger().info(
                    'All synchronized depth inputs were processed; '
                    'starting complete-map export')
                return
            previous_counts = counts
            time.sleep(max(poll_sec, 0.1))

        raise RuntimeError(
            'Timed out waiting for nvblox input to drain; refusing to save '
            'a partial map. Check TF and queue-drop messages.')

    def _expected_depth_callbacks(self):
        """Read the bag depth count so transport loss is visible."""
        bag_path = str(self.get_parameter('bag_path').value)
        if not bag_path:
            return None
        depth_topic = str(self.get_parameter('depth_topic').value)
        try:
            import rosbag2_py

            metadata = rosbag2_py.Info().read_metadata(bag_path, 'sqlite3')
        except Exception as error:  # noqa: B902 - rosbag bindings vary
            raise RuntimeError(
                f'Cannot read ROS bag metadata from {bag_path}: {error}')
        topic_counts = {
            item.topic_metadata.name: int(item.message_count)
            for item in metadata.topics_with_message_count
        }
        if depth_topic not in topic_counts:
            raise RuntimeError(
                f'Depth topic {depth_topic!r} is absent from {bag_path}')
        expected = topic_counts[depth_topic]
        if expected <= 0:
            raise RuntimeError(
                f'Depth topic {depth_topic!r} contains no messages')
        self.get_logger().info(
            f'Bag completeness target: {expected} depth messages on '
            f'{depth_topic}')
        return expected

    def export(self):
        self._prepare_service_clients()
        service_call_timeout = float(
            self.get_parameter('service_call_timeout_sec').value)
        map_output = Path(
            self.get_parameter('map_output').value).expanduser().resolve()
        ply_output = Path(
            self.get_parameter('ply_output').value).expanduser().resolve()
        timings_output = Path(
            self.get_parameter('timings_output').value).expanduser().resolve()
        esdf_output = Path(
            self.get_parameter('esdf_output').value).expanduser().resolve()
        map_output.parent.mkdir(parents=True, exist_ok=True)
        ply_output.parent.mkdir(parents=True, exist_ok=True)
        timings_output.parent.mkdir(parents=True, exist_ok=True)
        esdf_output.parent.mkdir(parents=True, exist_ok=True)

        self.wait_until_input_drained(timings_output)

        # nvblox stores native maps in SQLite. Reusing an existing database can
        # leave stale schemas/tables and produce a partially written map, so a
        # requested one-shot export has explicit replace semantics.
        if map_output.exists():
            map_output.unlink()
            self.get_logger().info(
                f'Removed previous native map before export: {map_output}')
        if ply_output.exists():
            ply_output.unlink()
            self.get_logger().info(
                f'Removed previous PLY before export: {ply_output}')

        ply_client = self._get_client(FilePath, 'save_ply_service')
        ply_request = FilePath.Request()
        ply_request.file_path = str(ply_output)
        ply_response = self._call(
            ply_client, ply_request, 'save_ply', service_call_timeout)
        if ply_response is None or not ply_response.success:
            raise RuntimeError(
                'nvblox returned success=false while saving PLY')
        self.get_logger().info(f'Saved nvblox mesh PLY: {ply_output}')

        save_client = self._get_client(FilePath, 'save_map_service')
        save_request = FilePath.Request()
        save_request.file_path = str(map_output)
        save_response = self._call(
            save_client, save_request, 'save_map', service_call_timeout)
        if save_response is None or not save_response.success:
            raise RuntimeError(
                'nvblox returned success=false while saving map')
        self.get_logger().info(f'Saved native nvblox map: {map_output}')

        esdf_client = self._get_client(EsdfAndGradients, 'esdf_service')
        esdf_request = EsdfAndGradients.Request()
        esdf_request.update_esdf = True
        esdf_request.visualize_esdf = False
        # use_aabb=false makes nvblox return a dense grid over *all* allocated
        # ESDF blocks, i.e. the complete mapped scene instead of a fixed box.
        use_aabb = bool(self.get_parameter('esdf_use_aabb').value)
        esdf_request.use_aabb = use_aabb
        esdf_request.frame_id = self.get_parameter('frame_id').value
        if use_aabb:
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
        else:
            self.get_logger().info(
                'Requesting ESDF over all allocated nvblox blocks '
                '(use_aabb=false); the exported grid spans the full map')
        esdf_response = self._call(
            esdf_client, esdf_request, '3D ESDF', service_call_timeout)
        if esdf_response is None or not esdf_response.success:
            raise RuntimeError(
                'nvblox returned success=false for the ESDF query')

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
