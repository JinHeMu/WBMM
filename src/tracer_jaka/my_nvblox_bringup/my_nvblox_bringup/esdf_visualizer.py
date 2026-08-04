#!/usr/bin/env python3
"""Publish a robot-centered subset of nvblox's 3D ESDF for RViz."""

from geometry_msgs.msg import Point, Vector3
import numpy as np
from nvblox_msgs.srv import EsdfAndGradients
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener


class EsdfVisualizer(Node):
    """Convert the dense 3D ESDF service response to a bounded point cloud."""

    def __init__(self):
        """Create the ESDF service client, TF listener, and cloud publisher."""
        super().__init__('esdf_visualizer')
        self.declare_parameter(
            'service_name', '/nvblox_node/get_esdf_and_gradient')
        self.declare_parameter(
            'output_topic', '/nvblox_node/esdf_3d_pointcloud')
        self.declare_parameter('global_frame', 'odom')
        self.declare_parameter('tracking_frame', 'base_footprint')
        self.declare_parameter('follow_tracking_frame', True)
        self.declare_parameter('query_size_x_m', 4.0)
        self.declare_parameter('query_size_y_m', 4.0)
        self.declare_parameter('query_min_z_m', -0.2)
        self.declare_parameter('query_size_z_m', 3.0)
        self.declare_parameter('publish_rate_hz', 1.0)
        self.declare_parameter('max_visualized_distance_m', 1.5)
        self.declare_parameter('voxel_subsampling', 2)
        self.declare_parameter('unobserved_value', -1000.0)

        self._global_frame = self.get_parameter(
            'global_frame').get_parameter_value().string_value
        self._tracking_frame = self.get_parameter(
            'tracking_frame').get_parameter_value().string_value
        self._follow_tracking_frame = self.get_parameter(
            'follow_tracking_frame').get_parameter_value().bool_value
        self._fixed_query_center = None
        service_name = self.get_parameter(
            'service_name').get_parameter_value().string_value
        output_topic = self.get_parameter(
            'output_topic').get_parameter_value().string_value

        self._publisher = self.create_publisher(
            PointCloud2, output_topic, 1)
        self._client = self.create_client(EsdfAndGradients, service_name)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._request_pending = False
        self._warned_waiting_for_service = False

        rate_hz = max(
            0.1, self.get_parameter(
                'publish_rate_hz').get_parameter_value().double_value)
        self._timer = self.create_timer(1.0 / rate_hz, self._request_esdf)
        self.get_logger().info(
            f'3D ESDF visualization: {service_name} -> {output_topic} '
            f'at {rate_hz:.2f} Hz '
            f'({"robot-following" if self._follow_tracking_frame else "fixed"})')

    def _request_esdf(self):
        if self._request_pending:
            return
        if not self._client.service_is_ready():
            if not self._warned_waiting_for_service:
                self.get_logger().info('Waiting for the nvblox ESDF service')
                self._warned_waiting_for_service = True
            return

        try:
            transform = self._tf_buffer.lookup_transform(
                self._global_frame,
                self._tracking_frame,
                Time(),
                timeout=Duration(seconds=0.05))
        except TransformException as error:
            self.get_logger().warning(
                f'Waiting for {self._global_frame} -> '
                f'{self._tracking_frame} TF: {error}',
                throttle_duration_sec=5.0)
            return

        size_x = self.get_parameter(
            'query_size_x_m').get_parameter_value().double_value
        size_y = self.get_parameter(
            'query_size_y_m').get_parameter_value().double_value
        min_z = self.get_parameter(
            'query_min_z_m').get_parameter_value().double_value
        size_z = self.get_parameter(
            'query_size_z_m').get_parameter_value().double_value
        translation = transform.transform.translation
        if self._follow_tracking_frame:
            center_x = translation.x
            center_y = translation.y
            center_z = translation.z
        else:
            if self._fixed_query_center is None:
                self._fixed_query_center = (
                    translation.x, translation.y, translation.z)
                self.get_logger().info(
                    'Fixed ESDF visualization center at '
                    f'({translation.x:.2f}, {translation.y:.2f}, '
                    f'{translation.z:.2f}) in {self._global_frame}')
            center_x, center_y, center_z = self._fixed_query_center

        request = EsdfAndGradients.Request()
        request.update_esdf = False
        request.visualize_esdf = False
        request.use_aabb = True
        request.frame_id = self._global_frame
        request.aabb_min_m = Point(
            x=center_x - 0.5 * size_x,
            y=center_y - 0.5 * size_y,
            z=center_z + min_z)
        request.aabb_size_m = Vector3(x=size_x, y=size_y, z=size_z)

        self._request_pending = True
        future = self._client.call_async(request)
        future.add_done_callback(self._handle_response)

    def _handle_response(self, future):
        self._request_pending = False
        try:
            response = future.result()
        except Exception as error:  # noqa: B902 - ROS future exceptions vary
            self.get_logger().error(f'3D ESDF request failed: {error}')
            return
        if response is None or not response.success:
            self.get_logger().warning(
                'nvblox returned an empty ESDF grid',
                throttle_duration_sec=5.0)
            return

        dimensions = response.esdf_and_gradients.layout.dim
        if len(dimensions) != 3:
            self.get_logger().error('Expected a three-dimensional ESDF grid')
            return
        shape = tuple(int(dimension.size) for dimension in dimensions)
        values = np.asarray(
            response.esdf_and_gradients.data, dtype=np.float32)
        if values.size != int(np.prod(shape)):
            self.get_logger().error(
                f'ESDF grid shape {shape} does not match '
                f'{values.size} values')
            return
        values = values.reshape(shape)

        subsampling = max(
            1, self.get_parameter(
                'voxel_subsampling').get_parameter_value().integer_value)
        sampled_values = values[::subsampling, ::subsampling, ::subsampling]
        unobserved = self.get_parameter(
            'unobserved_value').get_parameter_value().double_value
        max_distance = self.get_parameter(
            'max_visualized_distance_m').get_parameter_value().double_value
        observed = sampled_values > (unobserved + 1.0)
        if max_distance > 0.0:
            observed &= np.abs(sampled_values) <= max_distance

        sampled_indices = np.argwhere(observed)
        if sampled_indices.size == 0:
            return
        voxel_size = float(response.voxel_size_m)
        origin = np.array([
            response.origin_m.x,
            response.origin_m.y,
            response.origin_m.z,
        ], dtype=np.float32)
        xyz = origin + (
            sampled_indices.astype(np.float32) * subsampling + 0.5
        ) * voxel_size
        distances = sampled_values[observed].reshape((-1, 1))
        points = np.hstack((xyz, distances)).astype(np.float32, copy=False)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32,
                       count=1),
            PointField(name='intensity', offset=12,
                       datatype=PointField.FLOAT32, count=1),
        ]
        header = Header(
            stamp=self.get_clock().now().to_msg(),
            frame_id=self._global_frame)
        self._publisher.publish(
            point_cloud2.create_cloud(header, fields, points))


def main(args=None):
    """Run the local ESDF visualization node."""
    rclpy.init(args=args)
    node = EsdfVisualizer()
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
