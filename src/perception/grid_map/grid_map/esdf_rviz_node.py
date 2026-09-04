#!/usr/bin/env python3

from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    HistoryPolicy,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker


class EsdfRvizPublisher(Node):
    def __init__(self):
        super().__init__("esdf_rviz_publisher")

        self.declare_parameter(
            "esdf_file", "maps/tracer_jaka_zu5_scene_esdf.npz"
        )
        self.declare_parameter("frame_id", "map")

        # z_slice < 0 表示显示完整 3D ESDF
        # z_slice >= 0 表示只显示某个高度的切片
        self.declare_parameter("z_slice", 0.1)

        self.declare_parameter("max_distance", 0.8)
        self.declare_parameter("stride", 2)
        self.declare_parameter("publish_period", 0.5)
        self.declare_parameter("include_unknown", False)
        self.declare_parameter("publish_surface_mesh", True)
        self.declare_parameter("ply_file", "")
        self.declare_parameter("publish_ply_mesh", False)
        self.declare_parameter("offset_x", 0.0)
        self.declare_parameter("offset_y", 0.0)
        self.declare_parameter("offset_z", 0.0)

        # 2D 投影高度范围，适合移动底盘
        self.declare_parameter("z_min_2d", 0.05)
        self.declare_parameter("z_max_2d", 0.6)

        self.esdf_file = (
            self.get_parameter("esdf_file")
            .get_parameter_value()
            .string_value
        )
        self.frame_id = (
            self.get_parameter("frame_id")
            .get_parameter_value()
            .string_value
        )
        self.z_slice = (
            self.get_parameter("z_slice")
            .get_parameter_value()
            .double_value
        )
        self.max_distance = (
            self.get_parameter("max_distance")
            .get_parameter_value()
            .double_value
        )
        self.stride = (
            self.get_parameter("stride")
            .get_parameter_value()
            .integer_value
        )
        self.publish_period = (
            self.get_parameter("publish_period")
            .get_parameter_value()
            .double_value
        )
        self.z_min_2d = (
            self.get_parameter("z_min_2d")
            .get_parameter_value()
            .double_value
        )
        self.z_max_2d = (
            self.get_parameter("z_max_2d")
            .get_parameter_value()
            .double_value
        )
        self.include_unknown = (
            self.get_parameter("include_unknown")
            .get_parameter_value()
            .bool_value
        )
        self.publish_surface_mesh = (
            self.get_parameter("publish_surface_mesh")
            .get_parameter_value()
            .bool_value
        )
        self.ply_file = (
            self.get_parameter("ply_file")
            .get_parameter_value()
            .string_value
        )
        self.publish_ply_mesh = (
            self.get_parameter("publish_ply_mesh")
            .get_parameter_value()
            .bool_value
        )
        self.offset = np.asarray([
            self.get_parameter("offset_x").value,
            self.get_parameter("offset_y").value,
            self.get_parameter("offset_z").value,
        ], dtype=float)

        if self.stride < 1:
            self.stride = 1

        data = np.load(self.esdf_file)

        self.esdf = data["esdf"]
        self.occ = data["occupancy"]
        self.observed = (
            data["observed"].astype(bool)
            if "observed" in data.files
            else np.ones_like(self.occ, dtype=bool)
        )
        # The offset is for a measured, fixed alignment between the saved ESDF
        # and the persistent map frame.  Leave all three values at zero unless
        # that alignment has been calibrated.
        self.origin = data["origin"].astype(float) + self.offset
        self.voxel_size = float(data["voxel_size"])

        # ROS2 中 latch 的近似等价是 transient_local durability。
        # 这样 RViz2 后打开时也能收到最后一次发布的地图。
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.cloud_pub = self.create_publisher(
            PointCloud2,
            "/esdf_cloud",
            qos,
        )

        self.grid_pub = self.create_publisher(
            OccupancyGrid,
            "/esdf_occ2d",
            qos,
        )

        self.mesh_pub = self.create_publisher(
            Marker,
            "/esdf_surface_mesh",
            qos,
        )

        self.ply_mesh_pub = self.create_publisher(
            Marker,
            "/nvblox_ply_mesh",
            qos,
        )

        self.surface_mesh = (
            self.make_surface_mesh()
            if self.publish_surface_mesh
            else None
        )
        self.surface_mesh_published = False
        self.ply_mesh = (
            self.make_ply_mesh()
            if self.publish_ply_mesh
            else None
        )
        self.ply_mesh_published = False

        self.timer = self.create_timer(
            self.publish_period,
            self.publish_maps,
        )

        self.get_logger().info(f"Loaded ESDF file: {self.esdf_file}")
        self.get_logger().info(f"ESDF shape: {self.esdf.shape}")
        self.get_logger().info(f"Origin: {self.origin}")
        self.get_logger().info(f"Visualization offset: {self.offset}")
        self.get_logger().info(f"Voxel size: {self.voxel_size}")
        self.get_logger().info(f"Frame id: {self.frame_id}")
        self.get_logger().info(
            "3D ESDF display: "
            f"stride={self.stride}, max_distance={self.max_distance}, "
            f"include_unknown={self.include_unknown}"
        )

    def make_header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        return header

    def make_esdf_cloud(self):
        """
        发布 PointCloud2:
          x, y, z: 体素中心坐标
          intensity: ESDF 值，单位 m
        """
        esdf = self.esdf
        origin = self.origin
        voxel_size = self.voxel_size
        stride = self.stride
        max_distance = self.max_distance

        nx, ny, nz = esdf.shape

        if self.z_slice >= 0.0:
            k = int(round((self.z_slice - origin[2]) / voxel_size))
            k = int(np.clip(k, 0, nz - 1))

            esdf_slice = esdf[::stride, ::stride, k]

            ii, jj = np.meshgrid(
                np.arange(0, nx, stride),
                np.arange(0, ny, stride),
                indexing="ij",
            )

            dd = esdf_slice

            if self.include_unknown:
                visible = np.ones_like(esdf_slice, dtype=bool)
            else:
                visible = self.observed[::stride, ::stride, k]
            mask = visible
            if max_distance > 0.0:
                mask &= np.abs(dd) <= max_distance

            xs = origin[0] + (ii[mask] + 0.5) * voxel_size
            ys = origin[1] + (jj[mask] + 0.5) * voxel_size
            zs = np.full_like(xs, origin[2] + (k + 0.5) * voxel_size)
            intensities = dd[mask].astype(np.float32)

            points = np.column_stack(
                (xs, ys, zs, intensities)
            ).astype(np.float32)

        else:
            esdf_sampled = esdf[::stride, ::stride, ::stride]

            ii, jj, kk = np.meshgrid(
                np.arange(0, nx, stride),
                np.arange(0, ny, stride),
                np.arange(0, nz, stride),
                indexing="ij",
            )

            dd = esdf_sampled
            if self.include_unknown:
                visible = np.ones_like(esdf_sampled, dtype=bool)
            else:
                visible = self.observed[::stride, ::stride, ::stride]
            mask = visible
            if max_distance > 0.0:
                mask &= np.abs(dd) <= max_distance

            xs = origin[0] + (ii[mask] + 0.5) * voxel_size
            ys = origin[1] + (jj[mask] + 0.5) * voxel_size
            zs = origin[2] + (kk[mask] + 0.5) * voxel_size
            intensities = dd[mask].astype(np.float32)

            points = np.column_stack(
                (xs, ys, zs, intensities)
            ).astype(np.float32)

        fields = [
            PointField(
                name="x", offset=0, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="y", offset=4, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="z", offset=8, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name="intensity", offset=12,
                datatype=PointField.FLOAT32, count=1
            ),
        ]

        return point_cloud2.create_cloud(
            self.make_header(),
            fields,
            points.tolist(),
        )

    def make_surface_mesh(self):
        """Build a voxel surface mesh from observed occupied ESDF cells.

        The REMANI NPZ does not contain nvblox's native triangle mesh. This
        marker reconstructs the collision surface without requiring CUDA or
        nvblox_ros on the validation host. Conservative unknown occupancy is
        deliberately excluded; otherwise the complete query AABB becomes a
        solid box and hides the measured scene.
        """
        solid = self.occ & self.observed
        marker = Marker()
        marker.header = self.make_header()
        marker.ns = "saved_esdf_surface"
        marker.id = 0
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 0.70
        marker.color.g = 0.76
        marker.color.b = 0.82
        marker.color.a = 0.72

        # Each template is two consistently wound triangles for one face of a
        # unit voxel. The coordinates are converted to metric positions below.
        faces = (
            (0, -1, ((0, 0, 0), (0, 0, 1), (0, 1, 1),
                     (0, 0, 0), (0, 1, 1), (0, 1, 0))),
            (0, 1, ((1, 0, 0), (1, 1, 0), (1, 1, 1),
                    (1, 0, 0), (1, 1, 1), (1, 0, 1))),
            (1, -1, ((0, 0, 0), (1, 0, 0), (1, 0, 1),
                     (0, 0, 0), (1, 0, 1), (0, 0, 1))),
            (1, 1, ((0, 1, 0), (0, 1, 1), (1, 1, 1),
                    (0, 1, 0), (1, 1, 1), (1, 1, 0))),
            (2, -1, ((0, 0, 0), (0, 1, 0), (1, 1, 0),
                     (0, 0, 0), (1, 1, 0), (1, 0, 0))),
            (2, 1, ((0, 0, 1), (1, 0, 1), (1, 1, 1),
                    (0, 0, 1), (1, 1, 1), (0, 1, 1))),
        )

        face_count = 0
        for axis, direction, corners in faces:
            neighbour = np.zeros_like(solid)
            current_slice = [slice(None)] * 3
            neighbour_slice = [slice(None)] * 3
            if direction < 0:
                current_slice[axis] = slice(1, None)
                neighbour_slice[axis] = slice(None, -1)
            else:
                current_slice[axis] = slice(None, -1)
                neighbour_slice[axis] = slice(1, None)
            neighbour[tuple(current_slice)] = solid[tuple(neighbour_slice)]

            boundary_cells = np.argwhere(solid & ~neighbour)
            face_count += int(boundary_cells.shape[0])
            if boundary_cells.size == 0:
                continue

            metric_cells = (
                self.origin[None, :]
                + boundary_cells.astype(float) * self.voxel_size
            )
            for corner in corners:
                vertices = metric_cells + (
                    np.asarray(corner, dtype=float)[None, :]
                    * self.voxel_size
                )
                marker.points.extend(
                    Point(x=float(vertex[0]),
                          y=float(vertex[1]),
                          z=float(vertex[2]))
                    for vertex in vertices
                )

        triangle_count = len(marker.points) // 3
        self.get_logger().info(
            f"Built ESDF surface mesh: {face_count} voxel faces, "
            f"{triangle_count} triangles"
        )
        return marker

    def make_ply_mesh(self):
        """Create an RViz mesh-resource marker for the saved nvblox PLY."""
        ply_path = Path(self.ply_file).expanduser() if self.ply_file else None
        if ply_path is None:
            esdf_path = Path(self.esdf_file).expanduser()
            ply_path = None
            for suffix in ("_remani_esdf.npz", "_remani.npz"):
                if esdf_path.name.endswith(suffix):
                    ply_name = esdf_path.name[:-len(suffix)] + "_mesh.ply"
                    ply_path = esdf_path.with_name(ply_name)
                    break
            if ply_path is None:
                ply_path = esdf_path.with_suffix(".ply")
        ply_path = ply_path.resolve()

        if not ply_path.is_file():
            self.get_logger().warning(
                f"Saved nvblox PLY does not exist: {ply_path}. "
                "Run d455_bag_esdf.launch.py again or pass ply_file."
            )
            return None

        marker = Marker()
        marker.header = self.make_header()
        marker.ns = "saved_nvblox_ply"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.mesh_resource = ply_path.as_uri()
        marker.mesh_use_embedded_materials = True
        self.get_logger().info(f"Using saved nvblox PLY: {ply_path}")
        return marker

    def make_2d_occupancy_grid(self):
        """
        把 3D occupancy 在 z_min_2d ~ z_max_2d 内投影成 2D OccupancyGrid。
        对移动底盘导航/可视化比较友好。
        """
        occ = self.occ
        origin = self.origin
        voxel_size = self.voxel_size

        nx, ny, nz = occ.shape

        k0 = int(np.floor((self.z_min_2d - origin[2]) / voxel_size))
        k1 = int(np.ceil((self.z_max_2d - origin[2]) / voxel_size))

        k0 = max(k0, 0)
        k1 = min(k1, nz)

        observed = self.observed[:, :, k0:k1]
        observed2d = observed.any(axis=2)
        occ2d = (occ[:, :, k0:k1] & observed).any(axis=2)

        grid = OccupancyGrid()
        grid.header = self.make_header()
        grid.info.map_load_time = self.get_clock().now().to_msg()
        grid.info.resolution = float(voxel_size)
        grid.info.width = int(nx)
        grid.info.height = int(ny)

        grid.info.origin = Pose()
        grid.info.origin.position.x = float(origin[0])
        grid.info.origin.position.y = float(origin[1])
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # OccupancyGrid 是 row-major:
        # index = x + y * width
        # occ2d shape 是 [x, y]，所以转置成 [y, x] 再 flatten。
        data_2d = np.full((ny, nx), -1, dtype=np.int8)
        data_2d[observed2d.T] = 0
        data_2d[occ2d.T] = 100

        grid.data = data_2d.flatten().tolist()
        return grid

    def publish_maps(self):
        cloud_msg = self.make_esdf_cloud()
        grid_msg = self.make_2d_occupancy_grid()

        self.cloud_pub.publish(cloud_msg)
        self.grid_pub.publish(grid_msg)
        if self.surface_mesh is not None and not self.surface_mesh_published:
            self.surface_mesh.header = self.make_header()
            self.mesh_pub.publish(self.surface_mesh)
            # The map is static and the publisher is transient-local, so RViz
            # receives this retained mesh even if it connects later.
            self.surface_mesh_published = True
        if self.ply_mesh is not None and not self.ply_mesh_published:
            self.ply_mesh.header = self.make_header()
            self.ply_mesh_pub.publish(self.ply_mesh)
            self.ply_mesh_published = True


def main(args=None):
    rclpy.init(args=args)

    node = EsdfRvizPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
