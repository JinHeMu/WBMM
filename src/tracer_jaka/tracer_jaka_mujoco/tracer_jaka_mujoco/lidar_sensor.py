#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
lidar_sensor.py —— MuJoCo-LiDAR 封装为 ROS2 传感器

要点
----
* 在 **独立线程** 中按 lidar.rate 跑光线追踪，不阻塞 500Hz 物理步进。
* 每次扫描前在锁内把 MjData 快照到私有副本，再在锁外做耗时的 trace_rays，
  既保证数据一致，又不长时间占用物理锁。
* 默认生成单层 360° 扫描并直接发布 sensor_msgs/LaserScan，供 slam_toolbox 使用。
* 可选同时发布 PointCloud2，方便在 RViz 中调试。
* 通过 MuJoCo geom group 掩码只扫描环境(group 0)，从源头避免机械臂和底盘自遮挡。
"""

import math
import time
import threading

import numpy as np
import mujoco
import rclpy

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from tf2_ros import StaticTransformBroadcaster

from . import sensors_common as sc


class LidarSensor:
    def __init__(self, node, cfg):
        self.node = node
        self.log = node.get_logger()
        self.model = node.model

        self.site_name = cfg["site_name"]
        self.frame_id = cfg["frame_id"]
        self.topic = cfg["topic"]
        self.points_topic = cfg["points_topic"]
        self.publish_points = bool(cfg["publish_points"])
        self.rate = float(cfg["rate"])
        self.backend = cfg["backend"]
        self.range_min = float(cfg["range_min"])
        self.cutoff = float(cfg["cutoff_dist"])
        self.pattern = cfg["pattern"]
        if self.pattern != "2d":
            self.log.warn(
                f"[lidar] LaserScan 仅支持单层 2D 模式，忽略 pattern='{self.pattern}'")
            self.pattern = "2d"
        self.cols = int(cfg["num_ray_cols"])
        self.rows = int(cfg["num_ray_rows"])
        self.angle_min = float(cfg["angle_min"])
        self.angle_max = float(cfg["angle_max"])
        self.geom_group = np.asarray(cfg["geom_group"], dtype=np.uint8)
        self.publish_static_tf = bool(cfg["publish_static_tf"])
        self.device_mem_gb = float(cfg["taichi_device_memory_gb"])

        self._ok = False
        self._thread = None
        self._running = False

        # ---- 校验 site ----
        sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, self.site_name)
        if sid < 0:
            self.log.error(f"[lidar] 模型中找不到 site: {self.site_name}，LiDAR 已禁用")
            return

        # ---- 导入并创建 MuJoCo-LiDAR ----
        try:
            from mujoco_lidar import MjLidarWrapper, scan_gen
        except Exception as e:               # noqa
            self.log.error(f"[lidar] 无法导入 mujoco_lidar（pip install mujoco_lidar）: {e}")
            return

        self._scan_gen = scan_gen
        try:
            self.theta, self.phi = self._build_pattern(scan_gen)
        except Exception as e:               # noqa
            self.log.error(f"[lidar] 生成扫描模式失败: {e}")
            return

        args = {"geomgroup": self.geom_group}
        if self.backend == "taichi":
            args["ti_init_args"] = {"device_memory_GB": self.device_mem_gb}
        try:
            self.lidar = MjLidarWrapper(
                self.model,
                site_name=self.site_name,
                backend=self.backend,
                cutoff_dist=self.cutoff,
                args=args,
            )
        except Exception as e:               # noqa
            self.log.error(f"[lidar] 创建 MjLidarWrapper 失败: {e}")
            return

        # ---- 私有数据快照 ----
        self._snap = mujoco.MjData(self.model)

        # ---- ROS 接口 ----
        self.pub = node.create_publisher(LaserScan, self.topic, qos_profile_sensor_data)
        self.points_pub = (
            node.create_publisher(PointCloud2, self.points_topic, qos_profile_sensor_data)
            if self.publish_points else None
        )
        self.static_tf = StaticTransformBroadcaster(node)
        self._tf_sent = False

        self._ok = True
        self.log.info(
            f"[lidar] enabled  rays={len(self.theta)}  rate={self.rate}Hz  "
            f"backend={self.backend}  topic={self.topic}  geom_group={self.geom_group.tolist()}")

    # ---------------------------------------------------------------- #
    def _build_pattern(self, scan_gen):
        """根据配置返回 (theta, phi)。2d 模式只有一条水平扫描线。"""
        if self.pattern == "2d":
            theta = np.linspace(
                self.angle_min, self.angle_max, self.cols, endpoint=False, dtype=np.float64)
            return theta, np.zeros_like(theta)
        if self.pattern == "grid":
            return scan_gen.generate_grid_scan_pattern(
                num_ray_cols=self.cols, num_ray_rows=self.rows)
        fn = getattr(scan_gen, f"generate_{self.pattern}", None)
        if fn is None:
            self.log.warn(
                f"[lidar] 未知扫描模式 '{self.pattern}'，回退到 grid {self.cols}x{self.rows}")
            return scan_gen.generate_grid_scan_pattern(
                num_ray_cols=self.cols, num_ray_rows=self.rows)
        return fn()

    # ---------------------------------------------------------------- #
    def start(self):
        if not self._ok:
            return
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=max(0.2, 2.0 / self.rate))

    # ---------------------------------------------------------------- #
    def _snapshot(self):
        """在物理锁内把 data 拷到私有副本（快），锁外再做光追。"""
        with self.node.physics_lock:
            try:
                mujoco.mj_copyData(self._snap, self.model, self.node.data)
            except Exception:                # noqa  老版本兜底
                self._snap.qpos[:] = self.node.data.qpos
                self._snap.qvel[:] = self.node.data.qvel
                mujoco.mj_forward(self.model, self._snap)

    def _publish_static_tf(self, stamp):
        """雷达固连底盘 -> 静态 TF，仅发一次。"""
        site = self._snap.site(self.site_name)
        R_w_s = site.xmat.reshape(3, 3).copy()
        t_w_s = site.xpos.copy()
        x = float(self.node.base_x)
        y = float(self.node.base_y)
        yaw = float(self.node.base_yaw)
        R_w_b, t_w_b = sc.base_world_pose(x, y, yaw)
        R_b_s, t_b_s = sc.world_to_base(R_w_b, t_w_b, R_w_s, t_w_s)
        tf = sc.make_tf(stamp, self.node.base_frame, self.frame_id, R_b_s, t_b_s)
        self.static_tf.sendTransform(tf)
        self._tf_sent = True

    def _worker(self):
        period = 1.0 / self.rate
        next_t = time.perf_counter()
        while self._running:
            self._snapshot()
            stamp = self.node.now_msg()
            try:
                ranges = self.lidar.trace_rays(self._snap, self.theta, self.phi)
                pts = np.asarray(self.lidar.get_hit_points(), dtype=np.float32).reshape(-1, 3)
            except Exception as e:           # noqa
                self.log.warn(f"[lidar] trace_rays 失败: {e}", throttle_duration_sec=5.0)
                pts = np.empty((0, 3), dtype=np.float32)
                ranges = None

            if self.publish_static_tf and not self._tf_sent:
                self._publish_static_tf(stamp)

            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = self.frame_id
            scan.angle_min = self.angle_min
            scan.angle_increment = (self.angle_max - self.angle_min) / self.cols
            scan.angle_max = self.angle_min + (self.cols - 1) * scan.angle_increment
            scan.scan_time = period
            scan.time_increment = period / self.cols
            scan.range_min = self.range_min
            scan.range_max = self.cutoff

            if ranges is None:
                values = np.full(self.cols, np.inf, dtype=np.float32)
                pts = np.zeros((self.cols, 3), dtype=np.float32)
            else:
                values = np.asarray(ranges, dtype=np.float32)
            invalid = (~np.isfinite(values)) | (values < self.range_min)
            values[invalid] = np.inf
            scan.ranges = values.tolist()
            try:
                self.pub.publish(scan)
                if self.points_pub is not None:
                    valid = np.isfinite(values)
                    valid_pts = pts[valid]
                    self.points_pub.publish(
                        sc.make_pointcloud2(stamp, self.frame_id, valid_pts))
            except Exception as exc:  # rclpy context may close during SIGINT
                if not rclpy.ok() or not self._running:
                    break
                self.log.warn(
                    f"[lidar] 发布扫描失败: {exc}", throttle_duration_sec=5.0)

            next_t += period
            sleep = next_t - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.perf_counter()
