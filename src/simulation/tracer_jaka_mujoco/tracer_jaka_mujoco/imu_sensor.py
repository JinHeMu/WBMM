#!/usr/bin/env python3
"""Publish the MuJoCo IMU site as a ROS 2 sensor_msgs/Imu stream."""

import numpy as np
import mujoco

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from tf2_ros import StaticTransformBroadcaster

from . import sensors_common as sc


class ImuSensor:
    def __init__(self, node, cfg):
        self.node = node
        self.log = node.get_logger()
        self.model = node.model
        self.site_name = cfg["site_name"]
        self.frame_id = cfg["frame_id"]
        self.topic = cfg["topic"]
        self.rate = float(cfg["rate"])
        self.orientation_stddev = float(cfg["orientation_stddev"])
        self.angular_velocity_stddev = float(cfg["angular_velocity_stddev"])
        self.linear_acceleration_stddev = float(cfg["linear_acceleration_stddev"])
        self.publish_static_tf = bool(cfg["publish_static_tf"])

        self.site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, self.site_name)
        if self.site_id < 0:
            raise RuntimeError(f"模型中找不到 IMU site: {self.site_name}")

        self.sensor_adr = {}
        for sensor_name in ("imu_accelerometer", "imu_gyro", "imu_orientation"):
            sensor_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
            if sensor_id < 0:
                raise RuntimeError(f"模型中找不到 MuJoCo sensor: {sensor_name}")
            adr = int(self.model.sensor_adr[sensor_id])
            dim = int(self.model.sensor_dim[sensor_id])
            self.sensor_adr[sensor_name] = (adr, dim)

        self.pub = node.create_publisher(Imu, self.topic, qos_profile_sensor_data)
        self.static_tf = StaticTransformBroadcaster(node)
        self._tf_sent = False
        self._last_publish_time = -1.0
        self.log.info(
            f"[imu] enabled  rate={self.rate}Hz  topic={self.topic}  frame={self.frame_id}")

    def _read_sensor(self, name):
        adr, dim = self.sensor_adr[name]
        return np.asarray(self.node.data.sensordata[adr:adr + dim], dtype=float).copy()

    def _publish_static_tf(self, stamp):
        site = self.node.data.site(self.site_name)
        r_world_site = site.xmat.reshape(3, 3).copy()
        t_world_site = site.xpos.copy()
        r_world_base, t_world_base = sc.base_world_pose(
            self.node.base_x, self.node.base_y, self.node.base_yaw)
        r_base_site, t_base_site = sc.world_to_base(
            r_world_base, t_world_base, r_world_site, t_world_site)
        self.static_tf.sendTransform(
            sc.make_tf(
                stamp, self.node.base_frame, self.frame_id,
                r_base_site, t_base_site))
        self._tf_sent = True

    def publish_if_due(self):
        period = 1.0 / self.rate
        if self._last_publish_time >= 0.0:
            if self.node.sim_time - self._last_publish_time + 1e-12 < period:
                return
        self._last_publish_time = self.node.sim_time
        stamp = self.node.now_msg()

        if self.publish_static_tf and not self._tf_sent:
            self._publish_static_tf(stamp)

        # MuJoCo framequat is [w, x, y, z]; ROS uses [x, y, z, w].
        quat = self._read_sensor("imu_orientation")
        gyro = self._read_sensor("imu_gyro")
        accel = self._read_sensor("imu_accelerometer")

        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.orientation.w = float(quat[0])
        msg.orientation.x = float(quat[1])
        msg.orientation.y = float(quat[2])
        msg.orientation.z = float(quat[3])
        msg.angular_velocity.x = float(gyro[0])
        msg.angular_velocity.y = float(gyro[1])
        msg.angular_velocity.z = float(gyro[2])
        msg.linear_acceleration.x = float(accel[0])
        msg.linear_acceleration.y = float(accel[1])
        msg.linear_acceleration.z = float(accel[2])

        o_var = self.orientation_stddev ** 2
        w_var = self.angular_velocity_stddev ** 2
        a_var = self.linear_acceleration_stddev ** 2
        msg.orientation_covariance = [o_var, 0.0, 0.0, 0.0, o_var, 0.0, 0.0, 0.0, o_var]
        msg.angular_velocity_covariance = [
            w_var, 0.0, 0.0, 0.0, w_var, 0.0, 0.0, 0.0, w_var]
        msg.linear_acceleration_covariance = [
            a_var, 0.0, 0.0, 0.0, a_var, 0.0, 0.0, 0.0, a_var]
        self.pub.publish(msg)
