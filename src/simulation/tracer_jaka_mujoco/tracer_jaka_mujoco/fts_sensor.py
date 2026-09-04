"""MuJoCo six-axis force/torque sensor with the real broadcaster contract."""

import numpy as np
import mujoco

from geometry_msgs.msg import WrenchStamped


class MujocoFtsBroadcaster:
    """Publish a MuJoCo F/T pair as real-robot-compatible WrenchStamped data."""

    def __init__(self, node, config):
        self.node = node
        self.model = node.model
        self.data = node.data
        self.frame_id = str(config['frame_id'])
        self.topic = str(config['topic'])
        self.raw_topic = str(config['raw_topic'])
        self.rate = max(1.0, float(config['rate']))
        self.period = 1.0 / self.rate
        self.next_publish_time = 0.0
        self.filter_alpha = float(np.clip(config['filter_alpha'], 0.0, 1.0))
        self.force_deadband = max(0.0, float(config['force_deadband']))
        self.torque_deadband = max(0.0, float(config['torque_deadband']))
        self.zero_on_start = bool(config['zero_on_start'])
        self.calibration_delay = max(0.0, float(config['calibration_delay']))
        self.calibration_samples = max(1, int(config['calibration_samples']))

        force_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR,
            str(config['force_sensor_name']))
        torque_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SENSOR,
            str(config['torque_sensor_name']))
        if force_id < 0 or torque_id < 0:
            raise RuntimeError(
                'MuJoCo F/T sensors not found: '
                f"{config['force_sensor_name']}, {config['torque_sensor_name']}")

        self.force_adr = int(self.model.sensor_adr[force_id])
        self.torque_adr = int(self.model.sensor_adr[torque_id])
        if int(self.model.sensor_dim[force_id]) != 3:
            raise RuntimeError('MuJoCo force sensor must have dimension 3')
        if int(self.model.sensor_dim[torque_id]) != 3:
            raise RuntimeError('MuJoCo torque sensor must have dimension 3')

        self.publisher = node.create_publisher(WrenchStamped, self.topic, 10)
        self.raw_publisher = node.create_publisher(
            WrenchStamped, self.raw_topic, 10)

        self.bias = np.zeros(6, dtype=float)
        self.filtered = np.zeros(6, dtype=float)
        self.calibration_sum = np.zeros(6, dtype=float)
        self.calibration_count = 0
        self.calibrated = not self.zero_on_start
        self.filter_initialized = False

        node.get_logger().info(
            '[fts] enabled  sensor=tcp_fts_sensor  '
            f'rate={self.rate:.1f}Hz  frame={self.frame_id}  '
            f'topic={self.topic}')

    def _read_raw(self):
        """Return [Fx, Fy, Fz, Tx, Ty, Tz] in tcp_fts_site coordinates."""
        force = self.data.sensordata[self.force_adr:self.force_adr + 3]
        torque = self.data.sensordata[self.torque_adr:self.torque_adr + 3]
        return np.concatenate((force, torque)).astype(float, copy=True)

    def _make_message(self, values):
        message = WrenchStamped()
        message.header.stamp = self.node.now_msg()
        message.header.frame_id = self.frame_id
        message.wrench.force.x = float(values[0])
        message.wrench.force.y = float(values[1])
        message.wrench.force.z = float(values[2])
        message.wrench.torque.x = float(values[3])
        message.wrench.torque.y = float(values[4])
        message.wrench.torque.z = float(values[5])
        return message

    def publish_if_due(self):
        """Sample, calibrate, filter, and publish at the configured rate."""
        sim_time = float(self.node.sim_time)
        if sim_time + 1e-12 < self.next_publish_time:
            return
        self.next_publish_time = sim_time + self.period

        raw = self._read_raw()
        self.raw_publisher.publish(self._make_message(raw))

        if not self.calibrated:
            if sim_time < self.calibration_delay:
                return
            self.calibration_sum += raw
            self.calibration_count += 1
            if self.calibration_count < self.calibration_samples:
                return
            self.bias = self.calibration_sum / float(self.calibration_count)
            self.calibrated = True
            self.filtered.fill(0.0)
            self.filter_initialized = True
            self.node.get_logger().info(
                '[fts] zero calibration complete: '
                + np.array2string(self.bias, precision=3))

        compensated = raw - self.bias
        if not self.filter_initialized:
            self.filtered = compensated
            self.filter_initialized = True
        else:
            alpha = self.filter_alpha
            self.filtered = alpha * compensated + (1.0 - alpha) * self.filtered

        output = self.filtered.copy()
        output[:3][np.abs(output[:3]) < self.force_deadband] = 0.0
        output[3:][np.abs(output[3:]) < self.torque_deadband] = 0.0
        self.publisher.publish(self._make_message(output))
