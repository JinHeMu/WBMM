#include "force_sensor_processor_node.hpp"

#include "wbmm_ros_interfaces/wbmm_ros_conversions.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

namespace whole_body_force_control {
namespace {

Vector6d vector6Parameter(rclcpp::Node &node, const std::string &name,
                          const Vector6d &defaults) {
  const auto values =
      node.declare_parameter<std::vector<double>>(name, std::vector<double>{});
  if (values.empty()) {
    return defaults;
  }
  if (values.size() != 6) {
    throw std::runtime_error(name + " must contain exactly 6 values");
  }
  Vector6d result;
  for (std::size_t i = 0; i < 6; ++i) {
    result[static_cast<Eigen::Index>(i)] = values[i];
  }
  if (!result.allFinite()) {
    throw std::runtime_error(name + " must contain only finite values");
  }
  return result;
}

Eigen::Vector3d vector3Parameter(rclcpp::Node &node, const std::string &name,
                                 const Eigen::Vector3d &defaults) {
  const auto values =
      node.declare_parameter<std::vector<double>>(name, std::vector<double>{});
  if (values.empty()) {
    return defaults;
  }
  if (values.size() != 3) {
    throw std::runtime_error(name + " must contain exactly 3 values");
  }
  Eigen::Vector3d result;
  for (std::size_t i = 0; i < 3; ++i) {
    result[static_cast<Eigen::Index>(i)] = values[i];
  }
  if (!result.allFinite()) {
    throw std::runtime_error(name + " must contain only finite values");
  }
  return result;
}

Eigen::Matrix3d
rotationFromTransform(const geometry_msgs::msg::TransformStamped &transform) {
  const auto &q = transform.transform.rotation;
  const Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-12) {
    throw std::invalid_argument("TF rotation is not a valid quaternion");
  }
  return quaternion.normalized().toRotationMatrix();
}

Eigen::Vector3d translationFromTransform(
    const geometry_msgs::msg::TransformStamped &transform) {
  const auto &t = transform.transform.translation;
  const Eigen::Vector3d translation(t.x, t.y, t.z);
  if (!translation.allFinite()) {
    throw std::invalid_argument("TF translation is non-finite");
  }
  return translation;
}

} // namespace

ForceSensorProcessorNode::ForceSensorProcessorNode()
    : Node("force_sensor_processor") {
  loadParameters();
  configureProcessor();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  processed_wrench_publisher_ =
      create_publisher<geometry_msgs::msg::WrenchStamped>(
          parameters_.processed_wrench_topic, rclcpp::SensorDataQoS());
  state_publisher_ = create_publisher<std_msgs::msg::String>(
      parameters_.state_topic, rclcpp::QoS(1).reliable().transient_local());
  raw_wrench_subscription_ =
      create_subscription<geometry_msgs::msg::WrenchStamped>(
          parameters_.raw_wrench_topic, rclcpp::SensorDataQoS(),
          std::bind(&ForceSensorProcessorNode::rawWrenchCallback, this,
                    std::placeholders::_1));
  reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/whole_body_force_control/force_sensor/reset",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        resetProcessor();
        response->success = true;
        response->message =
            "Force sensor processor reset; waiting for fresh raw data.";
      });

  const auto period =
      std::chrono::duration<double>(1.0 / parameters_.monitor_rate);
  monitor_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&ForceSensorProcessorNode::monitorRawInput, this));

  resetProcessor();
  RCLCPP_INFO(get_logger(), "Ready: raw=%s processed=%s frame=%s->%s",
              parameters_.raw_wrench_topic.c_str(),
              parameters_.processed_wrench_topic.c_str(),
              parameters_.sensor_frame.c_str(), parameters_.tcp_frame.c_str());
}

void ForceSensorProcessorNode::loadParameters() {
  parameters_.raw_wrench_topic = declare_parameter<std::string>(
      "topics.raw_wrench", "/fts_broadcaster/wrench");
  parameters_.processed_wrench_topic = declare_parameter<std::string>(
      "topics.processed_wrench", "/whole_body_force_control/processed_wrench");
  parameters_.state_topic = declare_parameter<std::string>(
      "topics.states", "/whole_body_force_control/force_sensor_states");
  parameters_.sensor_frame = declare_parameter<std::string>(
      "force_sensor.sensor_frame", "jk_se_vi_200_link");
  parameters_.tcp_frame =
      declare_parameter<std::string>("force_sensor.tcp_frame", "tool0");
  parameters_.load_base_frame = declare_parameter<std::string>(
      "force_sensor.load_compensation.base_frame", "base_footprint");
  if (parameters_.raw_wrench_topic.empty() ||
      parameters_.processed_wrench_topic.empty() ||
      parameters_.state_topic.empty() || parameters_.sensor_frame.empty() ||
      parameters_.tcp_frame.empty() || parameters_.load_base_frame.empty()) {
    throw std::runtime_error(
        "force sensor topics and frames must not be empty");
  }

  parameters_.tf_lookup_timeout =
      declare_parameter<double>("force_sensor.tf_lookup_timeout", 0.05);
  parameters_.tf_fallback_to_latest =
      declare_parameter<bool>("force_sensor.tf_fallback_to_latest", false);
  parameters_.raw_timeout =
      declare_parameter<double>("force_sensor.raw_timeout", 0.25);
  parameters_.monitor_rate =
      declare_parameter<double>("force_sensor.monitor_rate", 50.0);
  if (!std::isfinite(parameters_.tf_lookup_timeout) ||
      parameters_.tf_lookup_timeout < 0.0 ||
      !std::isfinite(parameters_.raw_timeout) ||
      parameters_.raw_timeout <= 0.0 ||
      !std::isfinite(parameters_.monitor_rate) ||
      parameters_.monitor_rate <= 0.0) {
    throw std::runtime_error(
        "force sensor TF timeout, raw timeout and monitor rate are invalid");
  }

  const int tare_samples =
      declare_parameter<int>("force_sensor.tare_samples", 50);
  if (tare_samples <= 0) {
    throw std::runtime_error("force_sensor.tare_samples must be positive");
  }
  parameters_.tare_samples = static_cast<std::size_t>(tare_samples);
  const double filter_alpha =
      declare_parameter<double>("force_sensor.filter_alpha", 0.25);
  if (!std::isfinite(filter_alpha) || filter_alpha < 0.0 ||
      filter_alpha > 1.0) {
    throw std::runtime_error("force_sensor.filter_alpha must be in [0, 1]");
  }
  parameters_.filter_alpha = Vector6d::Constant(filter_alpha);
  parameters_.wrench_scale =
      vector6Parameter(*this, "force_sensor.wrench_scale", Vector6d::Ones());
  parameters_.hard_wrench_limit = vector6Parameter(
      *this, "force_sensor.hard_wrench_limit",
      (Vector6d() << 20.0, 20.0, 20.0, 5.0, 5.0, 5.0).finished());
  if ((parameters_.hard_wrench_limit.array() <= 0.0).any()) {
    throw std::runtime_error(
        "force_sensor.hard_wrench_limit values must be positive");
  }
  parameters_.hard_force_norm_limit =
      declare_parameter<double>("force_sensor.hard_force_norm_limit", 20.0);
  parameters_.force_deadband_n =
      declare_parameter<double>("force_sensor.force_deadband_n", 1.0);
  parameters_.torque_deadband_nm =
      declare_parameter<double>("force_sensor.torque_deadband_nm", 0.1);
  if (!std::isfinite(parameters_.hard_force_norm_limit) ||
      parameters_.hard_force_norm_limit < 0.0 ||
      !std::isfinite(parameters_.force_deadband_n) ||
      parameters_.force_deadband_n < 0.0 ||
      !std::isfinite(parameters_.torque_deadband_nm) ||
      parameters_.torque_deadband_nm < 0.0) {
    throw std::runtime_error("force sensor limits/deadbands are invalid");
  }

  auto &load = parameters_.load_compensation;
  load.enable =
      declare_parameter<bool>("force_sensor.load_compensation.enable", false);
  load.gravity_m_s2 = declare_parameter<double>(
      "force_sensor.load_compensation.gravity_m_s2", 9.80665);
  load.mass_kg =
      declare_parameter<double>("force_sensor.load_compensation.mass_kg", 0.0);
  load.gravity_direction_base = vector3Parameter(
      *this, "force_sensor.load_compensation.gravity_direction_base",
      (Eigen::Vector3d() << 0.0, 0.0, -1.0).finished());
  load.center_of_mass_sensor_m = vector3Parameter(
      *this, "force_sensor.load_compensation.center_of_mass_sensor_m",
      Eigen::Vector3d::Zero());
  load.bias_sensor.head<3>() = vector3Parameter(
      *this, "force_sensor.load_compensation.force_bias_sensor_n",
      Eigen::Vector3d::Zero());
  load.bias_sensor.tail<3>() = vector3Parameter(
      *this, "force_sensor.load_compensation.torque_bias_sensor_nm",
      Eigen::Vector3d::Zero());
}

void ForceSensorProcessorNode::configureProcessor() {
  ForceProcessorConfig config;
  config.tare_samples = parameters_.tare_samples;
  config.filter_alpha = parameters_.filter_alpha;
  config.scale = parameters_.wrench_scale;
  config.hard_limit_enabled = true;
  config.hard_wrench_limit = parameters_.hard_wrench_limit;
  config.hard_force_norm_limit = parameters_.hard_force_norm_limit;
  config.force_deadband_n = parameters_.force_deadband_n;
  config.torque_deadband_nm = parameters_.torque_deadband_nm;
  config.load_compensation = parameters_.load_compensation;
  processor_.setConfig(config);
}

geometry_msgs::msg::TransformStamped
ForceSensorProcessorNode::lookupTransformWithFallback(
    const std::string &target_frame, const std::string &source_frame,
    const builtin_interfaces::msg::Time &stamp, const std::string &context) {
  const auto timeout =
      rclcpp::Duration::from_seconds(parameters_.tf_lookup_timeout);
  try {
    return tf_buffer_->lookupTransform(target_frame, source_frame,
                                       rclcpp::Time(stamp), timeout);
  } catch (const tf2::TransformException &exception) {
    if (!parameters_.tf_fallback_to_latest) {
      throw;
    }
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s: TF %s <- %s at stamp failed (%s); using latest TF",
        context.c_str(), target_frame.c_str(), source_frame.c_str(),
        exception.what());
    return tf_buffer_->lookupTransform(target_frame, source_frame,
                                       tf2::TimePointZero,
                                       tf2_ros::fromRclcpp(timeout));
  }
}

void ForceSensorProcessorNode::rawWrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr message) {
  if (fault_latched_) {
    return;
  }

  raw_received_ = true;
  last_raw_ = std::chrono::steady_clock::now();

  if (message->header.frame_id != parameters_.sensor_frame) {
    latchFault("WRENCH_FRAME");
    return;
  }
  auto raw =
      wbmm::ros_interfaces::wrenchFromRos(*message, parameters_.sensor_frame);
  if (!raw.has_value() || !wbmm::core::validate(*raw).ok) {
    latchFault("WRENCH_INVALID");
    return;
  }

  try {
    const auto tcp_sensor = lookupTransformWithFallback(
        parameters_.tcp_frame, parameters_.sensor_frame, message->header.stamp,
        "sensor-to-tcp");
    const Eigen::Matrix3d tcp_rotation_sensor =
        rotationFromTransform(tcp_sensor);
    const Eigen::Vector3d sensor_origin_in_tcp =
        translationFromTransform(tcp_sensor);

    Eigen::Matrix3d sensor_rotation_base = Eigen::Matrix3d::Identity();
    if (parameters_.load_compensation.enable) {
      const auto sensor_base = lookupTransformWithFallback(
          parameters_.sensor_frame, parameters_.load_base_frame,
          message->header.stamp, "base-to-sensor");
      sensor_rotation_base = rotationFromTransform(sensor_base);
    }

    const auto result = processor_.process(
        *raw, tcp_rotation_sensor, sensor_origin_in_tcp, sensor_rotation_base);
    if (result.taring) {
      publishState("TARING");
      return;
    }
    if (!result.ok) {
      latchFault(result.hard_limit_exceeded ? "WRENCH_LIMIT"
                                            : "WRENCH_INVALID");
      return;
    }

    geometry_msgs::msg::WrenchStamped output;
    output.header = message->header;
    output.header.frame_id = parameters_.tcp_frame;
    output.wrench.force.x = result.wrench.force.x;
    output.wrench.force.y = result.wrench.force.y;
    output.wrench.force.z = result.wrench.force.z;
    output.wrench.torque.x = result.wrench.torque.x;
    output.wrench.torque.y = result.wrench.torque.y;
    output.wrench.torque.z = result.wrench.torque.z;
    processed_wrench_publisher_->publish(output);
    publishState("ACTIVE");
  } catch (const tf2::TransformException &exception) {
    RCLCPP_ERROR(get_logger(), "Wrench TF lookup failed: %s", exception.what());
    latchFault("WRENCH_TF");
  } catch (const std::exception &exception) {
    RCLCPP_ERROR(get_logger(), "Wrench processing failed: %s",
                 exception.what());
    latchFault("WRENCH_INVALID");
  }
}

void ForceSensorProcessorNode::monitorRawInput() {
  if (fault_latched_ || !raw_received_) {
    return;
  }
  const double age = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - last_raw_)
                         .count();
  if (age > parameters_.raw_timeout) {
    latchFault("WRENCH_TIMEOUT");
  }
}

void ForceSensorProcessorNode::resetProcessor() {
  fault_latched_ = false;
  raw_received_ = false;
  processor_.reset();
  if (!parameters_.load_compensation.enable) {
    processor_.startTare();
    publishState("TARING");
  } else {
    publishState("WAITING_FOR_WRENCH");
  }
}

void ForceSensorProcessorNode::latchFault(const std::string &reason) {
  if (fault_latched_) {
    return;
  }
  fault_latched_ = true;
  RCLCPP_ERROR(get_logger(), "Force sensor fault latched: %s", reason.c_str());
  publishState("FAULT_" + reason);
}

void ForceSensorProcessorNode::publishState(const std::string &state) {
  if (state == last_state_) {
    return;
  }
  last_state_ = state;
  std_msgs::msg::String message;
  message.data = state;
  state_publisher_->publish(message);
}

} // namespace whole_body_force_control

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
        std::make_shared<whole_body_force_control::ForceSensorProcessorNode>());
  } catch (const std::exception &exception) {
    RCLCPP_FATAL(rclcpp::get_logger("force_sensor_processor"), "%s",
                 exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
