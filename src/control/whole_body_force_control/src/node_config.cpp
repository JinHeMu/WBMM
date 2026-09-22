#include "node.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace whole_body_force_control
{
namespace
{

constexpr std::array<const char *, 6> kAxisNames{
    "fx", "fy", "fz", "tx", "ty", "tz"};

Vector6d vector6Parameter(
    rclcpp::Node &node, const std::string &name,
    const Vector6d &defaults)
{
  const auto values = node.declare_parameter<std::vector<double>>(
      name, std::vector<double>{});
  if (values.empty()) {
    return defaults;
  }
  if (values.size() != 6) {
    throw std::runtime_error(name + " must contain exactly 6 values");
  }
  Vector6d result;
  for (std::size_t i = 0; i < 6; ++i) {
    result[i] = values[i];
  }
  if (!result.allFinite()) {
    throw std::runtime_error(name + " must contain only finite values");
  }
  return result;
}

Vector6d requireVector6(const std::vector<double> &values,
                        const std::string &name)
{
  if (values.size() != 6) {
    throw std::runtime_error(name + " must contain exactly 6 values");
  }
  Vector6d result;
  for (std::size_t i = 0; i < 6; ++i) {
    result[i] = values[i];
  }
  if (!result.allFinite()) {
    throw std::runtime_error(name + " must contain only finite values");
  }
  return result;
}

Eigen::Vector3d vector3Parameter(
    rclcpp::Node &node, const std::string &name,
    const Eigen::Vector3d &defaults)
{
  const auto values = node.declare_parameter<std::vector<double>>(
      name, std::vector<double>{});
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

AxisMask6d parseSelectedAxes(const std::vector<bool> &values)
{
  if (values.size() != 6) {
    throw std::runtime_error(
        "admittance.selected_axes must contain exactly 6 bool values");
  }
  AxisMask6d mask{};
  for (std::size_t i = 0; i < 6; ++i) {
    mask[i] = values[i];
  }
  return mask;
}

void requireFiniteNonNegative(const Vector6d &values, const std::string &name)
{
  if ((values.array() < 0.0).any()) {
    throw std::runtime_error(name + " values must be non-negative");
  }
}

}  // namespace

std::string WholeBodyForceControlNode::enabledAxes(const AxisMask6d &mask)
{
  std::ostringstream stream;
  bool first = true;
  for (std::size_t i = 0; i < 6; ++i) {
    if (!mask[i]) {
      continue;
    }
    if (!first) {
      stream << ',';
    }
    stream << kAxisNames[i];
    first = false;
  }
  return first ? "none" : stream.str();
}

void WholeBodyForceControlNode::loadParameters()
{
  parameters_.urdf_file = declare_parameter<std::string>("urdf_file", "");
  if (parameters_.urdf_file.empty()) {
    throw std::runtime_error("urdf_file is required");
  }

  parameters_.robot_name = declare_parameter<std::string>(
      "robot_name", "mobile_manipulator");
  parameters_.state_frame = declare_parameter<std::string>(
      "state_frame", "odom");
  if (parameters_.state_frame.empty()) {
    throw std::runtime_error("state_frame must not be empty");
  }

  parameters_.sensor_frame = declare_parameter<std::string>(
      "force_sensor.sensor_frame", "jk_se_vi_200_link");
  if (parameters_.sensor_frame.empty()) {
    throw std::runtime_error("force_sensor.sensor_frame must not be empty");
  }

  parameters_.tcp_frame = declare_parameter<std::string>(
      "force_sensor.tcp_frame", "tool0");
  if (parameters_.tcp_frame.empty()) {
    throw std::runtime_error("force_sensor.tcp_frame must not be empty");
  }

  parameters_.admittance_enabled = declare_parameter<bool>(
      "admittance.enable", false);
  parameters_.admittance_axes = parseSelectedAxes(
      declare_parameter<std::vector<bool>>(
          "admittance.selected_axes", std::vector<bool>(6, false)));
  if (parameters_.admittance_enabled &&
      std::none_of(parameters_.admittance_axes.begin(),
                   parameters_.admittance_axes.end(),
                   [](bool enabled) {return enabled;})) {
    throw std::runtime_error(
        "admittance.enable is true but no admittance.selected_axes are enabled");
  }

  parameters_.mass = vector6Parameter(
      *this, "admittance.mass", Vector6d::Constant(1.0));
  if ((parameters_.mass.array() <= 0.0).any()) {
    throw std::runtime_error("admittance.mass values must be positive");
  }

  parameters_.stiffness = vector6Parameter(
      *this, "admittance.stiffness", Vector6d::Zero());
  requireFiniteNonNegative(parameters_.stiffness, "admittance.stiffness");

  const auto damping_values = declare_parameter<std::vector<double>>(
      "admittance.damping", std::vector<double>{});
  const auto damping_ratio_values = declare_parameter<std::vector<double>>(
      "admittance.damping_ratio", std::vector<double>{});
  const bool has_damping = !damping_values.empty();
  const bool has_damping_ratio = !damping_ratio_values.empty();
  if (has_damping && has_damping_ratio) {
    throw std::runtime_error(
        "set only one of admittance.damping and admittance.damping_ratio");
  }
  if (has_damping) {
    parameters_.damping = requireVector6(
        damping_values, "admittance.damping");
    requireFiniteNonNegative(parameters_.damping, "admittance.damping");
  } else {
    const Vector6d ratios = has_damping_ratio
        ? requireVector6(damping_ratio_values, "admittance.damping_ratio")
        : Vector6d::Ones();
    requireFiniteNonNegative(ratios, "admittance.damping_ratio");
    for (std::size_t i = 0; i < 6; ++i) {
      if (parameters_.admittance_axes[i] &&
          parameters_.stiffness[i] <= 0.0) {
        throw std::runtime_error(
            "admittance.damping is required for selected axes with zero "
            "stiffness; damping_ratio would produce zero damping");
      }
      parameters_.damping[i] =
          2.0 * ratios[i] * std::sqrt(
              parameters_.mass[i] * parameters_.stiffness[i]);
    }
  }

  for (std::size_t i = 0; i < 6; ++i) {
    if (parameters_.admittance_axes[i] && parameters_.damping[i] <= 0.0) {
      throw std::runtime_error(
          "selected admittance axes require positive damping");
    }
  }

  parameters_.max_velocity = vector6Parameter(
      *this, "admittance.max_velocity",
      (Vector6d() << 0.035, 0.035, 0.035, 0.15, 0.15, 0.15).finished());
  if ((parameters_.max_velocity.array() <= 0.0).any()) {
    throw std::runtime_error("admittance.max_velocity values must be positive");
  }

  parameters_.filter_alpha = declare_parameter<double>(
      "force_sensor.filter_alpha", 0.25);
  if (!std::isfinite(parameters_.filter_alpha) ||
      parameters_.filter_alpha < 0.0 || parameters_.filter_alpha > 1.0) {
    throw std::runtime_error("force_sensor.filter_alpha must be in [0, 1]");
  }

  parameters_.tf_lookup_timeout = declare_parameter<double>(
      "force_sensor.tf_lookup_timeout", 0.05);
  parameters_.tf_fallback_to_latest = declare_parameter<bool>(
      "force_sensor.tf_fallback_to_latest", true);
  if (!std::isfinite(parameters_.tf_lookup_timeout) ||
      parameters_.tf_lookup_timeout < 0.0) {
    throw std::runtime_error(
        "force_sensor.tf_lookup_timeout must be non-negative");
  }

  parameters_.wrench_scale = vector6Parameter(
      *this, "force_sensor.wrench_scale", Vector6d::Ones());

  parameters_.hard_wrench_limit = vector6Parameter(
      *this, "force_sensor.hard_wrench_limit",
      (Vector6d() << 20.0, 20.0, 20.0, 5.0, 5.0, 5.0).finished());
  if ((parameters_.hard_wrench_limit.array() <= 0.0).any()) {
    throw std::runtime_error(
        "force_sensor.hard_wrench_limit values must be positive");
  }

  parameters_.hard_force_norm_limit = declare_parameter<double>(
      "force_sensor.hard_force_norm_limit", 20.0);
  if (!std::isfinite(parameters_.hard_force_norm_limit) ||
      parameters_.hard_force_norm_limit < 0.0) {
    throw std::runtime_error(
        "force_sensor.hard_force_norm_limit must be finite and "
        "non-negative (0 disables the norm check)");
  }

  const int tare_samples = declare_parameter<int>(
      "force_sensor.tare_samples", 50);
  if (tare_samples <= 0) {
    throw std::runtime_error("force_sensor.tare_samples must be positive");
  }
  parameters_.tare_samples = static_cast<std::size_t>(tare_samples);

  parameters_.load_compensation_enabled = declare_parameter<bool>(
      "force_sensor.load_compensation.enable", false);
  parameters_.load_gravity_m_s2 = declare_parameter<double>(
      "force_sensor.load_compensation.gravity_m_s2", 9.80665);
  parameters_.load_mass_kg = declare_parameter<double>(
      "force_sensor.load_compensation.mass_kg", 0.0);
  parameters_.load_gravity_direction_base = vector3Parameter(
      *this, "force_sensor.load_compensation.gravity_direction_base",
      (Eigen::Vector3d() << 0.0, 0.0, -1.0).finished());
  parameters_.load_center_of_mass_sensor_m = vector3Parameter(
      *this, "force_sensor.load_compensation.center_of_mass_sensor_m",
      Eigen::Vector3d::Zero());
  const Eigen::Vector3d load_force_bias_sensor = vector3Parameter(
      *this, "force_sensor.load_compensation.force_bias_sensor_n",
      Eigen::Vector3d::Zero());
  const Eigen::Vector3d load_torque_bias_sensor = vector3Parameter(
      *this, "force_sensor.load_compensation.torque_bias_sensor_nm",
      Eigen::Vector3d::Zero());
  parameters_.load_bias_sensor.head<3>() = load_force_bias_sensor;
  parameters_.load_bias_sensor.tail<3>() = load_torque_bias_sensor;

  if (!std::isfinite(parameters_.load_gravity_m_s2) ||
      parameters_.load_gravity_m_s2 <= 0.0) {
    throw std::runtime_error(
        "force_sensor.load_compensation.gravity_m_s2 must be positive");
  }
  if (!std::isfinite(parameters_.load_mass_kg) ||
      parameters_.load_mass_kg < 0.0) {
    throw std::runtime_error(
        "force_sensor.load_compensation.mass_kg must be non-negative");
  }
  const double gravity_direction_norm =
      parameters_.load_gravity_direction_base.norm();
  if (!std::isfinite(gravity_direction_norm) ||
      gravity_direction_norm < 1.0e-9) {
    throw std::runtime_error(
        "force_sensor.load_compensation.gravity_direction_base must be "
        "non-zero");
  }
  parameters_.load_gravity_direction_base.normalize();

  parameters_.force_deadband_n = declare_parameter<double>(
      "force_sensor.force_deadband_n", 1.0);
  parameters_.torque_deadband_nm = declare_parameter<double>(
      "force_sensor.torque_deadband_nm", 0.1);
  if (!std::isfinite(parameters_.force_deadband_n) ||
      parameters_.force_deadband_n < 0.0) {
    throw std::runtime_error(
        "force_sensor.force_deadband_n must be non-negative");
  }
  if (!std::isfinite(parameters_.torque_deadband_nm) ||
      parameters_.torque_deadband_nm < 0.0) {
    throw std::runtime_error(
        "force_sensor.torque_deadband_nm must be non-negative");
  }

  parameters_.force_timeout = declare_parameter<double>(
      "force_sensor.force_timeout", 0.25);
  if (!std::isfinite(parameters_.force_timeout) ||
      parameters_.force_timeout <= 0.0) {
    throw std::runtime_error(
        "force_sensor.force_timeout must be positive");
  }

  parameters_.base_share = declare_parameter<double>(
      "whole_body.base_share", 0.4);
  parameters_.max_base_velocity = declare_parameter<double>(
      "whole_body.max_base_velocity", 0.5);
  parameters_.max_joint_velocity = declare_parameter<double>(
      "whole_body.max_joint_velocity", 1.0);
  parameters_.max_ee_linear_velocity = declare_parameter<double>(
      "whole_body.max_ee_linear_velocity", 0.05);
  parameters_.max_ee_angular_velocity = declare_parameter<double>(
      "whole_body.max_ee_angular_velocity", 0.20);
  if (!std::isfinite(parameters_.base_share) ||
      !std::isfinite(parameters_.max_base_velocity) ||
      !std::isfinite(parameters_.max_joint_velocity) ||
      !std::isfinite(parameters_.max_ee_linear_velocity) ||
      !std::isfinite(parameters_.max_ee_angular_velocity)) {
    throw std::runtime_error("whole_body parameters must be finite");
  }
  if (parameters_.max_base_velocity <= 0.0 ||
      parameters_.max_joint_velocity <= 0.0 ||
      parameters_.max_ee_linear_velocity <= 0.0 ||
      parameters_.max_ee_angular_velocity <= 0.0) {
    throw std::runtime_error(
        "whole_body velocity limits must be positive");
  }

  parameters_.observation_timeout = declare_parameter<double>(
      "safety.observation_timeout", 0.25);
  parameters_.capture_settle_time = declare_parameter<double>(
      "safety.capture_settle_time", 1.0);
  parameters_.enforce_single_target_owner = declare_parameter<bool>(
      "safety.enforce_single_target_owner", true);
  parameters_.loop_rate = declare_parameter<double>(
      "safety.loop_rate", 50.0);
  if (!std::isfinite(parameters_.observation_timeout) ||
      parameters_.observation_timeout <= 0.0) {
    throw std::runtime_error("safety.observation_timeout must be positive");
  }
  if (!std::isfinite(parameters_.capture_settle_time) ||
      parameters_.capture_settle_time < 0.0) {
    throw std::runtime_error("safety.capture_settle_time must be non-negative");
  }
  if (!std::isfinite(parameters_.loop_rate) || parameters_.loop_rate <= 0.0) {
    throw std::runtime_error("safety.loop_rate must be positive");
  }

  parameters_.reference_output_enabled = declare_parameter<bool>(
      "admittance.output", false);
  const auto output_mode = declare_parameter<std::string>(
      "output.mode", "ee_pose");
  if (output_mode == "ee_pose") {
    parameters_.output_mode = ReferenceOutputMode::kEndEffectorPose;
  } else if (output_mode == "whole_body_state") {
    parameters_.output_mode = ReferenceOutputMode::kWholeBodyState;
  } else {
    throw std::runtime_error(
        "output.mode must be 'ee_pose' or 'whole_body_state'");
  }
  parameters_.reference_horizon = declare_parameter<double>(
      "output.reference_horizon", 1.0);
  parameters_.reference_dt = declare_parameter<double>(
      "output.reference_dt", 0.1);
  parameters_.input_dimension = declare_parameter<int>(
      "output.input_dimension", 8);
  if (!std::isfinite(parameters_.reference_horizon) ||
      parameters_.reference_horizon <= 0.0 ||
      !std::isfinite(parameters_.reference_dt) ||
      parameters_.reference_dt <= 0.0 ||
      parameters_.input_dimension <= 0) {
    throw std::runtime_error(
        "output.reference_horizon/reference_dt/input_dimension are invalid");
  }

  parameters_.correction_topic = declare_parameter<std::string>(
      "topics.correction", "/whole_body_force_control/correction");
  parameters_.state_topic = declare_parameter<std::string>(
      "topics.states", "/whole_body_force_control/states");
  parameters_.wrench_topic = declare_parameter<std::string>(
      "topics.wrench", "/whole_body_force_control/wrench");
  parameters_.target_topic = declare_parameter<std::string>(
      "topics.target", parameters_.robot_name + "_whole_body_target");
  parameters_.ee_target_topic = declare_parameter<std::string>(
      "topics.ee_target", parameters_.robot_name + "_ee_target");
}

}  // namespace whole_body_force_control
