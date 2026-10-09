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

  parameters_.force_timeout = declare_parameter<double>(
      "force_sensor.force_timeout", 0.25);
  if (!std::isfinite(parameters_.force_timeout) ||
      parameters_.force_timeout <= 0.0) {
    throw std::runtime_error(
        "force_sensor.force_timeout must be positive");
  }

  parameters_.max_ee_linear_velocity = declare_parameter<double>(
      "end_effector.max_linear_velocity", 0.05);
  parameters_.max_ee_angular_velocity = declare_parameter<double>(
      "end_effector.max_angular_velocity", 0.20);
  if (!std::isfinite(parameters_.max_ee_linear_velocity) ||
      !std::isfinite(parameters_.max_ee_angular_velocity)) {
    throw std::runtime_error("whole_body parameters must be finite");
  }
  if (parameters_.max_ee_linear_velocity <= 0.0 ||
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
  parameters_.max_tracking_error_m = declare_parameter<double>(
      "safety.max_tracking_error_m", 0.0);
  parameters_.max_tracking_error_rad = declare_parameter<double>(
      "safety.max_tracking_error_rad", 0.0);
  if (!std::isfinite(parameters_.max_tracking_error_m) ||
      !std::isfinite(parameters_.max_tracking_error_rad) ||
      parameters_.max_tracking_error_m < 0.0 ||
      parameters_.max_tracking_error_rad < 0.0) {
    throw std::runtime_error("tracking error limits must be finite and non-negative");
  }
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
  parameters_.input_dimension = declare_parameter<int>(
      "output.input_dimension", 8);
  if (parameters_.input_dimension <= 0) {
    throw std::runtime_error("output.input_dimension must be positive");
  }

  parameters_.correction_topic = declare_parameter<std::string>(
      "topics.correction", "/whole_body_force_control/correction");
  parameters_.state_topic = declare_parameter<std::string>(
      "topics.states", "/whole_body_force_control/states");
  parameters_.wrench_topic = declare_parameter<std::string>(
      "topics.wrench",
      "/whole_body_force_control/processed_wrench");
  parameters_.force_sensor_state_topic = declare_parameter<std::string>(
      "topics.force_sensor_states",
      "/whole_body_force_control/force_sensor_states");
  parameters_.ee_target_topic = declare_parameter<std::string>(
      "topics.ee_target", parameters_.robot_name + "_ee_target");
}

}  // namespace whole_body_force_control
