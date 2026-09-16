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

std::size_t axisIndex(const std::string &axis)
{
  if (axis == "fx" || axis == "x")
  {
    return 0;
  }
  if (axis == "fy" || axis == "y")
  {
    return 1;
  }
  if (axis == "fz" || axis == "z")
  {
    return 2;
  }
  if (axis == "tx" || axis == "rx" || axis == "mx")
  {
    return 3;
  }
  if (axis == "ty" || axis == "ry" || axis == "my")
  {
    return 4;
  }
  if (axis == "tz" || axis == "rz" || axis == "mz")
  {
    return 5;
  }
  throw std::runtime_error(
      "Unknown compliance axis '" + axis +
      "'; use fx, fy, fz, tx, ty, or tz");
}

AxisMask6d parseAxisMask(
    const std::vector<std::string> &names, const std::string &legacy_axis)
{
  AxisMask6d mask{};
  if (names.empty() || (names.size() == 1 && names.front() == "none"))
  {
    return mask;
  }
  if (names.size() == 1 && names.front() == "legacy")
  {
    mask[axisIndex(legacy_axis)] = true;
    return mask;
  }
  for (const auto &name : names)
  {
    if (name == "legacy" || name == "none")
    {
      throw std::runtime_error("legacy/none cannot be combined with explicit axes");
    }
    mask[axisIndex(name)] = true;
  }
  return mask;
}

Vector6d vector6Parameter(
    rclcpp::Node &node, const std::string &name,
    const Vector6d &defaults)
{
  const auto values = node.declare_parameter<std::vector<double>>(
      name, std::vector<double>{});
  if (values.empty())
  {
    return defaults;
  }
  if (values.size() != 6)
  {
    throw std::runtime_error(name + " must contain exactly 6 values");
  }
  Vector6d result;
  for (std::size_t i = 0; i < 6; ++i)
  {
    result[i] = values[i];
  }
  if (!result.allFinite())
  {
    throw std::runtime_error(name + " must contain only finite values");
  }
  return result;
}

}  // namespace

std::string WholeBodyForceControlNode::enabledAxes(const AxisMask6d &mask)
{
  std::ostringstream stream;
  bool first = true;
  for (std::size_t i = 0; i < 6; ++i)
  {
    if (!mask[i])
    {
      continue;
    }
    if (!first)
    {
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
  parameters_.ee_frame = declare_parameter<std::string>("ee_frame", "tool0");
  parameters_.state_frame = declare_parameter<std::string>("state_frame", "odom");
  if (parameters_.urdf_file.empty())
  {
    throw std::runtime_error("urdf_file is required");
  }
  if (parameters_.state_frame.empty())
  {
    throw std::runtime_error("state_frame must not be empty");
  }

  parameters_.robot_name = declare_parameter<std::string>(
      "robot_name", "mobile_manipulator");
  parameters_.control_mode = declare_parameter<std::string>(
      "control_mode", "force_follow");
  if (parameters_.control_mode != "admittance" &&
      parameters_.control_mode != "constant_force" &&
      parameters_.control_mode != "force_follow")
  {
    throw std::runtime_error(
        "control_mode must be admittance, constant_force, or force_follow");
  }

  // Deprecated scalar compatibility parameters.  They remain the default
  // until admittance_axes is explicitly configured.
  parameters_.force_axis = declare_parameter<std::string>("force_axis", "x");
  if (parameters_.force_axis != "x" && parameters_.force_axis != "y" &&
      parameters_.force_axis != "z")
  {
    throw std::runtime_error("force_axis must be x, y, or z");
  }
  parameters_.force_axis_index = axisIndex(parameters_.force_axis);

  parameters_.absolute_force = declare_parameter<bool>("absolute_force", false);
  parameters_.configured_desired_force =
      declare_parameter<double>("desired_force", 12.0);
  parameters_.mass = declare_parameter<double>("mass", 3.0);
  parameters_.damping = declare_parameter<double>("damping", 45.0);
  parameters_.stiffness = declare_parameter<double>("stiffness", 150.0);
  parameters_.max_offset = declare_parameter<double>("max_offset", 0.08);
  parameters_.max_velocity = declare_parameter<double>("max_velocity", 0.035);
  parameters_.filter_alpha = declare_parameter<double>("filter_alpha", 0.25);
  parameters_.force_velocity_mode = declare_parameter<bool>(
      "force_velocity_mode", false);
  parameters_.force_deadband = declare_parameter<double>("force_deadband", 0.0);

  parameters_.loop_rate = declare_parameter<double>("loop_rate", 50.0);
  parameters_.reference_horizon = declare_parameter<double>(
      "reference_horizon", 1.0);
  parameters_.reference_dt = declare_parameter<double>("reference_dt", 0.1);
  parameters_.input_dimension = declare_parameter<int>("input_dimension", 8);
  parameters_.base_share = declare_parameter<double>("base_share", 0.4);
  parameters_.max_base_delta = declare_parameter<double>("max_base_delta", 0.04);
  parameters_.max_joint_delta = declare_parameter<double>("max_joint_delta", 0.25);
  parameters_.force_timeout = declare_parameter<double>("force_timeout", 0.25);
  parameters_.observation_timeout = declare_parameter<double>(
      "observation_timeout", 0.25);
  parameters_.capture_settle_time = declare_parameter<double>(
      "capture_settle_time", 1.0);
  const int tare_samples = declare_parameter<int>("tare_samples", 50);
  if (tare_samples <= 0)
  {
    throw std::runtime_error("tare_samples must be positive");
  }
  parameters_.tare_samples = static_cast<std::size_t>(tare_samples);
  parameters_.armed = declare_parameter<bool>("armed", false);
  parameters_.reference_output_enabled = declare_parameter<bool>(
      "reference_output_enabled", false);
  parameters_.enforce_single_target_owner = declare_parameter<bool>(
      "enforce_single_target_owner", true);
  parameters_.force_scale = declare_parameter<double>("force_scale", 1.0);
  if (!std::isfinite(parameters_.force_scale) ||
      std::abs(parameters_.force_scale) < 1.0e-9)
  {
    throw std::runtime_error("force_scale must be finite and non-zero");
  }

  const Eigen::Vector3d response_body(
      declare_parameter<double>("response_body_x", 1.0),
      declare_parameter<double>("response_body_y", 0.0),
      declare_parameter<double>("response_body_z", 0.0));
  if (!response_body.allFinite() || response_body.norm() < 1.0e-9)
  {
    throw std::runtime_error("response_body direction must be finite and non-zero");
  }
  parameters_.response_body = response_body.normalized();

  const auto admittance_axis_names =
      declare_parameter<std::vector<std::string>>("admittance_axes", {"legacy"});
  const auto constant_force_axis_names =
      declare_parameter<std::vector<std::string>>("constant_force_axes", {"legacy"});
  const auto absolute_wrench_axis_names =
      declare_parameter<std::vector<std::string>>("absolute_wrench_axes", {"legacy"});
  parameters_.cartesian_mode = !(
      admittance_axis_names.size() == 1 &&
      admittance_axis_names.front() == "legacy");
  parameters_.require_wrench_frame = declare_parameter<bool>(
      "require_wrench_frame", parameters_.cartesian_mode);
  parameters_.admittance_axes = parseAxisMask(
      admittance_axis_names, parameters_.force_axis);
  parameters_.constant_force_axes = parseAxisMask(
      constant_force_axis_names, parameters_.force_axis);
  parameters_.absolute_wrench_axes = parseAxisMask(
      absolute_wrench_axis_names, parameters_.force_axis);
  if (parameters_.control_mode != "constant_force")
  {
    parameters_.constant_force_axes.fill(false);
  }
  for (std::size_t i = 0; i < 6; ++i)
  {
    if (parameters_.constant_force_axes[i] &&
        !parameters_.admittance_axes[i])
    {
      throw std::runtime_error(
          "constant_force_axes must be a subset of admittance_axes");
    }
  }

  Vector6d desired_defaults = Vector6d::Zero();
  for (std::size_t i = 0; i < 3; ++i)
  {
    if (parameters_.constant_force_axes[i])
    {
      desired_defaults[i] = parameters_.configured_desired_force;
    }
  }
  // Torque targets deliberately default to zero.  Enabling tx/ty/tz
  // compliance therefore does not silently enable torque regulation.
  parameters_.desired_wrench = vector6Parameter(
      *this, "desired_wrench", desired_defaults);

  Vector6d mass_defaults;
  mass_defaults << parameters_.mass, parameters_.mass, parameters_.mass,
      0.30, 0.30, 0.30;
  Vector6d damping_defaults;
  damping_defaults << parameters_.damping, parameters_.damping,
      parameters_.damping, 4.5, 4.5, 4.5;
  Vector6d stiffness_defaults;
  stiffness_defaults << parameters_.stiffness, parameters_.stiffness,
      parameters_.stiffness, 15.0, 15.0, 15.0;
  Vector6d max_offset_defaults;
  max_offset_defaults << parameters_.max_offset, parameters_.max_offset,
      parameters_.max_offset, 0.15, 0.15, 0.15;
  Vector6d max_velocity_defaults;
  max_velocity_defaults << parameters_.max_velocity, parameters_.max_velocity,
      parameters_.max_velocity, 0.15, 0.15, 0.15;
  const Vector6d alpha_defaults =
      Vector6d::Constant(parameters_.filter_alpha);

  parameters_.mass_6d = vector6Parameter(*this, "mass_6d", mass_defaults);
  parameters_.damping_6d = vector6Parameter(
      *this, "damping_6d", damping_defaults);
  parameters_.stiffness_6d = vector6Parameter(
      *this, "stiffness_6d", stiffness_defaults);
  parameters_.max_offset_6d = vector6Parameter(
      *this, "max_offset_6d", max_offset_defaults);
  parameters_.max_velocity_6d = vector6Parameter(
      *this, "max_velocity_6d", max_velocity_defaults);
  parameters_.filter_alpha_6d = vector6Parameter(
      *this, "filter_alpha_6d", alpha_defaults);
  parameters_.wrench_scale_6d = vector6Parameter(
      *this, "wrench_scale_6d", Vector6d::Ones());

  Vector6d hard_wrench_defaults;
  hard_wrench_defaults << 40.0, 40.0, 40.0, 5.0, 5.0, 5.0;
  parameters_.hard_wrench_limit = vector6Parameter(
      *this, "hard_wrench_limit", hard_wrench_defaults);
  if ((parameters_.hard_wrench_limit.array() <= 0.0).any())
  {
    throw std::runtime_error("hard_wrench_limit values must be positive");
  }
  parameters_.max_wrench_rate = vector6Parameter(
      *this, "max_wrench_rate", Vector6d::Zero());
  if ((parameters_.max_wrench_rate.array() < 0.0).any())
  {
    throw std::runtime_error("max_wrench_rate values must be non-negative");
  }

  parameters_.target_topic = parameters_.robot_name + "_mpc_target";
  parameters_.status_topic = declare_parameter<std::string>(
      "status_topic", "/whole_body_force_control/status");
  parameters_.control_state_topic = declare_parameter<std::string>(
      "control_state_topic", "/whole_body_force_control/control_state");
  parameters_.wrench_topic = declare_parameter<std::string>(
      "wrench_topic", "/whole_body_force_control/wrench");
}

}  // namespace whole_body_force_control
