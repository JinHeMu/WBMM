#include "whole_body_force_control/force_processor.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace whole_body_force_control
{

ForceProcessor::ForceProcessor(const ForceProcessorConfig & config)
{
  setConfig(config);
}

void ForceProcessor::setConfig(const ForceProcessorConfig & config)
{
  config_ = config;
  config_.tare_samples = std::max<std::size_t>(1, config_.tare_samples);
  config_.hard_force_norm_limit =
    std::max(0.0, config_.hard_force_norm_limit);
  config_.force_deadband_n = std::max(0.0, config_.force_deadband_n);
  config_.torque_deadband_nm = std::max(0.0, config_.torque_deadband_nm);
  for (Eigen::Index i = 0; i < 6; ++i) {
    config_.filter_alpha[i] = std::clamp(config_.filter_alpha[i], 0.0, 1.0);
    config_.hard_wrench_limit[i] = std::abs(config_.hard_wrench_limit[i]);
  }

  if (config_.load_compensation.enable) {
    if (!std::isfinite(config_.load_compensation.gravity_m_s2) ||
        config_.load_compensation.gravity_m_s2 <= 0.0) {
      throw std::invalid_argument(
        "load_compensation.gravity_m_s2 must be positive");
    }
    if (!std::isfinite(config_.load_compensation.mass_kg) ||
        config_.load_compensation.mass_kg < 0.0) {
      throw std::invalid_argument(
        "load_compensation.mass_kg must be non-negative");
    }
    if (!config_.load_compensation.gravity_direction_base.allFinite()) {
      throw std::invalid_argument(
        "load_compensation.gravity_direction_base must be finite");
    }
    const double direction_norm =
      config_.load_compensation.gravity_direction_base.norm();
    if (!std::isfinite(direction_norm) || direction_norm < 1.0e-9) {
      throw std::invalid_argument(
        "load_compensation.gravity_direction_base must be non-zero");
    }
    config_.load_compensation.gravity_direction_base.normalize();
    if (!config_.load_compensation.center_of_mass_sensor_m.allFinite() ||
        !config_.load_compensation.bias_sensor.allFinite()) {
      throw std::invalid_argument(
        "load_compensation CoM and bias must be finite");
    }
    // Gravity compensation replaces the tare step.  Clear any tare state so a
    // previous startTare() call cannot silently subtract a second offset.
    tare_active_ = false;
    tare_sum_.setZero();
    tare_offset_.setZero();
    tare_count_ = 0;
    filter_initialized_ = false;
    filtered_wrench_.setZero();
    last_output_.setZero();
  }
}

void ForceProcessor::startTare()
{
  if (config_.load_compensation.enable) {
    tare_active_ = false;
    tare_sum_.setZero();
    tare_offset_.setZero();
    tare_count_ = 0;
    filter_initialized_ = false;
    filtered_wrench_.setZero();
    last_output_.setZero();
    return;
  }
  tare_active_ = true;
  tare_sum_.setZero();
  tare_offset_.setZero();
  tare_count_ = 0;
  filter_initialized_ = false;
  filtered_wrench_.setZero();
  last_output_.setZero();
}

void ForceProcessor::reset()
{
  tare_active_ = false;
  tare_sum_.setZero();
  tare_offset_.setZero();
  tare_count_ = 0;
  filter_initialized_ = false;
  filtered_wrench_.setZero();
  last_output_.setZero();
}

Vector6d ForceProcessor::toVector(const wbmm::core::Wrench & wrench)
{
  Vector6d values;
  values << wrench.force.x, wrench.force.y, wrench.force.z,
      wrench.torque.x, wrench.torque.y, wrench.torque.z;
  return values;
}

wbmm::core::Wrench ForceProcessor::toWrench(
  const Vector6d & values, const wbmm::core::Header & header)
{
  wbmm::core::Wrench wrench;
  wrench.header = header;
  wrench.force.x = values[0];
  wrench.force.y = values[1];
  wrench.force.z = values[2];
  wrench.torque.x = values[3];
  wrench.torque.y = values[4];
  wrench.torque.z = values[5];
  return wrench;
}

ForceProcessorResult ForceProcessor::process(
  const wbmm::core::Wrench & raw_source,
  const Eigen::Matrix3d & target_rotation_source,
  const Eigen::Vector3d & target_to_source,
  const Eigen::Matrix3d & source_rotation_base)
{
  ForceProcessorResult result;
  result.wrench.header = raw_source.header;

  const Vector6d raw = toVector(raw_source);
  if (!raw.allFinite() || !source_rotation_base.allFinite()) {
    return result;
  }

  const bool compensation_enabled = config_.load_compensation.enable;

  Vector6d compensated_raw = raw;
  if (compensation_enabled) {
    const auto & config = config_.load_compensation;
    const Eigen::Vector3d gravity_force_base =
      config.mass_kg * config.gravity_m_s2 * config.gravity_direction_base;
    const Eigen::Vector3d gravity_force_sensor =
      source_rotation_base * gravity_force_base;

    Vector6d gravity_wrench;
    gravity_wrench.head<3>() = gravity_force_sensor;
    gravity_wrench.tail<3>() =
      config.center_of_mass_sensor_m.cross(gravity_force_sensor);

    compensated_raw = raw - config.bias_sensor - gravity_wrench;
    if (!compensated_raw.allFinite()) {
      return result;
    }
  }

  // Legacy tare mode keeps the fail-closed raw hard limit before tare so a
  // large raw step cannot be filtered away.  In compensation mode the known
  // payload gravity/bias is removed first, otherwise a legitimate payload
  // weight could trip the raw limit.
  if (config_.hard_limit_enabled && !compensation_enabled) {
    Vector6d raw_for_limit = raw;
    for (Eigen::Index i = 0; i < 6; ++i) {
      raw_for_limit[i] *= config_.scale[i];
    }
    if (!raw_for_limit.allFinite()) {
      return result;
    }
    if (config_.hard_force_norm_limit > 0.0 &&
      raw_for_limit.head<3>().norm() > config_.hard_force_norm_limit)
    {
      result.hard_limit_exceeded = true;
      return result;
    }
    if ((raw_for_limit.cwiseAbs().array() >
      config_.hard_wrench_limit.array()).any())
    {
      result.hard_limit_exceeded = true;
      return result;
    }
  }

  if (!compensation_enabled && tare_active_) {
    tare_sum_ += raw;
    ++tare_count_;
    if (tare_count_ < config_.tare_samples) {
      result.taring = true;
      result.tare_samples_collected = tare_count_;
      return result;
    }
    tare_offset_ = tare_sum_ / static_cast<double>(tare_count_);
    tare_active_ = false;
    filter_initialized_ = false;
    filtered_wrench_.setZero();
    last_output_.setZero();
  }

  result.tare_samples_collected = tare_count_;

  Vector6d source = compensated_raw;
  if (!compensation_enabled) {
    source -= tare_offset_;
  }
  for (Eigen::Index i = 0; i < 6; ++i) {
    source[i] *= config_.scale[i];
  }

  if (config_.hard_limit_enabled && compensation_enabled) {
    if (config_.hard_force_norm_limit > 0.0 &&
      source.head<3>().norm() > config_.hard_force_norm_limit)
    {
      result.hard_limit_exceeded = true;
      return result;
    }
    if ((source.cwiseAbs().array() >
      config_.hard_wrench_limit.array()).any())
    {
      result.hard_limit_exceeded = true;
      return result;
    }
  }

  Vector6d processed;
  try {
    processed = transformWrench(source, target_rotation_source, target_to_source);
  } catch (const std::exception &) {
    return result;
  }

  if (!filter_initialized_) {
    filtered_wrench_ = processed;
    filter_initialized_ = true;
    last_output_ = processed;
  } else {
    for (Eigen::Index i = 0; i < 6; ++i) {
      const double alpha = config_.filter_alpha[i];
      filtered_wrench_[i] =
        alpha * processed[i] + (1.0 - alpha) * filtered_wrench_[i];
    }
  }

  if (!filtered_wrench_.allFinite()) {
    return result;
  }

  // Deadband after low-pass filtering: small residual noise is reported as
  // exactly zero so it cannot drive the admittance integrator.
  for (Eigen::Index i = 0; i < 3; ++i) {
    if (std::abs(filtered_wrench_[i]) < config_.force_deadband_n) {
      filtered_wrench_[i] = 0.0;
    }
  }
  for (Eigen::Index i = 3; i < 6; ++i) {
    if (std::abs(filtered_wrench_[i]) < config_.torque_deadband_nm) {
      filtered_wrench_[i] = 0.0;
    }
  }

  if (config_.hard_limit_enabled &&
    ((config_.hard_force_norm_limit > 0.0 &&
    filtered_wrench_.head<3>().norm() > config_.hard_force_norm_limit) ||
    (filtered_wrench_.cwiseAbs().array() >
    config_.hard_wrench_limit.array()).any()))
  {
    result.hard_limit_exceeded = true;
    return result;
  }

  last_output_ = filtered_wrench_;
  result.wrench = toWrench(filtered_wrench_, raw_source.header);
  result.ok = true;
  return result;
}

}  // namespace whole_body_force_control
