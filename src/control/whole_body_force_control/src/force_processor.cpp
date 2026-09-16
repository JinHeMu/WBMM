#include "whole_body_force_control/force_processor.hpp"

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
  for (Eigen::Index i = 0; i < 6; ++i) {
    config_.filter_alpha[i] = std::clamp(config_.filter_alpha[i], 0.0, 1.0);
    config_.hard_wrench_limit[i] = std::abs(config_.hard_wrench_limit[i]);
    config_.max_wrench_rate[i] = std::max(0.0, config_.max_wrench_rate[i]);
  }
}

void ForceProcessor::startTare()
{
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
  double dt)
{
  ForceProcessorResult result;
  result.wrench.header = raw_source.header;

  const Vector6d raw = toVector(raw_source);
  if (!raw.allFinite()) {
    return result;
  }

  if (tare_active_) {
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

  const Vector6d source = raw - tare_offset_;
  Vector6d processed;
  try {
    processed = transformWrench(source, target_rotation_source, target_to_source);
  } catch (const std::exception &) {
    return result;
  }

  for (Eigen::Index i = 0; i < 6; ++i) {
    processed[i] *= config_.scale[i];
    if (config_.absolute_axes[static_cast<std::size_t>(i)]) {
      processed[i] = std::abs(processed[i]);
    }
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

  if (config_.hard_limit_enabled &&
    (filtered_wrench_.cwiseAbs().array() > config_.hard_wrench_limit.array()).any())
  {
    result.hard_limit_exceeded = true;
    return result;
  }

  for (Eigen::Index i = 0; i < 6; ++i) {
    const double max_rate = config_.max_wrench_rate[i];
    if (max_rate <= 0.0 || dt <= 0.0) {
      continue;
    }
    const double max_step = max_rate * dt;
    const double delta = filtered_wrench_[i] - last_output_[i];
    if (std::abs(delta) > max_step) {
      filtered_wrench_[i] = last_output_[i] + std::copysign(max_step, delta);
      result.rate_limited = true;
    }
  }

  last_output_ = filtered_wrench_;
  result.wrench = toWrench(filtered_wrench_, raw_source.header);
  result.ok = true;
  return result;
}

}  // namespace whole_body_force_control
