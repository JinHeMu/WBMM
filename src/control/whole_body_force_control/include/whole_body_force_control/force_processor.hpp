#pragma once

#include "whole_body_force_control/controllers.hpp"

#include <wbmm_core/types.hpp>

#include <Eigen/Core>

#include <cstddef>

namespace whole_body_force_control
{

// Force preprocessing pipeline:
//   raw FTS -> finite/raw hard-limit checks -> tare -> coordinate transform
//           -> scale -> low-pass -> finite/hard-limit/rate-limit checks
//
// The controller layer receives an already processed wrench.
struct ForceProcessorConfig
{
  std::size_t tare_samples{50};
  Vector6d filter_alpha{Vector6d::Ones()};
  Vector6d scale{Vector6d::Ones()};
  bool hard_limit_enabled{true};
  Vector6d hard_wrench_limit{Vector6d::Constant(1000.0)};
  // Euclidean norm of raw Fx/Fy/Fz [N].  Checked before tare and filtering so
  // a large step stops immediately.  <=0 disables this particular check.
  double hard_force_norm_limit{20.0};
  Vector6d max_wrench_rate{Vector6d::Zero()};  // <=0 disables that axis
};

struct ForceProcessorResult
{
  bool ok{false};
  bool taring{false};
  bool hard_limit_exceeded{false};
  bool rate_limited{false};
  std::size_t tare_samples_collected{0};
  wbmm::core::Wrench wrench;
};

class ForceProcessor
{
public:
  ForceProcessor() = default;
  explicit ForceProcessor(const ForceProcessorConfig & config);

  void setConfig(const ForceProcessorConfig & config);
  void startTare();
  void reset();

  // target_rotation_source: rotation from source frame to target frame.
  // target_to_source: source origin position expressed in target frame.
  ForceProcessorResult process(
    const wbmm::core::Wrench & raw_source,
    const Eigen::Matrix3d & target_rotation_source,
    const Eigen::Vector3d & target_to_source,
    double dt);

  [[nodiscard]] bool taring() const {return tare_active_;}
  [[nodiscard]] std::size_t tareSamplesCollected() const {return tare_count_;}
  [[nodiscard]] std::size_t tareSamplesRequired() const
  {
    return config_.tare_samples;
  }

  [[nodiscard]] const Vector6d & tareOffset() const {return tare_offset_;}

private:
  static Vector6d toVector(const wbmm::core::Wrench & wrench);
  static wbmm::core::Wrench toWrench(
    const Vector6d & values, const wbmm::core::Header & header);

  ForceProcessorConfig config_{};
  bool tare_active_{false};
  Vector6d tare_sum_{Vector6d::Zero()};
  Vector6d tare_offset_{Vector6d::Zero()};
  std::size_t tare_count_{0};

  bool filter_initialized_{false};
  Vector6d filtered_wrench_{Vector6d::Zero()};
  Vector6d last_output_{Vector6d::Zero()};
};

}  // namespace whole_body_force_control
