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
// End-effector gravity-load compensation.
//
// The identified static model is:
//   f_s = R_sb * h_b + b_f
//   tau_s = r_sc x (R_sb * h_b) + b_tau
// where:
//   h_b  : signed payload gravity force in the robot base frame [N]
//   R_sb : rotation from base frame to sensor frame
//   r_sc : sensor-origin-to-payload-CoM vector, expressed in sensor frame [m]
//   b_f  : constant force bias in sensor frame [N]
//   b_tau: constant torque bias in sensor frame [Nm]
//
// If mass_kg and gravity_direction_base are configured, h_b is computed as
// mass_kg * gravity_m_s2 * gravity_direction_base.
struct LoadCompensationConfig
{
  bool enable{false};
  double gravity_m_s2{9.80665};
  double mass_kg{0.0};
  // Unit vector of the signed gravity force in the robot base frame.
  Eigen::Vector3d gravity_direction_base{0.0, 0.0, -1.0};
  Eigen::Vector3d center_of_mass_sensor_m{0.0, 0.0, 0.0};
  Vector6d bias_sensor{Vector6d::Zero()};
};

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

  LoadCompensationConfig load_compensation{};
  double force_deadband_n{1.0};
  double torque_deadband_nm{0.1};
};

struct ForceProcessorResult
{
  bool ok{false};
  bool taring{false};
  bool hard_limit_exceeded{false};
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
  // source_rotation_base: rotation from base frame to source frame, used only
  // when load compensation is enabled.  Defaults to identity so existing
  // callers/tests keep the original behaviour.
  ForceProcessorResult process(
    const wbmm::core::Wrench & raw_source,
    const Eigen::Matrix3d & target_rotation_source,
    const Eigen::Vector3d & target_to_source,
    const Eigen::Matrix3d & source_rotation_base =
      Eigen::Matrix3d::Identity());

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
