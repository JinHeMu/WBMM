#include "whole_body_force_control/controllers.hpp"

#include <Eigen/Geometry>

#include <stdexcept>

namespace whole_body_force_control
{

Vector6d transformWrench(
  const Vector6d & source_wrench,
  const Eigen::Matrix3d & target_rotation_source,
  const Eigen::Vector3d & target_to_source)
{
  if (!source_wrench.allFinite() || !target_rotation_source.allFinite() ||
    !target_to_source.allFinite())
  {
    throw std::invalid_argument("wrench transform input is non-finite");
  }
  const Eigen::Vector3d force =
    target_rotation_source * source_wrench.head<3>();
  const Eigen::Vector3d torque =
    target_rotation_source * source_wrench.tail<3>();
  Vector6d target;
  target.head<3>() = force;
  target.tail<3>() = torque + target_to_source.cross(force);
  return target;
}

AdmittanceController::AdmittanceController(
  double mass,
  double damping,
  double stiffness,
  double max_velocity)
: mass_(std::max(1.0e-6, mass)),
  damping_(std::max(0.0, damping)),
  stiffness_(std::max(0.0, stiffness)),
  max_velocity_(std::abs(max_velocity))
{}

double AdmittanceController::update(double measured_force, double dt)
{
  measured_force_ = measured_force;
  dt = std::clamp(dt, 0.0, 0.05);

  const double acceleration =
    (measured_force_ - damping_ * velocity_ - stiffness_ * offset_) / mass_;
  velocity_ = std::clamp(
    velocity_ + dt * acceleration, -max_velocity_, max_velocity_);
  const double previous_offset = offset_;
  offset_ += dt * velocity_;
  if (dt <= 0.0) {
    offset_ = previous_offset;
  }
  return offset_;
}

void AdmittanceController::reset(double measured_force)
{
  measured_force_ = measured_force;
  offset_ = 0.0;
  velocity_ = 0.0;
}

bool AdmittanceController::limitOffset(double reachable_offset)
{
  if (!std::isfinite(reachable_offset)) {
    return false;
  }
  const double bounded = reachable_offset;

  // Only shrink an existing correction in the same direction.  Letting a
  // saturated reachable value flip the sign would itself create a jump, and
  // the regular admittance dynamics must remain free to reverse the motion.
  const bool same_direction =
    offset_ == 0.0 || bounded == 0.0 ||
    std::signbit(offset_) == std::signbit(bounded);
  // The IK/kinematics solve carries a few micrometres of numerical error.
  // A 0.1 mm / 0.0057 deg deadband prevents that error from looking like
  // saturation while still stopping real workspace windup almost immediately.
  constexpr double kAntiWindupDeadband = 1.0e-4;
  if (same_direction &&
    std::abs(bounded) + kAntiWindupDeadband < std::abs(offset_))
  {
    offset_ = bounded;
    velocity_ = 0.0;
    return true;
  }
  return false;
}

CartesianComplianceController::CartesianComplianceController(
  const AxisMask6d & admittance_axes,
  const Vector6d & mass,
  const Vector6d & damping,
  const Vector6d & stiffness,
  const Vector6d & max_velocity)
: admittance_axes_(admittance_axes)
{
  for (std::size_t i = 0; i < 6; ++i) {
    admittance_[i] = std::make_unique<AdmittanceController>(
      mass[i], damping[i], stiffness[i], max_velocity[i]);
  }
}

Vector6d CartesianComplianceController::update(
  const Vector6d & measured_wrench, double dt)
{
  measured_wrench_ = measured_wrench;
  for (std::size_t i = 0; i < 6; ++i) {
    if (!admittance_axes_[i]) {
      admittance_[i]->reset(measured_wrench[i]);
      offset_[i] = 0.0;
      velocity_[i] = 0.0;
      continue;
    }
    offset_[i] = admittance_[i]->update(measured_wrench[i], dt);
    velocity_[i] = admittance_[i]->velocity();
  }
  return offset_;
}

bool CartesianComplianceController::clampOffset(
  const Vector6d & reachable_offset)
{
  bool changed = false;
  for (std::size_t i = 0; i < 6; ++i) {
    if (!admittance_axes_[i]) {
      continue;
    }
    changed = admittance_[i]->limitOffset(reachable_offset[i]) || changed;
    offset_[i] = admittance_[i]->offset();
    velocity_[i] = admittance_[i]->velocity();
  }
  return changed;
}

void CartesianComplianceController::reset(const Vector6d & measured_wrench)
{
  for (std::size_t i = 0; i < 6; ++i) {
    admittance_[i]->reset(measured_wrench[i]);
  }
  offset_.setZero();
  velocity_.setZero();
  measured_wrench_ = measured_wrench;
}

}  // namespace whole_body_force_control
