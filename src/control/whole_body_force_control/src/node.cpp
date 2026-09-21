#include "node.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace whole_body_force_control
{
namespace
{

Eigen::Vector3d log3(const Eigen::Matrix3d &rotation)
{
  const Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle())) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.angle() * angle_axis.axis();
}

}  // namespace

WholeBodyForceControlNode::WholeBodyForceControlNode()
    : Node("whole_body_force_control")
{
  loadParameters();

  robot_model_ = std::make_shared<PinocchioRobotModel>(
      parameters_.urdf_file);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  kinematics_ = std::make_unique<WholeBodyKinematics>(
      robot_model_, parameters_.tcp_frame, parameters_.state_frame);

  configureForceProcessor();

  cartesian_controller_ = std::make_unique<CartesianComplianceController>(
      parameters_.admittance_axes, parameters_.mass, parameters_.damping,
      parameters_.stiffness, parameters_.max_velocity);

  createRosInterfaces();
  force_processor_.startTare();

  const auto start_time = std::chrono::steady_clock::now();
  last_update_ = start_time;
  last_wrench_ = start_time;
  last_observation_ = start_time;
  capture_requested_at_ = start_time;
  const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, parameters_.loop_rate));
  timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WholeBodyForceControlNode::update, this));
  RCLCPP_INFO(
      get_logger(),
      "Ready: frame=%s axes=[%s] admittance_enable=%s output=%s",
      parameters_.state_frame.c_str(),
      enabledAxes(parameters_.admittance_axes).c_str(),
      parameters_.admittance_enabled ? "true" : "false",
      parameters_.reference_output_enabled ? "true" : "false");
  publishState(
      parameters_.admittance_enabled ? "WAITING_FOR_DATA" : "DISABLED");
}

void WholeBodyForceControlNode::requestFault(const std::string &reason)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!fault_latched_ && !pending_fault_) {
    pending_fault_ = true;
    pending_fault_reason_ = reason;
  }
}

void WholeBodyForceControlNode::latchFault(const std::string &reason)
{
  if (fault_latched_) {
    return;
  }
  RCLCPP_ERROR(get_logger(), "Force-control fault latched: %s", reason.c_str());
  fault_latched_ = true;
  fault_reason_ = reason;
  parameters_.admittance_enabled = false;
  nominal_captured_ = false;
  hold_state_valid_ = false;
  force_processor_.reset();
  cartesian_controller_->reset(measuredWrenchVector());
  publishState("FAULT_" + reason);
  publishHoldReference();
}

void WholeBodyForceControlNode::publishHoldReference()
{
  if (!observation_received_) {
    return;
  }
  if (parameters_.output_mode == ReferenceOutputMode::kEndEffectorPose) {
    publishHoldEndEffectorReference();
    return;
  }
  if (!hold_state_valid_) {
    hold_state_ = observationStateLocked();
    hold_state_valid_ = true;
  }
  publishReference(hold_state_);
}

void WholeBodyForceControlNode::checkFaults(
    bool observation_timed_out, bool wrench_timed_out)
{
  if (parameters_.admittance_enabled && observation_timed_out)
  {
    latchFault("OBSERVATION_TIMEOUT");
  }
  else if (parameters_.admittance_enabled && wrench_timed_out)
  {
    latchFault("WRENCH_TIMEOUT");
  }
  else if (parameters_.admittance_enabled &&
           parameters_.enforce_single_target_owner &&
           foreignTargetPublisherPresent())
  {
    latchFault("TARGET_OWNER");
  }
}

void WholeBodyForceControlNode::captureNominalState(
    const Eigen::VectorXd &measured_state)
{
  nominal_state_ = measured_state;
  last_reference_state_ = nominal_state_;
  nominal_tcp_ = kinematics_->framePosition(nominal_state_);
  nominal_tcp_rotation_ = kinematics_->frameRotation(nominal_state_);
  cartesian_controller_->reset(measuredWrenchVector());
  last_ee_correction_.setZero();
  ee_correction_valid_ = false;
  nominal_captured_ = true;
  RCLCPP_INFO(
      get_logger(),
      "Captured nominal state; admittance wrench is expressed in %s",
      parameters_.state_frame.c_str());
  publishState("ACTIVE");
}

Vector6d WholeBodyForceControlNode::correctionFromReferencePose(
    const Eigen::VectorXd &reference) const
{
  Vector6d correction = Vector6d::Zero();
  if (!nominal_captured_ || nominal_state_.size() != reference.size()) {
    return correction;
  }

  const Eigen::Vector3d reference_tcp = kinematics_->framePosition(reference);
  const Eigen::Matrix3d reference_rotation =
      kinematics_->frameRotation(reference);

  // Kinematics produced a target in the nominal TCP frame:
  //   p_ref = p_nom + R_nom * dx
  //   R_ref = exp3(R_nom * dtheta) * R_nom
  correction.head<3>() =
      nominal_tcp_rotation_.transpose() * (reference_tcp - nominal_tcp_);
  const Eigen::Matrix3d world_rotation =
      reference_rotation * nominal_tcp_rotation_.transpose();
  correction.tail<3>() =
      nominal_tcp_rotation_.transpose() * log3(world_rotation);
  return correction;
}

void WholeBodyForceControlNode::updateReference(
    const Vector6d &measured_wrench,
    double dt,
    Vector6d &correction,
    Vector6d &filtered_wrench,
    Eigen::VectorXd &reference,
    double &primary_offset,
    double &primary_force)
{
  correction = cartesian_controller_->update(measured_wrench, dt);
  filtered_wrench = cartesian_controller_->measuredWrench();

  // The admittance offset is defined relative to the captured nominal TCP
  // frame.  Use that fixed nominal rotation (not the instantaneous measured
  // TCP TF) so a tracking deviation cannot rotate the force direction and
  // create a feedback loop or reference jump.
  const auto makeReference = [&](const Vector6d &local_correction) {
      Vector6d world_correction;
      world_correction.head<3>() =
          nominal_tcp_rotation_ * local_correction.head<3>();
      world_correction.tail<3>() =
          nominal_tcp_rotation_ * local_correction.tail<3>();
      return kinematics_->correctedStateWorld6D(
          nominal_state_, world_correction, parameters_.base_share);
    };

  reference = makeReference(correction);

  // Anti-windup for a force request that the arm/whole-body workspace cannot
  // realize.  Without this, K=0 force following integrates an ever-larger
  // offset while the IK output is clamped, and releasing the force leaves a
  // large hidden reference; a later command then appears as a jump.
  const Vector6d achieved_correction =
      correctionFromReferencePose(reference);
  const bool correction_limited =
      cartesian_controller_->clampOffset(achieved_correction);
  if (correction_limited) {
    const double desired_norm = correction.norm();
    const double reached_norm = achieved_correction.norm();
    const double max_axis_delta =
        (correction - achieved_correction).cwiseAbs().maxCoeff();
    correction = cartesian_controller_->offset();
    reference = makeReference(correction);
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Admittance correction saturated (desired=%.6f reached=%.6f "
        "max_axis_delta=%.6f); integrator reduced to prevent windup",
        desired_norm, reached_norm, max_axis_delta);
  }

  // Rate-limit the complete reference, including base translation.  The IK
  // can jump to another solution branch near singularities; MPC must receive
  // a continuous target rather than a step.
  if (last_reference_state_.size() == reference.size()) {
    const double limited_dt = std::clamp(dt, 0.0, 0.05);
    const double base_step = parameters_.max_base_velocity * limited_dt;
    for (Eigen::Index i = 0; i < 2; ++i) {
      const double delta = reference[i] - last_reference_state_[i];
      reference[i] = last_reference_state_[i] +
          std::clamp(delta, -base_step, base_step);
    }
    const int arm_dimension = kinematics_->armDimension();
    const double joint_step =
        parameters_.max_joint_velocity * limited_dt;
    for (int i = 0; i < arm_dimension; ++i) {
      const Eigen::Index index = 3 + i;
      const double delta = reference[index] - last_reference_state_[index];
      reference[index] = last_reference_state_[index] +
          std::clamp(delta, -joint_step, joint_step);
    }
  }
  last_reference_state_ = reference;

  const Eigen::Vector3d measured_force = filtered_wrench.head<3>();
  const Eigen::Vector3d correction_translation = correction.head<3>();
  if (measured_force.norm() > 1.0e-9)
  {
    const Eigen::Vector3d direction = measured_force.normalized();
    primary_force = measured_force.norm();
    primary_offset = correction_translation.dot(direction);
  }
  else
  {
    for (std::size_t i = 0; i < 6; ++i)
    {
      if (parameters_.admittance_axes[i])
      {
        primary_offset = correction[i];
        primary_force = filtered_wrench[i];
        break;
      }
    }
  }
}

void WholeBodyForceControlNode::clampEndEffectorCorrection(
    Vector6d & correction)
{
  Vector6d clamped = correction;
  bool limited = false;

  if (parameters_.max_ee_translation_offset > 0.0) {
    for (Eigen::Index i = 0; i < 3; ++i) {
      const double value = std::clamp(
          clamped[i], -parameters_.max_ee_translation_offset,
          parameters_.max_ee_translation_offset);
      limited = limited || (value != clamped[i]);
      clamped[i] = value;
    }
  }
  if (parameters_.max_ee_rotation_offset > 0.0) {
    for (Eigen::Index i = 3; i < 6; ++i) {
      const double value = std::clamp(
          clamped[i], -parameters_.max_ee_rotation_offset,
          parameters_.max_ee_rotation_offset);
      limited = limited || (value != clamped[i]);
      clamped[i] = value;
    }
  }

  if (limited) {
    // Reduce the hidden admittance integrator to the reachable task-space
    // offset so a released force cannot leave a large hidden target.
    cartesian_controller_->clampOffset(clamped);
    correction = cartesian_controller_->offset();
  } else {
    correction = clamped;
  }
}

void WholeBodyForceControlNode::updateEndEffectorReference(
    const Vector6d & measured_wrench,
    double dt,
    Vector6d & correction,
    Vector6d & filtered_wrench,
    wbmm::core::EndEffectorPose & target,
    double & primary_offset,
    double & primary_force)
{
  correction = cartesian_controller_->update(measured_wrench, dt);
  filtered_wrench = cartesian_controller_->measuredWrench();

  if (!ee_correction_valid_) {
    last_ee_correction_ = correction;
    ee_correction_valid_ = true;
  }

  // Rate-limit the task-space correction in the nominal TCP frame.  This
  // prevents a discontinuous target when admittance is enabled or when a
  // force step occurs, without reintroducing base/arm allocation.
  const double limited_dt = std::clamp(dt, 0.0, 0.05);
  const double linear_step =
      parameters_.max_ee_linear_velocity * limited_dt;
  const double angular_step =
      parameters_.max_ee_angular_velocity * limited_dt;

  Vector6d limited = correction;
  for (Eigen::Index i = 0; i < 3; ++i) {
    limited[i] = last_ee_correction_[i] +
        std::clamp(
            correction[i] - last_ee_correction_[i],
            -linear_step, linear_step);
  }
  for (Eigen::Index i = 3; i < 6; ++i) {
    limited[i] = last_ee_correction_[i] +
        std::clamp(
            correction[i] - last_ee_correction_[i],
            -angular_step, angular_step);
  }

  clampEndEffectorCorrection(limited);
  correction = limited;
  last_ee_correction_ = correction;

  wbmm::core::Pose nominal_pose;
  nominal_pose.header.frame_id = parameters_.state_frame;
  nominal_pose.header.stamp = observation_time_;
  nominal_pose.position.x = nominal_tcp_.x();
  nominal_pose.position.y = nominal_tcp_.y();
  nominal_pose.position.z = nominal_tcp_.z();
  const Eigen::Quaterniond nominal_quaternion(nominal_tcp_rotation_);
  nominal_pose.orientation.w = nominal_quaternion.w();
  nominal_pose.orientation.x = nominal_quaternion.x();
  nominal_pose.orientation.y = nominal_quaternion.y();
  nominal_pose.orientation.z = nominal_quaternion.z();

  target = makeEndEffectorPoseTarget(
      nominal_pose, correction, parameters_.state_frame,
      observation_time_ + 0.02);

  const Eigen::Vector3d measured_force = filtered_wrench.head<3>();
  const Eigen::Vector3d correction_translation = correction.head<3>();
  if (measured_force.norm() > 1.0e-9) {
    const Eigen::Vector3d direction = measured_force.normalized();
    primary_force = measured_force.norm();
    primary_offset = correction_translation.dot(direction);
  } else {
    for (std::size_t i = 0; i < 6; ++i) {
      if (parameters_.admittance_axes[i]) {
        primary_offset = correction[i];
        primary_force = filtered_wrench[i];
        break;
      }
    }
  }
}

void WholeBodyForceControlNode::configureForceProcessor()
{
  ForceProcessorConfig config;
  config.tare_samples = parameters_.tare_samples;
  config.filter_alpha = Vector6d::Constant(parameters_.filter_alpha);
  config.scale = parameters_.wrench_scale;
  config.hard_limit_enabled = true;
  config.hard_wrench_limit = parameters_.hard_wrench_limit;
  config.hard_force_norm_limit = parameters_.hard_force_norm_limit;
  force_processor_.setConfig(config);
}

void WholeBodyForceControlNode::update()
{
  const auto wall_now = std::chrono::steady_clock::now();
  const double dt = std::chrono::duration<double>(wall_now - last_update_).count();
  last_update_ = wall_now;
  std::lock_guard<std::mutex> lock(mutex_);

  if (pending_fault_)
  {
    const std::string reason = pending_fault_reason_;
    pending_fault_ = false;
    pending_fault_reason_.clear();
    latchFault(reason);
  }

  if (!observation_received_)
  {
    publishState(
        parameters_.admittance_enabled ? "WAITING_FOR_OBSERVATION" : "DISABLED");
    return;
  }

  if (force_processor_.taring())
  {
    publishState("TARING");
    publishHoldReference();
    return;
  }

  const Eigen::VectorXd measured_state = observationStateLocked();
  const Vector6d measured_wrench = measuredWrenchVector();
  const bool observation_timed_out =
      std::chrono::duration<double>(wall_now - last_observation_).count() >
          parameters_.observation_timeout;
  const bool wrench_timed_out =
      !wrench_received_ ||
      std::chrono::duration<double>(wall_now - last_wrench_).count() >
          parameters_.force_timeout;

  if (parameters_.admittance_enabled && !wrench_received_)
  {
    publishState("WAITING_FOR_WRENCH");
    publishHoldReference();
    return;
  }

  checkFaults(observation_timed_out, wrench_timed_out);
  if (!parameters_.admittance_enabled)
  {
    publishState(
        fault_latched_ ? "FAULT_" + fault_reason_ : "DISABLED");
    publishHoldReference();
    return;
  }

  if (!nominal_captured_)
  {
    const double elapsed =
        std::chrono::duration<double>(wall_now - capture_requested_at_).count();
    if (elapsed < parameters_.capture_settle_time)
    {
      publishState("SETTLING");
      return;
    }
    captureNominalState(measured_state);
  }

  Vector6d correction = Vector6d::Zero();
  Vector6d filtered_wrench = Vector6d::Zero();
  double primary_offset = 0.0;
  double primary_force = 0.0;

  if (parameters_.output_mode == ReferenceOutputMode::kEndEffectorPose) {
    wbmm::core::EndEffectorPose target;
    updateEndEffectorReference(
        measured_wrench, dt, correction, filtered_wrench, target,
        primary_offset, primary_force);
    publishEndEffectorReference(target);
    publishEndEffectorCorrection(
        target, measured_state, primary_force, primary_offset,
        filtered_wrench, correction);
    return;
  }

  Eigen::VectorXd reference;
  updateReference(
      measured_wrench, dt, correction, filtered_wrench, reference,
      primary_offset, primary_force);

  publishReference(reference);
  publishCorrection(
      reference, measured_state, primary_force, primary_offset,
      filtered_wrench, correction);
}

}  // namespace whole_body_force_control

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(
        std::make_shared<whole_body_force_control::WholeBodyForceControlNode>());
  }
  catch (const std::exception &exception)
  {
    RCLCPP_FATAL(
        rclcpp::get_logger("whole_body_force_control"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
