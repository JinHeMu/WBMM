#include "node.hpp"

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

WholeBodyForceControlNode::WholeBodyForceControlNode()
    : Node("whole_body_force_control")
{
  loadParameters();

  robot_model_ = std::make_shared<PinocchioRobotModel>(
      parameters_.urdf_file);
  kinematics_ = std::make_unique<WholeBodyKinematics>(
      robot_model_, parameters_.ee_frame, parameters_.state_frame,
      wbmm::core::ClockDomain::kOcs2Mpc);

  configureForceProcessor();

  cartesian_controller_ = std::make_unique<CartesianComplianceController>(
      parameters_.admittance_axes, parameters_.constant_force_axes,
      parameters_.desired_wrench, parameters_.mass_6d,
      parameters_.damping_6d, parameters_.stiffness_6d,
      parameters_.max_offset_6d, parameters_.max_velocity_6d,
      parameters_.control_mode == "force_follow");

  const double legacy_desired =
      parameters_.control_mode == "constant_force"
          ? parameters_.configured_desired_force
          : 0.0;
  admittance_ = std::make_unique<AdmittanceController>(
      legacy_desired, parameters_.mass, parameters_.damping,
      parameters_.stiffness, parameters_.max_offset,
      parameters_.max_velocity, parameters_.absolute_force);
  force_follower_ = std::make_unique<ForceFollower>(
      legacy_desired, parameters_.stiffness, parameters_.max_offset,
      parameters_.max_velocity, parameters_.absolute_force,
      parameters_.force_velocity_mode, parameters_.force_deadband);

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
      "Ready: mode=%s interface=%s axes=[%s] constant_force=[%s] armed=%s output=%s",
      parameters_.control_mode.c_str(),
      parameters_.cartesian_mode ? "6d" : "legacy",
      enabledAxes(parameters_.admittance_axes).c_str(),
      enabledAxes(parameters_.constant_force_axes).c_str(),
      parameters_.armed ? "true" : "false",
      parameters_.reference_output_enabled ? "true" : "false");
  publishControlState(parameters_.armed ? "WAITING_FOR_DATA" : "DISABLED");
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
  if (!fault_latched_)
  {
    RCLCPP_ERROR(get_logger(), "Force-control fault latched: %s", reason.c_str());
  }
  fault_latched_ = true;
  fault_reason_ = reason;
  parameters_.armed = false;
  nominal_captured_ = false;
  force_processor_.reset();
  admittance_->reset(measured_force_);
  force_follower_->reset(measured_force_);
  cartesian_controller_->reset(measuredWrenchVector());
  publishControlState("FAULT_" + reason);
}

void WholeBodyForceControlNode::checkFaults(
    bool observation_timed_out, bool wrench_timed_out)
{
  if (parameters_.armed && observation_timed_out)
  {
    latchFault("OBSERVATION_TIMEOUT");
  }
  else if (parameters_.armed && wrench_timed_out)
  {
    latchFault("WRENCH_TIMEOUT");
  }
  else if (parameters_.armed && parameters_.enforce_single_target_owner &&
           foreignTargetPublisherPresent())
  {
    latchFault("TARGET_OWNER");
  }
}

void WholeBodyForceControlNode::captureNominalState(
    const Eigen::VectorXd &measured_state)
{
  nominal_state_ = measured_state;
  response_world_ =
      Eigen::AngleAxisd(nominal_state_[2], Eigen::Vector3d::UnitZ()) *
      parameters_.response_body;
  nominal_ee_ = kinematics_->framePosition(nominal_state_);
  nominal_ee_rotation_ = kinematics_->frameRotation(nominal_state_);
  admittance_->reset(0.0);
  force_follower_->reset(0.0);
  cartesian_controller_->reset(measuredWrenchVector());
  nominal_captured_ = true;
  RCLCPP_INFO(
      get_logger(), "Captured nominal state; accepting %s wrench",
      parameters_.cartesian_mode ? "6D" : "legacy scalar");
  publishControlState("ACTIVE");
}

void WholeBodyForceControlNode::updateCartesianReference(
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
  reference = kinematics_->correctedState6D(
      nominal_state_, correction, parameters_.base_share,
      parameters_.max_base_delta, parameters_.max_joint_delta);
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

void WholeBodyForceControlNode::updateLegacyReference(
    double dt,
    Vector6d &correction,
    Vector6d &filtered_wrench,
    Eigen::VectorXd &reference,
    double &primary_offset,
    double &primary_force)
{
  const bool force_follow = parameters_.control_mode == "force_follow";
  primary_offset = force_follow
      ? force_follower_->update(measured_force_, dt)
      : admittance_->update(measured_force_, dt);
  primary_force = force_follow
      ? force_follower_->measuredForce()
      : admittance_->measuredForce();
  reference = kinematics_->correctedState(
      nominal_state_, response_world_, primary_offset,
      parameters_.base_share, parameters_.max_base_delta,
      parameters_.max_joint_delta);
  const std::size_t index = parameters_.force_axis_index;
  correction[index] = primary_offset;
  filtered_wrench[index] = primary_force;
}

void WholeBodyForceControlNode::configureForceProcessor()
{
  ForceProcessorConfig config;
  config.tare_samples = parameters_.tare_samples;
  config.filter_alpha = parameters_.filter_alpha_6d;
  config.scale = parameters_.wrench_scale_6d;
  config.absolute_axes = parameters_.absolute_wrench_axes;
  config.hard_limit_enabled = true;
  config.hard_wrench_limit = parameters_.hard_wrench_limit;
  config.max_wrench_rate = parameters_.max_wrench_rate;
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

  // Data check.
  if (!observation_received_)
  {
    publishControlState(
        parameters_.armed ? "WAITING_FOR_OBSERVATION" : "DISABLED");
    return;
  }

  if (force_processor_.taring())
  {
    publishControlState("TARING");
    publishReference(observationStateLocked());
    return;
  }

  const Eigen::VectorXd measured_state = observationStateLocked();
  const Vector6d measured_wrench = measuredWrenchVector();
  const bool observation_timed_out =
      !observation_received_ ||
      std::chrono::duration<double>(wall_now - last_observation_).count() >
          parameters_.observation_timeout;
  const bool wrench_timed_out =
      !wrench_received_ ||
      std::chrono::duration<double>(wall_now - last_wrench_).count() >
          parameters_.force_timeout;

  // Initial startup is not a sensor-loss fault.  Wait for the first valid
  // sample; after a stream has started, any timeout is latched below.
  if (parameters_.armed && !wrench_received_)
  {
    publishControlState("WAITING_FOR_WRENCH");
    publishReference(measured_state);
    return;
  }

  // Safety checks.
  checkFaults(observation_timed_out, wrench_timed_out);
  if (!parameters_.armed)
  {
    if (!fault_latched_)
    {
      publishControlState("DISABLED");
    }
    publishReference(measured_state);
    return;
  }

  // Nominal-state capture.
  if (!nominal_captured_)
  {
    const double elapsed =
        std::chrono::duration<double>(wall_now - capture_requested_at_).count();
    if (elapsed < parameters_.capture_settle_time)
    {
      publishControlState("SETTLING");
      return;
    }
    captureNominalState(measured_state);
  }

  // Control calculation.
  Vector6d correction = Vector6d::Zero();
  Vector6d filtered_wrench = Vector6d::Zero();
  Eigen::VectorXd reference;
  double primary_offset = 0.0;
  double primary_force = 0.0;
  if (parameters_.cartesian_mode)
  {
    updateCartesianReference(
        measured_wrench, dt, correction, filtered_wrench, reference,
        primary_offset, primary_force);
  }
  else
  {
    updateLegacyReference(
        dt, correction, filtered_wrench, reference,
        primary_offset, primary_force);
  }

  // Publish reference and status.
  publishReference(reference);
  publishStatus(
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
