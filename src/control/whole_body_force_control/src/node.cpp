#include "node.hpp"

#include <wbmm_robot_model/wbmm_robot_model.hpp>
#include <wbmm_ros_interfaces/wbmm_conversions.hpp>

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
WholeBodyForceControlNode::WholeBodyForceControlNode()
    : Node("whole_body_force_control")
{
  loadParameters();
  configured_admittance_enabled_ = parameters_.admittance_enabled;

  // 统一模型来源：urdfdom 读取 URDF，配置给出受控关节顺序。力控不需要
  // 碰撞几何，因此不要求 URDF 球或 base_collision_link。
  const auto robot_description =
    wbmm::robot_model::loadRobotDescription(parameters_.urdf_file);
  auto config =
    wbmm::robot_model::RobotModelConfig::defaultsFor(robot_description);
  config.state_base_frame = robot_description.root_link;
  robot_model_ = std::make_shared<PinocchioRobotModel>(
      wbmm::pinocchio::KinematicModel::create(
        std::make_shared<const wbmm::robot_model::RobotDescription>(
          robot_description),
        config));
  cartesian_controller_ = std::make_unique<CartesianComplianceController>(
      parameters_.admittance_axes, parameters_.mass, parameters_.damping,
      parameters_.stiffness, parameters_.max_velocity);

  createRosInterfaces();

  const auto start_time = std::chrono::steady_clock::now();
  last_update_ = start_time;
  last_control_time_ = now();
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
      parameters_.admittance_enabled ? "WAITING_FOR_OBSERVATION" : "DISABLED");
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
  hold_ee_target_valid_ = false;
  ee_correction_valid_ = false;
  last_ee_correction_.setZero();
  cartesian_controller_->reset(measuredWrenchVector());
  publishState("FAULT_" + reason);
  publishHoldReference();
}

void WholeBodyForceControlNode::resetForceControl()
{
  std::lock_guard<std::mutex> lock(mutex_);

  fault_latched_ = false;
  pending_fault_ = false;
  fault_reason_.clear();
  pending_fault_reason_.clear();
  nominal_captured_ = false;
  hold_ee_target_valid_ = false;
  ee_correction_valid_ = false;
  last_ee_correction_.setZero();

  parameters_.admittance_enabled = configured_admittance_enabled_;
  cartesian_controller_->reset(measuredWrenchVector());

  wrench_received_ = false;
  capture_requested_at_ = std::chrono::steady_clock::now();
  publishState(
      parameters_.admittance_enabled ? "WAITING_FOR_OBSERVATION" : "DISABLED");
}

void WholeBodyForceControlNode::publishHoldReference()
{
  if (observation_received_) {
    publishHoldEndEffectorReference();
  }
}

void WholeBodyForceControlNode::captureNominalState(
    const Eigen::VectorXd &measured_state)
{
  const auto pose = tcpPose(measured_state);
  nominal_tcp_ = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  nominal_tcp_rotation_ = Eigen::Quaterniond(
      pose.orientation.w, pose.orientation.x, pose.orientation.y,
      pose.orientation.z).normalized().toRotationMatrix();
  cartesian_controller_->reset(measuredWrenchVector());
  last_ee_correction_.setZero();
  ee_correction_valid_ = false;
  hold_ee_target_valid_ = false;
  nominal_captured_ = true;
  RCLCPP_INFO(
      get_logger(),
      "Captured nominal TCP axes in %s",
      parameters_.state_frame.c_str());
}

wbmm::core::Pose WholeBodyForceControlNode::tcpPose(
    const Eigen::VectorXd & state) const
{
  wbmm::core::Header header;
  header.frame_id = parameters_.state_frame;
  header.stamp = observation_time_;
  const auto core_state = wbmm::ros_interfaces::toCoreState(
      state, robot_model_->jointNames(), header);
  if (!core_state.has_value()) {
    throw std::invalid_argument("TCP pose requires a full robot state");
  }
  wbmm::core::Pose pose;
  if (!robot_model_->forwardKinematics(*core_state, parameters_.tcp_frame, pose)) {
    throw std::runtime_error("TCP forward kinematics failed");
  }
  return pose;
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

  // The published wrench follows the moving TCP axes. The integrated
  // correction uses fixed nominal TCP axes, so it must not rotate on its own.
  correction = rateLimitCorrection(
      correction, last_ee_correction_, dt,
      parameters_.max_ee_linear_velocity, parameters_.max_ee_angular_velocity);
  cartesian_controller_->clampOffset(correction);
  last_ee_correction_ = correction;
  ee_correction_valid_ = true;

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

void WholeBodyForceControlNode::update()
{
  const auto wall_now = std::chrono::steady_clock::now();
  const double wall_dt = std::chrono::duration<double>(wall_now - last_update_).count();
  last_update_ = wall_now;
  const auto control_now = now();
  const bool sim_clock = get_parameter("use_sim_time").as_bool();
  const double control_dt = (control_now - last_control_time_).seconds();
  last_control_time_ = control_now;
  const double dt = sim_clock ? std::clamp(control_dt, 0.0, 0.05) : wall_dt;
  std::lock_guard<std::mutex> lock(mutex_);
  if (sim_clock && control_dt < 0.0 && nominal_captured_) {
    latchFault("CLOCK_RESET");
  }

  if (pending_fault_)
  {
    const std::string reason = pending_fault_reason_;
    pending_fault_ = false;
    pending_fault_reason_.clear();
    latchFault(reason);
  }

  if (fault_latched_) {
    publishState("FAULT_" + fault_reason_);
    publishHoldReference();
    return;
  }

  if (!observation_received_)
  {
    publishState(
        parameters_.admittance_enabled ? "WAITING_FOR_OBSERVATION" : "DISABLED");
    return;
  }

  const bool observation_timed_out =
      std::chrono::duration<double>(wall_now - last_observation_).count() >
          parameters_.observation_timeout;

  if (parameters_.admittance_enabled && observation_timed_out)
  {
    latchFault("OBSERVATION_TIMEOUT");
    publishState("FAULT_OBSERVATION_TIMEOUT");
    publishHoldReference();
    return;
  }

  if (parameters_.admittance_enabled && !wrench_received_)
  {
    publishState(
        force_sensor_state_received_ ? "WAITING_FOR_WRENCH" :
        "WAITING_FOR_FORCE_SENSOR_STATE");
    publishHoldReference();
    return;
  }

  if (parameters_.admittance_enabled && !force_sensor_active_)
  {
    publishState(
        force_sensor_state_received_ ? force_sensor_state_ :
        "WAITING_FOR_FORCE_SENSOR_STATE");
    publishHoldReference();
    return;
  }

  const bool wrench_timed_out =
      parameters_.admittance_enabled && wrench_received_ &&
      std::chrono::duration<double>(wall_now - last_wrench_).count() >
          parameters_.force_timeout;

  if (wrench_timed_out)
  {
    latchFault("WRENCH_TIMEOUT");
    publishState("FAULT_WRENCH_TIMEOUT");
    publishHoldReference();
    return;
  }

  if (parameters_.admittance_enabled &&
      parameters_.enforce_single_target_owner &&
      foreignTargetPublisherPresent())
  {
    latchFault("TARGET_OWNER");
    publishState("FAULT_TARGET_OWNER");
    publishHoldReference();
    return;
  }

  if (!parameters_.admittance_enabled)
  {
    publishState(
        fault_latched_ ? "FAULT_" + fault_reason_ : "DISABLED");
    publishHoldReference();
    return;
  }

  // Keep wall-clock heartbeat/watchdogs alive while the simulator is paused.
  // Admittance motion advances only with the same /clock used by MPC/MRT.
  if (sim_clock && dt == 0.0 && nominal_captured_) {
    publishState("ACTIVE");
    return;
  }

  const Eigen::VectorXd measured_state = observationStateLocked();
  const auto measured_pose = tcpPose(measured_state);
  const Eigen::Matrix3d measured_rotation = Eigen::Quaterniond(
      measured_pose.orientation.w, measured_pose.orientation.x,
      measured_pose.orientation.y, measured_pose.orientation.z)
      .normalized().toRotationMatrix();

  if (!nominal_captured_)
  {
    const double elapsed =
        std::chrono::duration<double>(wall_now - capture_requested_at_).count();
    if (elapsed < parameters_.capture_settle_time)
    {
      publishState("SETTLING");
      publishHoldReference();
      return;
    }
    captureNominalState(measured_state);
  }

  // Re-express forces and moments at the *same current TCP origin* in the
  // fixed nominal axes. No moment-arm shift to the startup TCP is appropriate.
  const Vector6d measured_wrench = transformWrench(
      measuredWrenchVector(), nominal_tcp_rotation_.transpose() * measured_rotation,
      Eigen::Vector3d::Zero());

  if (ee_correction_valid_) {
    const Eigen::Vector3d expected_position =
        nominal_tcp_ + nominal_tcp_rotation_ * last_ee_correction_.head<3>();
    const Eigen::Vector3d measured_position(
        measured_pose.position.x, measured_pose.position.y, measured_pose.position.z);
    wbmm::core::Pose nominal_pose = measured_pose;
    const Eigen::Quaterniond nominal_q(nominal_tcp_rotation_);
    nominal_pose.orientation.w = nominal_q.w();
    nominal_pose.orientation.x = nominal_q.x();
    nominal_pose.orientation.y = nominal_q.y();
    nominal_pose.orientation.z = nominal_q.z();
    const auto last_target = makeEndEffectorPoseTarget(
        nominal_pose, last_ee_correction_, parameters_.state_frame, observation_time_);
    const Eigen::Matrix3d target_rotation = Eigen::Quaterniond(
        last_target.orientation.w, last_target.orientation.x,
        last_target.orientation.y, last_target.orientation.z).toRotationMatrix();
    const double angular_error = Eigen::AngleAxisd(
        target_rotation.transpose() * measured_rotation).angle();
    const double linear_error = (expected_position - measured_position).norm();
    if ((parameters_.max_tracking_error_m > 0.0 &&
         linear_error > parameters_.max_tracking_error_m) ||
        (parameters_.max_tracking_error_rad > 0.0 &&
         angular_error > parameters_.max_tracking_error_rad)) {
      RCLCPP_ERROR(get_logger(),
                   "TCP tracking error: %.4f m (limit %.4f), %.4f rad (limit %.4f)",
                   linear_error, parameters_.max_tracking_error_m,
                   angular_error, parameters_.max_tracking_error_rad);
      latchFault("TRACKING_ERROR");
      return;
    }
  }

  Vector6d correction = Vector6d::Zero();
  Vector6d filtered_wrench = Vector6d::Zero();
  double primary_offset = 0.0;
  double primary_force = 0.0;

  wbmm::core::EndEffectorPose target;
  updateEndEffectorReference(
      measured_wrench, dt, correction, filtered_wrench, target,
      primary_offset, primary_force);
  const bool published = publishEndEffectorReference(target);
  publishEndEffectorCorrection(
      target, measured_state, primary_force, primary_offset,
      filtered_wrench, correction);
  if (published || !parameters_.reference_output_enabled) {
    publishState("ACTIVE");
  } else {
    pending_fault_ = true;
    pending_fault_reason_ = "REFERENCE_INVALID";
    publishState("FAULT_REFERENCE_INVALID");
  }
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
