#include "node.hpp"

#include "wbmm_ros_interfaces/wbmm_conversions.hpp"
#include "wbmm_ros_interfaces/wbmm_ros_conversions.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace whole_body_force_control
{
using wbmm::ros_interfaces::makeZeroWholeBodyInput;
using wbmm::ros_interfaces::toCoreState;
using wbmm::ros_interfaces::toCoreWrench;
using wbmm::ros_interfaces::toEigenState;
using wbmm::ros_interfaces::toEigenWrench;
using wbmm::ros_interfaces::toMpcTargetTrajectories;
using wbmm::ros_interfaces::wholeBodyStateFromMpcObservation;
using wbmm::ros_interfaces::wrenchFromRos;

void WholeBodyForceControlNode::createRosInterfaces()
{
  if (parameters_.cartesian_mode)
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

  const auto reliable = rclcpp::QoS(1).reliable();
  target_publisher_ = create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
      parameters_.target_topic, reliable);
  status_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      parameters_.status_topic, rclcpp::QoS(10));
  control_state_publisher_ = create_publisher<std_msgs::msg::String>(
      parameters_.control_state_topic,
      rclcpp::QoS(1).reliable().transient_local());
  observation_subscription_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
      parameters_.robot_name + "_mpc_observation",
      rclcpp::QoS(1).best_effort(),
      std::bind(
          &WholeBodyForceControlNode::observationCallback, this,
          std::placeholders::_1));
  wrench_subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      parameters_.wrench_topic,
      rclcpp::SensorDataQoS(),
      std::bind(
          &WholeBodyForceControlNode::wrenchCallback, this,
          std::placeholders::_1));
}

void WholeBodyForceControlNode::observationCallback(
    const ocs2_msgs::msg::MpcObservation::SharedPtr message)
{
  auto converted = wholeBodyStateFromMpcObservation(
      *message, robot_model_->jointNames(), parameters_.state_frame,
      wbmm::core::ClockDomain::kOcs2Mpc);
  if (!converted.has_value())
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring malformed observation; expected %dD state in frame '%s'",
        kinematics_->stateDimension(), parameters_.state_frame.c_str());
    return;
  }
  const auto structural = wbmm::core::validate(*converted);
  if (!structural.ok)
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring structurally invalid observation: %s",
        structural.message.c_str());
    return;
  }
  std::string reason;
  if (!robot_model_->validate(*converted, &reason))
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring observation rejected by RobotModel: %s", reason.c_str());
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  observation_state_ = std::move(*converted);
  observation_time_ = message->time;
  observation_received_ = true;
  last_observation_ = std::chrono::steady_clock::now();
}

void WholeBodyForceControlNode::wrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr message)
{
  if (parameters_.cartesian_mode && parameters_.require_wrench_frame &&
      message->header.frame_id.empty())
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping 6D wrench without frame_id; expected %s",
        parameters_.ee_frame.c_str());
    return;
  }

  // 旧标量模式允许空 frame；core 合同要求非空，因此用 ee_frame_ 作为
  // 纯标签 fallback，不做任何坐标变换（与旧行为一致）。
  auto converted = wrenchFromRos(
      *message, wbmm::core::ClockDomain::kSystem, parameters_.ee_frame);
  if (!converted.has_value())
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping malformed wrench message");
    return;
  }
  auto validation = wbmm::core::validate(*converted);
  if (!validation.ok)
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping invalid wrench: %s", validation.message.c_str());
    return;
  }

  if (parameters_.cartesian_mode)
  {
    const auto transformed = wrenchInComplianceFrame(*message);
    if (!transformed)
    {
      return;
    }
    *converted = toCoreWrench(*transformed, converted->header);
    converted->header.frame_id = parameters_.ee_frame;
    validation = wbmm::core::validate(*converted);
    if (!validation.ok)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping invalid transformed wrench: %s",
          validation.message.c_str());
      return;
    }
  }

  applyWrenchScaleAndAbsolute(*converted);
  const Vector6d wrench = toEigenWrench(*converted);
  double measured_force =
      wrench[parameters_.force_axis_index] * parameters_.force_scale;
  if (parameters_.absolute_force)
  {
    measured_force = std::abs(measured_force);
  }

  std::lock_guard<std::mutex> lock(mutex_);
  measured_wrench_core_ = *converted;
  measured_force_ = measured_force;
  wrench_received_ = true;
  last_wrench_ = std::chrono::steady_clock::now();
}

void WholeBodyForceControlNode::applyWrenchScaleAndAbsolute(
    wbmm::core::Wrench &wrench) const
{
  std::array<double *, 6> values{
      &wrench.force.x, &wrench.force.y, &wrench.force.z,
      &wrench.torque.x, &wrench.torque.y, &wrench.torque.z};
  for (std::size_t i = 0; i < 6; ++i)
  {
    *values[i] *= parameters_.wrench_scale_6d[i];
    if (parameters_.absolute_wrench_axes[i])
    {
      *values[i] = std::abs(*values[i]);
    }
  }
}

Vector6d WholeBodyForceControlNode::measuredWrenchVector() const
{
  return toEigenWrench(measured_wrench_core_);
}

std::optional<Vector6d> WholeBodyForceControlNode::wrenchInComplianceFrame(
    const geometry_msgs::msg::WrenchStamped &message)
{
  Vector6d source;
  source << message.wrench.force.x, message.wrench.force.y,
      message.wrench.force.z, message.wrench.torque.x,
      message.wrench.torque.y, message.wrench.torque.z;
  if (!parameters_.cartesian_mode)
  {
    return source;
  }
  if (message.header.frame_id.empty())
  {
    if (parameters_.require_wrench_frame)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping 6D wrench without frame_id; expected %s",
          parameters_.ee_frame.c_str());
      return std::nullopt;
    }
    return source;
  }
  if (message.header.frame_id == parameters_.ee_frame)
  {
    return source;
  }
  try
  {
    const auto transform = tf_buffer_->lookupTransform(
        parameters_.ee_frame, message.header.frame_id, tf2::TimePointZero);
    const auto &rotation_message = transform.transform.rotation;
    Eigen::Quaterniond quaternion(
        rotation_message.w, rotation_message.x,
        rotation_message.y, rotation_message.z);
    if (quaternion.norm() < 1.0e-9)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping 6D wrench: %s -> %s has invalid rotation",
          message.header.frame_id.c_str(), parameters_.ee_frame.c_str());
      return std::nullopt;
    }
    quaternion.normalize();
    const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
    const auto &translation_message = transform.transform.translation;
    const Eigen::Vector3d target_to_source(
        translation_message.x, translation_message.y, translation_message.z);
    return transformWrench(source, rotation, target_to_source);
  }
  catch (const tf2::TransformException &exception)
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping 6D wrench: cannot transform %s -> %s: %s",
        message.header.frame_id.c_str(), parameters_.ee_frame.c_str(),
        exception.what());
    return std::nullopt;
  }
}

Eigen::VectorXd WholeBodyForceControlNode::observationStateLocked() const
{
  return toEigenState(observation_state_);
}

bool WholeBodyForceControlNode::foreignTargetPublisherPresent() const
{
  const auto publishers = get_publishers_info_by_topic(parameters_.target_topic);
  for (const auto &publisher : publishers)
  {
    if (publisher.node_name() != get_name() ||
        publisher.node_namespace() != get_namespace())
    {
      return true;
    }
  }
  return false;
}

void WholeBodyForceControlNode::publishControlState(const std::string &state)
{
  if (state == last_control_state_)
  {
    return;
  }
  last_control_state_ = state;
  std_msgs::msg::String message;
  message.data = state;
  control_state_publisher_->publish(message);
}

void WholeBodyForceControlNode::publishReference(
    const Eigen::VectorXd &reference)
{
  if (!parameters_.reference_output_enabled)
  {
    return;
  }

  // 参考轨迹先在 wbmm_core 里构造并统一校验，再转换为 OCS2 消息。
  // 几何/时间语义与旧实现一致：hold 轨迹 + 相对起点 0.02s 的等间隔点。
  const int count = std::max(
      2, static_cast<int>(std::ceil(
             parameters_.reference_horizon / parameters_.reference_dt)) + 1);
  wbmm::core::WholeBodyTrajectory trajectory;
  trajectory.trajectory_id = "whole_body_force_control_hold";
  trajectory.environment_revision = 1;
  trajectory.collision_model_revision = 1;
  trajectory.points.reserve(static_cast<std::size_t>(count));

  for (int i = 0; i < count; ++i)
  {
    const double time_from_start =
        0.02 + static_cast<double>(i) * parameters_.reference_dt;
    const double stamp = observation_time_ + time_from_start;
    wbmm::core::Header header;
    header.frame_id = parameters_.state_frame;
    header.stamp = stamp;
    header.clock = wbmm::core::ClockDomain::kOcs2Mpc;

    auto state = toCoreState(reference, robot_model_->jointNames(), header);
    if (!state.has_value())
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Refusing to publish reference: state size does not match RobotModel");
      return;
    }
    wbmm::core::WholeBodyTrajectoryPoint point;
    point.time_from_start = time_from_start;
    point.state = *state;
    point.feedforward_input = makeZeroWholeBodyInput(
        robot_model_->jointNames(), stamp, wbmm::core::ClockDomain::kOcs2Mpc);
    point.phase = wbmm::core::ExecutionPhase::kExecution;
    trajectory.points.push_back(point);
  }

  const auto validation = wbmm::core::validate(trajectory);
  if (!validation.ok)
  {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Refusing to publish invalid reference trajectory: %s",
        validation.message.c_str());
    return;
  }
  target_publisher_->publish(
      toMpcTargetTrajectories(
          trajectory, observation_time_,
          static_cast<std::size_t>(parameters_.input_dimension)));
}

void WholeBodyForceControlNode::publishStatus(
    const Eigen::VectorXd &reference,
    const Eigen::VectorXd &measured_state,
    double primary_force,
    double primary_offset,
    const Vector6d &filtered_wrench,
    const Vector6d &correction)
{
  Eigen::Vector3d report_direction = response_world_;
  if (parameters_.cartesian_mode && correction.head<3>().norm() > 1.0e-12)
  {
    report_direction =
        (nominal_ee_rotation_ * correction.head<3>()).normalized();
  }
  const Eigen::Vector2d heading(
      std::cos(nominal_state_[2]), std::sin(nominal_state_[2]));
  const Eigen::Vector2d base_delta =
      reference.head<2>() - nominal_state_.head<2>();
  const double base_reference = base_delta.dot(report_direction.head<2>());
  const double ee_reference =
      (kinematics_->framePosition(reference) - nominal_ee_).dot(report_direction);
  const double measured_base =
      (measured_state.head<2>() - nominal_state_.head<2>())
          .dot(report_direction.head<2>());
  const double measured_ee =
      (kinematics_->framePosition(measured_state) - nominal_ee_)
          .dot(report_direction);
  const double lateral_base =
      (measured_state.head<2>() - nominal_state_.head<2>())
          .dot(Eigen::Vector2d(-heading.y(), heading.x()));
  const double max_joint_motion =
      (measured_state.tail(kinematics_->armDimension()) -
       nominal_state_.tail(kinematics_->armDimension()))
          .cwiseAbs()
          .maxCoeff();

  std_msgs::msg::Float64MultiArray status;
  status.data = {
      primary_force, primary_offset, base_reference,
      ee_reference - base_reference, measured_base, measured_ee,
      measured_ee - measured_base, max_joint_motion, lateral_base};
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    status.data.push_back(filtered_wrench[i]);
  }
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    status.data.push_back(correction[i]);
  }
  const Vector6d velocity =
      parameters_.cartesian_mode ? cartesian_controller_->velocity()
                                 : Vector6d::Zero();
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    status.data.push_back(velocity[i]);
  }
  for (std::size_t i = 0; i < 6; ++i)
  {
    status.data.push_back(parameters_.admittance_axes[i] ? 1.0 : 0.0);
  }
  for (std::size_t i = 0; i < 6; ++i)
  {
    status.data.push_back(parameters_.constant_force_axes[i] ? 1.0 : 0.0);
  }
  status.data.push_back(parameters_.armed ? 1.0 : 0.0);
  status.data.push_back(parameters_.reference_output_enabled ? 1.0 : 0.0);
  status.data.push_back(fault_latched_ ? 1.0 : 0.0);
  status_publisher_->publish(status);
}

}  // namespace whole_body_force_control
