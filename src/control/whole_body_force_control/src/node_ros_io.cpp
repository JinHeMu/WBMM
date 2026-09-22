#include "node.hpp"

#include "wbmm_ros_interfaces/wbmm_conversions.hpp"
#include "wbmm_ros_interfaces/wbmm_ros_conversions.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace whole_body_force_control
{
using wbmm::ros_interfaces::makeZeroWholeBodyInput;
using wbmm::ros_interfaces::toCoreState;
using wbmm::ros_interfaces::toEigenState;
using wbmm::ros_interfaces::toEigenWrench;
using wbmm::ros_interfaces::toMpcTargetTrajectories;
using wbmm::ros_interfaces::wholeBodyStateFromMpcObservation;
using wbmm::ros_interfaces::wrenchFromRos;

void WholeBodyForceControlNode::createRosInterfaces()
{
  const auto reliable = rclcpp::QoS(1).reliable();
  target_publisher_ = create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
      parameters_.target_topic, reliable);
  ee_target_publisher_ =
      create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
      parameters_.ee_target_topic, reliable);
  correction_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      parameters_.correction_topic, rclcpp::QoS(10));
  state_publisher_ = create_publisher<std_msgs::msg::String>(
      parameters_.state_topic,
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
  force_sensor_state_subscription_ =
      create_subscription<std_msgs::msg::String>(
      parameters_.force_sensor_state_topic,
      rclcpp::QoS(1).reliable().transient_local(),
      std::bind(
          &WholeBodyForceControlNode::forceSensorStateCallback, this,
          std::placeholders::_1));
  reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/whole_body_force_control/reset",
      [this](
          const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        resetForceControl();
        response->success = true;
        response->message = "Force control reset; waiting for fresh data.";
      });
}

void WholeBodyForceControlNode::observationCallback(
    const ocs2_msgs::msg::MpcObservation::SharedPtr message)
{
  auto converted = wholeBodyStateFromMpcObservation(
      *message, robot_model_->jointNames(), parameters_.state_frame);
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
  if (message->header.frame_id != parameters_.tcp_frame)
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping processed wrench in frame '%s'; expected '%s'",
        message->header.frame_id.c_str(), parameters_.tcp_frame.c_str());
    requestFault("WRENCH_FRAME");
    return;
  }

  auto wrench = wrenchFromRos(*message, parameters_.tcp_frame);
  if (!wrench.has_value())
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping malformed processed wrench message");
    requestFault("WRENCH_INVALID");
    return;
  }

  const auto validation = wbmm::core::validate(*wrench);
  if (!validation.ok)
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping invalid processed wrench: %s", validation.message.c_str());
    requestFault("WRENCH_INVALID");
    return;
  }

  const auto wall_now = std::chrono::steady_clock::now();

  std::lock_guard<std::mutex> lock(mutex_);
  measured_wrench_core_ = *wrench;
  wrench_received_ = true;
  last_wrench_ = wall_now;
}

void WholeBodyForceControlNode::forceSensorStateCallback(
    const std_msgs::msg::String::SharedPtr message)
{
  std::lock_guard<std::mutex> lock(mutex_);
  force_sensor_state_ = message->data;
  force_sensor_state_received_ = true;
  force_sensor_active_ = force_sensor_state_ == "ACTIVE";

  constexpr char kFaultPrefix[] = "FAULT_";
  if (!fault_latched_ && !pending_fault_ &&
      force_sensor_state_.rfind(kFaultPrefix, 0) == 0)
  {
    pending_fault_ = true;
    pending_fault_reason_ = force_sensor_state_.substr(
        sizeof(kFaultPrefix) - 1);
  }
}

Vector6d WholeBodyForceControlNode::measuredWrenchVector() const
{
  return toEigenWrench(measured_wrench_core_);
}

Eigen::VectorXd WholeBodyForceControlNode::observationStateLocked() const
{
  return toEigenState(observation_state_);
}

bool WholeBodyForceControlNode::foreignTargetPublisherPresent() const
{
  const std::string & topic =
      parameters_.output_mode == ReferenceOutputMode::kEndEffectorPose
      ? parameters_.ee_target_topic
      : parameters_.target_topic;
  const auto publishers = get_publishers_info_by_topic(topic);
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

void WholeBodyForceControlNode::publishState(const std::string &state)
{
  last_state_ = state;
  std_msgs::msg::String message;
  message.data = state;
  state_publisher_->publish(message);
}

bool WholeBodyForceControlNode::publishReference(
    const Eigen::VectorXd &reference)
{
  if (!parameters_.reference_output_enabled)
  {
    return false;
  }

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

    auto state = toCoreState(reference, robot_model_->jointNames(), header);
    if (!state.has_value())
    {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Refusing to publish reference: state size does not match RobotModel");
      return false;
    }
    wbmm::core::WholeBodyTrajectoryPoint point;
    point.time_from_start = time_from_start;
    point.state = *state;
    point.feedforward_input = makeZeroWholeBodyInput(
        robot_model_->jointNames(), stamp);
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
    return false;
  }
  target_publisher_->publish(
      toMpcTargetTrajectories(
          trajectory, observation_time_,
          static_cast<std::size_t>(parameters_.input_dimension)));
  return true;
}

bool WholeBodyForceControlNode::publishEndEffectorReference(
    const wbmm::core::EndEffectorPose & target)
{
  if (!parameters_.reference_output_enabled)
  {
    return false;
  }

  const auto validation = wbmm::core::validate(target);
  if (!validation.ok)
  {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Refusing to publish invalid end-effector target: %s",
        validation.message.c_str());
    return false;
  }

  ee_target_publisher_->publish(
      wbmm::ros_interfaces::toMpcTargetTrajectories(
          target, observation_time_,
          static_cast<std::size_t>(parameters_.input_dimension)));
  return true;
}

void WholeBodyForceControlNode::publishHoldEndEffectorReference()
{
  if (!observation_received_)
  {
    return;
  }

  if (!hold_ee_target_valid_)
  {
    const Eigen::VectorXd measured_state = observationStateLocked();
    const Eigen::Vector3d position = kinematics_->framePosition(measured_state);
    const Eigen::Matrix3d rotation = kinematics_->frameRotation(measured_state);
    const Eigen::Quaterniond orientation(rotation);

    hold_ee_target_.header.frame_id = parameters_.state_frame;
    hold_ee_target_.position.x = position.x();
    hold_ee_target_.position.y = position.y();
    hold_ee_target_.position.z = position.z();
    hold_ee_target_.orientation.w = orientation.w();
    hold_ee_target_.orientation.x = orientation.x();
    hold_ee_target_.orientation.y = orientation.y();
    hold_ee_target_.orientation.z = orientation.z();
    hold_ee_target_valid_ = true;

    RCLCPP_INFO(
        get_logger(),
        "Latched hold EE target: pos=(%.3f, %.3f, %.3f)",
        position.x(), position.y(), position.z());
  }

  // The pose is latched once; only the stamp follows the latest observation.
  hold_ee_target_.header.stamp = observation_time_;

  last_ee_correction_.setZero();
  ee_correction_valid_ = false;
  publishEndEffectorReference(hold_ee_target_);
}

void WholeBodyForceControlNode::publishEndEffectorCorrection(
    const wbmm::core::EndEffectorPose & target,
    const Eigen::VectorXd & /*measured_state*/,
    double primary_force,
    double primary_offset,
    const Vector6d & filtered_wrench,
    const Vector6d & correction)
{
  std_msgs::msg::Float64MultiArray correction_msg;
  correction_msg.data = {
      primary_force, primary_offset,
      target.position.x, target.position.y, target.position.z,
      target.orientation.w, target.orientation.x,
      target.orientation.y, target.orientation.z};
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    correction_msg.data.push_back(filtered_wrench[i]);
  }
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    correction_msg.data.push_back(correction[i]);
  }
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    correction_msg.data.push_back(cartesian_controller_->velocity()[i]);
  }
  for (std::size_t i = 0; i < 6; ++i)
  {
    correction_msg.data.push_back(parameters_.admittance_axes[i] ? 1.0 : 0.0);
  }
  correction_msg.data.push_back(parameters_.admittance_enabled ? 1.0 : 0.0);
  correction_msg.data.push_back(parameters_.reference_output_enabled ? 1.0 : 0.0);
  correction_msg.data.push_back(fault_latched_ ? 1.0 : 0.0);
  correction_publisher_->publish(correction_msg);
}

void WholeBodyForceControlNode::publishCorrection(
    const Eigen::VectorXd &reference,
    const Eigen::VectorXd &measured_state,
    double primary_force,
    double primary_offset,
    const Vector6d &filtered_wrench,
    const Vector6d &correction)
{
  Eigen::Vector3d report_direction_tcp = filtered_wrench.head<3>();
  if (report_direction_tcp.norm() < 1.0e-9)
  {
    report_direction_tcp = correction.head<3>();
  }
  if (report_direction_tcp.norm() < 1.0e-9)
  {
    report_direction_tcp = Eigen::Vector3d::UnitX();
    for (std::size_t i = 0; i < 3; ++i)
    {
      if (parameters_.admittance_axes[i])
      {
        report_direction_tcp = Eigen::Vector3d::Unit(
            static_cast<Eigen::Index>(i));
        break;
      }
    }
  }
  report_direction_tcp.normalize();
  const Eigen::Vector3d report_direction =
      (nominal_tcp_rotation_ * report_direction_tcp).normalized();

  const Eigen::Vector2d heading(
      std::cos(nominal_state_[2]), std::sin(nominal_state_[2]));
  const Eigen::Vector2d base_delta =
      reference.head<2>() - nominal_state_.head<2>();
  const double base_reference = base_delta.dot(report_direction.head<2>());
  const double ee_reference =
      (kinematics_->framePosition(reference) - nominal_tcp_).dot(report_direction);
  const double measured_base =
      (measured_state.head<2>() - nominal_state_.head<2>())
          .dot(report_direction.head<2>());
  const double measured_ee =
      (kinematics_->framePosition(measured_state) - nominal_tcp_)
          .dot(report_direction);
  const double lateral_base =
      (measured_state.head<2>() - nominal_state_.head<2>())
          .dot(Eigen::Vector2d(-heading.y(), heading.x()));
  const double max_joint_motion =
      (measured_state.tail(kinematics_->armDimension()) -
       nominal_state_.tail(kinematics_->armDimension()))
          .cwiseAbs()
          .maxCoeff();

  std_msgs::msg::Float64MultiArray correction_msg;
  correction_msg.data = {
      primary_force, primary_offset, base_reference,
      ee_reference - base_reference, measured_base, measured_ee,
      measured_ee - measured_base, max_joint_motion, lateral_base};
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    correction_msg.data.push_back(filtered_wrench[i]);
  }
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    correction_msg.data.push_back(correction[i]);
  }
  for (Eigen::Index i = 0; i < 6; ++i)
  {
    correction_msg.data.push_back(cartesian_controller_->velocity()[i]);
  }
  for (std::size_t i = 0; i < 6; ++i)
  {
    correction_msg.data.push_back(parameters_.admittance_axes[i] ? 1.0 : 0.0);
  }
  correction_msg.data.push_back(parameters_.admittance_enabled ? 1.0 : 0.0);
  correction_msg.data.push_back(parameters_.reference_output_enabled ? 1.0 : 0.0);
  correction_msg.data.push_back(fault_latched_ ? 1.0 : 0.0);
  correction_publisher_->publish(correction_msg);
}

}  // namespace whole_body_force_control
