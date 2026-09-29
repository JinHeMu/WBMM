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
using wbmm::ros_interfaces::toEigenState;
using wbmm::ros_interfaces::toEigenWrench;
using wbmm::ros_interfaces::wholeBodyStateFromMpcObservation;
using wbmm::ros_interfaces::wrenchFromRos;

void WholeBodyForceControlNode::createRosInterfaces()
{
  const auto reliable = rclcpp::QoS(1).reliable();
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
        "Ignoring malformed observation; expected %zuD state in frame '%s'",
        robot_model_->stateDimension(), parameters_.state_frame.c_str());
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
  const auto publishers = get_publishers_info_by_topic(parameters_.ee_target_topic);
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
  std_msgs::msg::String message;
  message.data = state;
  state_publisher_->publish(message);
}

bool WholeBodyForceControlNode::publishEndEffectorReference(
    const wbmm::core::EndEffectorPose & target)
{
  if (!parameters_.reference_output_enabled)
  {
    return false;
  }

  if (target.header.frame_id != parameters_.state_frame)
  {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Refusing EE target in %s; expected %s",
        target.header.frame_id.c_str(), parameters_.state_frame.c_str());
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
    const auto pose = tcpPose(measured_state);
    const Eigen::Vector3d position(
        pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Quaterniond orientation(
        pose.orientation.w, pose.orientation.x, pose.orientation.y,
        pose.orientation.z);

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

}  // namespace whole_body_force_control
