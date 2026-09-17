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
using wbmm::ros_interfaces::toEigenState;
using wbmm::ros_interfaces::toEigenWrench;
using wbmm::ros_interfaces::toMpcTargetTrajectories;
using wbmm::ros_interfaces::wholeBodyStateFromMpcObservation;
using wbmm::ros_interfaces::wrenchFromRos;

namespace
{

Eigen::Matrix3d rotationFromTransform(
    const geometry_msgs::msg::TransformStamped &transform)
{
  const auto &q = transform.transform.rotation;
  const Eigen::Quaterniond quaternion(q.w, q.x, q.y, q.z);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1.0e-12)
  {
    throw std::invalid_argument("TF rotation is not a valid quaternion");
  }
  return quaternion.normalized().toRotationMatrix();
}

Eigen::Vector3d translationFromTransform(
    const geometry_msgs::msg::TransformStamped &transform)
{
  const auto &t = transform.transform.translation;
  const Eigen::Vector3d translation(t.x, t.y, t.z);
  if (!translation.allFinite())
  {
    throw std::invalid_argument("TF translation is non-finite");
  }
  return translation;
}

}  // namespace

geometry_msgs::msg::TransformStamped
WholeBodyForceControlNode::lookupTransformWithFallback(
    const std::string &target_frame, const std::string &source_frame,
    const builtin_interfaces::msg::Time &stamp,
    const std::string &context)
{
  const auto timeout = rclcpp::Duration::from_seconds(
      parameters_.tf_lookup_timeout);
  const rclcpp::Time requested_time(stamp);
  try
  {
    return tf_buffer_->lookupTransform(
        target_frame, source_frame, requested_time, timeout);
  }
  catch (const tf2::TransformException &exception)
  {
    if (!parameters_.tf_fallback_to_latest)
    {
      throw;
    }
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "%s: TF %s <- %s at message stamp failed (%s); using latest TF",
        context.c_str(), target_frame.c_str(), source_frame.c_str(),
        exception.what());
    return tf_buffer_->lookupTransform(
        target_frame, source_frame, tf2::TimePointZero,
        tf2_ros::fromRclcpp(timeout));
  }
}

void WholeBodyForceControlNode::createRosInterfaces()
{
  const auto reliable = rclcpp::QoS(1).reliable();
  target_publisher_ = create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
      parameters_.target_topic, reliable);
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
  auto raw = wrenchFromRos(
      *message, wbmm::core::ClockDomain::kSystem, parameters_.sensor_frame);
  if (!raw.has_value())
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping malformed wrench message");
    return;
  }

  auto validation = wbmm::core::validate(*raw);
  if (!validation.ok)
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping invalid raw wrench: %s", validation.message.c_str());
    return;
  }

  // Raw wrench is measured in sensor_frame.  It is transformed to tcp_frame
  // with the full lever-arm term, then admittance is solved in tcp_frame.
  if (message->header.frame_id.empty() ||
      message->header.frame_id != parameters_.sensor_frame) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejecting wrench frame '%s'; expected sensor frame '%s'",
        message->header.frame_id.c_str(), parameters_.sensor_frame.c_str());
    if (parameters_.admittance_enabled) {
      requestFault("WRENCH_FRAME");
    }
    return;
  }

  geometry_msgs::msg::TransformStamped tf_tcp_sensor;
  try
  {
    tf_tcp_sensor = lookupTransformWithFallback(
        parameters_.tcp_frame, parameters_.sensor_frame,
        message->header.stamp, "sensor->tcp");
  }
  catch (const tf2::TransformException &exception)
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping wrench: TF lookup failed: %s", exception.what());
    if (parameters_.admittance_enabled) {
      requestFault("WRENCH_TF");
    }
    return;
  }
  catch (const std::exception &exception)
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping wrench: invalid TF: %s", exception.what());
    if (parameters_.admittance_enabled) {
      requestFault("WRENCH_TF");
    }
    return;
  }

  Eigen::Matrix3d tcp_rotation_sensor;
  Eigen::Vector3d sensor_origin_in_tcp;
  try
  {
    tcp_rotation_sensor = rotationFromTransform(tf_tcp_sensor);
    sensor_origin_in_tcp = translationFromTransform(tf_tcp_sensor);
  }
  catch (const std::exception &exception)
  {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping wrench: invalid TF data: %s", exception.what());
    if (parameters_.admittance_enabled) {
      requestFault("WRENCH_TF");
    }
    return;
  }

  const auto wall_now = std::chrono::steady_clock::now();
  const auto processed = force_processor_.process(
      *raw, tcp_rotation_sensor, sensor_origin_in_tcp);
  if (processed.taring) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Auto-tare in progress: %zu/%zu samples",
        processed.tare_samples_collected,
        force_processor_.tareSamplesRequired());
    return;
  }
  if (!processed.ok) {
    if (processed.hard_limit_exceeded) {
      requestFault("WRENCH_LIMIT");
    } else if (parameters_.admittance_enabled) {
      requestFault("WRENCH_INVALID");
    } else {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping wrench rejected by ForceProcessor");
    }
    return;
  }

  wbmm::core::Wrench wrench = processed.wrench;
  wrench.header.frame_id = parameters_.tcp_frame;
  validation = wbmm::core::validate(wrench);
  if (!validation.ok) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping invalid processed wrench: %s", validation.message.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  measured_wrench_core_ = wrench;
  wrench_received_ = true;
  last_wrench_ = wall_now;
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

void WholeBodyForceControlNode::publishState(const std::string &state)
{
  if (state == last_state_)
  {
    return;
  }
  last_state_ = state;
  std_msgs::msg::String message;
  message.data = state;
  state_publisher_->publish(message);
}

void WholeBodyForceControlNode::publishReference(
    const Eigen::VectorXd &reference)
{
  if (!parameters_.reference_output_enabled)
  {
    return;
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
