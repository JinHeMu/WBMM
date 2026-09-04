#include "whole_body_force_control/controllers.hpp"
#include "whole_body_force_control/whole_body_kinematics.hpp"

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ocs2_msgs/msg/mpc_input.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_state.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace whole_body_force_control
{
namespace
{
constexpr std::array<const char *, 6> kAxisNames{
  "fx", "fy", "fz", "tx", "ty", "tz"};

std::vector<float> toFloatVector(const Eigen::VectorXd & value)
{
  std::vector<float> result(static_cast<std::size_t>(value.size()));
  for (Eigen::Index i = 0; i < value.size(); ++i) {
    result[static_cast<std::size_t>(i)] = static_cast<float>(value[i]);
  }
  return result;
}

std::size_t axisIndex(const std::string & axis)
{
  if (axis == "fx" || axis == "x") {return 0;}
  if (axis == "fy" || axis == "y") {return 1;}
  if (axis == "fz" || axis == "z") {return 2;}
  if (axis == "tx" || axis == "rx" || axis == "mx") {return 3;}
  if (axis == "ty" || axis == "ry" || axis == "my") {return 4;}
  if (axis == "tz" || axis == "rz" || axis == "mz") {return 5;}
  throw std::runtime_error(
          "Unknown compliance axis '" + axis +
          "'; use fx, fy, fz, tx, ty, or tz");
}

AxisMask6d parseAxisMask(
  const std::vector<std::string> & names, const std::string & legacy_axis)
{
  AxisMask6d mask{};
  if (names.empty() || (names.size() == 1 && names.front() == "none")) {
    return mask;
  }
  if (names.size() == 1 && names.front() == "legacy") {
    mask[axisIndex(legacy_axis)] = true;
    return mask;
  }
  for (const auto & name : names) {
    if (name == "legacy" || name == "none") {
      throw std::runtime_error("legacy/none cannot be combined with explicit axes");
    }
    mask[axisIndex(name)] = true;
  }
  return mask;
}

Vector6d vector6Parameter(
  rclcpp::Node & node, const std::string & name,
  const Vector6d & defaults)
{
  const auto values = node.declare_parameter<std::vector<double>>(
    name, std::vector<double>{});
  if (values.empty()) {
    return defaults;
  }
  if (values.size() != 6) {
    throw std::runtime_error(name + " must contain exactly 6 values");
  }
  Vector6d result;
  for (std::size_t i = 0; i < 6; ++i) {
    result[i] = values[i];
  }
  if (!result.allFinite()) {
    throw std::runtime_error(name + " must contain only finite values");
  }
  return result;
}

std::string enabledAxes(const AxisMask6d & mask)
{
  std::ostringstream stream;
  bool first = true;
  for (std::size_t i = 0; i < 6; ++i) {
    if (!mask[i]) {continue;}
    if (!first) {stream << ',';}
    stream << kAxisNames[i];
    first = false;
  }
  return first ? "none" : stream.str();
}
}  // namespace

// Reference-side compliance node.  The default "legacy" axis selection keeps
// the previous scalar force_axis + response_body behavior byte-for-byte at the
// public interface.  Explicit admittance_axes switches to signed 6D wrench and
// full-pose IK in the nominal end-effector frame.
class WholeBodyForceControlNode final : public rclcpp::Node
{
public:
  WholeBodyForceControlNode()
  : Node("whole_body_force_control")
  {
    const auto urdf_file = declare_parameter<std::string>("urdf_file", "");
    ee_frame_ = declare_parameter<std::string>("ee_frame", "tool0");
    if (urdf_file.empty()) {
      throw std::runtime_error("urdf_file is required");
    }
    kinematics_ = std::make_unique<WholeBodyKinematics>(urdf_file, ee_frame_);

    robot_name_ = declare_parameter<std::string>("robot_name", "mobile_manipulator");
    control_mode_ = declare_parameter<std::string>("control_mode", "force_follow");
    if (control_mode_ != "admittance" && control_mode_ != "constant_force" &&
        control_mode_ != "force_follow")
    {
      throw std::runtime_error(
              "control_mode must be admittance, constant_force, or force_follow");
    }

    // Deprecated scalar compatibility parameters.  They remain the default
    // until admittance_axes is explicitly configured.
    force_axis_ = declare_parameter<std::string>("force_axis", "x");
    if (force_axis_ != "x" && force_axis_ != "y" && force_axis_ != "z") {
      throw std::runtime_error("force_axis must be x, y, or z");
    }
    absolute_force_ = declare_parameter<bool>("absolute_force", false);
    const double configured_desired_force =
      declare_parameter<double>("desired_force", 12.0);
    const double mass = declare_parameter<double>("mass", 3.0);
    const double damping = declare_parameter<double>("damping", 45.0);
    const double stiffness = declare_parameter<double>("stiffness", 150.0);
    const double max_offset = declare_parameter<double>("max_offset", 0.08);
    const double max_velocity = declare_parameter<double>("max_velocity", 0.035);
    const double filter_alpha = declare_parameter<double>("filter_alpha", 0.25);
    force_velocity_mode_ = declare_parameter<bool>(
      "force_velocity_mode", false);
    force_deadband_ = declare_parameter<double>("force_deadband", 0.0);

    loop_rate_ = declare_parameter<double>("loop_rate", 50.0);
    reference_horizon_ = declare_parameter<double>("reference_horizon", 1.0);
    reference_dt_ = declare_parameter<double>("reference_dt", 0.1);
    input_dimension_ = declare_parameter<int>("input_dimension", 8);
    base_share_ = declare_parameter<double>("base_share", 0.4);
    max_base_delta_ = declare_parameter<double>("max_base_delta", 0.04);
    max_joint_delta_ = declare_parameter<double>("max_joint_delta", 0.25);
    force_timeout_ = declare_parameter<double>("force_timeout", 0.25);
    observation_timeout_ = declare_parameter<double>("observation_timeout", 0.25);
    capture_settle_time_ = declare_parameter<double>("capture_settle_time", 1.0);
    armed_ = declare_parameter<bool>("armed", false);
    reference_output_enabled_ =
      declare_parameter<bool>("reference_output_enabled", false);
    enforce_single_target_owner_ =
      declare_parameter<bool>("enforce_single_target_owner", true);
    force_scale_ = declare_parameter<double>("force_scale", 1.0);
    if (!std::isfinite(force_scale_) || std::abs(force_scale_) < 1.0e-9) {
      throw std::runtime_error("force_scale must be finite and non-zero");
    }

    const Eigen::Vector3d response_body(
      declare_parameter<double>("response_body_x", 1.0),
      declare_parameter<double>("response_body_y", 0.0),
      declare_parameter<double>("response_body_z", 0.0));
    if (!response_body.allFinite() || response_body.norm() < 1.0e-9) {
      throw std::runtime_error("response_body direction must be finite and non-zero");
    }
    response_body_ = response_body.normalized();

    const auto admittance_axis_names =
      declare_parameter<std::vector<std::string>>("admittance_axes", {"legacy"});
    const auto constant_force_axis_names =
      declare_parameter<std::vector<std::string>>("constant_force_axes", {"legacy"});
    const auto absolute_wrench_axis_names =
      declare_parameter<std::vector<std::string>>("absolute_wrench_axes", {"legacy"});
    cartesian_mode_ = !(
      admittance_axis_names.size() == 1 &&
      admittance_axis_names.front() == "legacy");
    require_wrench_frame_ = declare_parameter<bool>(
      "require_wrench_frame", cartesian_mode_);
    admittance_axes_ = parseAxisMask(admittance_axis_names, force_axis_);
    constant_force_axes_ = parseAxisMask(constant_force_axis_names, force_axis_);
    absolute_wrench_axes_ = parseAxisMask(absolute_wrench_axis_names, force_axis_);
    if (control_mode_ != "constant_force") {
      constant_force_axes_.fill(false);
    }
    for (std::size_t i = 0; i < 6; ++i) {
      if (constant_force_axes_[i] && !admittance_axes_[i]) {
        throw std::runtime_error("constant_force_axes must be a subset of admittance_axes");
      }
    }

    Vector6d desired_defaults = Vector6d::Zero();
    for (std::size_t i = 0; i < 3; ++i) {
      if (constant_force_axes_[i]) {
        desired_defaults[i] = configured_desired_force;
      }
    }
    // Torque targets deliberately default to zero.  Enabling tx/ty/tz
    // compliance therefore does not silently enable torque regulation.
    const Vector6d desired_wrench = vector6Parameter(
      *this, "desired_wrench", desired_defaults);
    Vector6d mass_defaults;
    mass_defaults << mass, mass, mass, 0.30, 0.30, 0.30;
    Vector6d damping_defaults;
    damping_defaults << damping, damping, damping, 4.5, 4.5, 4.5;
    Vector6d stiffness_defaults;
    stiffness_defaults << stiffness, stiffness, stiffness, 15.0, 15.0, 15.0;
    Vector6d max_offset_defaults;
    max_offset_defaults << max_offset, max_offset, max_offset, 0.15, 0.15, 0.15;
    Vector6d max_velocity_defaults;
    max_velocity_defaults <<
      max_velocity, max_velocity, max_velocity, 0.15, 0.15, 0.15;
    const Vector6d alpha_defaults = Vector6d::Constant(filter_alpha);
    const Vector6d wrench_scale_defaults = Vector6d::Ones();
    wrench_scale_6d_ = vector6Parameter(
      *this, "wrench_scale_6d", wrench_scale_defaults);
    Vector6d hard_wrench_defaults;
    hard_wrench_defaults << 40.0, 40.0, 40.0, 5.0, 5.0, 5.0;
    hard_wrench_limit_ = vector6Parameter(
      *this, "hard_wrench_limit", hard_wrench_defaults);
    if ((hard_wrench_limit_.array() <= 0.0).any()) {
      throw std::runtime_error("hard_wrench_limit values must be positive");
    }

    cartesian_controller_ = std::make_unique<CartesianComplianceController>(
      admittance_axes_, constant_force_axes_, desired_wrench,
      vector6Parameter(*this, "mass_6d", mass_defaults),
      vector6Parameter(*this, "damping_6d", damping_defaults),
      vector6Parameter(*this, "stiffness_6d", stiffness_defaults),
      vector6Parameter(*this, "max_offset_6d", max_offset_defaults),
      vector6Parameter(*this, "max_velocity_6d", max_velocity_defaults),
      vector6Parameter(*this, "filter_alpha_6d", alpha_defaults),
      control_mode_ == "force_follow");

    const double legacy_desired = control_mode_ == "constant_force" ?
      configured_desired_force : 0.0;
    admittance_ = std::make_unique<AdmittanceController>(
      legacy_desired, mass, damping, stiffness, max_offset, max_velocity,
      filter_alpha, absolute_force_);
    force_follower_ = std::make_unique<ForceFollower>(
      legacy_desired, stiffness, max_offset, max_velocity, filter_alpha,
      absolute_force_, force_velocity_mode_, force_deadband_);

    if (cartesian_mode_) {
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    const auto reliable = rclcpp::QoS(1).reliable();
    target_topic_ = robot_name_ + "_mpc_target";
    target_publisher_ = create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
      target_topic_, reliable);
    status_publisher_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      declare_parameter<std::string>(
        "status_topic", "/whole_body_force_control/status"),
      rclcpp::QoS(10));
    control_state_publisher_ = create_publisher<std_msgs::msg::String>(
      declare_parameter<std::string>(
        "control_state_topic", "/whole_body_force_control/control_state"),
      rclcpp::QoS(1).reliable().transient_local());
    observation_subscription_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
      robot_name_ + "_mpc_observation", rclcpp::QoS(1).best_effort(),
      std::bind(
        &WholeBodyForceControlNode::observationCallback, this,
        std::placeholders::_1));
    wrench_subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      declare_parameter<std::string>(
        "wrench_topic", "/whole_body_force_control/wrench"),
      rclcpp::SensorDataQoS(),
      std::bind(
        &WholeBodyForceControlNode::wrenchCallback, this,
        std::placeholders::_1));
    enable_service_ = create_service<std_srvs::srv::SetBool>(
      declare_parameter<std::string>(
        "enable_service", "/whole_body_force_control/enable"),
      std::bind(
        &WholeBodyForceControlNode::enableCallback, this,
        std::placeholders::_1, std::placeholders::_2));

    start_time_ = std::chrono::steady_clock::now();
    last_update_ = start_time_;
    last_wrench_ = start_time_;
    last_observation_ = start_time_;
    capture_requested_at_ = start_time_;
    const auto period = std::chrono::duration<double>(
      1.0 / std::max(1.0, loop_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&WholeBodyForceControlNode::update, this));
    RCLCPP_INFO(
      get_logger(),
      "Ready: mode=%s interface=%s axes=[%s] constant_force=[%s] armed=%s output=%s",
      control_mode_.c_str(), cartesian_mode_ ? "6d" : "legacy",
      enabledAxes(admittance_axes_).c_str(),
      enabledAxes(constant_force_axes_).c_str(),
      armed_ ? "true" : "false",
      reference_output_enabled_ ? "true" : "false");
    publishControlState(armed_ ? "WAITING_FOR_DATA" : "DISABLED");
  }

private:
  void observationCallback(const ocs2_msgs::msg::MpcObservation::SharedPtr message)
  {
    if (message->state.value.size() !=
        static_cast<std::size_t>(kinematics_->stateDimension()))
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring %zuD observation; expected %dD",
        message->state.value.size(), kinematics_->stateDimension());
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    observation_ = message;
    observation_received_ = true;
    last_observation_ = std::chrono::steady_clock::now();
  }

  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr message)
  {
    const auto transformed = wrenchInComplianceFrame(*message);
    if (!transformed) {
      return;
    }
    Vector6d wrench = *transformed;
    if (!wrench.allFinite()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping non-finite wrench message");
      return;
    }
    for (std::size_t i = 0; i < 6; ++i) {
      wrench[i] *= wrench_scale_6d_[i];
      if (absolute_wrench_axes_[i]) {
        wrench[i] = std::abs(wrench[i]);
      }
    }
    std::lock_guard<std::mutex> lock(mutex_);
    measured_wrench_ = wrench;
    measured_force_ = wrench[axisIndex(force_axis_)] * force_scale_;
    if (absolute_force_) {
      measured_force_ = std::abs(measured_force_);
    }
    wrench_received_ = true;
    last_wrench_ = std::chrono::steady_clock::now();
  }

  void enableCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!request->data) {
      armed_ = false;
      nominal_captured_ = false;
      fault_latched_ = false;
      fault_reason_.clear();
      publishControlState("DISABLED");
      response->success = true;
      response->message = "force-follow disabled; current observation is held";
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    if (!observation_received_ ||
        std::chrono::duration<double>(now - last_observation_).count() >
        observation_timeout_)
    {
      response->success = false;
      response->message = "cannot enable: OCS2 observation is missing or stale";
      return;
    }
    if (!wrench_received_ ||
        std::chrono::duration<double>(now - last_wrench_).count() > force_timeout_)
    {
      response->success = false;
      response->message = "cannot enable: wrench is missing or stale";
      return;
    }
    if (enforce_single_target_owner_ && foreignTargetPublisherPresent()) {
      response->success = false;
      response->message = "cannot enable: another OCS2 target publisher exists";
      return;
    }

    armed_ = true;
    nominal_captured_ = false;
    fault_latched_ = false;
    fault_reason_.clear();
    capture_requested_at_ = now;
    admittance_->reset(measured_force_);
    force_follower_->reset(measured_force_);
    cartesian_controller_->reset(measured_wrench_);
    publishControlState("SETTLING");
    response->success = true;
    response->message = reference_output_enabled_ ?
      "force-follow armed; reference output enabled" :
      "force-follow armed in shadow mode; reference output disabled";
  }

  std::optional<Vector6d> wrenchInComplianceFrame(
    const geometry_msgs::msg::WrenchStamped & message)
  {
    Vector6d source;
    source << message.wrench.force.x, message.wrench.force.y,
      message.wrench.force.z, message.wrench.torque.x,
      message.wrench.torque.y, message.wrench.torque.z;
    if (!cartesian_mode_) {
      return source;
    }
    if (message.header.frame_id.empty()) {
      if (require_wrench_frame_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping 6D wrench without frame_id; expected %s",
          ee_frame_.c_str());
        return std::nullopt;
      }
      return source;
    }
    if (message.header.frame_id == ee_frame_) {
      return source;
    }
    try {
      const auto transform = tf_buffer_->lookupTransform(
        ee_frame_, message.header.frame_id, tf2::TimePointZero);
      const auto & rotation_message = transform.transform.rotation;
      Eigen::Quaterniond quaternion(
        rotation_message.w, rotation_message.x,
        rotation_message.y, rotation_message.z);
      if (quaternion.norm() < 1.0e-9) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Dropping 6D wrench: %s -> %s has invalid rotation",
          message.header.frame_id.c_str(), ee_frame_.c_str());
        return std::nullopt;
      }
      quaternion.normalize();
      const Eigen::Matrix3d rotation = quaternion.toRotationMatrix();
      const auto & translation_message = transform.transform.translation;
      const Eigen::Vector3d target_to_source(
        translation_message.x, translation_message.y, translation_message.z);
      return transformWrench(source, rotation, target_to_source);
    } catch (const tf2::TransformException & exception) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping 6D wrench: cannot transform %s -> %s: %s",
        message.header.frame_id.c_str(), ee_frame_.c_str(), exception.what());
      return std::nullopt;
    }
  }

  Eigen::VectorXd observationStateLocked() const
  {
    Eigen::VectorXd state(kinematics_->stateDimension());
    for (Eigen::Index i = 0; i < state.size(); ++i) {
      state[i] = observation_->state.value[static_cast<std::size_t>(i)];
    }
    return state;
  }

  bool foreignTargetPublisherPresent() const
  {
    const auto publishers = get_publishers_info_by_topic(target_topic_);
    for (const auto & publisher : publishers) {
      if (publisher.node_name() != get_name() ||
          publisher.node_namespace() != get_namespace())
      {
        return true;
      }
    }
    return false;
  }

  bool wrenchLimitExceeded() const
  {
    return (measured_wrench_.cwiseAbs().array() >
      hard_wrench_limit_.array()).any();
  }

  void latchFault(const std::string & reason)
  {
    if (!fault_latched_) {
      RCLCPP_ERROR(get_logger(), "Force-control fault latched: %s", reason.c_str());
    }
    fault_latched_ = true;
    fault_reason_ = reason;
    armed_ = false;
    nominal_captured_ = false;
    admittance_->reset(measured_force_);
    force_follower_->reset(measured_force_);
    cartesian_controller_->reset(measured_wrench_);
    publishControlState("FAULT_" + reason);
  }

  void publishControlState(const std::string & state)
  {
    if (state == last_control_state_) {
      return;
    }
    last_control_state_ = state;
    std_msgs::msg::String message;
    message.data = state;
    control_state_publisher_->publish(message);
  }

  void update()
  {
    const auto wall_now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(wall_now - last_update_).count();
    last_update_ = wall_now;
    std::lock_guard<std::mutex> lock(mutex_);
    if (!observation_) {
      publishControlState(armed_ ? "WAITING_FOR_OBSERVATION" : "DISABLED");
      return;
    }
    const Eigen::VectorXd measured_state = observationStateLocked();
    const bool observation_timed_out =
      !observation_received_ ||
      std::chrono::duration<double>(wall_now - last_observation_).count() >
      observation_timeout_;
    const bool wrench_timed_out =
      !wrench_received_ ||
      std::chrono::duration<double>(wall_now - last_wrench_).count() >
      force_timeout_;

    // Initial startup is not a sensor-loss fault.  Wait for the first valid
    // sample; after a stream has started, any timeout is latched below.
    if (armed_ && !wrench_received_) {
      publishControlState("WAITING_FOR_WRENCH");
      publishReference(measured_state);
      return;
    }

    if (armed_ && observation_timed_out) {
      latchFault("OBSERVATION_TIMEOUT");
    } else if (armed_ && wrench_timed_out) {
      latchFault("WRENCH_TIMEOUT");
    } else if (armed_ && wrenchLimitExceeded()) {
      latchFault("WRENCH_LIMIT");
    } else if (armed_ && enforce_single_target_owner_ &&
      foreignTargetPublisherPresent())
    {
      latchFault("TARGET_OWNER");
    }

    if (!armed_) {
      if (!fault_latched_) {
        publishControlState("DISABLED");
      }
      publishReference(measured_state);
      return;
    }

    if (!nominal_captured_) {
      const double elapsed =
        std::chrono::duration<double>(wall_now - capture_requested_at_).count();
      if (elapsed < capture_settle_time_) {
        publishControlState("SETTLING");
        return;
      }
      nominal_state_ = measured_state;
      response_world_ =
        Eigen::AngleAxisd(nominal_state_[2], Eigen::Vector3d::UnitZ()) *
        response_body_;
      nominal_ee_ = kinematics_->framePosition(nominal_state_);
      nominal_ee_rotation_ = kinematics_->frameRotation(nominal_state_);
      admittance_->reset(0.0);
      force_follower_->reset(0.0);
      cartesian_controller_->reset(measured_wrench_);
      nominal_captured_ = true;
      RCLCPP_INFO(
        get_logger(), "Captured nominal state; accepting %s wrench",
        cartesian_mode_ ? "6D" : "legacy scalar");
      publishControlState("ACTIVE");
    }
    Vector6d correction = Vector6d::Zero();
    Vector6d filtered_wrench = Vector6d::Zero();
    Eigen::VectorXd reference;
    double primary_offset = 0.0;
    double primary_force = 0.0;

    if (cartesian_mode_) {
      correction = cartesian_controller_->update(measured_wrench_, dt);
      filtered_wrench = cartesian_controller_->measuredWrench();
      reference = kinematics_->correctedState6D(
        nominal_state_, correction, base_share_, max_base_delta_,
        max_joint_delta_);
      for (std::size_t i = 0; i < 6; ++i) {
        if (admittance_axes_[i]) {
          primary_offset = correction[i];
          primary_force = filtered_wrench[i];
          break;
        }
      }
    } else {
      const bool force_follow = control_mode_ == "force_follow";
      primary_offset = force_follow ?
        force_follower_->update(measured_force_, dt) :
        admittance_->update(measured_force_, dt);
      primary_force = force_follow ?
        force_follower_->measuredForce() : admittance_->measuredForce();
      reference = kinematics_->correctedState(
        nominal_state_, response_world_, primary_offset, base_share_,
        max_base_delta_, max_joint_delta_);
      const std::size_t index = axisIndex(force_axis_);
      correction[index] = primary_offset;
      filtered_wrench[index] = primary_force;
    }

    publishReference(reference);
    publishStatus(
      reference, measured_state, primary_force, primary_offset,
      filtered_wrench, correction);
  }

  void publishReference(const Eigen::VectorXd & reference)
  {
    if (!reference_output_enabled_) {
      return;
    }
    ocs2_msgs::msg::MpcTargetTrajectories target;
    const int count = std::max(
      2, static_cast<int>(std::ceil(reference_horizon_ / reference_dt_)) + 1);
    for (int i = 0; i < count; ++i) {
      target.time_trajectory.push_back(
        observation_->time + 0.02 + static_cast<double>(i) * reference_dt_);
      ocs2_msgs::msg::MpcState state_message;
      state_message.value = toFloatVector(reference);
      target.state_trajectory.push_back(std::move(state_message));
      ocs2_msgs::msg::MpcInput input_message;
      input_message.value = std::vector<float>(
        static_cast<std::size_t>(input_dimension_), 0.0F);
      target.input_trajectory.push_back(std::move(input_message));
    }
    target_publisher_->publish(target);
  }

  void publishStatus(
    const Eigen::VectorXd & reference,
    const Eigen::VectorXd & measured_state,
    double primary_force, double primary_offset,
    const Vector6d & filtered_wrench,
    const Vector6d & correction)
  {
    Eigen::Vector3d report_direction = response_world_;
    if (cartesian_mode_ && correction.head<3>().norm() > 1.0e-12) {
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
      .cwiseAbs().maxCoeff();

    std_msgs::msg::Float64MultiArray status;
    status.data = {
      primary_force, primary_offset, base_reference,
      ee_reference - base_reference, measured_base, measured_ee,
      measured_ee - measured_base, max_joint_motion, lateral_base};
    for (Eigen::Index i = 0; i < 6; ++i) {
      status.data.push_back(filtered_wrench[i]);
    }
    for (Eigen::Index i = 0; i < 6; ++i) {
      status.data.push_back(correction[i]);
    }
    const Vector6d velocity = cartesian_mode_ ?
      cartesian_controller_->velocity() : Vector6d::Zero();
    for (Eigen::Index i = 0; i < 6; ++i) {
      status.data.push_back(velocity[i]);
    }
    for (std::size_t i = 0; i < 6; ++i) {
      status.data.push_back(admittance_axes_[i] ? 1.0 : 0.0);
    }
    for (std::size_t i = 0; i < 6; ++i) {
      status.data.push_back(constant_force_axes_[i] ? 1.0 : 0.0);
    }
    status.data.push_back(armed_ ? 1.0 : 0.0);
    status.data.push_back(reference_output_enabled_ ? 1.0 : 0.0);
    status.data.push_back(fault_latched_ ? 1.0 : 0.0);
    status_publisher_->publish(status);
  }

  std::string robot_name_;
  std::string ee_frame_;
  std::string target_topic_;
  std::string control_mode_;
  std::string force_axis_;
  bool absolute_force_{false};
  bool cartesian_mode_{false};
  bool require_wrench_frame_{false};
  bool armed_{false};
  bool reference_output_enabled_{false};
  bool enforce_single_target_owner_{true};
  bool observation_received_{false};
  bool wrench_received_{false};
  bool fault_latched_{false};
  bool force_velocity_mode_{false};
  double force_deadband_{0.0};
  AxisMask6d admittance_axes_{};
  AxisMask6d constant_force_axes_{};
  AxisMask6d absolute_wrench_axes_{};
  double loop_rate_{50.0};
  double reference_horizon_{1.0};
  double reference_dt_{0.1};
  int input_dimension_{8};
  double base_share_{0.4};
  double max_base_delta_{0.04};
  double max_joint_delta_{0.25};
  double force_timeout_{0.25};
  double observation_timeout_{0.25};
  double capture_settle_time_{1.0};
  double force_scale_{1.0};
  double measured_force_{0.0};
  Vector6d measured_wrench_{Vector6d::Zero()};
  Vector6d wrench_scale_6d_{Vector6d::Ones()};
  Vector6d hard_wrench_limit_{Vector6d::Ones()};
  bool nominal_captured_{false};
  std::string fault_reason_;
  std::string last_control_state_;
  Eigen::Vector3d response_body_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d response_world_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d nominal_ee_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d nominal_ee_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::VectorXd nominal_state_;
  std::unique_ptr<WholeBodyKinematics> kinematics_;
  std::unique_ptr<AdmittanceController> admittance_;
  std::unique_ptr<ForceFollower> force_follower_;
  std::unique_ptr<CartesianComplianceController> cartesian_controller_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  ocs2_msgs::msg::MpcObservation::SharedPtr observation_;
  std::mutex mutex_;
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_update_;
  std::chrono::steady_clock::time_point last_wrench_;
  std::chrono::steady_clock::time_point last_observation_;
  std::chrono::steady_clock::time_point capture_requested_at_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr target_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_state_publisher_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscription_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace whole_body_force_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(
      std::make_shared<whole_body_force_control::WholeBodyForceControlNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(
      rclcpp::get_logger("whole_body_force_control"), "%s", exception.what());
  }
  rclcpp::shutdown();
  return 0;
}
