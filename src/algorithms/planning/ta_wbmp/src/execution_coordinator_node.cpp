#include "ta_wbmp/planner.hpp"
#include "whole_body_force_control/controllers.hpp"
#include "whole_body_force_control/whole_body_kinematics.hpp"

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ocs2_msgs/msg/mpc_input.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_state.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <traj_utils/msg/whole_body_goal.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace ta_wbmp
{
class ExecutionCoordinatorNode final : public rclcpp::Node
{
public:
  ExecutionCoordinatorNode()
  : Node("ta_wbmp_execution_coordinator"),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter("urdf_file", "");
    declare_parameter("task_file", "");
    declare_parameter("ee_frame", "tool0");
    declare_parameter("execution_enabled", false);
    declare_parameter("auto_start", false);
    declare_parameter("whole_body_goal_topic", "/remani_planner/whole_body_goal");
    declare_parameter("mpc_target_topic", "/mobile_manipulator_mpc_target");
    declare_parameter("mpc_observation_topic", "/mobile_manipulator_mpc_observation");
    declare_parameter("remani_task_service", "/remani_planner/set_task_execution");
    declare_parameter(
      "bridge_reference_service", "/remani_bridge/set_reference_enabled");
    declare_parameter("reference_handoff_timeout", 3.0);
    declare_parameter("control_frame", "odom");
    declare_parameter("arrival_squared_tolerance", 0.04);
    declare_parameter("arrival_hold", 0.35);
    declare_parameter<std::vector<std::string>>("joint_names", {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});

    const std::string urdf = get_parameter("urdf_file").as_string();
    const std::string task = get_parameter("task_file").as_string();
    if (urdf.empty() || task.empty()) {
      throw std::runtime_error("urdf_file and task_file are required");
    }
    planner_ = std::make_unique<TaskAwarePlanner>(
      urdf, get_parameter("ee_frame").as_string(), task);
    plan_ = planner_->plan();
    const auto & mpc = plan_.task_trajectory.execution.mpc;
    const auto & force = plan_.task_trajectory.execution.force;
    declare_parameter("reference_rate", mpc.reference_rate);
    declare_parameter("reference_horizon", mpc.reference_horizon);
    declare_parameter("reference_dt", mpc.reference_dt);
    declare_parameter(
      "tracking_slow_squared_tolerance",
      mpc.tracking_slow_squared_tolerance);
    declare_parameter(
      "tracking_stop_squared_tolerance",
      mpc.tracking_stop_squared_tolerance);
    force_control_enabled_ = declare_parameter(
      "force_control_enabled", force.enabled);
    force_mode_ = declare_parameter("force_control_mode", force.mode);
    force_axis_ = declare_parameter("force_axis", force.force_axis);
    declare_parameter("wrench_topic", force.wrench_topic);
    if (force_mode_ != "admittance" && force_mode_ != "constant_force" &&
        force_mode_ != "force_follow")
    {
      throw std::runtime_error(
        "force_control_mode must be admittance, constant_force, or force_follow");
    }
    if (force_axis_ != "x" && force_axis_ != "y" && force_axis_ != "z") {
      throw std::runtime_error("force_axis must be x, y, or z");
    }
    force_absolute_ = declare_parameter(
      "force_absolute", force.absolute_force);
    desired_force_ = declare_parameter("desired_force", force.desired_force);
    force_sensor_timeout_ = declare_parameter(
      "force_sensor_timeout", force.sensor_timeout);
    force_full_speed_error_ = declare_parameter(
      "force_progress_full_speed_error", force.progress_full_speed_error);
    force_pause_error_ = declare_parameter(
      "force_progress_pause_error", force.progress_pause_error);
    force_pause_error_ = std::max(
      force_full_speed_error_ + 1.0e-6, force_pause_error_);
    force_min_progress_scale_ = declare_parameter(
      "force_progress_min_scale", force.progress_min_scale);
    force_hard_limit_ = declare_parameter("force_hard_limit", force.hard_limit);
    force_spike_rejection_n_ = declare_parameter(
      "force_spike_rejection_n", force.spike_rejection_n);
    force_spike_confirm_samples_ = std::max<int>(
      1, declare_parameter(
        "force_spike_confirm_samples", force.spike_confirm_samples));
    force_base_share_ = declare_parameter("force_base_share", force.base_share);
    force_max_base_delta_ = declare_parameter(
      "force_max_base_delta", force.max_base_delta);
    force_max_joint_delta_ = declare_parameter(
      "force_max_joint_delta", force.max_joint_delta);
    const double controller_desired_force =
      force_mode_ == "admittance" ? 0.0 : desired_force_;
    admittance_ = std::make_unique<
      whole_body_force_control::AdmittanceController>(
      controller_desired_force, force.mass, force.damping, force.stiffness,
      force.max_offset, force.max_velocity, force.filter_alpha);
    force_follower_ = std::make_unique<
      whole_body_force_control::ForceFollower>(
      controller_desired_force, force.stiffness, force.max_offset,
      force.max_velocity, force.filter_alpha);
    force_kinematics_ = std::make_unique<
      whole_body_force_control::WholeBodyKinematics>(
      urdf, get_parameter("ee_frame").as_string());

    goal_publisher_ = create_publisher<traj_utils::msg::WholeBodyGoal>(
      get_parameter("whole_body_goal_topic").as_string(), rclcpp::QoS(1).reliable());
    mpc_target_topic_ = get_parameter("mpc_target_topic").as_string();
    observation_subscription_ =
      create_subscription<ocs2_msgs::msg::MpcObservation>(
      get_parameter("mpc_observation_topic").as_string(),
      rclcpp::QoS(1).best_effort(),
      std::bind(&ExecutionCoordinatorNode::observationCallback, this,
      std::placeholders::_1));
    wrench_subscription_ =
      create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_parameter("wrench_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&ExecutionCoordinatorNode::wrenchCallback, this,
      std::placeholders::_1));
    remani_task_client_ = create_client<std_srvs::srv::SetBool>(
      get_parameter("remani_task_service").as_string());
    bridge_reference_client_ = create_client<std_srvs::srv::SetBool>(
      get_parameter("bridge_reference_service").as_string());
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/ta_wbmp/execution/status", rclcpp::QoS(1).reliable().transient_local());
    start_service_ = create_service<std_srvs::srv::Trigger>(
      "/ta_wbmp/execution/start",
      std::bind(&ExecutionCoordinatorNode::startCallback, this,
      std::placeholders::_1, std::placeholders::_2));
    force_service_ = create_service<std_srvs::srv::SetBool>(
      "/ta_wbmp/execution/enable_force_control",
      std::bind(&ExecutionCoordinatorNode::forceControlCallback, this,
      std::placeholders::_1, std::placeholders::_2));
    force_status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/ta_wbmp/execution/force_state",
      rclcpp::QoS(1).reliable().transient_local());
    const double rate = std::max(
      1.0, get_parameter("reference_rate").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate),
      std::bind(&ExecutionCoordinatorNode::update, this));
    publishStatus();

    RCLCPP_INFO(
      get_logger(),
      "Prepared task '%s': REMANI q_pre + %zu-point transition/task MPC "
      "reference. execution_enabled=%s force_control=%s mode=%s",
      plan_.report.task_name.c_str(),
      plan_.waypoints.size() - plan_.execution_start_index,
      get_parameter("execution_enabled").as_bool() ? "true" : "false",
      force_control_enabled_ ? "true" : "false", force_mode_.c_str());
    if (get_parameter("execution_enabled").as_bool() &&
      get_parameter("auto_start").as_bool())
    {
      std::string reason;
      if (!startNavigation(reason)) {
        phase_ = Phase::FAILED;
        RCLCPP_ERROR(get_logger(), "Auto-start readiness failed: %s", reason.c_str());
        publishStatus();
      }
    }
  }

private:
  enum class Phase {
    READY,
    NAVIGATING,
    REQUESTING_BRIDGE_RELEASE,
    WAITING_BRIDGE_RELEASE,
    REQUESTING_TASK,
    WAITING_COORDINATOR_OWNERSHIP,
    TASK_EXEC,
    COMPLETE,
    FAILED
  };

  std::size_t mpcTargetPublisherCount() const
  {
    return get_publishers_info_by_topic(mpc_target_topic_).size();
  }

  bool navigationReadiness(std::string & reason) const
  {
    if (!planner_->environmentCollisionChecked()) {
      reason =
        "Production execution is fail-closed: shared REMANI/ESDF environment "
        "collision checker is not configured";
      return false;
    }
    if (!bridge_reference_client_->service_is_ready()) {
      reason = "REMANI bridge reference-ownership service is not ready";
      return false;
    }
    const std::size_t owners = mpcTargetPublisherCount();
    if (owners != 1U) {
      reason = "NAVIGATING requires exactly one MPC target publisher; found " +
        std::to_string(owners);
      return false;
    }
    return true;
  }

  bool handoffTimedOut() const
  {
    return std::chrono::steady_clock::now() > handoff_deadline_;
  }

  void resetHandoffDeadline()
  {
    handoff_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(
        0.1, get_parameter("reference_handoff_timeout").as_double())));
  }

  static double stateError(const Eigen::VectorXd & current,
                           const Eigen::VectorXd & goal)
  {
    if (current.size() != 9 || goal.size() != 9) {
      return std::numeric_limits<double>::infinity();
    }
    Eigen::VectorXd delta = current - goal;
    delta[2] = wrapAngle(current[2] - goal[2]);
    return delta.squaredNorm();
  }

  Eigen::VectorXd toControlFrame(const Eigen::VectorXd & source) const
  {
    if (source.size() != 9) {
      throw std::runtime_error("whole-body state must be 9D");
    }
    const std::string control_frame = get_parameter("control_frame").as_string();
    if (control_frame.empty() || control_frame == plan_.frame_id) {
      return source;
    }
    const auto transform = tf_buffer_.lookupTransform(
      control_frame, plan_.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(0.05));
    const auto & rotation = transform.transform.rotation;
    const double yaw = std::atan2(
      2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    Eigen::VectorXd result = source;
    result[0] = transform.transform.translation.x + c * source[0] - s * source[1];
    result[1] = transform.transform.translation.y + s * source[0] + c * source[1];
    result[2] = wrapAngle(source[2] + yaw);
    return result;
  }

  Eigen::Vector3d directionToControlFrame(
    const Eigen::Vector3d & source) const
  {
    const std::string control_frame = get_parameter("control_frame").as_string();
    if (control_frame.empty() || control_frame == plan_.frame_id) {
      return source;
    }
    const auto transform = tf_buffer_.lookupTransform(
      control_frame, plan_.frame_id, tf2::TimePointZero,
      tf2::durationFromSec(0.05));
    const auto & rotation = transform.transform.rotation;
    const double yaw = std::atan2(
      2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
      1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z));
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return Eigen::Vector3d(
      c * source.x() - s * source.y(),
      s * source.x() + c * source.y(), source.z());
  }

  void observationCallback(const ocs2_msgs::msg::MpcObservation::SharedPtr message)
  {
    if (message->state.value.size() != 9U) {
      return;
    }
    observation_state_.resize(9);
    for (std::size_t index = 0; index < 9; ++index) {
      observation_state_[static_cast<Eigen::Index>(index)] =
        message->state.value[index];
    }
    observation_time_ = message->time;
  }

  void wrenchCallback(
    const geometry_msgs::msg::WrenchStamped::SharedPtr message)
  {
    double force = message->wrench.force.z;
    if (force_axis_ == "x") {
      force = message->wrench.force.x;
    } else if (force_axis_ == "y") {
      force = message->wrench.force.y;
    }
    const double raw_force = force_absolute_ ? std::abs(force) : force;
    // Reject isolated F/T spikes before they can latch the hard limit or feed
    // the admittance/follower. A real sustained force change is accepted after
    // N consecutive samples remain outside the spike band.
    if (force_received_ &&
        std::abs(raw_force - measured_force_) > force_spike_rejection_n_)
    {
      ++force_spike_count_;
      if (force_spike_count_ < force_spike_confirm_samples_) {
        return;
      }
    } else {
      force_spike_count_ = 0;
    }
    measured_force_ = raw_force;
    force_received_ = true;
    last_force_wall_time_ = std::chrono::steady_clock::now();
    if (force_control_enabled_ && measured_force_ >= force_hard_limit_) {
      force_hard_stop_ = true;
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Force hard limit latched at %.2f N; task progress stopped",
        measured_force_);
    }
  }

  void forceControlCallback(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    force_control_enabled_ = request->data;
    force_hard_stop_ = false;
    force_received_ = false;
    force_spike_count_ = 0;
    admittance_->reset(0.0);
    force_follower_->reset(0.0);
    response->success = true;
    response->message = force_control_enabled_ ?
      "TA-WBMP force execution enabled; safety latch reset" :
      "TA-WBMP force execution disabled";
    publishForceStatus("operator_toggle");
  }

  void startCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (!get_parameter("execution_enabled").as_bool()) {
      response->success = false;
      response->message = "execution_enabled=false; no live topic was written";
      return;
    }
    if (phase_ != Phase::READY) {
      response->success = false;
      response->message = "execution is not in READY state";
      return;
    }
    std::string reason;
    if (!startNavigation(reason)) {
      response->success = false;
      response->message = reason;
      return;
    }
    response->success = true;
    response->message = "REMANI navigation goal published";
  }

  bool startNavigation(std::string & reason)
  {
    if (!navigationReadiness(reason)) {
      return false;
    }
    traj_utils::msg::WholeBodyGoal goal;
    goal.header.frame_id = plan_.frame_id;
    goal.header.stamp = now();
    goal.base_pose.position.x = plan_.remani_navigation_goal[0];
    goal.base_pose.position.y = plan_.remani_navigation_goal[1];
    goal.base_pose.orientation.z = std::sin(
      0.5 * plan_.remani_navigation_goal[2]);
    goal.base_pose.orientation.w = std::cos(
      0.5 * plan_.remani_navigation_goal[2]);
    goal.joint_names = get_parameter("joint_names").as_string_array();
    goal.joint_positions.assign(
      plan_.remani_navigation_goal.data() + 3,
      plan_.remani_navigation_goal.data() + 9);
    goal_publisher_->publish(goal);
    phase_ = Phase::NAVIGATING;
    arrival_since_.reset();
    publishStatus();
    return true;
  }

  Waypoint sampleExecution(double relative_time) const
  {
    const double origin = plan_.waypoints[plan_.execution_start_index].time;
    const double query = origin + std::clamp(
      relative_time, 0.0, plan_.waypoints.back().time - origin);
    const auto begin = std::next(
      plan_.waypoints.begin(), static_cast<std::ptrdiff_t>(plan_.execution_start_index));
    const auto second = std::lower_bound(
      begin, plan_.waypoints.end(), query,
      [](const Waypoint & waypoint, double value) {return waypoint.time < value;});
    if (second == begin) {
      return *begin;
    }
    if (second == plan_.waypoints.end()) {
      return plan_.waypoints.back();
    }
    const auto first = std::prev(second);
    const double ratio = std::clamp(
      (query - first->time) / std::max(1.0e-9, second->time - first->time),
      0.0, 1.0);
    Waypoint result = *first;
    result.time = query;
    result.state = first->state + ratio * (second->state - first->state);
    result.state[2] = wrapAngle(
      first->state[2] + ratio * wrapAngle(second->state[2] - first->state[2]));
    result.task_target =
      first->task_target + ratio * (second->task_target - first->task_target);
    result.has_task_target = first->has_task_target || second->has_task_target;
    result.task_normal =
      first->task_normal + ratio * (second->task_normal - first->task_normal);
    if (result.task_normal.norm() > 1.0e-9) {
      result.task_normal.normalize();
    }
    result.contact = ratio >= 1.0 - 1.0e-9 ?
      second->contact : first->contact && second->contact;
    if (ratio >= 1.0 - 1.0e-9) {
      result.phase = second->phase;
    }
    return result;
  }

  void requestBridgeRelease()
  {
    if (!bridge_reference_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for REMANI bridge ownership service");
      return;
    }
    phase_ = Phase::REQUESTING_BRIDGE_RELEASE;
    resetHandoffDeadline();
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = false;
    bridge_reference_client_->async_send_request(
      request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        if (phase_ != Phase::REQUESTING_BRIDGE_RELEASE) {
          return;
        }
        const auto response = future.get();
        if (!response->success) {
          phase_ = Phase::FAILED;
          RCLCPP_ERROR(get_logger(), "Bridge rejected ownership release: %s",
            response->message.c_str());
        } else {
          phase_ = Phase::WAITING_BRIDGE_RELEASE;
          RCLCPP_INFO(
            get_logger(),
            "Bridge release confirmed; waiting for MPC target publisher count=0");
        }
        publishStatus();
      });
    publishStatus();
  }

  void requestTaskOwnership()
  {
    if (!remani_task_client_->service_is_ready()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for REMANI task-execution service");
      return;
    }
    phase_ = Phase::REQUESTING_TASK;
    resetHandoffDeadline();
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    remani_task_client_->async_send_request(
      request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
        if (phase_ != Phase::REQUESTING_TASK) {
          return;
        }
        const auto response = future.get();
        if (!response->success) {
          phase_ = Phase::FAILED;
          RCLCPP_ERROR(get_logger(), "REMANI rejected TASK_EXEC: %s",
            response->message.c_str());
        } else {
          target_publisher_ =
            create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
            mpc_target_topic_, rclcpp::QoS(1).reliable());
          phase_ = Phase::WAITING_COORDINATOR_OWNERSHIP;
          resetHandoffDeadline();
          RCLCPP_INFO(
            get_logger(),
            "REMANI TASK_EXEC confirmed; waiting for Coordinator publisher count=1");
        }
        publishStatus();
      });
    publishStatus();
  }

  double updateForceControl(
    const Waypoint & reference, double dt, double & progress_scale)
  {
    if (!force_control_enabled_ || !reference.contact ||
        reference.phase != kPhaseTask)
    {
      force_offset_ = 0.0;
      admittance_->reset(measured_force_);
      force_follower_->reset(measured_force_);
      publishForceStatus(force_control_enabled_ ? "inactive" : "disabled");
      return force_offset_;
    }

    const double sensor_age = force_received_ ?
      std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_force_wall_time_).count() :
      std::numeric_limits<double>::infinity();
    if (sensor_age > force_sensor_timeout_) {
      progress_scale = 0.0;
      publishForceStatus("sensor_stale_hold");
      return force_offset_;
    }

    force_offset_ = force_mode_ == "force_follow" ?
      force_follower_->update(measured_force_, dt) :
      admittance_->update(measured_force_, dt);
    const double target_force = force_mode_ == "admittance" ?
      0.0 : desired_force_;
    const double error = std::abs(measured_force_ - target_force);
    double force_scale = 1.0;
    if (force_hard_stop_ || error >= force_pause_error_) {
      force_scale = 0.0;
    } else if (error > force_full_speed_error_) {
      const double ratio = (error - force_full_speed_error_) /
        std::max(1.0e-9, force_pause_error_ - force_full_speed_error_);
      force_scale = 1.0 - std::clamp(ratio, 0.0, 1.0) *
        (1.0 - std::clamp(force_min_progress_scale_, 0.0, 1.0));
    }
    progress_scale *= force_scale;
    publishForceStatus(force_hard_stop_ ? "hard_limit_hold" :
      (force_scale <= 0.0 ? "force_error_hold" :
      (force_scale < 1.0 ? "force_error_throttle" : "tracking")));
    return force_offset_;
  }

  Eigen::VectorXd forceCorrectedState(
    const Waypoint & waypoint, const Eigen::VectorXd & nominal,
    double force_offset) const
  {
    if (!force_control_enabled_ || !waypoint.contact ||
        waypoint.phase != kPhaseTask || std::abs(force_offset) < 1.0e-12)
    {
      return nominal;
    }
    return force_kinematics_->correctedState(
      nominal, directionToControlFrame(waypoint.task_normal), force_offset,
      force_base_share_, force_max_base_delta_, force_max_joint_delta_);
  }

  void publishForceStatus(const std::string & state)
  {
    if (!force_status_publisher_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = state + ";enabled=" +
      (force_control_enabled_ ? "true" : "false") +
      ";force_N=" + std::to_string(measured_force_) +
      ";offset_m=" + std::to_string(force_offset_);
    force_status_publisher_->publish(message);
  }

  void publishMpcReference()
  {
    if (!observation_time_ || !task_start_observation_time_) {
      return;
    }
    const Eigen::VectorXd current_reference = toControlFrame(
      sampleExecution(virtual_progress_).state);
    const double tracking_error = stateError(observation_state_, current_reference);
    const double slow = std::max(
      0.0, get_parameter("tracking_slow_squared_tolerance").as_double());
    const double stop = std::max(
      slow + 1.0e-9,
      get_parameter("tracking_stop_squared_tolerance").as_double());
    double progress_scale = 1.0;
    if (tracking_error >= stop) {
      progress_scale = 0.0;
    } else if (tracking_error > slow) {
      progress_scale = (stop - tracking_error) / (stop - slow);
    }
    const auto force_wall_now = std::chrono::steady_clock::now();
    const double force_dt = std::clamp(
      std::chrono::duration<double>(
      force_wall_now - force_update_wall_time_).count(), 0.0, 0.05);
    force_update_wall_time_ = force_wall_now;
    const Waypoint active_waypoint = sampleExecution(virtual_progress_);
    const double force_offset = updateForceControl(
      active_waypoint, force_dt, progress_scale);
    if (last_progress_observation_time_) {
      const double elapsed = std::clamp(
        *observation_time_ - *last_progress_observation_time_, 0.0, 0.2);
      virtual_progress_ += progress_scale * elapsed;
    }
    last_progress_observation_time_ = observation_time_;
    const double progress = virtual_progress_;
    const double duration = plan_.waypoints.back().time -
      plan_.waypoints[plan_.execution_start_index].time;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "TA-WBMP TASK_EXEC progress=%.3f/%.3f tracking_err=%.4f "
      "progress_scale=%.3f", progress, duration, tracking_error,
      progress_scale);
    const double horizon = get_parameter("reference_horizon").as_double();
    const double dt = std::max(0.01, get_parameter("reference_dt").as_double());
    const int count = std::max(2, static_cast<int>(std::ceil(horizon / dt)) + 1);
    ocs2_msgs::msg::MpcTargetTrajectories target;
    for (int index = 0; index < count; ++index) {
      const double offset = index * dt;
      const Waypoint first = sampleExecution(progress + offset);
      const Waypoint second = sampleExecution(progress + offset + dt);
      const Eigen::VectorXd first_state = forceCorrectedState(
        first, toControlFrame(first.state), force_offset);
      const Eigen::VectorXd second_state = toControlFrame(second.state);
      const Eigen::VectorXd corrected_second_state = forceCorrectedState(
        second, second_state, force_offset);
      target.time_trajectory.push_back(*observation_time_ + offset);
      ocs2_msgs::msg::MpcState state;
      for (Eigen::Index value = 0; value < first_state.size(); ++value) {
        state.value.push_back(static_cast<float>(first_state[value]));
      }
      target.state_trajectory.push_back(std::move(state));
      Eigen::VectorXd input = Eigen::VectorXd::Zero(8);
      const Eigen::Vector2d delta =
        corrected_second_state.head<2>() - first_state.head<2>();
      input[0] = (std::cos(first_state[2]) * delta.x() +
        std::sin(first_state[2]) * delta.y()) / dt;
      input[1] = wrapAngle(corrected_second_state[2] - first_state[2]) / dt;
      input.tail(6) =
        (corrected_second_state.tail(6) - first_state.tail(6)) / dt;
      ocs2_msgs::msg::MpcInput command;
      for (Eigen::Index value = 0; value < input.size(); ++value) {
        command.value.push_back(static_cast<float>(input[value]));
      }
      target.input_trajectory.push_back(std::move(command));
    }
    target_publisher_->publish(target);
    if (progress >= duration) {
      phase_ = Phase::COMPLETE;
      RCLCPP_INFO(
        get_logger(),
        "TA-WBMP execution COMPLETE: reached end of task trajectory "
        "(progress=%.3f s, duration=%.3f s)", progress, duration);
      publishStatus();
    }
  }

  void update()
  {
    if (phase_ == Phase::NAVIGATING && observation_state_.size() == 9) {
      double error = std::numeric_limits<double>::infinity();
      try {
        error = stateError(
          observation_state_, toControlFrame(plan_.remani_navigation_goal));
      } catch (const tf2::TransformException & exception) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for task-to-control TF: %s", exception.what());
        return;
      }
      if (error <= get_parameter("arrival_squared_tolerance").as_double()) {
        if (!arrival_since_) {
          arrival_since_ = now();
        }
        if ((now() - *arrival_since_).seconds() >=
          get_parameter("arrival_hold").as_double())
        {
          requestBridgeRelease();
        }
      } else {
        arrival_since_.reset();
      }
    } else if (phase_ == Phase::REQUESTING_BRIDGE_RELEASE ||
      phase_ == Phase::REQUESTING_TASK)
    {
      if (handoffTimedOut()) {
        phase_ = Phase::FAILED;
        RCLCPP_ERROR(get_logger(), "Reference handoff service timed out");
        publishStatus();
      }
    } else if (phase_ == Phase::WAITING_BRIDGE_RELEASE) {
      if (mpcTargetPublisherCount() == 0U) {
        requestTaskOwnership();
      } else if (handoffTimedOut()) {
        phase_ = Phase::FAILED;
        RCLCPP_ERROR(
          get_logger(),
          "Reference handoff failed: bridge publisher did not disappear");
        publishStatus();
      }
    } else if (phase_ == Phase::WAITING_COORDINATOR_OWNERSHIP) {
      const std::size_t owners = mpcTargetPublisherCount();
      if (owners == 1U) {
        phase_ = Phase::TASK_EXEC;
        task_start_observation_time_ = observation_time_;
        last_progress_observation_time_ = observation_time_;
        virtual_progress_ = 0.0;
        RCLCPP_INFO(
          get_logger(),
          "Coordinator exclusively owns MPC reference; starting task trajectory");
        publishStatus();
      } else if (owners > 1U || handoffTimedOut()) {
        phase_ = Phase::FAILED;
        RCLCPP_ERROR(
          get_logger(),
          "Reference handoff failed: TASK_EXEC publisher count=%zu", owners);
        publishStatus();
      }
    } else if (phase_ == Phase::TASK_EXEC) {
      try {
        publishMpcReference();
      } catch (const tf2::TransformException & exception) {
        RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Cannot publish task reference without TF: %s", exception.what());
      }
    }
  }

  void publishStatus()
  {
    const char * names[] = {"READY", "NAVIGATING",
      "REQUESTING_BRIDGE_RELEASE", "WAITING_BRIDGE_RELEASE",
      "REQUESTING_TASK", "WAITING_COORDINATOR_OWNERSHIP",
      "TASK_EXEC", "COMPLETE", "FAILED"};
    std_msgs::msg::String message;
    message.data = names[static_cast<int>(phase_)];
    status_publisher_->publish(message);
  }

  std::unique_ptr<TaskAwarePlanner> planner_;
  std::unique_ptr<whole_body_force_control::AdmittanceController> admittance_;
  std::unique_ptr<whole_body_force_control::ForceFollower> force_follower_;
  std::unique_ptr<whole_body_force_control::WholeBodyKinematics>
    force_kinematics_;
  Plan plan_;
  Phase phase_{Phase::READY};
  Eigen::VectorXd observation_state_;
  std::optional<double> observation_time_;
  std::optional<double> task_start_observation_time_;
  std::optional<double> last_progress_observation_time_;
  double virtual_progress_{0.0};
  bool force_control_enabled_{false};
  bool force_absolute_{true};
  bool force_received_{false};
  bool force_hard_stop_{false};
  std::string force_mode_{"constant_force"};
  std::string force_axis_{"z"};
  double desired_force_{12.0};
  double measured_force_{0.0};
  double force_offset_{0.0};
  double force_sensor_timeout_{0.20};
  double force_full_speed_error_{2.0};
  double force_pause_error_{8.0};
  double force_min_progress_scale_{0.10};
  double force_hard_limit_{35.0};
  double force_spike_rejection_n_{8.0};
  int force_spike_confirm_samples_{3};
  int force_spike_count_{0};
  double force_base_share_{0.0};
  double force_max_base_delta_{0.02};
  double force_max_joint_delta_{0.10};
  std::optional<rclcpp::Time> arrival_since_;
  std::chrono::steady_clock::time_point last_force_wall_time_{
    std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point force_update_wall_time_{
    std::chrono::steady_clock::now()};
  std::chrono::steady_clock::time_point handoff_deadline_{
    std::chrono::steady_clock::now()};
  std::string mpc_target_topic_;
  rclcpp::Publisher<traj_utils::msg::WholeBodyGoal>::SharedPtr goal_publisher_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr target_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr force_status_publisher_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr
    observation_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
    wrench_subscription_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr remani_task_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr bridge_reference_client_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr force_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  mutable tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};
}  // namespace ta_wbmp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ta_wbmp::ExecutionCoordinatorNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("ta_wbmp_execution_coordinator"),
      "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
