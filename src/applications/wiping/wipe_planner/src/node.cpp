#include "wipe_planner/planner.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <ocs2_msgs/msg/mpc_input.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_state.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <quadrotor_msgs/msg/polynomial_traj.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <traj_utils/msg/whole_body_goal.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace wipe_planner
{
using PolynomialTraj = quadrotor_msgs::msg::PolynomialTraj;

class RemaniTrajectory
{
public:
  explicit RemaniTrajectory(std::vector<PolynomialTraj> sections, double start_time)
  : sections_(std::move(sections)), start_time_(start_time)
  {
    std::sort(sections_.begin(), sections_.end(),
      [](const auto & a, const auto & b) {return a.trajectory_id < b.trajectory_id;});
    for (const auto & section : sections_) {
      for (const auto & piece : section.trajectory) {
        duration_ += std::max(0.0, piece.duration);
      }
    }
  }

  std::pair<Eigen::VectorXd, Eigen::VectorXd> sample(
    double relative_time, double yaw_reference) const
  {
    double remaining = std::clamp(relative_time, 0.0, duration_);
    const PolynomialTraj * selected_section = nullptr;
    const quadrotor_msgs::msg::PolynomialMatrix * selected_piece = nullptr;
    for (const auto & section : sections_) {
      for (const auto & piece : section.trajectory) {
        selected_section = &section;
        selected_piece = &piece;
        if (remaining <= piece.duration) {
          return evaluate(section, piece, remaining, yaw_reference);
        }
        remaining -= std::max(0.0, piece.duration);
      }
    }
    if (selected_section == nullptr || selected_piece == nullptr) {
      throw std::runtime_error("Empty REMANI trajectory");
    }
    return evaluate(*selected_section, *selected_piece,
                    selected_piece->duration, yaw_reference);
  }

  double startTime() const {return start_time_;}
  double duration() const {return duration_;}

private:
  static std::pair<Eigen::VectorXd, Eigen::VectorXd> evaluate(
    const PolynomialTraj & section,
    const quadrotor_msgs::msg::PolynomialMatrix & piece,
    double time, double yaw_reference)
  {
    const int dimensions = static_cast<int>(piece.num_dim);
    const int degree = static_cast<int>(piece.num_order);
    if (dimensions != 8 ||
        piece.data.size() != static_cast<std::size_t>(dimensions * (degree + 1)))
    {
      throw std::runtime_error("Malformed REMANI PolynomialMatrix");
    }
    time = std::clamp(time, 0.0, piece.duration);
    Eigen::VectorXd position = Eigen::VectorXd::Zero(dimensions);
    Eigen::VectorXd velocity = Eigen::VectorXd::Zero(dimensions);
    Eigen::VectorXd acceleration = Eigen::VectorXd::Zero(dimensions);
    for (int row = 0; row <= degree; ++row) {
      const int power = degree - row;
      for (int column = 0; column < dimensions; ++column) {
        const double coefficient = piece.data[row * dimensions + column];
        position[column] += coefficient * std::pow(time, power);
        if (power >= 1) {
          velocity[column] += power * coefficient * std::pow(time, power - 1);
        }
        if (power >= 2) {
          acceleration[column] += power * (power - 1) * coefficient *
            std::pow(time, power - 2);
        }
      }
    }
    Eigen::VectorXd state = Eigen::VectorXd::Zero(9);
    Eigen::VectorXd input = Eigen::VectorXd::Zero(8);
    state.head<2>() = position.head<2>();
    const int direction = section.singul >= 0 ? 1 : -1;
    const double speed_squared = velocity.head<2>().squaredNorm();
    double yaw = yaw_reference;
    if (speed_squared > 1.0e-8) {
      yaw = yaw_reference + wrapAngle(
        std::atan2(direction * velocity[1], direction * velocity[0]) - yaw_reference);
      input[0] = direction * std::sqrt(speed_squared);
      input[1] = (velocity[0] * acceleration[1] -
                  velocity[1] * acceleration[0]) / speed_squared;
    }
    state[2] = yaw;
    state.tail<6>() = position.tail<6>();
    input.tail<6>() = velocity.tail<6>();
    return {state, input};
  }

  std::vector<PolynomialTraj> sections_;
  double start_time_{0.0};
  double duration_{0.0};
};

class WipePlannerNode final : public rclcpp::Node
{
public:
  WipePlannerNode()
  : Node("wipe_planner")
  {
    declareParameters();
    const auto urdf_file = get_parameter("urdf_file").as_string();
    const auto task_file = get_parameter("task_file").as_string();
    if (urdf_file.empty() || task_file.empty()) {
      throw std::runtime_error("urdf_file and task_file parameters are required");
    }
    planner_ = std::make_unique<Planner>(
      urdf_file, get_parameter("ee_frame").as_string(), task_file);
    admittance_ = std::make_unique<ForceAdmittance>(
      planner_->desiredForce(), get_parameter("admittance_mass").as_double(),
      get_parameter("admittance_damping").as_double(),
      get_parameter("admittance_stiffness").as_double(),
      get_parameter("admittance_max_offset").as_double(),
      get_parameter("admittance_max_velocity").as_double(),
      get_parameter("force_filter_alpha").as_double());

    const std::string robot_name = get_parameter("robot_name").as_string();
    const auto reliable = rclcpp::QoS(1).reliable();
    target_publisher_ = create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
      robot_name + "_mpc_target", reliable);
    whole_body_goal_publisher_ = create_publisher<traj_utils::msg::WholeBodyGoal>(
      get_parameter("whole_body_goal_topic").as_string(), reliable);
    observation_subscription_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
      robot_name + "_mpc_observation", rclcpp::QoS(1).best_effort(),
      std::bind(&WipePlannerNode::observationCallback, this, std::placeholders::_1));
    trajectory_subscription_ = create_subscription<PolynomialTraj>(
      get_parameter("trajectory_topic").as_string(), rclcpp::QoS(50),
      std::bind(&WipePlannerNode::trajectoryCallback, this, std::placeholders::_1));

    force_control_enabled_ = get_parameter("force_control_enabled").as_bool();
    wrench_subscription_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      get_parameter("wrench_topic").as_string(), rclcpp::SensorDataQoS(),
      std::bind(&WipePlannerNode::wrenchCallback, this, std::placeholders::_1));
    const auto transient = rclcpp::QoS(1).reliable().transient_local();
    path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/wipe_planner/base_path", transient);
    ee_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/wipe_planner/ee_coverage_path", transient);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/wipe_planner/trajectory", transient);
    active_reference_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      "/wipe_planner/active_whole_body_reference", rclcpp::QoS(1).reliable());
    phase_publisher_ = create_publisher<std_msgs::msg::String>(
      "/wipe_planner/phase", transient);
    force_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/wipe_planner/normal_force", 10);
    offset_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/wipe_planner/admittance_offset", 10);
    force_state_publisher_ = create_publisher<std_msgs::msg::String>(
      "/wipe_planner/force_control_state", 10);
    force_progress_scale_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/wipe_planner/force_progress_scale", 10);
    contact_arm_reference_publisher_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
      "/wipe_planner/contact_arm_reference", 10);
    base_error_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/wipe_planner/base_tracking_error", 10);
    joint_error_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/wipe_planner/joint_tracking_error", 10);
    ee_error_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/wipe_planner/ee_tracking_error", 10);
    progress_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/wipe_planner/virtual_progress", 10);
    progress_rate_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/wipe_planner/virtual_progress_rate", 10);
    lag_error_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/wipe_planner/progress_lag_error", 10);
    contouring_error_publisher_ = create_publisher<std_msgs::msg::Float64>(
      "/wipe_planner/contouring_error", 10);
    remani_task_client_ = create_client<std_srvs::srv::SetBool>(
      get_parameter("remani_task_service").as_string());
    force_control_service_ = create_service<std_srvs::srv::SetBool>(
      "/wipe_planner/enable_force_control",
      std::bind(&WipePlannerNode::forceControlService, this,
        std::placeholders::_1, std::placeholders::_2));

    const double rate = std::max(1.0, get_parameter("publish_rate").as_double());
    timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
      std::bind(&WipePlannerNode::timerCallback, this));
    RCLCPP_INFO(get_logger(),
      "REMANI -> WipePlanner(C++ differential drive) -> MPC ready; "
      "state=[base_x base_y base_yaw joint_1..joint_6] (9D), "
      "input=[v yaw_rate joint_vel_1..joint_vel_6] (8D), force=%s",
      force_control_enabled_ ? "enabled" : "disabled");
  }

  ~WipePlannerNode() override
  {
    shutting_down_.store(true);
    if (planning_thread_.joinable()) {
      planning_thread_.join();
    }
  }

private:
  enum class Phase {WAITING, NAVIGATING, PLANNING, WIPING, FAILED};

  void declareParameters()
  {
    declare_parameter("robot_name", "mobile_manipulator");
    declare_parameter("trajectory_topic", "/planning/trajectory");
    declare_parameter("wrench_topic", "/jaka_fts_broadcaster/wrench");
    declare_parameter("remani_task_service", "/remani_planner/set_task_execution");
    declare_parameter("world_frame", "odom");
    declare_parameter("ee_frame", "tool0");
    declare_parameter("urdf_file", "");
    declare_parameter("task_file", "");
    declare_parameter("publish_rate", 20.0);
    declare_parameter("reference_horizon", 3.0);
    declare_parameter("reference_dt", 0.08);
    declare_parameter("assembly_timeout", 0.08);
    declare_parameter("start_lead", 0.15);
    declare_parameter("contact_start_lead", 0.30);
    declare_parameter("append_wipe_task", true);
    declare_parameter("auto_navigation_goal", true);
    declare_parameter("whole_body_goal_topic", "/remani_planner/whole_body_goal");
    declare_parameter<std::vector<std::string>>("joint_names", {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    declare_parameter("navigation_whole_body_squared_tolerance", 0.20);
    declare_parameter("navigation_arrival_hold", 0.35);
    declare_parameter("navigation_base_tolerance", 0.08);
    declare_parameter("navigation_yaw_tolerance", 0.25);
    declare_parameter("navigation_partial_remaining", 1.50);
    declare_parameter("navigation_partial_arrival_hold", 0.0);
    declare_parameter("alignment_joint_speed", 0.05);
    declare_parameter("task_settle_duration", 4.0);
    declare_parameter("progress_min_rate", 0.0);
    declare_parameter("progress_max_rate", 1.15);
    declare_parameter("progress_rate_filter", 0.20);
    declare_parameter("progress_lag_gain", 0.70);
    declare_parameter("progress_ahead_gain", 0.20);
    declare_parameter("progress_projection_backtrack", 2.0);
    declare_parameter("progress_projection_lookahead", 4.0);
    declare_parameter("progress_projection_dt", 0.10);
    declare_parameter("progress_base_scale", 0.20);
    declare_parameter("progress_yaw_scale", 0.45);
    declare_parameter("progress_joint_scale", 0.30);
    declare_parameter("progress_ee_scale", 0.08);
    declare_parameter("progress_error_soft", 0.70);
    declare_parameter("progress_error_hard", 1.80);
    declare_parameter("active_visualization_rate", 4.0);
    declare_parameter("active_visualization_snapshots", 3);
    declare_parameter("force_control_enabled", true);
    declare_parameter("force_axis", "z");
    declare_parameter("force_use_absolute", true);
    declare_parameter("force_filter_alpha", 0.20);
    declare_parameter("force_contact_threshold", 0.5);
    declare_parameter("force_contact_detection_window", 1.0);
    declare_parameter("force_contact_plane_max_offset", 0.030);
    declare_parameter("force_precontact_max_lead", 0.0001);
    declare_parameter("force_progress_tolerance", 5.0);
    declare_parameter("force_settle_hold", 2.0);
    declare_parameter("force_progress_pause_error", 8.0);
    declare_parameter("force_progress_resume_error", 5.0);
    declare_parameter("force_progress_pause_hold", 0.25);
    declare_parameter("force_progress_resume_hold", 0.50);
    declare_parameter("force_progress_min_scale", 0.15);
    declare_parameter("force_progress_scale_filter", 0.08);
    declare_parameter("admittance_mass", 2.0);
    declare_parameter("admittance_damping", 200.0);
    declare_parameter("admittance_stiffness", 50.0);
    declare_parameter("admittance_max_offset", 0.015);
    declare_parameter("admittance_max_velocity", 0.001);
    declare_parameter("max_joint_force_correction", 0.10);
    declare_parameter("force_hard_limit", 35.0);
    declare_parameter("force_sensor_timeout", 0.20);
    declare_parameter("force_safety_retreat", 0.020);
    declare_parameter("force_safety_retreat_speed", 0.010);
    declare_parameter("visualization_snapshots", 6);
    declare_parameter("visualization_alpha", 0.16);
  }

  static Eigen::VectorXd toEigen(const std::vector<float> & values)
  {
    Eigen::VectorXd result(values.size());
    for (std::size_t i = 0; i < values.size(); ++i) {
      result[static_cast<Eigen::Index>(i)] = values[i];
    }
    return result;
  }

  static std::vector<float> toFloatVector(const Eigen::VectorXd & values)
  {
    std::vector<float> result(values.size());
    for (Eigen::Index i = 0; i < values.size(); ++i) {
      result[static_cast<std::size_t>(i)] = static_cast<float>(values[i]);
    }
    return result;
  }

  void observationCallback(const ocs2_msgs::msg::MpcObservation::SharedPtr message)
  {
    if (message->state.value.size() != 9) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    observation_ = *message;
    observation_ros_time_ = now();
  }

  bool forceContactActiveLocked() const
  {
    return force_control_enabled_ && phase_ == Phase::WIPING &&
      (force_contact_detected_ ||
      virtual_progress_ >= wipe_contact_start_time_);
  }

  bool forceSafetyStopLocked() const
  {
    return force_hard_stop_ || force_sensor_stale_;
  }

  bool forceReadyForProgressLocked() const
  {
    if (!force_control_enabled_) {
      return true;
    }
    return initial_force_settled_ && !forceSafetyStopLocked();
  }

  void captureForceSafetyAnchorLocked()
  {
    if (!force_safety_anchor_) {
      if (last_commanded_reference_) {
        // Start the retreat from the exact reference sent on the preceding
        // cycle.  Anchoring at the measured state can itself introduce a
        // discontinuity whenever the position controller has normal lag.
        force_safety_anchor_ = *last_commanded_reference_;
      } else if (observation_) {
        force_safety_anchor_ = toEigen(observation_->state.value);
      }
    }
  }

  void resetForceSafetyRetreatLocked()
  {
    force_safety_retreat_offset_ = 0.0;
    force_safety_retreat_initialized_ = false;
  }

  void refreshForceSafetyAnchorLocked()
  {
    if (forceSafetyStopLocked()) {
      captureForceSafetyAnchorLocked();
    } else {
      force_safety_anchor_.reset();
      resetForceSafetyRetreatLocked();
    }
  }

  void updateForceSafetyRetreatLocked()
  {
    if (!forceSafetyStopLocked() || !forceContactActiveLocked()) {
      resetForceSafetyRetreatLocked();
      return;
    }
    captureForceSafetyAnchorLocked();
    const auto current = std::chrono::steady_clock::now();
    if (!force_safety_retreat_initialized_) {
      force_safety_retreat_wall_time_ = current;
      force_safety_retreat_initialized_ = true;
      return;
    }
    const double dt = std::clamp(
      std::chrono::duration<double>(
        current - force_safety_retreat_wall_time_).count(), 0.0, 0.10);
    force_safety_retreat_wall_time_ = current;
    force_safety_retreat_offset_ = rateLimitedStep(
      force_safety_retreat_offset_,
      std::abs(get_parameter("force_safety_retreat").as_double()),
      std::abs(get_parameter("force_safety_retreat_speed").as_double()), dt);
  }

  void resetForceProgressSupervisorLocked()
  {
    initial_force_settled_ = false;
    force_motion_paused_ = false;
    force_progress_scale_ = 0.0;
    force_in_tolerance_since_.reset();
    force_pause_since_.reset();
    force_resume_since_.reset();
  }

  void updateForceProgressSupervisorLocked(
    const rclcpp::Time & current, double force_error)
  {
    if (!forceContactActiveLocked()) {
      resetForceProgressSupervisorLocked();
      return;
    }
    if (forceSafetyStopLocked()) {
      // Safety stop ownership is handled separately and must remain immediate.
      force_progress_scale_ = 0.0;
      force_pause_since_.reset();
      force_resume_since_.reset();
      return;
    }

    const double full_speed_error = std::max(
      0.0, get_parameter("force_progress_tolerance").as_double());
    if (!initial_force_settled_) {
      if (force_error <= full_speed_error) {
        if (!force_in_tolerance_since_) {
          force_in_tolerance_since_ = current;
        }
        const double settle_hold = std::max(
          0.0, get_parameter("force_settle_hold").as_double());
        if ((current - *force_in_tolerance_since_).seconds() >= settle_hold) {
          initial_force_settled_ = true;
          force_progress_scale_ = 1.0;
          RCLCPP_INFO(get_logger(),
            "Initial contact force settled; continuous wiping supervisor enabled");
        }
      } else {
        force_in_tolerance_since_.reset();
      }
      return;
    }

    // After the initial contact has settled, ordinary force ripple must not
    // restart the two-second contact-establishment hold.  Only a sustained,
    // much larger error enters a hysteretic pause; moderate error continuously
    // throttles path time while admittance restores the normal force.
    const double pause_error = std::max(
      full_speed_error + 1.0e-6,
      get_parameter("force_progress_pause_error").as_double());
    const double resume_error = std::clamp(
      get_parameter("force_progress_resume_error").as_double(),
      full_speed_error, pause_error);
    if (!force_motion_paused_) {
      force_resume_since_.reset();
      if (force_error >= pause_error) {
        if (!force_pause_since_) {
          force_pause_since_ = current;
        }
        const double pause_hold = std::max(
          0.0, get_parameter("force_progress_pause_hold").as_double());
        if ((current - *force_pause_since_).seconds() >= pause_hold) {
          force_motion_paused_ = true;
          force_pause_since_.reset();
          RCLCPP_WARN(get_logger(),
            "Sustained force error %.2f N; smoothly pausing path progress",
            force_error);
        }
      } else {
        force_pause_since_.reset();
      }
    } else {
      force_pause_since_.reset();
      if (force_error <= resume_error) {
        if (!force_resume_since_) {
          force_resume_since_ = current;
        }
        const double resume_hold = std::max(
          0.0, get_parameter("force_progress_resume_hold").as_double());
        if ((current - *force_resume_since_).seconds() >= resume_hold) {
          force_motion_paused_ = false;
          force_resume_since_.reset();
          RCLCPP_INFO(get_logger(),
            "Force error recovered to %.2f N; resuming path progress",
            force_error);
        }
      } else {
        force_resume_since_.reset();
      }
    }

    const double target_scale = force_motion_paused_ ? 0.0 : forceProgressScale(
      force_error, full_speed_error, pause_error,
      get_parameter("force_progress_min_scale").as_double());
    const double filter = std::clamp(
      get_parameter("force_progress_scale_filter").as_double(), 0.0, 1.0);
    force_progress_scale_ = std::clamp(
      (1.0 - filter) * force_progress_scale_ + filter * target_scale,
      0.0, 1.0);
  }

  void forceControlService(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    force_control_enabled_ = request->data;
    admittance_->reset(admittance_->measuredForce());
    force_hard_stop_ = false;
    force_sensor_stale_ = false;
    force_contact_detected_ = false;
    force_contact_plane_offset_ = 0.0;
    resetForceProgressSupervisorLocked();
    force_safety_anchor_.reset();
    resetForceSafetyRetreatLocked();
    response->success = true;
    response->message = force_control_enabled_ ?
      "force admittance enabled and safety latch reset" :
      "force admittance disabled";
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr message)
  {
    const std::string axis = get_parameter("force_axis").as_string();
    double force = axis == "x" ? message->wrench.force.x :
      (axis == "y" ? message->wrench.force.y : message->wrench.force.z);
    if (get_parameter("force_use_absolute").as_bool()) {
      force = std::abs(force);
    }
    const rclcpp::Time current = now();
    double offset = 0.0;
    double filtered_force = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const double dt = force_message_received_ ?
        std::max(0.0, (current - last_force_time_).seconds()) : 0.0;
      last_force_time_ = current;
      last_force_wall_time_ = std::chrono::steady_clock::now();
      force_message_received_ = true;
      force_sensor_stale_ = false;
      const double detection_window = std::max(
        0.0, get_parameter("force_contact_detection_window").as_double());
      if (force_control_enabled_ && phase_ == Phase::WIPING &&
          !force_contact_detected_ && !wipe_trajectory_.empty() &&
          virtual_progress_ >= wipe_contact_start_time_ - detection_window &&
          force >= get_parameter("force_contact_threshold").as_double())
      {
        force_contact_detected_ = true;
        if (observation_) {
          const Eigen::VectorXd measured = toEigen(observation_->state.value);
          const Eigen::VectorXd nominal_contact =
            interpolate(wipe_trajectory_, wipe_contact_start_time_).first;
          const Eigen::Vector3d measured_ee = planner_->framePosition(
            measured, get_parameter("ee_frame").as_string());
          const Eigen::Vector3d nominal_ee = planner_->framePosition(
            nominal_contact, get_parameter("ee_frame").as_string());
          const double maximum = std::abs(
            get_parameter("force_contact_plane_max_offset").as_double());
          force_contact_plane_offset_ = std::clamp(
            planner_->surfaceNormal().dot(measured_ee - nominal_ee),
            -maximum, maximum);
        }
        // The first real contact defines the normal plane. Skip the remaining
        // geometric approach; adding the captured normal offset to the nominal
        // contact state makes this transition continuous.
        virtual_progress_ = std::max(virtual_progress_, wipe_contact_start_time_);
        projected_progress_ = virtual_progress_;
        admittance_->reset(force);
        resetForceProgressSupervisorLocked();
        RCLCPP_INFO(get_logger(),
          "Force contact captured at %.2f N with plane offset %.2f mm",
          force, 1000.0 * force_contact_plane_offset_);
      }
      if (forceContactActiveLocked() && !force_hard_stop_) {
        offset = admittance_->update(force, dt);
      } else {
        // Never preload the controller while navigating or approaching. A
        // missing contact force must not accumulate a maximum wall penetration
        // before the first contact sample arrives.
        admittance_->reset(force);
        offset = 0.0;
      }
      filtered_force = admittance_->measuredForce();
      if (force_control_enabled_ &&
          filtered_force >= get_parameter("force_hard_limit").as_double())
      {
        if (!force_hard_stop_) {
          RCLCPP_ERROR(get_logger(),
            "Force hard limit reached: %.2f N; stopping progress and retreating",
            filtered_force);
        }
        force_hard_stop_ = true;
      }
      const double force_error = std::abs(
        filtered_force - planner_->desiredForce());
      updateForceProgressSupervisorLocked(current, force_error);
      // An over-force event is latched. Automatic recovery caused a repeated
      // nominal-reference -> impact -> retreat cycle in contact testing. The
      // operator must explicitly reset it through enable_force_control.
      refreshForceSafetyAnchorLocked();
      offset += force_contact_plane_offset_;
    }
    std_msgs::msg::Float64 force_message;
    force_message.data = filtered_force;
    force_publisher_->publish(force_message);
    std_msgs::msg::Float64 offset_message;
    offset_message.data = offset;
    offset_publisher_->publish(offset_message);
  }

  void trajectoryCallback(const PolynomialTraj::SharedPtr message)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != Phase::WAITING && phase_ != Phase::NAVIGATING) {
      return;  // Hard ownership latch: REMANI can no longer reach the MPC topic.
    }
    if (message->action == PolynomialTraj::ACTION_ABORT ||
        message->action == PolynomialTraj::ACTION_WARN_IMPOSSIBLE)
    {
      sections_.clear();
      remani_.reset();
      phase_ = Phase::WAITING;
      return;
    }
    if (message->action != PolynomialTraj::ACTION_ADD || message->trajectory.empty()) {
      return;
    }
    if (std::any_of(message->trajectory.begin(), message->trajectory.end(),
      [](const auto & piece) {return piece.num_dim != 8;}))
    {
      RCLCPP_ERROR(get_logger(), "Rejected REMANI trajectory: expected 8 dimensions");
      return;
    }
    if (message->trajectory_id == 1) {
      sections_.clear();
    } else if (sections_.empty()) {
      return;
    }
    sections_[message->trajectory_id] = *message;
    assembly_deadline_ = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(get_parameter("assembly_timeout").as_double()));
  }

  double observationTimeNowLocked() const
  {
    if (!observation_) {
      return 0.0;
    }
    return observation_->time + std::max(0.0, (now() - observation_ros_time_).seconds());
  }

  void finishAssembly()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (sections_.empty()) {
      return;
    }
    std::vector<PolynomialTraj> messages;
    uint32_t expected = 1;
    for (const auto & [identifier, message] : sections_) {
      if (identifier != expected++) {
        RCLCPP_ERROR(get_logger(), "Incomplete REMANI trajectory batch");
        sections_.clear();
        return;
      }
      messages.push_back(message);
    }
    const double start = observationTimeNowLocked() +
      get_parameter("start_lead").as_double();
    remani_ = std::make_shared<RemaniTrajectory>(std::move(messages), start);
    sections_.clear();
    assembly_deadline_.reset();
    phase_ = Phase::NAVIGATING;
    arrival_since_.reset();
    RCLCPP_INFO(get_logger(), "Assembled REMANI navigation trajectory: %.2f s",
                remani_->duration());
  }

  bool maybeStartCoveragePreview(Eigen::VectorXd & seed)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (preview_started_ || phase_ != Phase::WAITING || !observation_) {
      return false;
    }
    preview_started_ = true;
    seed = toEigen(observation_->state.value);
    return true;
  }

  void startCoveragePreview(Eigen::VectorXd seed)
  {
    if (planning_thread_.joinable()) {
      planning_thread_.join();
    }
    planning_thread_ = std::thread([this, seed = std::move(seed)]() {
      try {
        // The planner returns a wall-normal pre-contact prefix followed by the
        // contact-constrained coverage. REMANI receives only the first safe
        // pre-contact state; WipePlanner owns the slow normal approach.
        PlanReport report;
        auto preview = planner_->plan(seed, report);
        if (preview.empty()) {
          throw std::runtime_error("Coverage preview is empty");
        }
        const auto first_contact = std::find_if(
          preview.begin(), preview.end(),
          [](const Waypoint & point) {return point.in_contact;});
        if (first_contact == preview.begin() || first_contact == preview.end() ||
            !std::none_of(preview.begin(), first_contact,
              [](const Waypoint & point) {return point.in_contact;}) ||
            !std::all_of(first_contact, preview.end(),
              [](const Waypoint & point) {return point.in_contact;}))
        {
          throw std::runtime_error(
            "Coverage preview must contain pre-contact approach followed by contact");
        }
        const double settle_duration = std::max(
          0.0, get_parameter("task_settle_duration").as_double());
        if (settle_duration > 1.0e-6 && preview.size() > 1) {
          const std::size_t contact_index = static_cast<std::size_t>(
            std::distance(preview.begin(), first_contact));
          for (std::size_t index = contact_index + 1;
               index < preview.size(); ++index)
          {
            preview[index].time += settle_duration;
          }
          Waypoint hold = preview[contact_index];
          hold.time += settle_duration;
          hold.input.setZero();
          preview.insert(
            std::next(preview.begin(), static_cast<std::ptrdiff_t>(contact_index + 1)),
            std::move(hold));
        }
        report.points = preview.size();
        report.duration = preview.back().time;
        publishVisualization(preview);
        const Eigen::VectorXd first_state = preview.front().state;
        const Eigen::Vector3d goal = first_state.head<3>();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          task_start_state_ = first_state;
          preview_trajectory_ = preview;
          preview_report_ = report;
        }
        RCLCPP_INFO(get_logger(),
          "Coverage planned before navigation: %zu points, %.1f s; "
          "Hybrid A* expanded=%d, IK rejected=%d, collision rejected=%d; "
          "pre-contact whole-body goal base=(%.3f, %.3f, %.3f), "
          "normal approach starts %.2f s before first contact",
          report.points, report.duration, report.hybrid_expanded_nodes,
          report.reachability_rejections, report.collision_rejections,
          goal.x(), goal.y(), goal.z(), first_contact->time);
        if (get_parameter("auto_navigation_goal").as_bool() &&
            !shutting_down_.load())
        {
          traj_utils::msg::WholeBodyGoal message;
          message.header.frame_id = get_parameter("world_frame").as_string();
          message.header.stamp = now();
          message.base_pose.position.x = goal.x();
          message.base_pose.position.y = goal.y();
          message.base_pose.orientation.z = std::sin(0.5 * goal.z());
          message.base_pose.orientation.w = std::cos(0.5 * goal.z());
          message.joint_names = get_parameter("joint_names").as_string_array();
          message.joint_positions.resize(6);
          for (std::size_t joint = 0; joint < 6; ++joint) {
            message.joint_positions[joint] = first_state[3 + joint];
          }
          whole_body_goal_publisher_->publish(message);
          RCLCPP_INFO(get_logger(),
            "Sent wall-normal pre-contact state (base + 6 joints) to REMANI on %s",
            get_parameter("whole_body_goal_topic").as_string().c_str());
        }
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Coverage preview failed: %s", error.what());
        std::lock_guard<std::mutex> lock(mutex_);
        phase_ = Phase::FAILED;
      }
    });
  }

  bool enterRemaniTaskState()
  {
    if (remani_task_requested_.exchange(true)) {
      return remani_task_confirmed_.load();
    }
    if (!remani_task_client_->wait_for_service(std::chrono::seconds(2))) {
      remani_task_requested_.store(false);
      RCLCPP_ERROR(get_logger(), "REMANI TASK_EXEC service unavailable");
      return false;
    }
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    auto future = remani_task_client_->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      remani_task_requested_.store(false);
      RCLCPP_ERROR(get_logger(), "Timed out waiting for REMANI TASK_EXEC acknowledgement");
      return false;
    }
    const auto response = future.get();
    if (!response->success) {
      remani_task_requested_.store(false);
      RCLCPP_ERROR(get_logger(), "REMANI TASK_EXEC rejected: %s", response->message.c_str());
      return false;
    }
    remani_task_confirmed_.store(true);
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    return true;
  }

  bool maybeStartPlanning(double current_time, Eigen::VectorXd & seed)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (phase_ != Phase::NAVIGATING || !remani_ || !observation_ ||
        !task_start_state_ ||
        !get_parameter("append_wipe_task").as_bool())
    {
      return false;
    }
    if (current_time < remani_->startTime()) {
      arrival_since_.reset();
      return false;
    }
    seed = toEigen(observation_->state.value);
    const Eigen::VectorXd & task_start = *task_start_state_;
    const double position_error =
      (seed.head<2>() - task_start.head<2>()).norm();
    const double yaw_error = std::abs(wrapAngle(seed[2] - task_start[2]));
    const double joint_error =
      (seed.tail<6>() - task_start.tail<6>()).cwiseAbs().maxCoeff();
    const double whole_body_squared_error =
      wholeBodySquaredError(seed, task_start);
    const double squared_tolerance = std::max(0.0, get_parameter(
      "navigation_whole_body_squared_tolerance").as_double());
    const double remaining =
      remani_->startTime() + remani_->duration() - current_time;
    const bool whole_body_ready = whole_body_squared_error <= squared_tolerance;
    // Near the terminal part of REMANI navigation, allow WipePlanner to take
    // over once base/yaw are settled even if the arm is still lagging. The task
    // trajectory then inserts a collision-checked, base-hold arm alignment.
    // This avoids asking REMANI's non-singular mobile-base optimizer to solve a
    // pure stationary-arm residual, which otherwise creates yaw loops.
    const bool base_yaw_ready =
      position_error <= std::max(0.0, get_parameter(
        "navigation_base_tolerance").as_double()) &&
      yaw_error <= std::max(0.0, get_parameter(
        "navigation_yaw_tolerance").as_double()) &&
      remaining <= std::max(0.0, get_parameter(
        "navigation_partial_remaining").as_double());
    const bool arrived = whole_body_ready || base_yaw_ready;
    if (!arrived) {
      if (current_time - last_navigation_log_ >= 2.0) {
        last_navigation_log_ = current_time;
        RCLCPP_INFO(get_logger(),
          "Waiting for pre-contact handoff: squared=%.4f/%.4f, "
          "base=%.3f m, yaw=%.3f rad, max_joint=%.3f rad, remaining=%.2f s",
          whole_body_squared_error, squared_tolerance,
          position_error, yaw_error, joint_error, remaining);
      }
      arrival_since_.reset();
      return false;
    }
    if (!arrival_since_) {
      arrival_since_ = current_time;
    }
    const double required_hold = whole_body_ready ?
      get_parameter("navigation_arrival_hold").as_double() :
      get_parameter("navigation_partial_arrival_hold").as_double();
    if (current_time - *arrival_since_ < std::max(0.0, required_hold))
    {
      return false;
    }
    if (remaining > 0.0) {
      RCLCPP_INFO(get_logger(),
        "%s handoff reached (squared %.4f); taking task ownership "
        "%.2f s before REMANI trajectory end",
        whole_body_ready ? "Whole-body" : "Base/yaw",
        whole_body_squared_error,
        remaining);
    } else {
      RCLCPP_INFO(get_logger(),
        "%s handoff reached (squared %.4f); taking task ownership at "
        "REMANI trajectory end",
        whole_body_ready ? "Whole-body" : "Base/yaw",
        whole_body_squared_error);
    }
    phase_ = Phase::PLANNING;
    sections_.clear();
    assembly_deadline_.reset();
    return true;
  }

  void startPlanning(Eigen::VectorXd seed)
  {
    if (planning_thread_.joinable()) {
      planning_thread_.join();
    }
    planning_thread_ = std::thread([this, seed = std::move(seed)]() {
      try {
        PlanReport report;
        std::vector<Waypoint> trajectory;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          trajectory = preview_trajectory_;
          report = preview_report_;
        }
        if (trajectory.empty()) {
          throw std::runtime_error(
            "The pre-navigation contact coverage plan is unavailable");
        }
        try {
          trajectory = planner_->prependMeasuredAlignment(
            trajectory, seed,
            get_parameter("alignment_joint_speed").as_double(), report);
        } catch (const std::exception & error) {
          RCLCPP_WARN(get_logger(),
            "Deferring WipePlanner handoff because arm alignment is unsafe: %s",
            error.what());
          std::lock_guard<std::mutex> lock(mutex_);
          phase_ = Phase::NAVIGATING;
          arrival_since_.reset();
          return;
        }
        if (!enterRemaniTaskState()) {
          throw std::runtime_error(
            "Cannot execute task trajectory before REMANI enters TASK_EXEC");
        }
        const double initial_error =
          (seed - trajectory.front().state).cwiseAbs().maxCoeff();
        RCLCPP_INFO(get_logger(),
          "Executing base-hold arm alignment, wall-normal approach, then contact "
          "coverage (handoff max state error %.3f)", initial_error);
        if (shutting_down_.load()) {
          return;
        }
        publishVisualization(trajectory);
        {
          std::lock_guard<std::mutex> lock(mutex_);
          const auto contact_start = std::find_if(
            trajectory.begin(), trajectory.end(),
            [](const Waypoint & point) {return point.in_contact;});
          wipe_contact_start_time_ = contact_start == trajectory.end() ?
            0.0 : contact_start->time;
          wipe_trajectory_ = std::move(trajectory);
          const double observation_time = observationTimeNowLocked();
          wipe_start_time_ = observation_time +
            get_parameter("contact_start_lead").as_double();
          virtual_progress_ = 0.0;
          projected_progress_ = 0.0;
          progress_lag_error_ = 0.0;
          contouring_error_ = 0.0;
          virtual_progress_rate_ = 0.0;
          last_wipe_clock_time_ = observation_time;
          admittance_->reset(admittance_->measuredForce());
          force_sensor_stale_ = false;
          force_contact_detected_ = false;
          force_contact_plane_offset_ = 0.0;
          resetForceProgressSupervisorLocked();
          force_safety_anchor_.reset();
          resetForceSafetyRetreatLocked();
          phase_ = Phase::WIPING;
        }
        RCLCPP_INFO(get_logger(),
          "Wipe trajectory ready: %zu points, %.1f s, max EE %.1f mm, "
          "orientation %.3f rad, lateral slip %.2g m/s, wheel %.2f rad/s, "
          "Hybrid A* expanded=%d, IK rejected=%d, collision rejected=%d",
          report.points, report.duration, report.max_position_error * 1000.0,
          report.max_axis_error, report.max_lateral_velocity, report.max_wheel_speed,
          report.hybrid_expanded_nodes, report.reachability_rejections,
          report.collision_rejections);
      } catch (const std::exception & error) {
        RCLCPP_ERROR(get_logger(), "Wipe planning failed: %s", error.what());
        std::lock_guard<std::mutex> lock(mutex_);
        phase_ = Phase::FAILED;
      }
    });
  }

  static std::pair<Eigen::VectorXd, Eigen::VectorXd> interpolate(
    const std::vector<Waypoint> & trajectory, double time)
  {
    if (time <= trajectory.front().time) {
      return {trajectory.front().state, trajectory.front().input};
    }
    if (time >= trajectory.back().time) {
      return {trajectory.back().state,
              Eigen::VectorXd::Zero(trajectory.back().input.size())};
    }
    const auto right = std::lower_bound(trajectory.begin(), trajectory.end(), time,
      [](const Waypoint & point, double value) {return point.time < value;});
    const auto & left = *(right - 1);
    const double ratio = (time - left.time) / std::max(1.0e-9, right->time - left.time);
    Eigen::VectorXd state = left.state + ratio * (right->state - left.state);
    state[2] = left.state[2] + ratio * wrapAngle(right->state[2] - left.state[2]);
    Eigen::VectorXd input = left.input + ratio * (right->input - left.input);
    return {state, input};
  }

  void updateForceSensorSafetyLocked()
  {
    const bool was_stale = force_sensor_stale_;
    if (!forceContactActiveLocked()) {
      force_sensor_stale_ = false;
    } else {
      const double timeout = std::max(
        0.01, get_parameter("force_sensor_timeout").as_double());
      force_sensor_stale_ = !force_message_received_ ||
        std::chrono::duration<double>(
          std::chrono::steady_clock::now() - last_force_wall_time_).count() >
        timeout;
    }
    if (force_sensor_stale_ && !was_stale) {
      RCLCPP_ERROR(get_logger(),
        "Force sensor timed out during contact; stopping progress and retreating");
    } else if (!force_sensor_stale_ && was_stale) {
      RCLCPP_WARN(get_logger(), "Force sensor data recovered");
    }
    refreshForceSafetyAnchorLocked();
  }

  Eigen::VectorXd forceCorrectedReferenceLocked(const Eigen::VectorXd & state) const
  {
    if (!forceContactActiveLocked() || forceSafetyStopLocked()) {
      return state;
    }
    const double max_joint_correction =
      get_parameter("max_joint_force_correction").as_double();
    double total_offset = force_contact_plane_offset_ + admittance_->offset();
    Eigen::VectorXd corrected = planner_->forceCorrectedState(
      state, total_offset, max_joint_correction);

    // Before the first force sample, never let the OCS2 reference get far
    // ahead of the measured tool along the wall normal.  The admittance keeps
    // integrating toward contact, but this position-error guard turns the
    // final approach into a compliant crawl.  Without it, several millimetres
    // of queued position error can still be executed while a newly captured
    // contact reference propagates through MPC, producing a stiff-wall impact.
    if (!force_contact_detected_ && observation_) {
      const Eigen::VectorXd measured = toEigen(observation_->state.value);
      const Eigen::Vector3d measured_ee = planner_->framePosition(
        measured, get_parameter("ee_frame").as_string());
      const Eigen::Vector3d corrected_ee = planner_->framePosition(
        corrected, get_parameter("ee_frame").as_string());
      const Eigen::Vector3d normal = planner_->surfaceNormal();
      const double normal_lead = normal.dot(corrected_ee - measured_ee);
      const double max_wallward_lead = std::max(
        0.0, get_parameter("force_precontact_max_lead").as_double());
      if (normal_lead < -max_wallward_lead) {
        total_offset += -max_wallward_lead - normal_lead;
        corrected = planner_->forceCorrectedState(
          state, total_offset, max_joint_correction);
      }
    }
    return corrected;
  }

  std::pair<Eigen::VectorXd, Eigen::VectorXd> sampleReferenceLocked(
    double sample_time, double yaw_reference) const
  {
    const Eigen::VectorXd hold = toEigen(observation_->state.value);
    if (phase_ == Phase::WAITING || phase_ == Phase::PLANNING ||
        phase_ == Phase::FAILED)
    {
      return {hold, Eigen::VectorXd::Zero(8)};
    }
    if (phase_ == Phase::NAVIGATING && remani_) {
      auto result = remani_->sample(sample_time - remani_->startTime(), yaw_reference);
      if (sample_time >= remani_->startTime() + remani_->duration()) {
        result.second.setZero();
      }
      return result;
    }
    if (phase_ != Phase::WIPING || wipe_trajectory_.empty()) {
      return {hold, Eigen::VectorXd::Zero(8)};
    }
    const double preview_dt = std::max(0.0, sample_time - progress_anchor_time_);
    double relative_time = virtual_progress_ + virtual_progress_rate_ * preview_dt;
    if (forceSafetyStopLocked() && forceContactActiveLocked()) {
      Eigen::VectorXd retreat = force_safety_anchor_.value_or(hold);
      retreat = planner_->forceCorrectedState(
        retreat, force_safety_retreat_offset_,
        get_parameter("max_joint_force_correction").as_double());
      return {retreat, Eigen::VectorXd::Zero(8)};
    }
    auto result = interpolate(wipe_trajectory_, relative_time);
    if (forceContactActiveLocked()) {
      result.first = forceCorrectedReferenceLocked(result.first);
    }
    return result;
  }

  double progressMetric(const Eigen::VectorXd & measured,
                        const Eigen::VectorXd & reference,
                        bool include_ee = true) const
  {
    const double base = (measured.head<2>() - reference.head<2>()).norm() /
      std::max(1.0e-6, get_parameter("progress_base_scale").as_double());
    const double yaw = std::abs(wrapAngle(measured[2] - reference[2])) /
      std::max(1.0e-6, get_parameter("progress_yaw_scale").as_double());
    const double joint = (measured.tail<6>() - reference.tail<6>())
      .cwiseAbs().maxCoeff() /
      std::max(1.0e-6, get_parameter("progress_joint_scale").as_double());
    const double ee = include_ee ? (planner_->framePosition(measured,
      get_parameter("ee_frame").as_string()) - planner_->framePosition(
      reference, get_parameter("ee_frame").as_string())).norm() /
      std::max(1.0e-6, get_parameter("progress_ee_scale").as_double()) : 0.0;
    return std::sqrt(base * base + 0.25 * yaw * yaw + joint * joint + ee * ee);
  }

  double projectProgress(const Eigen::VectorXd & measured) const
  {
    const double begin = std::max(
      wipe_trajectory_.front().time,
      virtual_progress_ - get_parameter("progress_projection_backtrack").as_double());
    const double end = std::min(
      wipe_trajectory_.back().time,
      virtual_progress_ + get_parameter("progress_projection_lookahead").as_double());
    const double step = std::max(
      0.02, get_parameter("progress_projection_dt").as_double());
    double best_progress = begin;
    double best_metric = std::numeric_limits<double>::infinity();
    for (double candidate = begin; candidate <= end + 0.5 * step; candidate += step) {
      const double tau = std::min(candidate, end);
      const Eigen::VectorXd reference = forceCorrectedReferenceLocked(
        interpolate(wipe_trajectory_, tau).first);
      const double metric = progressMetric(
        measured, reference, false);
      const double metric_tie_tolerance = 1.0e-6;
      const bool nearer_virtual_progress =
        std::abs(tau - virtual_progress_) <
        std::abs(best_progress - virtual_progress_);
      if (metric < best_metric - metric_tie_tolerance ||
          (std::abs(metric - best_metric) <= metric_tie_tolerance &&
           nearer_virtual_progress))
      {
        best_metric = metric;
        best_progress = tau;
      }
    }
    return best_progress;
  }

  void updateVirtualProgressLocked(double current_time)
  {
    if (phase_ != Phase::WIPING || wipe_trajectory_.empty()) {
      return;
    }
    const double dt = std::clamp(current_time - last_wipe_clock_time_, 0.0, 0.10);
    last_wipe_clock_time_ = current_time;
    progress_anchor_time_ = current_time;
    if (current_time < wipe_start_time_) {
      virtual_progress_rate_ = 0.0;
      return;
    }

    const Eigen::VectorXd measured = toEigen(observation_->state.value);
    if (forceSafetyStopLocked() && forceContactActiveLocked())
    {
      virtual_progress_rate_ = 0.0;
      return;
    }
    if (force_control_enabled_ && (force_contact_detected_ ||
        virtual_progress_ >= wipe_contact_start_time_) &&
        !forceReadyForProgressLocked())
    {
      // The pre-contact guard is already active during the detection window,
      // but nominal time may advance to the planned contact point.  Only then
      // pause path progress, decoupling normal force establishment from
      // tangential wiping motion.
      virtual_progress_rate_ = 0.0;
      return;
    }
    const Eigen::VectorXd current_reference = forceCorrectedReferenceLocked(
      interpolate(wipe_trajectory_, virtual_progress_).first);
    const bool contact_phase = virtual_progress_ >= wipe_contact_start_time_;
    const bool contact_settle_phase = contact_phase &&
      virtual_progress_ < wipe_contact_start_time_ +
      std::max(0.0, get_parameter("task_settle_duration").as_double());
    double tracking_error = 0.0;
    if (contact_phase && !contact_settle_phase) {
      // Once contact starts, geometric projection is meaningful: the tool is
      // following a continuous Cartesian curve on the wall.  Its lag is then
      // useful for slowing virtual time without imposing nominal waypoint
      // timestamps as hard deadlines.
      projected_progress_ = projectProgress(measured);
      progress_lag_error_ = virtual_progress_ - projected_progress_;
      const Eigen::VectorXd projected_reference = forceCorrectedReferenceLocked(
        interpolate(wipe_trajectory_, projected_progress_).first);
      const Eigen::Vector3d ee_error = planner_->framePosition(measured,
        get_parameter("ee_frame").as_string()) - planner_->framePosition(
        projected_reference, get_parameter("ee_frame").as_string());
      const Eigen::Vector3d normal = planner_->surfaceNormal();
      contouring_error_ = (ee_error - normal * normal.dot(ee_error)).norm();
      tracking_error = progressMetric(measured, current_reference);
    } else {
      // The alignment prefix is a collision-free joint-space path; the normal
      // approach and first-contact settling hold contain repeated/nearby
      // Cartesian positions.  A
      // nearest-EE projection is ambiguous there (it repeatedly selected
      // tau=0 and deadlocked tau_dot), so advance it from whole-body tracking
      // error only.  Joint tracking already bounds EE motion kinematically.
      projected_progress_ = virtual_progress_;
      progress_lag_error_ = 0.0;
      contouring_error_ = (planner_->framePosition(measured,
        get_parameter("ee_frame").as_string()) - planner_->framePosition(
        current_reference, get_parameter("ee_frame").as_string())).norm();
      tracking_error = progressMetric(measured, current_reference, false);
    }
    const double progress_force_scale =
      force_control_enabled_ && forceContactActiveLocked() ?
      force_progress_scale_ : 1.0;
    virtual_progress_rate_ = adaptiveProgressRate(
      tracking_error, progress_lag_error_, virtual_progress_rate_,
      get_parameter("progress_min_rate").as_double() * progress_force_scale,
      get_parameter("progress_max_rate").as_double() * progress_force_scale,
      get_parameter("progress_rate_filter").as_double(),
      get_parameter("progress_lag_gain").as_double(),
      get_parameter("progress_ahead_gain").as_double(),
      get_parameter("progress_error_soft").as_double(),
      get_parameter("progress_error_hard").as_double());
    double next_progress = std::min(
      wipe_trajectory_.back().time,
      virtual_progress_ + virtual_progress_rate_ * dt);

    virtual_progress_ = next_progress;
  }

  std::string phaseName(Phase phase) const
  {
    switch (phase) {
      case Phase::WAITING: return "waiting_navigation";
      case Phase::NAVIGATING: return "remani_navigation";
      case Phase::PLANNING: return "wipe_planning";
      case Phase::WIPING: return "continuous_contact_wiping";
      case Phase::FAILED: return "failed";
    }
    return "unknown";
  }

  std::string forceStateLocked() const
  {
    if (!force_control_enabled_) {
      return "disabled";
    }
    if (force_hard_stop_) {
      return "over_force_retreat";
    }
    if (force_sensor_stale_) {
      return "sensor_timeout_retreat";
    }
    if (!forceContactActiveLocked()) {
      const double guard_window = std::max(
        0.0, get_parameter("force_contact_detection_window").as_double());
      if (phase_ == Phase::WIPING && !wipe_trajectory_.empty() &&
          virtual_progress_ >= wipe_contact_start_time_ - guard_window)
      {
        // During the final slow contact search, publish the planner's smooth
        // absolute arm reference directly instead of integrating noisy MPC
        // joint velocities.  The same command path remains active after force
        // appears, so first contact no longer changes controller ownership.
        return "guarded_approach";
      }
      return "armed_waiting_for_contact";
    }
    if (!forceReadyForProgressLocked())
    {
      return "active_force_settling";
    }
    if (force_motion_paused_) {
      return "active_force_paused";
    }
    if (force_progress_scale_ < 0.98) {
      return "active_force_throttled";
    }
    return "active";
  }

  void timerCallback()
  {
    bool assembly_ready = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      assembly_ready = assembly_deadline_ &&
        std::chrono::steady_clock::now() >= *assembly_deadline_;
    }
    if (assembly_ready) {
      finishAssembly();
    }
    double current_time = 0.0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!observation_) {
        return;
      }
      current_time = observationTimeNowLocked();
    }
    Eigen::VectorXd seed;
    if (maybeStartCoveragePreview(seed)) {
      startCoveragePreview(std::move(seed));
      return;
    }
    if (maybeStartPlanning(current_time, seed)) {
      startPlanning(std::move(seed));
    }

    ocs2_msgs::msg::MpcTargetTrajectories target;
    Eigen::VectorXd current_reference;
    std::vector<Eigen::VectorXd> active_reference_states;
    Phase current_phase;
    double current_progress = 0.0;
    double current_progress_rate = 0.0;
    double current_lag_error = 0.0;
    double current_contouring_error = 0.0;
    double current_force = 0.0;
    double current_force_offset = 0.0;
    double current_force_progress_scale = 1.0;
    std::string current_force_state;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!observation_) {
        return;
      }
      updateForceSensorSafetyLocked();
      updateForceSafetyRetreatLocked();
      updateVirtualProgressLocked(current_time);
      const double horizon = get_parameter("reference_horizon").as_double();
      const double step = get_parameter("reference_dt").as_double();
      const int count = std::max(2, static_cast<int>(std::ceil(horizon / step)) + 1);
      double yaw_reference = observation_->state.value[2];
      for (int i = 0; i < count; ++i) {
        const double sample_time = current_time + i * step;
        auto [state, input] = sampleReferenceLocked(sample_time, yaw_reference);
        if (state.size() != 9 || input.size() != 8) {
          throw std::runtime_error(
                  "WipePlanner produced a non-whole-body MPC reference: expected 9D state and 8D input");
        }
        yaw_reference = state[2];
        if (i == 0) {
          current_reference = state;
          last_commanded_reference_ = state;
        }
        active_reference_states.push_back(state);
        target.time_trajectory.push_back(sample_time);
        ocs2_msgs::msg::MpcState state_message;
        state_message.value = toFloatVector(state);
        target.state_trajectory.push_back(std::move(state_message));
        ocs2_msgs::msg::MpcInput input_message;
        input_message.value = toFloatVector(input);
        target.input_trajectory.push_back(std::move(input_message));
      }
      current_phase = phase_;
      current_progress = virtual_progress_;
      current_progress_rate = virtual_progress_rate_;
      current_lag_error = progress_lag_error_;
      current_contouring_error = contouring_error_;
      current_force = admittance_->measuredForce();
      current_force_offset = force_contact_plane_offset_ + admittance_->offset();
      current_force_progress_scale = force_progress_scale_;
      current_force_state = forceStateLocked();
    }
    target_publisher_->publish(target);
    if (current_reference.size() >= 9) {
      std_msgs::msg::Float64MultiArray arm_reference_message;
      arm_reference_message.data.reserve(6);
      for (Eigen::Index index = 3; index < 9; ++index) {
        arm_reference_message.data.push_back(current_reference[index]);
      }
      contact_arm_reference_publisher_->publish(arm_reference_message);
    }
    std_msgs::msg::String phase_message;
    phase_message.data = phaseName(current_phase);
    phase_publisher_->publish(phase_message);
    std_msgs::msg::String force_state_message;
    force_state_message.data = current_force_state;
    force_state_publisher_->publish(force_state_message);
    if (current_phase == Phase::WIPING) {
      for (const auto & item : std::vector<std::pair<
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, double>>{
          {progress_publisher_, current_progress},
          {progress_rate_publisher_, current_progress_rate},
          {lag_error_publisher_, current_lag_error},
          {contouring_error_publisher_, current_contouring_error},
          {force_progress_scale_publisher_, current_force_progress_scale}})
      {
        std_msgs::msg::Float64 message;
        message.data = item.second;
        item.first->publish(message);
      }
      if (current_time - last_progress_log_ >= 2.0) {
        last_progress_log_ = current_time;
        RCLCPP_INFO(get_logger(),
          "Path progress: tau=%.2f s, tau_dot=%.2f, projected_tau=%.2f s, "
          "lag=%.2f s, contour=%.3f m, force=%.2f N, offset=%.2f mm, "
          "force_scale=%.2f, force_state=%s",
          current_progress, current_progress_rate,
          current_progress - current_lag_error,
          current_lag_error, current_contouring_error, current_force,
          1000.0 * current_force_offset, current_force_progress_scale,
          current_force_state.c_str());
      }
      publishActiveReference(active_reference_states, current_time);
    }
    publishTrackingErrors(current_reference, current_time, current_phase);
  }

  void publishTrackingErrors(const Eigen::VectorXd & reference, double current_time,
                             Phase current_phase)
  {
    Eigen::VectorXd measured;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!observation_) {return;}
      measured = toEigen(observation_->state.value);
    }
    const double base_error = (measured.head<2>() - reference.head<2>()).norm();
    const double joint_error =
      (measured.tail<6>() - reference.tail<6>()).cwiseAbs().maxCoeff();
    const double ee_error = (planner_->framePosition(measured,
      get_parameter("ee_frame").as_string()) - planner_->framePosition(
      reference, get_parameter("ee_frame").as_string())).norm();
    for (const auto & item : std::vector<std::pair<
      rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, double>>{
        {base_error_publisher_, base_error},
        {joint_error_publisher_, joint_error},
        {ee_error_publisher_, ee_error}})
    {
      std_msgs::msg::Float64 message;
      message.data = item.second;
      item.first->publish(message);
    }
    if (current_phase == Phase::WIPING && current_time - last_tracking_log_ >= 2.0) {
      last_tracking_log_ = current_time;
      const std::string ee_frame = get_parameter("ee_frame").as_string();
      const Eigen::Vector3d reference_tool_z =
        planner_->frameRotation(reference, ee_frame).col(2);
      const Eigen::Vector3d measured_tool_z =
        planner_->frameRotation(measured, ee_frame).col(2);
      const Eigen::Vector3d reference_ee = planner_->framePosition(reference, ee_frame);
      const Eigen::Vector3d measured_ee = planner_->framePosition(measured, ee_frame);
      const Eigen::Vector3d wall_direction = -planner_->surfaceNormal();
      RCLCPP_INFO(get_logger(), "MPC tracking: base=%.3f m, joint=%.3f rad, EE=%.3f m",
                  base_error, joint_error, ee_error);
      RCLCPP_INFO(get_logger(),
        "Whole-body joints ref=[%.2f %.2f %.2f %.2f %.2f %.2f] "
        "meas=[%.2f %.2f %.2f %.2f %.2f %.2f]",
        reference[3], reference[4], reference[5], reference[6], reference[7], reference[8],
        measured[3], measured[4], measured[5], measured[6], measured[7], measured[8]);
      RCLCPP_INFO(get_logger(),
        "tool0 +Z ref=[%.2f %.2f %.2f] meas=[%.2f %.2f %.2f] "
        "wall-direction=[%.2f %.2f %.2f]",
        reference_tool_z.x(), reference_tool_z.y(), reference_tool_z.z(),
        measured_tool_z.x(), measured_tool_z.y(), measured_tool_z.z(),
        wall_direction.x(), wall_direction.y(), wall_direction.z());
      RCLCPP_INFO(get_logger(),
        "tool0 position ref=[%.3f %.3f %.3f] meas=[%.3f %.3f %.3f], "
        "surface-normal gap=%.2f mm",
        reference_ee.x(), reference_ee.y(), reference_ee.z(),
        measured_ee.x(), measured_ee.y(), measured_ee.z(),
        1000.0 * planner_->surfaceNormal().dot(
          measured_ee - planner_->surfaceCenter()));
    }
  }

  visualization_msgs::msg::Marker lineMarker(
    int id, const std::string & ns, double scale,
    float red, float green, float blue, float alpha) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = get_parameter("world_frame").as_string();
    marker.header.stamp = now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = alpha;
    return marker;
  }

  static geometry_msgs::msg::Point point(const Eigen::Vector3d & value)
  {
    geometry_msgs::msg::Point result;
    result.x = value.x(); result.y = value.y(); result.z = value.z();
    return result;
  }

  void publishActiveReference(const std::vector<Eigen::VectorXd> & states,
                              double current_time)
  {
    if (states.empty()) {
      return;
    }
    const double rate = std::max(
      0.5, get_parameter("active_visualization_rate").as_double());
    if (current_time - last_active_visualization_time_ < 1.0 / rate) {
      return;
    }
    last_active_visualization_time_ = current_time;

    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = get_parameter("world_frame").as_string();
    clear.header.stamp = now();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    const auto active_header = clear.header;
    markers.markers.push_back(std::move(clear));
    auto base_line = lineMarker(
      0, "active_reference_base", 0.045, 0.05F, 0.85F, 1.0F, 1.0F);
    auto ee_line = lineMarker(
      0, "active_reference_ee", 0.035, 1.0F, 0.1F, 0.85F, 1.0F);
    for (const auto & state : states) {
      base_line.points.push_back(point(Eigen::Vector3d(state[0], state[1], 0.06)));
      ee_line.points.push_back(point(planner_->framePosition(
        state, get_parameter("ee_frame").as_string())));
    }
    markers.markers.push_back(std::move(base_line));
    markers.markers.push_back(std::move(ee_line));

    const int requested = std::max(
      1, static_cast<int>(get_parameter("active_visualization_snapshots").as_int()));
    const int count = std::min<int>(requested, states.size());
    int marker_id = 100;
    for (int snapshot = 0; snapshot < count; ++snapshot) {
      const std::size_t index = static_cast<std::size_t>(std::llround(
        static_cast<double>(snapshot) * (states.size() - 1) /
        std::max(1, count - 1)));
      for (const auto & geometry : planner_->visualGeometry(states[index])) {
        visualization_msgs::msg::Marker marker;
        marker.header = active_header;
        marker.ns = "active_reference_robot";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.mesh_resource = geometry.mesh_path;
        marker.mesh_use_embedded_materials = false;
        marker.pose.position = point(geometry.position);
        marker.pose.orientation.x = geometry.orientation.x();
        marker.pose.orientation.y = geometry.orientation.y();
        marker.pose.orientation.z = geometry.orientation.z();
        marker.pose.orientation.w = geometry.orientation.w();
        marker.scale.x = geometry.mesh_scale.x();
        marker.scale.y = geometry.mesh_scale.y();
        marker.scale.z = geometry.mesh_scale.z();
        marker.color.r = 0.05F;
        marker.color.g = 0.80F;
        marker.color.b = 1.0F;
        marker.color.a = 0.28F;
        markers.markers.push_back(std::move(marker));
      }
    }
    active_reference_publisher_->publish(markers);
  }

  void publishVisualization(const std::vector<Waypoint> & trajectory)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = get_parameter("world_frame").as_string();
    path.header.stamp = now();
    nav_msgs::msg::Path ee_path;
    ee_path.header = path.header;
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker delete_all;
    delete_all.header = path.header;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(delete_all);
    auto base_line = lineMarker(0, "base_path", 0.035, 0.55F, 0.55F, 0.60F, 1.0F);
    auto ee_line = lineMarker(0, "constrained_ee_path", 0.025, 0.1F, 1.0F, 0.25F, 1.0F);
    auto contact_line = lineMarker(0, "contact_path", 0.018, 1.0F, 0.2F, 0.1F, 1.0F);
    std::vector<std::size_t> contact_indices;
    for (std::size_t i = 0; i < trajectory.size(); ++i) {
      const auto & waypoint = trajectory[i];
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position.x = waypoint.state[0];
      pose.pose.position.y = waypoint.state[1];
      pose.pose.orientation.z = std::sin(0.5 * waypoint.state[2]);
      pose.pose.orientation.w = std::cos(0.5 * waypoint.state[2]);
      path.poses.push_back(pose);
      base_line.points.push_back(point(Eigen::Vector3d(
        waypoint.state[0], waypoint.state[1], 0.04)));
      const auto ee_position = planner_->framePosition(
        waypoint.state, get_parameter("ee_frame").as_string());
      if (waypoint.in_contact) {
        ee_line.points.push_back(point(ee_position));
        contact_line.points.push_back(point(waypoint.contact_target));
        contact_indices.push_back(i);
        geometry_msgs::msg::PoseStamped ee_pose;
        ee_pose.header = ee_path.header;
        ee_pose.pose.position = point(ee_position);
        const Eigen::Quaterniond ee_orientation(planner_->frameRotation(
          waypoint.state, get_parameter("ee_frame").as_string()));
        ee_pose.pose.orientation.x = ee_orientation.x();
        ee_pose.pose.orientation.y = ee_orientation.y();
        ee_pose.pose.orientation.z = ee_orientation.z();
        ee_pose.pose.orientation.w = ee_orientation.w();
        ee_path.poses.push_back(std::move(ee_pose));
      }
    }
    markers.markers.push_back(std::move(base_line));
    markers.markers.push_back(std::move(ee_line));
    markers.markers.push_back(std::move(contact_line));

    if (!trajectory.empty()) {
      // Show the exact base component copied from the first whole-body state
      // into REMANI's whole-body goal. The green sphere below is the end-effector contact
      // point on the wall and must not be interpreted as a mobile-base goal.
      visualization_msgs::msg::Marker navigation_start;
      navigation_start.header = path.header;
      navigation_start.ns = "trajectory_front_base_goal";
      navigation_start.id = 0;
      navigation_start.type = visualization_msgs::msg::Marker::ARROW;
      navigation_start.action = visualization_msgs::msg::Marker::ADD;
      navigation_start.pose.position.x = trajectory.front().state[0];
      navigation_start.pose.position.y = trajectory.front().state[1];
      navigation_start.pose.position.z = 0.08;
      navigation_start.pose.orientation.z =
        std::sin(0.5 * trajectory.front().state[2]);
      navigation_start.pose.orientation.w =
        std::cos(0.5 * trajectory.front().state[2]);
      navigation_start.scale.x = 0.32;
      navigation_start.scale.y = 0.08;
      navigation_start.scale.z = 0.08;
      navigation_start.color.r = 0.05F;
      navigation_start.color.g = 0.55F;
      navigation_start.color.b = 1.0F;
      navigation_start.color.a = 1.0F;
      markers.markers.push_back(std::move(navigation_start));
    }

    if (!contact_indices.empty()) {
      for (int endpoint = 0; endpoint < 2; ++endpoint) {
        const std::size_t selected = endpoint == 0 ?
          contact_indices.front() : contact_indices.back();
        visualization_msgs::msg::Marker marker;
        marker.header = path.header;
        marker.ns = endpoint == 0 ? "wipe_start" : "wipe_end";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = point(trajectory[selected].contact_target);
        marker.pose.orientation.w = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.075;
        marker.color.r = endpoint == 0 ? 0.1F : 1.0F;
        marker.color.g = endpoint == 0 ? 1.0F : 0.1F;
        marker.color.b = 0.1F;
        marker.color.a = 1.0F;
        markers.markers.push_back(std::move(marker));
      }
    }

    // Complete meshes are expensive in RViz. Publish a bounded, evenly-spaced
    // set once (transient-local), instead of hundreds of meshes every timer tick.
    const int requested = std::max(
      2, static_cast<int>(get_parameter("visualization_snapshots").as_int()));
    const int snapshot_count = std::min<int>(requested, contact_indices.size());
    const double alpha = get_parameter("visualization_alpha").as_double();
    int marker_id = 100;
    for (int snapshot = 0; snapshot < snapshot_count; ++snapshot) {
      const std::size_t selected = contact_indices[static_cast<std::size_t>(std::llround(
        static_cast<double>(snapshot) * (contact_indices.size() - 1) /
        std::max(1, snapshot_count - 1)))];
      for (const auto & geometry : planner_->visualGeometry(trajectory[selected].state)) {
        visualization_msgs::msg::Marker marker;
        marker.header = path.header;
        marker.ns = "robot_snapshots";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.mesh_resource = geometry.mesh_path;
        marker.mesh_use_embedded_materials = true;
        marker.pose.position.x = geometry.position.x();
        marker.pose.position.y = geometry.position.y();
        marker.pose.position.z = geometry.position.z();
        marker.pose.orientation.x = geometry.orientation.x();
        marker.pose.orientation.y = geometry.orientation.y();
        marker.pose.orientation.z = geometry.orientation.z();
        marker.pose.orientation.w = geometry.orientation.w();
        marker.scale.x = geometry.mesh_scale.x();
        marker.scale.y = geometry.mesh_scale.y();
        marker.scale.z = geometry.mesh_scale.z();
        marker.color.a = alpha;
        markers.markers.push_back(std::move(marker));
      }
      const Eigen::Vector3d tool_position = planner_->framePosition(
        trajectory[selected].state, get_parameter("ee_frame").as_string());
      const Eigen::Vector3d tool_z = planner_->frameRotation(
        trajectory[selected].state, get_parameter("ee_frame").as_string()).col(2);
      visualization_msgs::msg::Marker tool_axis;
      tool_axis.header = path.header;
      tool_axis.ns = "planned_tool0_z_axis";
      tool_axis.id = marker_id++;
      tool_axis.type = visualization_msgs::msg::Marker::ARROW;
      tool_axis.action = visualization_msgs::msg::Marker::ADD;
      tool_axis.points.push_back(point(tool_position));
      tool_axis.points.push_back(point(tool_position + 0.20 * tool_z));
      tool_axis.scale.x = 0.015;
      tool_axis.scale.y = 0.035;
      tool_axis.scale.z = 0.050;
      tool_axis.color.r = 0.05F;
      tool_axis.color.g = 0.25F;
      tool_axis.color.b = 1.0F;
      tool_axis.color.a = 1.0F;
      markers.markers.push_back(std::move(tool_axis));
    }
    path_publisher_->publish(path);
    ee_path_publisher_->publish(ee_path);
    marker_publisher_->publish(markers);
    RCLCPP_INFO(get_logger(), "RViz: published %d complete robot snapshots (%zu markers total)",
                snapshot_count, markers.markers.size());
  }

  std::unique_ptr<Planner> planner_;
  std::unique_ptr<ForceAdmittance> admittance_;
  mutable std::mutex mutex_;
  std::optional<ocs2_msgs::msg::MpcObservation> observation_;
  rclcpp::Time observation_ros_time_{0, 0, RCL_ROS_TIME};
  std::map<uint32_t, PolynomialTraj> sections_;
  std::optional<std::chrono::steady_clock::time_point> assembly_deadline_;
  std::shared_ptr<RemaniTrajectory> remani_;
  std::optional<Eigen::VectorXd> task_start_state_;
  std::vector<Waypoint> wipe_trajectory_;
  std::vector<Waypoint> preview_trajectory_;
  PlanReport preview_report_;
  double wipe_start_time_{0.0};
  double wipe_contact_start_time_{0.0};
  double virtual_progress_{0.0};
  double projected_progress_{0.0};
  double progress_lag_error_{0.0};
  double contouring_error_{0.0};
  double virtual_progress_rate_{0.0};
  double progress_anchor_time_{0.0};
  double last_wipe_clock_time_{0.0};
  double last_active_visualization_time_{-1.0e9};
  double last_progress_log_{-1.0e9};
  Phase phase_{Phase::WAITING};
  std::optional<double> arrival_since_;
  std::thread planning_thread_;
  bool preview_started_{false};
  std::atomic_bool shutting_down_{false};
  std::atomic_bool remani_task_requested_{false};
  std::atomic_bool remani_task_confirmed_{false};
  bool force_hard_stop_{false};
  bool force_sensor_stale_{false};
  bool force_message_received_{false};
  bool force_contact_detected_{false};
  bool force_control_enabled_{true};
  double force_contact_plane_offset_{0.0};
  rclcpp::Time last_force_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_force_wall_time_{};
  std::optional<rclcpp::Time> force_in_tolerance_since_;
  std::optional<rclcpp::Time> force_pause_since_;
  std::optional<rclcpp::Time> force_resume_since_;
  bool initial_force_settled_{false};
  bool force_motion_paused_{false};
  double force_progress_scale_{0.0};
  std::optional<Eigen::VectorXd> force_safety_anchor_;
  std::optional<Eigen::VectorXd> last_commanded_reference_;
  double force_safety_retreat_offset_{0.0};
  bool force_safety_retreat_initialized_{false};
  std::chrono::steady_clock::time_point force_safety_retreat_wall_time_{};
  double last_tracking_log_{-1.0e9};
  double last_navigation_log_{-1.0e9};

  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr target_publisher_;
  rclcpp::Publisher<traj_utils::msg::WholeBodyGoal>::SharedPtr whole_body_goal_publisher_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_subscription_;
  rclcpp::Subscription<PolynomialTraj>::SharedPtr trajectory_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ee_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    active_reference_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr force_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr offset_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
    force_progress_scale_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr force_state_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
    contact_arm_reference_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr base_error_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr joint_error_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr ee_error_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr progress_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr progress_rate_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lag_error_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr contouring_error_publisher_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr remani_task_client_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr force_control_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace wipe_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<wipe_planner::WipePlannerNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("wipe_planner"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
