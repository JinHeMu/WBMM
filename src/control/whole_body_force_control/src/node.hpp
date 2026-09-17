#pragma once

#include "whole_body_force_control/controllers.hpp"
#include "whole_body_force_control/force_processor.hpp"
#include "wbmm_pinocchio/pinocchio_robot_model.hpp"
#include "wbmm_pinocchio/whole_body_kinematics.hpp"

#include <wbmm_core/wbmm_core.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <Eigen/Core>

#include <chrono>
#include <cstddef>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace whole_body_force_control
{

using wbmm::pinocchio::PinocchioRobotModel;
using wbmm::pinocchio::WholeBodyKinematics;

// Reference-side six-axis admittance controller.
//
// The force sensor reports in sensor_frame.  The node transforms the measured
// wrench to the nominal TCP frame (tool0), solves admittance there, then maps
// the resulting correction into state_frame before publishing it to MPC.
class WholeBodyForceControlNode final : public rclcpp::Node
{
public:
  WholeBodyForceControlNode();

private:
  struct Parameters
  {
    std::string urdf_file;
    std::string robot_name;
    std::string state_frame;
    std::string sensor_frame;
    std::string tcp_frame;
    std::string target_topic;
    std::string correction_topic;
    std::string state_topic;
    std::string wrench_topic;

    bool admittance_enabled{false};
    bool tf_fallback_to_latest{true};
    bool reference_output_enabled{false};
    bool enforce_single_target_owner{true};

    AxisMask6d admittance_axes{};
    Vector6d mass{Vector6d::Zero()};
    Vector6d damping{Vector6d::Zero()};
    Vector6d stiffness{Vector6d::Zero()};
    Vector6d max_velocity{Vector6d::Zero()};

    double filter_alpha{0.25};
    double tf_lookup_timeout{0.05};
    Vector6d wrench_scale{Vector6d::Ones()};
    Vector6d hard_wrench_limit{Vector6d::Ones()};
    double hard_force_norm_limit{20.0};
    std::size_t tare_samples{50};

    double loop_rate{50.0};
    double force_timeout{0.25};
    double observation_timeout{0.25};
    double capture_settle_time{1.0};
    double base_share{0.4};
    double max_base_velocity{0.5};
    double max_joint_velocity{1.0};
    double reference_horizon{1.0};
    double reference_dt{0.1};
    int input_dimension{8};
  };

  void loadParameters();
  void configureForceProcessor();
  void createRosInterfaces();
  static std::string enabledAxes(const AxisMask6d &mask);

  void observationCallback(
      const ocs2_msgs::msg::MpcObservation::SharedPtr message);
  void wrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr message);
  geometry_msgs::msg::TransformStamped lookupTransformWithFallback(
      const std::string &target_frame, const std::string &source_frame,
      const builtin_interfaces::msg::Time &stamp,
      const std::string &context);
  Vector6d measuredWrenchVector() const;
  Eigen::VectorXd observationStateLocked() const;
  bool foreignTargetPublisherPresent() const;
  void requestFault(const std::string &reason);

  void latchFault(const std::string &reason);
  void publishHoldReference();
  void publishState(const std::string &state);
  void checkFaults(bool observation_timed_out, bool wrench_timed_out);
  void captureNominalState(const Eigen::VectorXd &measured_state);
  Vector6d correctionFromReferencePose(
      const Eigen::VectorXd &reference) const;
  void updateReference(
      const Vector6d &measured_wrench,
      double dt,
      Vector6d &correction,
      Vector6d &filtered_wrench,
      Eigen::VectorXd &reference,
      double &primary_offset,
      double &primary_force);
  void update();
  void publishReference(const Eigen::VectorXd &reference);
  void publishCorrection(
      const Eigen::VectorXd &reference,
      const Eigen::VectorXd &measured_state,
      double primary_force,
      double primary_offset,
      const Vector6d &filtered_wrench,
      const Vector6d &correction);

  // Configuration.
  Parameters parameters_;

  // Robot model and control algorithms.
  wbmm::core::RobotModelPtr robot_model_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<WholeBodyKinematics> kinematics_;
  std::unique_ptr<CartesianComplianceController> cartesian_controller_;
  ForceProcessor force_processor_;

  // Input cache.
  mutable std::mutex mutex_;
  wbmm::core::WholeBodyState observation_state_{};
  double observation_time_{0.0};
  wbmm::core::Wrench measured_wrench_core_{};
  bool observation_received_{false};
  bool wrench_received_{false};
  std::chrono::steady_clock::time_point last_update_;
  std::chrono::steady_clock::time_point last_wrench_;
  std::chrono::steady_clock::time_point last_observation_;
  std::chrono::steady_clock::time_point capture_requested_at_;

  // Control state.
  bool fault_latched_{false};
  bool hold_state_valid_{false};
  bool nominal_captured_{false};
  bool pending_fault_{false};
  std::string fault_reason_;
  std::string pending_fault_reason_;
  std::string last_state_;
  Eigen::Vector3d nominal_tcp_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d nominal_tcp_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::VectorXd nominal_state_;
  Eigen::VectorXd last_reference_state_;
  Eigen::VectorXd hold_state_;

  // ROS resources.
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr
      target_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      correction_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr
      observation_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      wrench_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace whole_body_force_control
