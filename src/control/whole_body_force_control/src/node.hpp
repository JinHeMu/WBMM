#pragma once

#include "whole_body_force_control/controllers.hpp"
#include "whole_body_force_control/force_processor.hpp"
#include "wbmm_pinocchio/pinocchio_robot_model.hpp"
#include "wbmm_pinocchio/whole_body_kinematics.hpp"

#include <wbmm_core/wbmm_core.hpp>

#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

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

// Reference-side compliance node.  The default "legacy" axis selection keeps
// the previous scalar force_axis + response_body behavior byte-for-byte at the
// public interface.  Explicit admittance_axes switches to signed 6D wrench and
// full-pose IK in the nominal end-effector frame.
class WholeBodyForceControlNode final : public rclcpp::Node
{
public:
  WholeBodyForceControlNode();

private:
  struct Parameters
  {
    std::string urdf_file;
    std::string ee_frame;
    std::string state_frame;
    std::string robot_name;
    std::string control_mode;
    std::string force_axis;
    std::size_t force_axis_index{0};
    std::string target_topic;
    std::string status_topic;
    std::string control_state_topic;
    std::string wrench_topic;

    bool absolute_force{false};
    bool force_velocity_mode{false};
    bool cartesian_mode{false};
    bool require_wrench_frame{false};
    bool armed{false};
    bool reference_output_enabled{false};
    bool enforce_single_target_owner{true};

    double configured_desired_force{12.0};
    double mass{3.0};
    double damping{45.0};
    double stiffness{150.0};
    double max_offset{0.08};
    double max_velocity{0.035};
    double filter_alpha{0.25};
    double force_deadband{0.0};
    double loop_rate{50.0};
    double reference_horizon{1.0};
    double reference_dt{0.1};
    int input_dimension{8};
    double base_share{0.4};
    double max_base_delta{0.04};
    double max_joint_delta{0.25};
    double force_timeout{0.25};
    double observation_timeout{0.25};
    double capture_settle_time{1.0};
    double force_scale{1.0};
    std::size_t tare_samples{50};

    Eigen::Vector3d response_body{Eigen::Vector3d::UnitX()};
    AxisMask6d admittance_axes{};
    AxisMask6d constant_force_axes{};
    AxisMask6d absolute_wrench_axes{};
    Vector6d desired_wrench{Vector6d::Zero()};
    Vector6d mass_6d{Vector6d::Zero()};
    Vector6d damping_6d{Vector6d::Zero()};
    Vector6d stiffness_6d{Vector6d::Zero()};
    Vector6d max_offset_6d{Vector6d::Zero()};
    Vector6d max_velocity_6d{Vector6d::Zero()};
    Vector6d filter_alpha_6d{Vector6d::Zero()};
    Vector6d wrench_scale_6d{Vector6d::Ones()};
    Vector6d hard_wrench_limit{Vector6d::Ones()};
    Vector6d max_wrench_rate{Vector6d::Zero()};
  };

  void loadParameters();
  void configureForceProcessor();
  void createRosInterfaces();
  static std::string enabledAxes(const AxisMask6d &mask);

  void observationCallback(
      const ocs2_msgs::msg::MpcObservation::SharedPtr message);
  void wrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr message);
  Vector6d measuredWrenchVector() const;
  bool getWrenchTransform(
      const geometry_msgs::msg::WrenchStamped &message,
      Eigen::Matrix3d &rotation,
      Eigen::Vector3d &translation);
  Eigen::VectorXd observationStateLocked() const;
  bool foreignTargetPublisherPresent() const;
  void requestFault(const std::string &reason);

  void latchFault(const std::string &reason);
  void publishControlState(const std::string &state);
  void checkFaults(bool observation_timed_out, bool wrench_timed_out);
  void captureNominalState(const Eigen::VectorXd &measured_state);
  void updateCartesianReference(
      const Vector6d &measured_wrench,
      double dt,
      Vector6d &correction,
      Vector6d &filtered_wrench,
      Eigen::VectorXd &reference,
      double &primary_offset,
      double &primary_force);
  void updateLegacyReference(
      double dt,
      Vector6d &correction,
      Vector6d &filtered_wrench,
      Eigen::VectorXd &reference,
      double &primary_offset,
      double &primary_force);
  void update();
  void publishReference(const Eigen::VectorXd &reference);
  void publishStatus(
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
  std::unique_ptr<WholeBodyKinematics> kinematics_;
  std::unique_ptr<AdmittanceController> admittance_;
  std::unique_ptr<ForceFollower> force_follower_;
  std::unique_ptr<CartesianComplianceController> cartesian_controller_;
  ForceProcessor force_processor_;

  // Input cache.
  std::mutex mutex_;
  wbmm::core::WholeBodyState observation_state_{};
  double observation_time_{0.0};
  wbmm::core::Wrench measured_wrench_core_{};
  double measured_force_{0.0};
  bool observation_received_{false};
  bool wrench_received_{false};
  std::chrono::steady_clock::time_point last_update_;
  std::chrono::steady_clock::time_point last_wrench_;
  std::chrono::steady_clock::time_point last_observation_;
  std::chrono::steady_clock::time_point capture_requested_at_;

  // Control state.
  bool fault_latched_{false};
  bool nominal_captured_{false};
  bool pending_fault_{false};
  std::string fault_reason_;
  std::string pending_fault_reason_;
  std::string last_control_state_;
  Eigen::Vector3d response_world_{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d nominal_ee_{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d nominal_ee_rotation_{Eigen::Matrix3d::Identity()};
  Eigen::VectorXd nominal_state_;

  // ROS resources.
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr
      target_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      status_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr control_state_publisher_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr
      observation_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      wrench_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace whole_body_force_control
