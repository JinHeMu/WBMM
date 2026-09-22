#pragma once

#include "whole_body_force_control/force_processor.hpp"

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Core>

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>

namespace whole_body_force_control {

// Thin ROS adapter around ForceProcessor.  It owns sensor topics, TF lookup,
// preprocessing state and sensor faults; no admittance or reference generation
// is performed here.
class ForceSensorProcessorNode final : public rclcpp::Node {
public:
  ForceSensorProcessorNode();

private:
  struct Parameters {
    std::string raw_wrench_topic;
    std::string processed_wrench_topic;
    std::string state_topic;
    std::string sensor_frame;
    std::string tcp_frame;
    std::string load_base_frame;

    double tf_lookup_timeout{0.05};
    bool tf_fallback_to_latest{false};
    double raw_timeout{0.25};
    double monitor_rate{50.0};

    std::size_t tare_samples{50};
    Vector6d filter_alpha{Vector6d::Constant(0.25)};
    Vector6d wrench_scale{Vector6d::Ones()};
    Vector6d hard_wrench_limit{Vector6d::Constant(20.0)};
    double hard_force_norm_limit{20.0};
    double force_deadband_n{1.0};
    double torque_deadband_nm{0.1};
    LoadCompensationConfig load_compensation{};
  };

  void loadParameters();
  void configureProcessor();
  void rawWrenchCallback(
      const geometry_msgs::msg::WrenchStamped::SharedPtr message);
  void monitorRawInput();
  void resetProcessor();
  void latchFault(const std::string &reason);
  void publishState(const std::string &state);

  geometry_msgs::msg::TransformStamped lookupTransformWithFallback(
      const std::string &target_frame, const std::string &source_frame,
      const builtin_interfaces::msg::Time &stamp, const std::string &context);

  Parameters parameters_;
  ForceProcessor processor_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  bool raw_received_{false};
  bool fault_latched_{false};
  std::string last_state_;
  std::chrono::steady_clock::time_point last_raw_;

  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr
      processed_wrench_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      raw_wrench_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr monitor_timer_;
};

} // namespace whole_body_force_control
