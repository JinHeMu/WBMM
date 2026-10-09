/*
 * tracer_base_ros.hpp
 *
 * Created on: 3 2, 2022 16:38
 * Description:
 *
 * Copyright (c) 2022 Agilex Robot Pte. Ltd.
 */

#ifndef TRACER_BASE_ROS_HPP
#define TRACER_BASE_ROS_HPP

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "ugv_sdk/mobile_robot/tracer_robot.hpp"

namespace westonrobot {
class TracerBaseRos : public rclcpp::Node {
 public:
  TracerBaseRos(std::string node_name);

  bool Initialize();
  void Run();
  void Stop();

 private:
  std::string port_name_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string odom_topic_name_;

  bool is_tracer_mini_ = false;
  bool publish_odom_tf_ = true;
  // bool is_omni_wheel_ = false;

  bool simulated_robot_ = false;
  int sim_control_rate_ = 50;


  std::shared_ptr<TracerRobot> robot_;
  // std::shared_ptr<TracerMiniOmniRobot> omni_robot_;

  rclcpp::executors::SingleThreadedExecutor executor_;
  double state_publish_rate_{50.0};

  void LoadParameters();
};
}  // namespace westonrobot

#endif /* SCOUT_BASE_ROS_HPP */
