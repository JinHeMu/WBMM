// Copyright (c) 2022, Stogl Robotics Consulting UG (haftungsbeschränkt) (template)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JAKA_HARDWARE_INTERFACE__JAKA_HARDWARE_INTERFACE_HPP_
#define JAKA_HARDWARE_INTERFACE__JAKA_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

// JAKA SDK Headers
#include "jaka_driver/JAKAZuRobot.h"
#include "jaka_driver/jktypes.h"

namespace jaka_hardware_interface
{
class JakaHardwareInterface : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  // JAKA Robot Object
  JAKAZuRobot robot_;

  // IP Config
  std::string robot_ip_;
  std::string local_ip_; // PC IP for EDG UDP
  bool hardware_write_ = false; // true: enable servo and write commands; false: read-only EDG telemetry

  // Data storage
  EDGState edg_state_;        // EDG 全量状态
  JointValue joint_cmd_;      // 发送给 EDG 的指令

  // States (Position, Velocity)
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;

  // Raw force/torque states.
  // The hardware interface only exposes the raw values from EDG.
  // Zeroing, coordinate transform, filtering, deadband and stale detection
  // are intentionally handled by the separate force process.
  std::vector<double> hw_fts_states_;

  // Commands (Position only as per URDF)
  std::vector<double> hw_position_commands_;
};

}  // namespace jaka_hardware_interface

#endif  // JAKA_HARDWARE_INTERFACE__JAKA_HARDWARE_INTERFACE_HPP_
