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

#include "jaka_hardware_interface/jaka_hardware_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <thread>

using namespace std;

namespace jaka_hardware_interface
{
  hardware_interface::CallbackReturn JakaHardwareInterface::on_init(
      const hardware_interface::HardwareInfo &info)
  {
    if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
      return CallbackReturn::ERROR;
    }

    // 1. 获取 Robot IP
    auto it = info_.hardware_parameters.find("robot_ip");
    if (it != info_.hardware_parameters.end())
    {
      robot_ip_ = it->second;
    }
    else
    {
      RCLCPP_FATAL(rclcpp::get_logger("JakaHardwareInterface"), "Parameter'robot_ip' not set");
      return CallbackReturn::ERROR;
    }

    // 2. 获取 Local IP (EDG 必需)
    auto it_local = info_.hardware_parameters.find("local_ip");
    if (it_local != info_.hardware_parameters.end())
    {
      local_ip_ = it_local->second;
    }
    else
    {
      RCLCPP_FATAL(rclcpp::get_logger("JakaHardwareInterface"), "Parameter'local_ip' not set (Required for EDG)");
      return CallbackReturn::ERROR;
    }

    // Real-motion gate.  The interface always logs in and reads EDG data.
    // When hardware_write is false it never enables servo mode or sends joint
    // commands.
    auto it_hardware_write = info_.hardware_parameters.find("hardware_write");
    if (it_hardware_write != info_.hardware_parameters.end())
    {
      const auto &value = it_hardware_write->second;
      hardware_write_ = value == "true" || value == "True" || value == "1";
    }

    hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
    hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

    // Raw force/torque values are exposed through state interfaces.
    // All force processing is handled by a separate force process.
    hw_fts_states_.resize(6, 0.0);

    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"),
                "Jaka EDG Interface Init: Robot=%s, Local=%s, HardwareWrite=%s",
                robot_ip_.c_str(), local_ip_.c_str(), hardware_write_ ? "true" : "false");

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn JakaHardwareInterface::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Connecting to robot...");

    // 登录
    if (robot_.login_in(robot_ip_.c_str()) != ERR_SUCC)
    {
      RCLCPP_ERROR(rclcpp::get_logger("JakaHardwareInterface"), "Login failed!");
      return CallbackReturn::ERROR;
    }

    // 初始化 EDG 模式
    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Initializing EDG UDP Stream...");
    if (robot_.edg_init(true, local_ip_.c_str(), 10010, 0) != ERR_SUCC)
    {
      RCLCPP_ERROR(rclcpp::get_logger("JakaHardwareInterface"), "Failed to init EDG! Check firewall/IP.");
      return CallbackReturn::ERROR;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // 读取初始状态 (使用 EDG 接口)
    int retry_count = 0;
    while (true)
    {
      if (robot_.edg_get_stat(&edg_state_) == ERR_SUCC)
      {
        break;
      }
      retry_count++;
      if (retry_count > 20)
      {
        RCLCPP_ERROR(rclcpp::get_logger("JakaHardwareInterface"), "Timeout waiting for initial EDG data.");
        return CallbackReturn::ERROR;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 同步 Command 和 State，防止启动飞车
    for (size_t i = 0; i < info_.joints.size() && i < 6; ++i)
    {
      hw_position_states_[i] = edg_state_.jointVal.jVal[i];
      hw_velocity_states_[i] = edg_state_.jointVel.jVel[i]; // rad/s
      hw_position_commands_[i] = hw_position_states_[i]; // 初始指令 = 当前位置
      RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Joint %zu init pos: %.4f", i, hw_position_states_[i]);
    }

    return CallbackReturn::SUCCESS;
  }

  std::vector<hardware_interface::StateInterface> JakaHardwareInterface::export_state_interfaces()
  {
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
    }

    if (info_.sensors.size() > 0)
    {
      const auto &sensor = info_.sensors[0]; 

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, "force.x", &hw_fts_states_[0]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, "force.y", &hw_fts_states_[1]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, "force.z", &hw_fts_states_[2]));

      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, "torque.x", &hw_fts_states_[3]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, "torque.y", &hw_fts_states_[4]));
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          sensor.name, "torque.z", &hw_fts_states_[5]));

      RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"),
                  "Exported FT Sensor interfaces for: %s", sensor.name.c_str());
    }

    return state_interfaces;
  }

  std::vector<hardware_interface::CommandInterface> JakaHardwareInterface::export_command_interfaces()
  {
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for (size_t i = 0; i < info_.joints.size(); ++i)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
    }

    return command_interfaces;
  }

  hardware_interface::CallbackReturn JakaHardwareInterface::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Activating... (Ensuring EDG is running)");
    if (!hardware_write_)
    {
      RCLCPP_WARN(rclcpp::get_logger("JakaHardwareInterface"),
                  "hardware_write=false: servo mode will not be enabled and no joint commands will be sent.");
    }
    else
    {
      robot_.servo_move_enable(true); // 机器人上电/使能
    }

    // FTS raw data is intentionally not zeroed or processed here.
    // The separate force process owns tare, transform, filtering and safety.

    // 再次同步，因为从 Configure 到 Activate 可能有时间差
    robot_.edg_get_stat(&edg_state_);
    for (size_t i = 0; i < info_.joints.size() && i < 6; ++i)
    {
      hw_position_commands_[i] = edg_state_.jointVal.jVal[i];
    }

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::CallbackReturn JakaHardwareInterface::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    RCLCPP_INFO(rclcpp::get_logger("JakaHardwareInterface"), "Deactivating... Stopping EDG");

    if (hardware_write_)
    {
      robot_.servo_move_enable(false);
    }
    // 关闭 EDG 模式
    robot_.edg_init(false);

    return CallbackReturn::SUCCESS;
  }

  hardware_interface::return_type JakaHardwareInterface::read(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    // 使用 EDG 接口读取全量数据
    errno_t ret = robot_.edg_get_stat(&edg_state_);
    if (ret != ERR_SUCC)
    {
      // Keep the last measured values.  A separate force process owns FTS
      // validity/stale detection; this hardware interface no longer performs
      // any force processing.
      return hardware_interface::return_type::OK;
    }

    // A. 更新关节状态
    for (size_t i = 0; i < info_.joints.size() && i < 6; ++i)
    {
      hw_position_states_[i] = edg_state_.jointVal.jVal[i];
      hw_velocity_states_[i] = edg_state_.jointVel.jVel[i];
    }

    // B. Raw FTS passthrough.
    // No zeroing, no coordinate transform, no filtering, no deadband and no
    // stale detection here.  Those belong to the separate force process.
    if (hw_fts_states_.size() == 6)
    {
      hw_fts_states_[0] = edg_state_.torqSensor.fx;
      hw_fts_states_[1] = edg_state_.torqSensor.fy;
      hw_fts_states_[2] = edg_state_.torqSensor.fz;
      hw_fts_states_[3] = edg_state_.torqSensor.tx;
      hw_fts_states_[4] = edg_state_.torqSensor.ty;
      hw_fts_states_[5] = edg_state_.torqSensor.tz;
    }

    return hardware_interface::return_type::OK;
  }

  hardware_interface::return_type JakaHardwareInterface::write(
      const rclcpp::Time &, const rclcpp::Duration & /*period*/)
  {
    if (!hardware_write_)
    {
      return hardware_interface::return_type::OK;
    }

    // 准备指令数据
    for (size_t i = 0; i < info_.joints.size() && i < 6; ++i)
    {
      // NaN 检查
      if (std::isnan(hw_position_commands_[i]))
      {
        joint_cmd_.jVal[i] = hw_position_states_[i];
      }
      else
      {
        joint_cmd_.jVal[i] = hw_position_commands_[i];
      }
    }

    static std::array<double, 6> last_sent{};
    static bool inited = false;

    double eps = 1e-5; // 约0.0001 rad ≈ 0.0057°
    bool changed = false;
    for (int i = 0; i < 6; i++)
    {
      if (!inited || std::fabs(joint_cmd_.jVal[i] - last_sent[i]) > eps)
      {
        changed = true;
      }
    }

    if (changed)
    {
      robot_.edg_servo_j(&joint_cmd_, MoveMode::ABS, 1);
      for (int i = 0; i < 6; i++)
        last_sent[i] = joint_cmd_.jVal[i];
      inited = true;
    }
    
    return hardware_interface::return_type::OK;
  }
} // namespace jaka_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    jaka_hardware_interface::JakaHardwareInterface, hardware_interface::SystemInterface)
