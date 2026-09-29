#pragma once

#include "wbmm_robot_model/robot_model_config.hpp"

#include <string>

namespace wbmm::robot_model
{

struct RobotModelConfigLoadResult
{
  bool success{false};
  std::string message;
  RobotModelConfig config;
};

// 机器人模型配置的统一读取入口（yaml-cpp，ROS-free）。
//
// YAML 形如：
//   state_base_frame: base_footprint
//   controlled_joints: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6]
//   locked_joints: {wheel_left: 0.0}
//   end_effectors: {tool: tool0}
//   base_collision_link: base_link
//   collision_groups: {arm: [Link_1, Link_2]}
//
// 任何字段缺省时回落到 RobotModelConfig::defaultsFor(description)。
class RobotModelConfigLoader
{
public:
  [[nodiscard]] static RobotModelConfigLoadResult fromFile(
    const std::string & yaml_path, const RobotDescription & description);

  [[nodiscard]] static RobotModelConfigLoadResult fromYaml(
    const std::string & yaml, const RobotDescription & description,
    const std::string & source = "<yaml>");
};

}  // namespace wbmm::robot_model
