#pragma once

#include "wbmm_robot_model/collision_sphere_model.hpp"
#include "wbmm_robot_model/robot_description.hpp"
#include "wbmm_robot_model/robot_description_loader.hpp"
#include "wbmm_robot_model/robot_model_config.hpp"
#include "wbmm_robot_model/robot_model_config_loader.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace wbmm::robot_model
{

// ============================================================================
// RobotModelDescription —— URDF、模型配置和碰撞球的组合。
//
//   一次加载，三类消费者共享：
//     description       URDF 的完整只读内容（link/joint/visual/collision）
//     config            控制/锁定/末端/分组等配置
//     collision_spheres 共享碰撞球模型
//
//   content_id 把 URDF 文本、模型配置、碰撞球策略一起哈希，作为模型版本。
//   同进程内以 shared_ptr<const RobotModelDescription> 共享即可；
//   运行时可变数据（FK 缓存、Data）不放在这里，见 wbmm_pinocchio 的
//   KinematicsData。
// ============================================================================

struct RobotModelDescription
{
  RobotDescription description;
  RobotModelConfig config;
  CollisionSphereModel collision_spheres;

  std::uint64_t content_id{0};

  [[nodiscard]] std::string contentIdHex() const;
};

using RobotModelDescriptionPtr = std::shared_ptr<const RobotModelDescription>;

struct RobotModelDescriptionResult
{
  bool success{false};
  std::string message;
  RobotModelDescription model;
};

// 校验配置并构建碰撞球；配置非法时返回失败。
[[nodiscard]] RobotModelDescriptionResult buildRobotModelDescription(
  const RobotDescription & description, const RobotModelConfig & config);

}  // namespace wbmm::robot_model
