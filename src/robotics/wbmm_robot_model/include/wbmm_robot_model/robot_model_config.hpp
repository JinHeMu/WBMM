#pragma once

// ============================================================================
// RobotModelConfig —— URDF 表达不了的机器人配置，收敛成一个显式值对象。
//
//   为什么需要它：URDF 只说"有哪些 link/joint"，不说
//     - 9D 状态里的 [x, y, yaw] 描述的是哪个 link；
//     - 哪些关节受控、按什么顺序；
//     - 哪些关节被锁定、锁在什么位置（轮子、夹爪…）；
//     - 末端/TCP 是哪个 frame；
//     - 哪些 link 属于底盘碰撞组；
// //   这些规则以前分散在 planner、visualization、OCS2 各自的参数里，是本次
//   统一的重点。所有消费者只读同一个 RobotModelConfig。
//
//   加载方式：
//     - RobotModelConfig::defaultsFor(description) 给出从 URDF 可推导的保守默认值；
//     - RobotModelConfigLoader::fromYaml 读取显式 YAML（yaml-cpp，ROS-free）。
// ============================================================================

#include "wbmm_robot_model/robot_description.hpp"

#include <map>
#include <string>
#include <vector>

namespace wbmm::robot_model
{

struct RobotModelConfig
{
  // 9D 状态中 [x, y, yaw] 描述的那个 link。默认 = URDF root。
  // KinematicModel 由此计算 root_from_state_base，保证所有消费者一致。
  std::string state_base_frame;

  // 受控 1-DoF 关节，顺序即状态里 q 的顺序。必须显式给出，不允许消费者
  // 各自按 URDF 顺序猜。
  std::vector<std::string> controlled_joints;

  // 未受控关节的锁定位置：关节名 -> 位置。可视化需要它来摆放轮子/夹爪，
  // 运动学在把状态映射成 q 时也用它补齐。
  std::map<std::string, double> locked_joints;

  // 末端：名字 -> frame。例如 {"tool": "tool0", "camera": "d435i_link"}。
  std::map<std::string, std::string> end_effectors;

  // 底盘碰撞组所在的 link（旧 planner 的 base_collision_link）。
  // 为空时表示没有底盘碰撞组。
  std::string base_collision_link;

  // 碰撞分组标签：组名 -> link 名列表。组只是集合视图，球始终绑定原始
  // link；未列出的球归入其所属 link 的默认组。
  std::map<std::string, std::vector<std::string>> collision_groups;

  // 从 URDF 推导默认值：
  //   state_base_frame = root
  //   controlled_joints = 全部非 fixed 的标量关节（按名称排序）
  //   locked_joints = {}
  //   end_effectors = {"tool0": "tool0"}（存在时）
  //   base_collision_link = ""（碰撞模型需要时由 YAML/参数显式指定）
  [[nodiscard]] static RobotModelConfig defaultsFor(
    const RobotDescription & description);
};

// 配置校验结果。校验失败必须让消费者显式失败，而不是静默回退。
struct ModelConfigValidation
{
  bool ok{false};
  std::string message;
};

// 针对给定 URDF 校验配置：关节存在且类型正确、没有重复、
// frame 存在、受控/锁定不冲突。
[[nodiscard]] ModelConfigValidation validate(
  const RobotModelConfig & config, const RobotDescription & description);

// 配置内容标识，用于判断模型配置是否改变。
[[nodiscard]] std::uint64_t contentId(const RobotModelConfig & config) noexcept;

}  // namespace wbmm::robot_model
