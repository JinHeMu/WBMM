#pragma once

// ============================================================================
// CollisionSphereModel —— 共享的只读碰撞球模型。
//
//   核心规则（对应方案第 7 节）：
//   - 球永远绑定它真正所属的 link（owner_link），坐标 center_in_link 表达在
//     该 link 系；
//   - 分组只是集合视图：底盘组与每个受控关节一个臂组，group_tags 记录
//     球同时属于哪些组；
//   - v1 的球心、半径、覆盖范围与旧 UrdfCollisionModel 逐字一致，先保证
//     行为不变，再谈 box/cylinder 的球近似。
//
//   本文件只描述几何，不做运动学（球心的世界位置属于 wbmm_pinocchio 的
//   SphereKinematics，MINCO 轨迹变量上的导数属于 wbmm_collision 的适配器）。
// ============================================================================

#include "wbmm_robot_model/robot_description.hpp"
#include "wbmm_robot_model/robot_model_config.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace wbmm::robot_model
{

struct CollisionSphere
{
  // 稳定 id：base 组为 "<base_link>_sphere_<i>"，臂组为
  // "<joint>_sphere_<k>"，与旧 loader 保持一致。
  std::string id;
  std::string owner_link;
  Eigen::Vector3d center_in_link{Eigen::Vector3d::Zero()};
  double radius{0.0};
  // 例如 {"base"}、{"arm", "joint_3"}。分组只是视图。
  std::vector<std::string> group_tags;
  std::string source_geometry;   // "<link>/<collision name|index>"
};

struct CollisionSphereGroup
{
  std::string name;              // "base" 或受控关节名
  std::string owner_link;        // 组的参考 link（球心已表达在该链上）
  std::vector<CollisionSphere> spheres;
};

struct CollisionSphereModel
{
  bool success{false};
  std::string message;

  std::string root_link;
  std::string base_collision_link;

  // root_from_base_collision：URDF root 到 base_collision_link 的 fixed 链。
  // 规划器的 (x, y, yaw) 描述 root（通常就是 base_footprint），这个偏移
  // 不能丢。
  Eigen::Isometry3d root_from_base_collision{Eigen::Isometry3d::Identity()};

  // base_collision_link 到第一个受控关节 parent 的 fixed 链（REMANI 的
  // T_q_0_）。之后每一跳都经过可动关节。
  Eigen::Isometry3d base_from_arm_mount{Eigen::Isometry3d::Identity()};

  CollisionSphereGroup base;
  // 与 arm_joint_names 一一对应。
  std::vector<CollisionSphereGroup> arm;
  std::vector<std::string> arm_joint_names;

  // 被跳过的非球 collision（box/cylinder/mesh），仅用于诊断与后续扩展。
  std::vector<std::string> skipped_geometries;

  // 全部臂球半径的最大值（REMANI 的 manipulator_thickness_）。
  double max_arm_radius{0.0};

  // 分组策略标识；内容标识会纳入。
  std::string strategy{"remani-compat-v1"};

  [[nodiscard]] std::size_t baseSphereCount() const noexcept
  {return base.spheres.size();}

  [[nodiscard]] std::size_t armSphereCount() const noexcept;

  [[nodiscard]] std::size_t sphereCount() const noexcept
  {return baseSphereCount() + armSphereCount();}
};

// 从共享描述和语义构建球模型。
//   base 组：config.base_collision_link 上的球（为空则不建底盘组）；
//   arm[i]：第 i 个受控关节 child_link 的 fixed 子树里的球，外加 i==0 时
//           位于关节轴上的 parent_link 球（REMANI 折叠规则，几何上精确）。
[[nodiscard]] CollisionSphereModel buildCollisionSphereModel(
  const RobotDescription & description, const RobotModelConfig & config);

}  // namespace wbmm::robot_model
