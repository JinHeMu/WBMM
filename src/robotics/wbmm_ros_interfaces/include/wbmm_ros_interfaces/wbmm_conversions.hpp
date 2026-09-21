#pragma once

#include <wbmm_core/wbmm_core.hpp>

#include <Eigen/Core>

#include <optional>
#include <string>
#include <vector>

namespace wbmm::ros_interfaces
{

// ============================================================================
// wbmm_conversions —— 旧 9D/8D Eigen 向量与 wbmm_core 合同之间的转换层。
//
//   本文件不依赖 ROS、Pinocchio 或具体机器人，只回答一个问题：
//   "控制算法使用的 Eigen 向量，如何无损地进入/离开 wbmm_core 类型？"
//
//   转换本身不做运动学校验；调用方在转换后统一调用
//   wbmm::core::validate(...) 和 RobotModel::validate(...)(fail-closed)。
// ============================================================================

// Eigen 9D state [x, y, yaw, q1..qN] -> WholeBodyState。
// 维度不匹配时返回 nullopt；不做数值校验。
[[nodiscard]] std::optional<wbmm::core::WholeBodyState> toCoreState(
  const Eigen::Ref<const Eigen::VectorXd> & state,
  const std::vector<std::string> & joint_names,
  const wbmm::core::Header & header);

// WholeBodyState -> Eigen 9D state [x, y, yaw, q1..qN]。
// joints.names 与 joints.positions 维度不一致时抛出 std::invalid_argument。
[[nodiscard]] Eigen::VectorXd toEigenState(
  const wbmm::core::WholeBodyState & state);

// Eigen 6D wrench [Fx,Fy,Fz,Tx,Ty,Tz] <-> wbmm::core::Wrench。
[[nodiscard]] wbmm::core::Wrench toCoreWrench(
  const Eigen::Matrix<double, 6, 1> & wrench,
  const wbmm::core::Header & header);

[[nodiscard]] Eigen::Matrix<double, 6, 1> toEigenWrench(
  const wbmm::core::Wrench & wrench);

// Eigen 8D input [v, omega, qdot1..qN] -> WholeBodyInput。
// 维度必须是 2 + joint_names.size()，否则返回 nullopt。
[[nodiscard]] std::optional<wbmm::core::WholeBodyInput> toCoreInput(
  const Eigen::Ref<const Eigen::VectorXd> & input,
  const std::vector<std::string> & joint_names,
  double stamp);

// WholeBodyInput -> Eigen [base_command..., joint_velocities...]。
[[nodiscard]] Eigen::VectorXd toEigenInput(
  const wbmm::core::WholeBodyInput & input);

// 构造"零输入" WholeBodyInput，用于仅有状态、没有前馈的名义轨迹点。
[[nodiscard]] wbmm::core::WholeBodyInput makeZeroWholeBodyInput(
  const std::vector<std::string> & joint_names,
  double stamp);

}  // namespace wbmm::ros_interfaces
