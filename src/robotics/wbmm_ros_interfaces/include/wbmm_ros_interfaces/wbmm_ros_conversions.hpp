#pragma once

#include <wbmm_core/wbmm_core.hpp>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <std_msgs/msg/header.hpp>

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace wbmm::ros_interfaces
{

// ============================================================================
// wbmm_ros_conversions —— ROS 消息与 wbmm_core 合同之间的唯一转换入口。
//
//   节点回调只负责：
//     1. 调这里的转换函数；
//     2. 调 wbmm::core::validate(...) / RobotModel::validate(...)；
//     3. 把结果交给算法。
//
//   这里集中处理 ROS 边界上的差异：
//   - builtin_interfaces/Time -> double 秒；
//   - geometry_msgs 四元数 xyzw <-> core 四元数 wxyz；
//   - ROS 空 frame 的显式 fallback 策略。
// ============================================================================

[[nodiscard]] wbmm::core::Quaternion quaternionFromRos(
  const geometry_msgs::msg::Quaternion & quaternion);

[[nodiscard]] geometry_msgs::msg::Quaternion quaternionToRos(
  const wbmm::core::Quaternion & quaternion);

[[nodiscard]] wbmm::core::Header headerFromRos(
  const std_msgs::msg::Header & header);

// WrenchStamped -> Wrench。消息 frame_id 为空时使用 fallback_frame；
// fallback_frame 仍为空、时间戳非法时返回 nullopt。
[[nodiscard]] std::optional<wbmm::core::Wrench> wrenchFromRos(
  const geometry_msgs::msg::WrenchStamped & message,
  const std::string & fallback_frame = {});

// MpcObservation -> WholeBodyState。OCS2 observation 不携带 frame_id 和关节名，
// 因此由调用方(节点)显式提供。维度/时间戳非法时返回 nullopt。
[[nodiscard]] std::optional<wbmm::core::WholeBodyState>
wholeBodyStateFromMpcObservation(
  const ocs2_msgs::msg::MpcObservation & message,
  const std::vector<std::string> & joint_names,
  const std::string & frame_id);

// WholeBodyTrajectory -> OCS2 MpcTargetTrajectories。
// 发布时刻 = start_time + point.time_from_start。
// input_dimension 是旧 OCS2 适配器的兼容参数：轨迹自带输入先转换，
// 再按该维度补零/截断；缺失输入时补全为零。
[[nodiscard]] ocs2_msgs::msg::MpcTargetTrajectories toMpcTargetTrajectories(
  const wbmm::core::WholeBodyTrajectory & trajectory,
  double start_time,
  std::size_t input_dimension);

// EndEffectorPose -> OCS2 MpcTargetTrajectories (single 7D target point).
// State order is [x, y, z, qx, qy, qz, qw].
[[nodiscard]] ocs2_msgs::msg::MpcTargetTrajectories toMpcTargetTrajectories(
  const wbmm::core::EndEffectorPose & pose,
  double time,
  std::size_t input_dimension);

}  // namespace wbmm::ros_interfaces
