#pragma once

// ============================================================================
// TrajectorySpheres —— MINCO 轨迹变量上的球运动学适配器。
//
//   这是"共享球模型 + 轨迹变量映射"的那一层：它复用 wbmm_robot_model 的
//   CollisionSphereModel，但对自变量是
//       z = [x, y, vx, vy, q_1..q_n]
//   而不是状态 x = [x, y, yaw, q_1..q_n]，也不是输入 u = [v, omega, qdot]。
//
//   差速底盘非完整：航向由行进方向恢复，yaw = atan2(gear * vy, gear * vx)，
//   对 yaw 的导数必须一起解析求出。这正是本适配器存在的原因：把这份映射从
//   通用球运动学里隔离出来。
//
//   Jacobian 约定：d(position)/d(z)，3 x TrajectoryVariables::size(n)。
//   OCS2/通用正运动学使用 wbmm_pinocchio 的 SphereKinematics，两者消费同一份
//   球模型，并在测试中交叉校验。
// ============================================================================

#include <wbmm_robot_model/collision_sphere_model.hpp>
#include <wbmm_robot_model/robot_description.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace wbmm::collision
{

// Layout of the variables a whole-body trajectory optimizer differentiates
// against.
//
// The base yaw is deliberately NOT a variable: it is reconstructed from the
// base velocity direction.
struct TrajectoryVariables
{
  static constexpr std::size_t kBasePositionDim = 2;   // [0] x, [1] y
  static constexpr std::size_t kBaseVelocityDim = 2;   // [2] vx, [3] vy
  static constexpr std::size_t kFirstJointIndex = 4;

  [[nodiscard]] static constexpr std::size_t size(
    std::size_t joint_count) noexcept
  {
    return kFirstJointIndex + joint_count;
  }
};

struct SphereSample
{
  std::string id;
  double radius{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  // d(position)/d(variable), 3 x TrajectoryVariables::size(joint_count).
  Eigen::MatrixXd jacobian;
};

struct TrajectorySpheres
{
  bool success{false};
  std::string message;

  // Reconstructed base heading and its gradient w.r.t. the base velocity.
  double yaw{0.0};
  Eigen::Vector2d yaw_gradient{Eigen::Vector2d::Zero()};

  // Spheres that move with the base only.
  std::vector<SphereSample> base;
  // Arm spheres, flattened over the groups, in CollisionSphereModel::arm order.
  std::vector<SphereSample> arm;
  // Owning joint index for each entry of `arm`.
  std::vector<std::size_t> arm_group;

  std::size_t variable_count{0};
};

// Evaluates every collision sphere and its analytic Jacobian.
//
//   description       unified URDF description (joint origins/axes)
//   sphere_model      unified collision spheres (geometry + groups)
//   base_position_xy  base_footprint position in the world frame
//   base_velocity_xy  direction of travel; together with `gear` it defines the
//                     heading as atan2(gear * vy, gear * vx)
//   gear              +1 forward, -1 reverse
//   joint_angles      one entry per sphere_model.arm_joint_names entry
//   held_yaw          heading to use when the speed is too low for the velocity
//                     direction to mean anything. Pass a finite value to get the
//                     same hold-at-rest behaviour the trajectory builder uses;
//                     leave it NaN to derive the heading from the velocity
//                     unconditionally.
//
// Gradients are analytic, not finite differences, because the optimizer calls
// this thousands of times per plan. The convention is verified against central
// differences in test_trajectory_spheres.cpp.
[[nodiscard]] TrajectorySpheres evaluateTrajectorySpheres(
  const wbmm::robot_model::RobotDescription & description,
  const wbmm::robot_model::CollisionSphereModel & sphere_model,
  const Eigen::Vector2d & base_position_xy,
  const Eigen::Vector2d & base_velocity_xy, int gear,
  const Eigen::VectorXd & joint_angles,
  double held_yaw = std::numeric_limits<double>::quiet_NaN());

}  // namespace wbmm::collision
