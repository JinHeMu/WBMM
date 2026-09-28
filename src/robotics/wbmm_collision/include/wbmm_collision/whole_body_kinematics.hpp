#pragma once

#include "wbmm_collision/urdf_collision_model.hpp"

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
// The base yaw is deliberately NOT a variable. A differential-drive base is
// nonholonomic, so its heading follows the direction of travel and is
// reconstructed from the base velocity. Differentiating through that
// reconstruction is the entire reason this evaluator exists: without it a
// collision cost could only be differentiated against x/y and the joints, and
// the base could not be steered away from an obstacle by its velocity.
struct WholeBodyVariableLayout
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

struct SphereKinematics
{
  std::string name;
  double radius{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  // d(position)/d(variable), 3 x WholeBodyVariableLayout::size(joint_count).
  Eigen::MatrixXd jacobian;
};

struct WholeBodyKinematics
{
  bool success{false};
  std::string message;

  // Reconstructed base heading and its gradient w.r.t. the base velocity.
  double yaw{0.0};
  Eigen::Vector2d yaw_gradient{Eigen::Vector2d::Zero()};

  // Spheres that move with the base only.
  std::vector<SphereKinematics> base;
  // Arm spheres, flattened over the groups, in UrdfCollisionModel::arm order.
  std::vector<SphereKinematics> arm;
  // Owning joint index for each entry of `arm`.
  std::vector<std::size_t> arm_group;

  std::size_t variable_count{0};
};

// Evaluates every collision sphere and its analytic Jacobian.
//
//   base_position_xy  base_footprint position in the world frame
//   base_velocity_xy  direction of travel; together with `gear` it defines the
//                     heading as atan2(gear * vy, gear * vx)
//   gear              +1 forward, -1 reverse
//   joint_angles      one entry per UrdfCollisionModel::joints entry
//   held_yaw          heading to use when the speed is too low for the velocity
//                     direction to mean anything. Pass a finite value to get the
//                     same hold-at-rest behaviour the trajectory builder uses;
//                     leave it NaN to derive the heading from the velocity
//                     unconditionally (the previous behaviour).
//
// Gradients are analytic, not finite differences, because the optimizer calls
// this thousands of times per plan. The convention is verified against central
// differences in test_whole_body_kinematics.cpp.
[[nodiscard]] WholeBodyKinematics evaluateWholeBodyKinematics(
  const UrdfCollisionModel & model, const Eigen::Vector2d & base_position_xy,
  const Eigen::Vector2d & base_velocity_xy, int gear,
  const Eigen::VectorXd & joint_angles,
  double held_yaw = std::numeric_limits<double>::quiet_NaN());

}  // namespace wbmm::collision
