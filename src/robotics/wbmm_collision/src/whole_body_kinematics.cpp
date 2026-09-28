#include "wbmm_collision/whole_body_kinematics.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

namespace wbmm::collision
{
namespace
{

// Below this speed the heading is not defined by the direction of travel, so
// its derivative w.r.t. the velocity is reported as zero. The trajectory
// builder holds the heading in the same regime, so the two agree.
constexpr double kMinSpeedForHeading = 1.0e-6;

Eigen::Matrix4d placementOf(const UrdfJointKinematics & joint, double angle)
{
  Eigen::Matrix4d placement = Eigen::Matrix4d::Identity();
  placement.block<3, 3>(0, 0) =
    joint.origin_rotation *
    Eigen::AngleAxisd(angle, joint.axis).toRotationMatrix();
  placement.block<3, 1>(0, 3) = joint.origin_translation;
  return placement;
}

// d/d(theta) of placementOf. Because the origin translation does not depend on
// theta and the rotation is a pure rotation about the joint axis:
//   dT = origin * skew(axis) * R(theta, axis)
Eigen::Matrix4d placementDerivativeOf(
  const UrdfJointKinematics & joint, double angle)
{
  const Eigen::Matrix3d rotation =
    Eigen::AngleAxisd(angle, joint.axis).toRotationMatrix();
  Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
  skew(0, 1) = -joint.axis.z();
  skew(0, 2) = joint.axis.y();
  skew(1, 0) = joint.axis.z();
  skew(1, 2) = -joint.axis.x();
  skew(2, 0) = -joint.axis.y();
  skew(2, 1) = joint.axis.x();

  Eigen::Matrix4d derivative = Eigen::Matrix4d::Zero();
  derivative.block<3, 3>(0, 0) = joint.origin_rotation * skew * rotation;
  return derivative;
}

Eigen::Matrix4d baseTransform(
  const Eigen::Vector2d & position_xy, double yaw)
{
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  transform(0, 3) = position_xy.x();
  transform(1, 3) = position_xy.y();
  return transform;
}

// d(baseTransform)/d(yaw). The translation does not depend on yaw.
Eigen::Matrix4d baseTransformDerivative(double yaw)
{
  Eigen::Matrix3d skew = Eigen::Matrix3d::Zero();
  skew(0, 1) = -1.0;
  skew(1, 0) = 1.0;

  Eigen::Matrix4d derivative = Eigen::Matrix4d::Zero();
  derivative.block<3, 3>(0, 0) =
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() * skew;
  return derivative;
}

void fillPositionGradient(Eigen::MatrixXd & jacobian)
{
  // The base translation enters every sphere position identically.
  jacobian(0, 0) = 1.0;
  jacobian(1, 1) = 1.0;
}

void fillHeadingGradient(
  Eigen::MatrixXd & jacobian, const Eigen::Vector3d & dPositionDYaw,
  const Eigen::Vector2d & yaw_gradient)
{
  for (int axis = 0; axis < 3; ++axis) {
    jacobian(axis, 2) = dPositionDYaw(axis) * yaw_gradient.x();
    jacobian(axis, 3) = dPositionDYaw(axis) * yaw_gradient.y();
  }
}

}  // namespace

WholeBodyKinematics evaluateWholeBodyKinematics(
  const UrdfCollisionModel & model, const Eigen::Vector2d & base_position_xy,
  const Eigen::Vector2d & base_velocity_xy, int gear,
  const Eigen::VectorXd & joint_angles, double held_yaw)
{
  WholeBodyKinematics result;

  if (!model.success) {
    result.message = "The URDF collision model is not valid.";
    return result;
  }
  if (gear != 1 && gear != -1) {
    result.message = "gear must be +1 or -1.";
    return result;
  }
  const auto joint_count = static_cast<std::size_t>(joint_angles.size());
  if (joint_count != model.joints.size()) {
    result.message = "joint_angles has " + std::to_string(joint_count) +
      " entries but the model has " + std::to_string(model.joints.size()) + ".";
    return result;
  }
  if (!base_position_xy.allFinite() || !base_velocity_xy.allFinite() ||
    !joint_angles.allFinite())
  {
    result.message = "Non-finite input.";
    return result;
  }

  result.variable_count = WholeBodyVariableLayout::size(joint_count);
  const auto variables = static_cast<Eigen::Index>(result.variable_count);

  // ---- Heading and its gradient -------------------------------------------
  const double gear_value = static_cast<double>(gear);
  const double vx = gear_value * base_velocity_xy.x();
  const double vy = gear_value * base_velocity_xy.y();
  const double speed_squared = vx * vx + vy * vy;
  if (std::isfinite(held_yaw) && speed_squared < kMinSpeedForHeading * kMinSpeedForHeading) {
    // Direction undefined. Holding the caller's heading is both what the
    // trajectory builder does at rest and the only choice that keeps the cost
    // continuous: atan2(0, 0) is 0, so a vanishing perturbation would otherwise
    // pick an arbitrary direction and swing the arm spheres.
    result.yaw = held_yaw;
    result.yaw_gradient.setZero();
  } else {
    result.yaw = std::atan2(vy, vx);
    // d/dv of atan2(gear*vy, gear*vx) is (-vy, vx)/|v|^2, independent of gear:
    // the two gear factors cancel. The denominator is floored rather than
    // switched on a threshold, so the gradient stays continuous and bounded.
    const double softened = std::max(
      speed_squared, kMinSpeedForHeading * kMinSpeedForHeading);
    result.yaw_gradient = Eigen::Vector2d(
      -base_velocity_xy.y() / softened,
      base_velocity_xy.x() / softened);
  }

  const Eigen::Matrix4d world_from_root = baseTransform(base_position_xy, result.yaw);
  const Eigen::Matrix4d dWorldFromRoot = baseTransformDerivative(result.yaw);
  const Eigen::Matrix4d root_from_base = model.root_from_base_collision;

  // ---- Base spheres --------------------------------------------------------
  result.base.reserve(model.base.spheres.size());
  for (const auto & sphere : model.base.spheres) {
    const Eigen::Vector4d point(
      sphere.center.x(), sphere.center.y(), sphere.center.z(), 1.0);

    SphereKinematics kinematics;
    kinematics.name = sphere.name;
    kinematics.radius = sphere.radius;
    kinematics.position =
      (world_from_root * root_from_base * point).head<3>();
    kinematics.jacobian = Eigen::MatrixXd::Zero(3, variables);
    fillPositionGradient(kinematics.jacobian);
    fillHeadingGradient(
      kinematics.jacobian, (dWorldFromRoot * root_from_base * point).head<3>(),
      result.yaw_gradient);
    result.base.push_back(std::move(kinematics));
  }

  // ---- Arm spheres ---------------------------------------------------------
  // T_world = world_from_root * root_from_base * mount * T_0 * ... * T_i
  const Eigen::Matrix4d mount =
    root_from_base * model.base_from_arm_mount;

  std::vector<Eigen::Matrix4d> placements(joint_count);
  std::vector<Eigen::Matrix4d> derivatives(joint_count);
  for (std::size_t j = 0U; j < joint_count; ++j) {
    placements[j] = placementOf(model.joints[j], joint_angles(static_cast<Eigen::Index>(j)));
    derivatives[j] =
      placementDerivativeOf(model.joints[j], joint_angles(static_cast<Eigen::Index>(j)));
  }

  // prefix[j] = mount * T_0 * ... * T_{j-1}; prefix[0] = mount.
  std::vector<Eigen::Matrix4d> prefix(joint_count + 1U, Eigen::Matrix4d::Identity());
  prefix[0] = mount;
  for (std::size_t j = 0U; j < joint_count; ++j) {
    prefix[j + 1U] = prefix[j] * placements[j];
  }

  for (std::size_t group = 0U; group < model.arm.size(); ++group) {
    // suffix[j] = T_j * ... * T_group; suffix[group + 1] = identity.
    std::vector<Eigen::Matrix4d> suffix(
      group + 2U, Eigen::Matrix4d::Identity());
    for (std::size_t j = group + 1U; j-- > 0U;) {
      suffix[j] = placements[j] * suffix[j + 1U];
    }
    const Eigen::Matrix4d chain = prefix[group] * suffix[group];

    for (const auto & sphere : model.arm[group].spheres) {
      const Eigen::Vector4d point(
        sphere.center.x(), sphere.center.y(), sphere.center.z(), 1.0);

      SphereKinematics kinematics;
      kinematics.name = sphere.name;
      kinematics.radius = sphere.radius;
      kinematics.jacobian = Eigen::MatrixXd::Zero(3, variables);

      const Eigen::Vector4d local = chain * point;
      kinematics.position = (world_from_root * local).head<3>();
      fillPositionGradient(kinematics.jacobian);
      fillHeadingGradient(
        kinematics.jacobian, (dWorldFromRoot * local).head<3>(),
        result.yaw_gradient);

      // d/dq_j = world_from_root * prefix[j] * dT_j * suffix[j + 1] * point.
      // The derivative is a rotation about an axis through the joint origin, so
      // only the rotational block of dT_j is non-zero and the point keeps its
      // homogeneous coordinate.
      for (std::size_t j = 0U; j <= group; ++j) {
        const Eigen::Vector4d moved = suffix[j + 1U] * point;
        const Eigen::Vector4d rotated =
          derivatives[j] * moved;
        const Eigen::Vector4d world =
          world_from_root * prefix[j] * rotated;
        kinematics.jacobian.col(
          static_cast<Eigen::Index>(WholeBodyVariableLayout::kFirstJointIndex + j)) =
          world.head<3>();
      }

      result.arm_group.push_back(group);
      result.arm.push_back(std::move(kinematics));
    }
  }

  result.success = true;
  result.message = "ok";
  return result;
}

}  // namespace wbmm::collision
