#include "wbmm_collision/trajectory_spheres.hpp"

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

    struct JointKinematics
    {
      Eigen::Vector3d origin_translation{Eigen::Vector3d::Zero()};
      Eigen::Matrix3d origin_rotation{Eigen::Matrix3d::Identity()};
      Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
    };

    Eigen::Matrix4d placementOf(const JointKinematics &joint, double angle)
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
    Eigen::Matrix4d placementDerivativeOf(const JointKinematics &joint, double angle)
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

    Eigen::Matrix4d baseTransform(const Eigen::Vector2d &position_xy, double yaw)
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

    void fillPositionGradient(Eigen::MatrixXd &jacobian)
    {
      // The base translation enters every sphere position identically.
      jacobian(0, 0) = 1.0;
      jacobian(1, 1) = 1.0;
    }

    void fillHeadingGradient(
        Eigen::MatrixXd &jacobian, const Eigen::Vector3d &dPositionDYaw,
        const Eigen::Vector2d &yaw_gradient)
    {
      for (int axis = 0; axis < 3; ++axis)
      {
        jacobian(axis, 2) = dPositionDYaw(axis) * yaw_gradient.x();
        jacobian(axis, 3) = dPositionDYaw(axis) * yaw_gradient.y();
      }
    }

  } // namespace

  TrajectorySpheres evaluateTrajectorySpheres(
      const wbmm::robot_model::RobotDescription &description,
      const wbmm::robot_model::CollisionSphereModel &sphere_model,
      const Eigen::Vector2d &base_position_xy,
      const Eigen::Vector2d &base_velocity_xy, int gear,
      const Eigen::VectorXd &joint_angles, double held_yaw)
  {
    TrajectorySpheres result;

    if (!sphere_model.success)
    {
      result.message = "The collision sphere model is not valid: " +
                       sphere_model.message;
      return result;
    }
    if (gear != 1 && gear != -1)
    {
      result.message = "gear must be +1 or -1.";
      return result;
    }
    const auto joint_count = static_cast<std::size_t>(joint_angles.size());
    if (joint_count != sphere_model.arm_joint_names.size())
    {
      result.message = "joint_angles has " + std::to_string(joint_count) +
                       " entries but the model has " +
                       std::to_string(sphere_model.arm_joint_names.size()) + ".";
      return result;
    }
    if (!base_position_xy.allFinite() || !base_velocity_xy.allFinite() ||
        !joint_angles.allFinite())
    {
      result.message = "Non-finite input.";
      return result;
    }

    // 从统一描述取每个受控关节的 origin/axis，保持 REMANI 的链式约定：
    //   T_parent_child(theta) = origin * AngleAxis(theta, axis)
    std::vector<JointKinematics> joints(joint_count);
    for (std::size_t j = 0U; j < joint_count; ++j)
    {
      const auto *joint =
          description.findJoint(sphere_model.arm_joint_names[j]);
      if (joint == nullptr)
      {
        result.message = "Joint '" + sphere_model.arm_joint_names[j] +
                         "' is not in the robot description.";
        return result;
      }
      joints[j].origin_translation = joint->origin.translation();
      joints[j].origin_rotation = joint->origin.linear();
      joints[j].axis = joint->axis;
    }

    result.variable_count = TrajectoryVariables::size(joint_count);
    const auto variables = static_cast<Eigen::Index>(result.variable_count);

    // ---- Heading and its gradient -------------------------------------------
    const double gear_value = static_cast<double>(gear);
    const double vx = gear_value * base_velocity_xy.x();
    const double vy = gear_value * base_velocity_xy.y();
    const double speed_squared = vx * vx + vy * vy;
    if (std::isfinite(held_yaw) &&
        speed_squared < kMinSpeedForHeading * kMinSpeedForHeading)
    {
      // Direction undefined. Holding the caller's heading is both what the
      // trajectory builder does at rest and the only choice that keeps the cost
      // continuous.
      result.yaw = held_yaw;
      result.yaw_gradient.setZero();
    }
    else
    {
      result.yaw = std::atan2(vy, vx);
      // d/dv of atan2(gear*vy, gear*vx) is (-vy, vx)/|v|^2, independent of gear.
      // The denominator is floored rather than switched on a threshold, so the
      // gradient stays continuous and bounded.
      const double softened = std::max(
          speed_squared, kMinSpeedForHeading * kMinSpeedForHeading);
      result.yaw_gradient = Eigen::Vector2d(
          -base_velocity_xy.y() / softened,
          base_velocity_xy.x() / softened);
    }

    const Eigen::Matrix4d world_from_root = baseTransform(base_position_xy, result.yaw);
    const Eigen::Matrix4d dWorldFromRoot = baseTransformDerivative(result.yaw);
    const Eigen::Matrix4d root_from_base =
        sphere_model.root_from_base_collision.matrix();

    // ---- Base spheres --------------------------------------------------------
    result.base.reserve(sphere_model.base.spheres.size());
    for (const auto &sphere : sphere_model.base.spheres)
    {
      const Eigen::Vector4d point(
          sphere.center_in_link.x(), sphere.center_in_link.y(),
          sphere.center_in_link.z(), 1.0);

      SphereSample kinematics;
      kinematics.id = sphere.id;
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
        root_from_base * sphere_model.base_from_arm_mount.matrix();

    std::vector<Eigen::Matrix4d> placements(joint_count);
    std::vector<Eigen::Matrix4d> derivatives(joint_count);
    for (std::size_t j = 0U; j < joint_count; ++j)
    {
      placements[j] = placementOf(
          joints[j], joint_angles(static_cast<Eigen::Index>(j)));
      derivatives[j] = placementDerivativeOf(
          joints[j], joint_angles(static_cast<Eigen::Index>(j)));
    }

    // prefix[j] = mount * T_0 * ... * T_{j-1}; prefix[0] = mount.
    std::vector<Eigen::Matrix4d> prefix(joint_count + 1U, Eigen::Matrix4d::Identity());
    prefix[0] = mount;
    for (std::size_t j = 0U; j < joint_count; ++j)
    {
      prefix[j + 1U] = prefix[j] * placements[j];
    }

    for (std::size_t group = 0U; group < sphere_model.arm.size(); ++group)
    {
      // suffix[j] = T_j * ... * T_group; suffix[group + 1] = identity.
      std::vector<Eigen::Matrix4d> suffix(
          group + 2U, Eigen::Matrix4d::Identity());
      for (std::size_t j = group + 1U; j-- > 0U;)
      {
        suffix[j] = placements[j] * suffix[j + 1U];
      }
      const Eigen::Matrix4d chain = prefix[group] * suffix[group];

      for (const auto &sphere : sphere_model.arm[group].spheres)
      {
        const Eigen::Vector4d point(
            sphere.center_in_link.x(), sphere.center_in_link.y(),
            sphere.center_in_link.z(), 1.0);

        SphereSample kinematics;
        kinematics.id = sphere.id;
        kinematics.radius = sphere.radius;
        kinematics.jacobian = Eigen::MatrixXd::Zero(3, variables);

        const Eigen::Vector4d local = chain * point;
        kinematics.position = (world_from_root * local).head<3>();
        fillPositionGradient(kinematics.jacobian);
        fillHeadingGradient(
            kinematics.jacobian, (dWorldFromRoot * local).head<3>(),
            result.yaw_gradient);

        // d/dq_j = world_from_root * prefix[j] * dT_j * suffix[j + 1] * point.
        for (std::size_t j = 0U; j <= group; ++j)
        {
          const Eigen::Vector4d moved = suffix[j + 1U] * point;
          const Eigen::Vector4d rotated = derivatives[j] * moved;
          const Eigen::Vector4d world = world_from_root * prefix[j] * rotated;
          kinematics.jacobian.col(
              static_cast<Eigen::Index>(TrajectoryVariables::kFirstJointIndex + j)) =
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

} // namespace wbmm::collision
