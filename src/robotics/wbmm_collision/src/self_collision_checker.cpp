#include "wbmm_collision/self_collision_checker.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

namespace wbmm::collision
{

SelfCollisionResult checkSelfCollision(
  const TrajectorySpheres & spheres, double safety_margin)
{
  SelfCollisionResult result;
  if (!spheres.success) {
    result.message = "Sphere kinematics is not valid: " + spheres.message;
    return result;
  }
  if (!std::isfinite(safety_margin) || safety_margin < 0.0) {
    result.message = "safety_margin must be finite and nonnegative.";
    return result;
  }
  if (spheres.variable_count < TrajectoryVariables::kFirstJointIndex ||
    spheres.variable_count > static_cast<std::size_t>(
      std::numeric_limits<Eigen::Index>::max()) ||
    spheres.arm.size() != spheres.arm_group.size())
  {
    result.message = "Invalid trajectory variable count or arm group count.";
    return result;
  }
  const auto variables = static_cast<Eigen::Index>(spheres.variable_count);
  const auto valid = [variables](const SphereSample & sphere) {
      return std::isfinite(sphere.radius) && sphere.radius >= 0.0 &&
             sphere.position.allFinite() && sphere.jacobian.rows() == 3 &&
             sphere.jacobian.cols() == variables && sphere.jacobian.allFinite();
    };
  for (const auto & sphere : spheres.base) {
    if (!valid(sphere)) {
      result.message = "Invalid base sphere: " + sphere.id;
      return result;
    }
  }
  for (std::size_t i = 0; i < spheres.arm.size(); ++i) {
    if (!valid(spheres.arm[i]) ||
      spheres.arm_group[i] >= spheres.variable_count - TrajectoryVariables::kFirstJointIndex)
    {
      result.message = "Invalid arm sphere or group: " + spheres.arm[i].id;
      return result;
    }
  }

  result.status = CollisionStatus::kFree;
  result.min_clearance = std::numeric_limits<double>::infinity();
  const auto checkPair = [&](const SphereSample & a, const SphereSample & b) {
      const Eigen::Vector3d delta = a.position - b.position;
      const double distance = delta.stableNorm();
      SelfCollisionPair pair;
      pair.first_sphere = a.id;
      pair.second_sphere = b.id;
      pair.clearance = distance - a.radius - b.radius - safety_margin;
      pair.gradient = Eigen::VectorXd::Zero(variables);
      if (distance > 0.0) {
        pair.gradient = (a.jacobian - b.jacobian).transpose() * (delta / distance);
      }
      if (!std::isfinite(pair.clearance) || !pair.gradient.allFinite()) {
        return false;
      }
      if (pair.clearance < result.min_clearance) {
        result.min_clearance = pair.clearance;
        result.closest_first_sphere = a.id;
        result.closest_second_sphere = b.id;
      }
      if (pair.clearance <= 0.0) {
        result.status = CollisionStatus::kCollision;
      }
      result.pairs.push_back(std::move(pair));
      return true;
    };
  for (std::size_t i = 0; i < spheres.arm.size(); ++i) {
    for (const auto & base : spheres.base) {
      if (!checkPair(spheres.arm[i], base)) {
        return SelfCollisionResult{CollisionStatus::kInvalidInput,
          std::numeric_limits<double>::quiet_NaN(), {}, {}, {},
          "Non-finite self-collision distance or gradient."};
      }
    }
    for (std::size_t j = i + 1; j < spheres.arm.size(); ++j) {
      const auto group_i = spheres.arm_group[i];
      const auto group_j = spheres.arm_group[j];
      const auto difference = group_i > group_j ? group_i - group_j : group_j - group_i;
      if (difference < 2U) {
        continue;
      }
      if (!checkPair(spheres.arm[i], spheres.arm[j])) {
        return SelfCollisionResult{CollisionStatus::kInvalidInput,
          std::numeric_limits<double>::quiet_NaN(), {}, {}, {},
          "Non-finite self-collision distance or gradient."};
      }
    }
  }
  result.message = result.isFree() ? "ok" : "Self-collision margin violated.";
  return result;
}

}  // namespace wbmm::collision
