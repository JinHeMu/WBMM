#include "wbmm_collision/environment_collision_checker.hpp"

#include "wbmm_core/validation.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

namespace wbmm::collision
{
namespace
{

Eigen::Vector3d toEigen(const wbmm::core::Vector3 & value)
{
  return Eigen::Vector3d(value.x, value.y, value.z);
}

bool isFiniteVector(const wbmm::core::Vector3 & value)
{
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

bool isFiniteQuaternion(const wbmm::core::Quaternion & value)
{
  return std::isfinite(value.w) && std::isfinite(value.x) &&
         std::isfinite(value.y) && std::isfinite(value.z);
}

bool appliesToScope(CollisionGroup group, CheckScope scope)
{
  switch (scope) {
    case CheckScope::kBase:
      return group == CollisionGroup::kBase;
    case CheckScope::kArm:
      return group == CollisionGroup::kArm;
    case CheckScope::kWholeBody:
      return true;
  }
  return false;
}

}  // namespace

EnvironmentCollisionChecker::EnvironmentCollisionChecker(
  wbmm::core::RobotModelPtr robot_model,
  std::shared_ptr<const wbmm::environment::EsdfGrid> environment,
  CollisionModel collision_model,
  CollisionCheckOptions options)
: robot_model_(std::move(robot_model)),
  environment_(std::move(environment)),
  collision_model_(std::move(collision_model)),
  options_(options)
{
}

CollisionResult EnvironmentCollisionChecker::checkBase(
  const wbmm::core::Header & header, const wbmm::core::BaseState & base) const
{
  // The search interface only exposes the base pose.  Build a structurally
  // valid whole-body state with zero arm positions so that the RobotModel FK
  // can resolve fixed links upstream of the arm joints.  For the current robot
  // the base collision geometry is attached before the arm chain, so the arm
  // values do not affect the base link pose.
  wbmm::core::WholeBodyState state;
  state.header = header;
  state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
  state.base = base;
  if (robot_model_) {
    state.joints.names = robot_model_->jointNames();
    state.joints.positions.assign(state.joints.names.size(), 0.0);
  }

  return checkWithScope(state, CheckScope::kBase, false);
}

CollisionResult EnvironmentCollisionChecker::check(
  const wbmm::core::WholeBodyState & state, CheckScope scope) const
{
  return checkWithScope(state, scope, true);
}

CollisionResult EnvironmentCollisionChecker::checkWithScope(
  const wbmm::core::WholeBodyState & state, CheckScope scope,
  bool validate_state) const
{
  CollisionResult result;
  result.message.clear();

  if (!robot_model_) {
    result.status = CollisionStatus::kModelError;
    result.message = "RobotModel is null.";
    return result;
  }
  if (!environment_) {
    result.status = CollisionStatus::kModelError;
    result.message = "ESDF environment is null.";
    return result;
  }
  if (!std::isfinite(options_.safety_margin) ||
    options_.safety_margin < 0.0)
  {
    result.status = CollisionStatus::kInvalidInput;
    result.message = "CollisionCheckOptions.safety_margin must be finite and non-negative.";
    return result;
  }
  if (state.header.frame_id.empty()) {
    result.status = CollisionStatus::kInvalidInput;
    result.message = "State header.frame_id must not be empty.";
    return result;
  }
  if (!std::isfinite(state.header.stamp) || state.header.stamp < 0.0) {
    result.status = CollisionStatus::kInvalidInput;
    result.message = "State header.stamp must be finite and non-negative.";
    return result;
  }
  if (!std::isfinite(state.base.x) || !std::isfinite(state.base.y) ||
    !std::isfinite(state.base.yaw) ||
    !std::isfinite(state.base.linear_velocity) ||
    !std::isfinite(state.base.lateral_velocity) ||
    !std::isfinite(state.base.yaw_rate))
  {
    result.status = CollisionStatus::kInvalidInput;
    result.message = "Base state must be finite.";
    return result;
  }
  if (state.base_model != wbmm::core::BaseModel::kDifferentialDrive) {
    result.status = CollisionStatus::kInvalidInput;
    result.message = "Only differential-drive base states are supported.";
    return result;
  }
  if (state.header.frame_id != environment_->info().frame_id) {
    result.status = CollisionStatus::kFrameMismatch;
    result.message = "State frame '" + state.header.frame_id +
      "' does not match ESDF frame '" + environment_->info().frame_id + "'.";
    return result;
  }
  if (collision_model_.spheres.empty()) {
    result.status = CollisionStatus::kInvalidInput;
    result.message = "CollisionModel contains no collision spheres.";
    return result;
  }

  if (validate_state) {
    const auto validation = wbmm::core::validate(state);
    if (!validation.ok) {
      result.status = CollisionStatus::kInvalidInput;
      result.message = validation.message;
      return result;
    }
    std::string model_message;
    if (!robot_model_->validate(state, &model_message)) {
      result.status = CollisionStatus::kInvalidInput;
      result.message = model_message.empty()
        ? "RobotModel rejected the whole-body state."
        : model_message;
      return result;
    }
  }

  std::unordered_map<std::string, wbmm::core::Pose> pose_cache;
  double min_clearance = std::numeric_limits<double>::infinity();
  std::string closest_sphere;
  std::size_t evaluated_spheres = 0U;

  for (const auto & sphere : collision_model_.spheres) {
    if (!appliesToScope(sphere.group, scope)) {
      continue;
    }

    if (sphere.name.empty() || sphere.link_name.empty() ||
      !sphere.center.array().isFinite().all() || !std::isfinite(sphere.radius) ||
      sphere.radius <= 0.0)
    {
      result.status = CollisionStatus::kInvalidInput;
      result.message = "Invalid collision sphere metadata.";
      return result;
    }

    auto pose_it = pose_cache.find(sphere.link_name);
    if (pose_it == pose_cache.end()) {
      wbmm::core::Pose pose;
      if (!robot_model_->forwardKinematics(state, sphere.link_name, pose)) {
        result.status = CollisionStatus::kModelError;
        result.message = "Forward kinematics failed for link '" +
          sphere.link_name + "'.";
        return result;
      }
      if (pose.header.frame_id != state.header.frame_id) {
        result.status = CollisionStatus::kFrameMismatch;
        result.message = "Link '" + sphere.link_name + "' pose is expressed in '" +
          pose.header.frame_id + "', expected '" + state.header.frame_id + "'.";
        return result;
      }
      pose_it = pose_cache.emplace(sphere.link_name, std::move(pose)).first;
    }
    const auto & pose = pose_it->second;

    if (!isFiniteVector(pose.position) || !isFiniteQuaternion(pose.orientation)) {
      result.status = CollisionStatus::kInvalidInput;
      result.message = "Link pose for '" + sphere.link_name + "' is not finite.";
      return result;
    }

    Eigen::Quaterniond orientation(
      pose.orientation.w, pose.orientation.x,
      pose.orientation.y, pose.orientation.z);
    if (!orientation.coeffs().allFinite() ||
      orientation.norm() < 1.0e-12)
    {
      result.status = CollisionStatus::kInvalidInput;
      result.message = "Link orientation for '" + sphere.link_name + "' is invalid.";
      return result;
    }
    orientation.normalize();

    const Eigen::Vector3d center_world =
      toEigen(pose.position) + orientation * sphere.center;

    const auto query = environment_->query(state.header.frame_id, center_world);
    switch (query.status) {
      case wbmm::environment::QueryStatus::kSuccess:
        break;
      case wbmm::environment::QueryStatus::kFrameMismatch:
        result.status = CollisionStatus::kFrameMismatch;
        result.message = query.message;
        return result;
      case wbmm::environment::QueryStatus::kOutOfBounds:
        result.status = CollisionStatus::kOutOfBounds;
        result.message = query.message;
        return result;
      case wbmm::environment::QueryStatus::kUnknown:
        result.status = CollisionStatus::kUnknownSpace;
        result.message = query.message;
        return result;
      case wbmm::environment::QueryStatus::kInvalidInput:
      case wbmm::environment::QueryStatus::kNotImplemented:
      default:
        result.status = CollisionStatus::kInvalidInput;
        result.message = query.message;
        return result;
    }

    if (!std::isfinite(query.distance)) {
      result.status = CollisionStatus::kInvalidInput;
      result.message = "ESDF query returned a non-finite distance.";
      return result;
    }

    const double clearance =
      query.distance - sphere.radius - options_.safety_margin;
    if (clearance < min_clearance) {
      min_clearance = clearance;
      closest_sphere = sphere.name;
    }
    ++evaluated_spheres;
  }

  if (evaluated_spheres == 0U) {
    result.status = CollisionStatus::kInvalidInput;
    result.message = std::string("No collision spheres apply to scope ") +
      (scope == CheckScope::kBase ? "kBase" :
      scope == CheckScope::kArm ? "kArm" : "kWholeBody") + ".";
    return result;
  }

  result.min_clearance = min_clearance;
  result.closest_sphere = std::move(closest_sphere);
  if (min_clearance > 0.0) {
    result.status = CollisionStatus::kFree;
    result.message = "free";
  } else {
    result.status = CollisionStatus::kCollision;
    result.message = "collision";
  }
  return result;
}

}  // namespace wbmm::collision
