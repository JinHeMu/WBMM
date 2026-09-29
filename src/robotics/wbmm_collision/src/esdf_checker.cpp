#include "wbmm_collision/esdf_checker.hpp"

#include "wbmm_core/validation.hpp"

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace wbmm::collision
{
namespace
{

bool hasTag(const std::vector<std::string> & tags, const std::string & tag)
{
  return std::find(tags.begin(), tags.end(), tag) != tags.end();
}

bool appliesToScope(
  const std::vector<std::string> & tags, CheckScope scope)
{
  const bool is_base = hasTag(tags, "base");
  const bool is_arm = !is_base || hasTag(tags, "arm");
  switch (scope) {
    case CheckScope::kBase:
      return is_base;
    case CheckScope::kArm:
      return is_arm;
    case CheckScope::kWholeBody:
      return true;
  }
  return false;
}

const char * scopeName(CheckScope scope)
{
  return scope == CheckScope::kBase ? "kBase" :
         scope == CheckScope::kArm ? "kArm" : "kWholeBody";
}

}  // namespace

EsdfChecker::EsdfChecker(
  wbmm::pinocchio::KinematicModelPtr kinematic_model,
  std::shared_ptr<const wbmm::environment::EsdfGrid> environment,
  const wbmm::robot_model::CollisionSphereModel & collision_model,
  CollisionCheckOptions options)
: model_(std::move(kinematic_model)),
  environment_(std::move(environment)),
  options_(options)
{
  if (model_ == nullptr) {
    model_message_ = "KinematicModel is null.";
    return;
  }
  kinematics_data_ = std::make_unique<wbmm::pinocchio::KinematicsData>(model_);

  if (!collision_model.success) {
    model_message_ = "CollisionSphereModel is not valid: " + collision_model.message;
    return;
  }
  if (collision_model.sphereCount() == 0U) {
    model_message_ = "CollisionSphereModel contains no collision spheres.";
    return;
  }
  try {
    sphere_kinematics_ = std::make_unique<wbmm::pinocchio::SphereKinematics>(
      model_, collision_model);
  } catch (const std::exception & exception) {
    model_message_ = std::string("Cannot build sphere kinematics: ") +
      exception.what();
  }
}

std::size_t EsdfChecker::sphereCount() const noexcept
{
  return sphere_kinematics_ == nullptr ? 0U : sphere_kinematics_->sphereCount();
}

CollisionResult EsdfChecker::checkBase(
  const wbmm::core::Header & header, const wbmm::core::BaseState & base) const
{
  // The search interface only exposes the base pose. Build a structurally valid
  // whole-body state with zero arm positions: the base spheres live before the
  // arm chain, so the arm values do not affect them.
  wbmm::core::WholeBodyState state;
  state.header = header;
  state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
  state.base = base;
  if (model_ != nullptr) {
    state.joints.names = model_->controlledJointNames();
    state.joints.positions.assign(state.joints.names.size(), 0.0);
  }

  return checkWithScope(state, CheckScope::kBase, false);
}

CollisionResult EsdfChecker::check(
  const wbmm::core::WholeBodyState & state, CheckScope scope) const
{
  return checkWithScope(state, scope, true);
}

CollisionResult EsdfChecker::checkWithScope(
  const wbmm::core::WholeBodyState & state, CheckScope scope,
  bool validate_state) const
{
  CollisionResult result;
  result.message.clear();

  if (model_ == nullptr) {
    result.status = CollisionStatus::kModelError;
    result.message = "KinematicModel is null.";
    return result;
  }
  if (environment_ == nullptr) {
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
  if (sphere_kinematics_ == nullptr) {
    result.status = CollisionStatus::kInvalidInput;
    result.message = model_message_.empty()
      ? "CollisionSphereModel contains no collision spheres."
      : model_message_;
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
    if (!model_->validateState(state, &model_message)) {
      result.status = CollisionStatus::kInvalidInput;
      result.message = model_message.empty()
        ? "KinematicModel rejected the whole-body state."
        : model_message;
      return result;
    }
  }

  // One full-tree forward kinematics for the whole query; every sphere center
  // is then read from the same Context.
  if (!kinematics_data_->update(state)) {
    result.status = CollisionStatus::kModelError;
    result.message = "Forward kinematics failed: " + kinematics_data_->lastMessage();
    return result;
  }
  std::vector<wbmm::pinocchio::SphereSample> samples;
  if (!sphere_kinematics_->centers(*kinematics_data_, samples)) {
    result.status = CollisionStatus::kModelError;
    result.message = "Cannot compute collision sphere centers.";
    return result;
  }

  double min_clearance = std::numeric_limits<double>::infinity();
  std::string closest_sphere;
  std::size_t evaluated_spheres = 0U;

  for (const auto & sphere : samples) {
    if (!appliesToScope(sphere.group_tags, scope)) {
      continue;
    }

    if (sphere.id.empty() || sphere.owner_link.empty() ||
      !sphere.center.array().isFinite().all() || !std::isfinite(sphere.radius) ||
      sphere.radius <= 0.0)
    {
      result.status = CollisionStatus::kInvalidInput;
      result.message = "Invalid collision sphere metadata.";
      return result;
    }

    const auto query = environment_->query(state.header.frame_id, sphere.center);
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

    // Unobserved corners are provenance, not invalidity. Only a caller that
    // opted into conservative handling rejects them.
    if (!query.fully_observed && options_.treat_unknown_as_occupied) {
      result.status = CollisionStatus::kUnknownSpace;
      result.message =
        "Sphere '" + sphere.id + "' reaches unobserved space and "
        "treat_unknown_as_occupied is enabled.";
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
      closest_sphere = sphere.id;
    }
    ++evaluated_spheres;
  }

  if (evaluated_spheres == 0U) {
    result.status = CollisionStatus::kInvalidInput;
    result.message = std::string("No collision spheres apply to scope ") +
      scopeName(scope) + ".";
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
