#include "wbmm_collision/environment_collision_checker.hpp"

#include <utility>

namespace wbmm::collision
{

EnvironmentCollisionChecker::EnvironmentCollisionChecker(
  wbmm::core::RobotModelPtr robot_model,
  std::shared_ptr<const wbmm::environment::EsdfGrid> environment,
  CollisionModel collision_model,
  CollisionCheckOptions options)
: robot_model_(std::move(robot_model)),
  environment_(std::move(environment)),
  collision_model_(std::move(collision_model)),
  options_(options)
{}

CollisionResult EnvironmentCollisionChecker::checkBase(
  const wbmm::core::Header &, const wbmm::core::BaseState &) const
{
  // TBD: base-only geometry placement without inventing an arm joint state.
  return {};
}

CollisionResult EnvironmentCollisionChecker::check(
  const wbmm::core::WholeBodyState &, CheckScope) const
{
  // TBD: validate state/frame/geometry, FK spheres, query ESDF and aggregate clearance.
  return {};
}

}  // namespace wbmm::collision
