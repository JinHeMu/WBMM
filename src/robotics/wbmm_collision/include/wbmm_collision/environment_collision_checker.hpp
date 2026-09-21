#pragma once

#include "wbmm_collision/collision_model.hpp"
#include "wbmm_core/robot_model.hpp"
#include "wbmm_environment/esdf_grid.hpp"

#include <memory>

namespace wbmm::collision
{

class EnvironmentCollisionChecker
{
public:
  EnvironmentCollisionChecker(
    wbmm::core::RobotModelPtr robot_model,
    std::shared_ptr<const wbmm::environment::EsdfGrid> environment,
    CollisionModel collision_model,
    CollisionCheckOptions options = {});

  // Matches the data available in wbmm_search::BaseCollisionChecker.
  // Base geometry only; it does not certify the arm's navigation configuration.
  [[nodiscard]] CollisionResult checkBase(
    const wbmm::core::Header & header, const wbmm::core::BaseState & base) const;

  // Environment only. No self-collision or continuous trajectory checks.
  [[nodiscard]] CollisionResult check(
    const wbmm::core::WholeBodyState & state,
    CheckScope scope = CheckScope::kWholeBody) const;

private:
  wbmm::core::RobotModelPtr robot_model_;
  std::shared_ptr<const wbmm::environment::EsdfGrid> environment_;
  CollisionModel collision_model_;
  CollisionCheckOptions options_;
};

}  // namespace wbmm::collision
