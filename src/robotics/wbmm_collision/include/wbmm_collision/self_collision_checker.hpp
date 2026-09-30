#pragma once

#include "wbmm_collision/check_types.hpp"
#include "wbmm_collision/trajectory_spheres.hpp"

#include <limits>
#include <string>
#include <vector>

namespace wbmm::collision
{

struct SelfCollisionPair
{
  std::string first_sphere;
  std::string second_sphere;
  // |p_a - p_b| - r_a - r_b - safety_margin, in metres.
  double clearance{0.0};
  // d(clearance)/dz, z = [x, y, vx, vy, q_1..q_n].
  // At coincident centers the norm has no unique derivative; use zero.
  Eigen::VectorXd gradient;
};

struct SelfCollisionResult
{
  CollisionStatus status{CollisionStatus::kInvalidInput};
  // +infinity means there are no eligible pairs. NaN means invalid input.
  double min_clearance{std::numeric_limits<double>::quiet_NaN()};
  std::string closest_first_sphere;
  std::string closest_second_sphere;
  // All eligible pairs, including free pairs, in the existing planner order.
  std::vector<SelfCollisionPair> pairs;
  std::string message;

  [[nodiscard]] bool isFree() const noexcept {return status == CollisionStatus::kFree;}
};

// Reuses evaluateTrajectorySpheres() output without repeating FK or ESDF queries.
// Matches the planner's existing policy: all arm/base pairs; arm/arm pairs only
// when group indices differ by at least two. No base/base or same/adjacent arm
// group checks. Touching the supplied margin counts as collision (clearance <= 0).
[[nodiscard]] SelfCollisionResult checkSelfCollision(
  const TrajectorySpheres & spheres, double safety_margin = 0.0);

}  // namespace wbmm::collision
