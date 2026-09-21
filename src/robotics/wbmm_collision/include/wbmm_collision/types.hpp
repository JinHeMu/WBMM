#pragma once

#include <Eigen/Core>

#include <limits>
#include <string>

namespace wbmm::collision
{

enum class CollisionGroup {kBase, kArm};
enum class CheckScope {kBase, kArm, kWholeBody};

// Geometry only: center is in link_name, radius is in m. FK belongs to RobotModel.
struct CollisionSphere
{
  std::string name;
  std::string link_name;
  CollisionGroup group{CollisionGroup::kArm};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
};

struct CollisionCheckOptions
{
  // Supplied by the caller; zero is a geometric boundary, not a validated safety margin.
  double safety_margin{0.0};
};

enum class CollisionStatus
{
  kNotImplemented = 0,
  kFree,
  kCollision,
  kInvalidInput,
  kFrameMismatch,
  kUnknownSpace,
  kOutOfBounds,
  kModelError,
};

struct CollisionResult
{
  CollisionStatus status{CollisionStatus::kNotImplemented};
  // Minimum d(center) - radius - safety_margin, in m; valid only for Free/Collision.
  double min_clearance{std::numeric_limits<double>::quiet_NaN()};
  std::string closest_sphere;
  std::string message{"TBD: environment collision checking is not implemented"};

  // Errors, unknown space and placeholders must never become a valid search node.
  [[nodiscard]] bool isFree() const noexcept {return status == CollisionStatus::kFree;}
};

}  // namespace wbmm::collision
