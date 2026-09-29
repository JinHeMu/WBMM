#pragma once

#include <Eigen/Core>

#include <limits>
#include <string>

namespace wbmm::collision
{

// 碰撞检查的几何范围。球的分组来自共享 CollisionSphereModel 的 group_tags：
// "base" 属于底盘，其余（"arm" 或关节名）属于机械臂。
enum class CheckScope {kBase, kArm, kWholeBody};

struct CollisionCheckOptions
{
  // Supplied by the caller; zero is a geometric boundary, not a validated safety margin.
  double safety_margin{0.0};

  // When true, a query whose interpolation stencil touches unobserved space is
  // reported as kUnknownSpace (not free) instead of being trusted.
  //
  // Default false, matching both the deployed map1 (whose own metadata says
  // unknown_is_occupied = false and whose unobserved voxels all hold the ESDF
  // clamp maximum) and the REMANI behaviour this planner replaces. Turning it
  // on is strictly more conservative but rejects roughly 89% of map1, so it is
  // only usable with a densely observed map.
  bool treat_unknown_as_occupied{false};
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
