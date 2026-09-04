#pragma once

#include "wipe_planner/planner.hpp"

#include <Eigen/Core>

#include <cmath>
#include <stdexcept>

namespace wipe_planner
{

// Rigid planar transform from a source frame into a target frame.  The
// translation is expressed in the target frame and yaw is target_yaw-source_yaw.
struct PlanarTransform
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

inline Eigen::VectorXd transformWholeBodyState(
  const Eigen::VectorXd & source, const PlanarTransform & target_from_source)
{
  if (source.size() != 9) {
    throw std::invalid_argument(
      "Whole-body frame conversion requires a 9D state");
  }
  const double cosine = std::cos(target_from_source.yaw);
  const double sine = std::sin(target_from_source.yaw);
  Eigen::VectorXd target = source;
  target[0] = target_from_source.x + cosine * source[0] - sine * source[1];
  target[1] = target_from_source.y + sine * source[0] + cosine * source[1];
  target[2] = wrapAngle(source[2] + target_from_source.yaw);
  // States 3..8 are joint angles and therefore frame invariant.
  return target;
}

inline PlanarTransform inversePlanarTransform(const PlanarTransform & transform)
{
  const double cosine = std::cos(transform.yaw);
  const double sine = std::sin(transform.yaw);
  PlanarTransform inverse;
  inverse.yaw = wrapAngle(-transform.yaw);
  inverse.x = -cosine * transform.x - sine * transform.y;
  inverse.y = sine * transform.x - cosine * transform.y;
  return inverse;
}

}  // namespace wipe_planner
