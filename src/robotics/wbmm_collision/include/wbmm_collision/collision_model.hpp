#pragma once

#include "wbmm_collision/types.hpp"

#include <vector>

namespace wbmm::collision
{

// A value type, not a second kinematics model. The caller supplies the
// robot-specific sphere approximation; this package only evaluates it.
struct CollisionModel
{
  std::vector<CollisionSphere> spheres;
};

}  // namespace wbmm::collision
