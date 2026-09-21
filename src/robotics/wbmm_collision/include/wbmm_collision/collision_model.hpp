#pragma once

#include "wbmm_collision/types.hpp"

#include <vector>

namespace wbmm::collision
{

// A value type, not a second kinematics model. Robot-specific spheres are TBD.
struct CollisionModel
{
  std::vector<CollisionSphere> spheres;
};

}  // namespace wbmm::collision
