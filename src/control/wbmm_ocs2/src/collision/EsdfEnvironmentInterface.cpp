/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "wbmm_ocs2/collision/EsdfEnvironmentInterface.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace wbmm_ocs2
{

using namespace ocs2;

EsdfEnvironmentInterface::EsdfEnvironmentInterface(
    const PinocchioInterface& pinocchioInterface,
    std::shared_ptr<const wbmm::environment::EsdfGrid> grid,
    const std::vector<std::string>& collisionLinks,
    const std::vector<scalar_t>& maxExcesses,
    scalar_t shrinkRatio,
    scalar_t defaultMinimumDistance)
    : grid_(std::move(grid)),
      sphereInterface_(pinocchioInterface, collisionLinks, maxExcesses, shrinkRatio),
      defaultMinimumDistance_(defaultMinimumDistance),
      frameId_(grid_ != nullptr ? grid_->info().frame_id : std::string())
{
    if (grid_ == nullptr) {
        throw std::invalid_argument(
            "[EsdfEnvironmentInterface] ESDF grid must not be null.");
    }
    if (collisionLinks.empty()) {
        throw std::invalid_argument(
            "[EsdfEnvironmentInterface] collisionLinks must not be empty.");
    }
    if (maxExcesses.empty()) {
        throw std::invalid_argument(
            "[EsdfEnvironmentInterface] maxExcesses must not be empty.");
    }
    if (!std::isfinite(defaultMinimumDistance_) ||
        defaultMinimumDistance_ < 0.0) {
        throw std::invalid_argument(
            "[EsdfEnvironmentInterface] defaultMinimumDistance must be finite "
            "and non-negative.");
    }
    if (!(shrinkRatio > 0.0) || !(shrinkRatio < 1.0)) {
        throw std::invalid_argument(
            "[EsdfEnvironmentInterface] shrinkRatio must be in (0, 1).");
    }

    sphereRadii_ = sphereInterface_.getSphereRadii();
    sphereLinks_.clear();
    sphereLinks_.reserve(sphereInterface_.getNumSpheresInTotal());

    const auto& primitiveLinks =
        sphereInterface_.getCollisionLinkOfEachPrimitveShape();
    const auto& numSpheres = sphereInterface_.getNumSpheres();
    for (std::size_t i = 0; i < sphereInterface_.getNumPrimitiveShapes(); ++i) {
        for (std::size_t j = 0; j < numSpheres[i]; ++j) {
            sphereLinks_.push_back(primitiveLinks[i]);
        }
    }

    if (sphereRadii_.size() != sphereLinks_.size()) {
        throw std::runtime_error(
            "[EsdfEnvironmentInterface] Sphere radius/link metadata mismatch.");
    }
}

std::vector<EsdfEnvironmentInterface::DistanceResult>
EsdfEnvironmentInterface::computeDistances(
    const PinocchioInterface& pinocchioInterface) const
{
    const std::vector<Eigen::Vector3d> sphereCenters =
        sphereInterface_.computeSphereCentersInWorldFrame(pinocchioInterface);

    std::vector<DistanceResult> results;
    results.reserve(sphereCenters.size());

    for (std::size_t i = 0; i < sphereCenters.size(); ++i) {
        const auto query = grid_->query(frameId_, sphereCenters[i]);

        DistanceResult result;
        result.radius = sphereRadii_[i];
        result.minimumDistance = defaultMinimumDistance_;
        result.linkName = sphereLinks_[i];

        if (query.status == wbmm::environment::QueryStatus::kSuccess &&
            std::isfinite(query.distance) && query.gradient_valid) {
            result.distance = query.distance;
            result.gradient = query.gradient;
            result.gradientValid = true;
        } else {
            // Unknown/out-of-bounds/frame-mismatch must not be treated as free.
            // Keep the constraint active, but do not expose a bogus gradient.
            result.distance = -defaultMinimumDistance_;
            result.gradient = Eigen::Vector3d::Zero();
            result.gradientValid = false;
        }

        results.push_back(std::move(result));
    }

    return results;
}

std::size_t EsdfEnvironmentInterface::getNumSpheres() const
{
    return sphereInterface_.getNumSpheresInTotal();
}

const PinocchioSphereInterface&
EsdfEnvironmentInterface::getSphereInterface() const
{
    return sphereInterface_;
}

std::string EsdfEnvironmentInterface::getFrameId() const
{
    return frameId_;
}

scalar_t EsdfEnvironmentInterface::getDefaultMinimumDistance() const
{
    return static_cast<scalar_t>(defaultMinimumDistance_);
}

}  // namespace wbmm_ocs2
