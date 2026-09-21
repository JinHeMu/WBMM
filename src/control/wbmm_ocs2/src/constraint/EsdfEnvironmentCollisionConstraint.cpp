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

#include "wbmm_ocs2/constraint/EsdfEnvironmentCollisionConstraint.h"

#include "wbmm_ocs2/PreComputation.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace wbmm_ocs2
{

using namespace ocs2;

EsdfEnvironmentCollisionConstraint::EsdfEnvironmentCollisionConstraint(
    const PinocchioStateInputMapping<scalar_t>& mapping,
    std::shared_ptr<const EsdfEnvironmentInterface> environment,
    scalar_t defaultMinimumDistance)
    : StateConstraint(ConstraintOrder::Linear),
      environment_(std::move(environment)),
      sphereKinematics_(nullptr),
      defaultMinimumDistance_(defaultMinimumDistance)
{
    if (environment_ == nullptr) {
        throw std::invalid_argument(
            "[EsdfEnvironmentCollisionConstraint] environment must not be "
            "null.");
    }
    if (!std::isfinite(defaultMinimumDistance_) ||
        defaultMinimumDistance_ < 0.0) {
        throw std::invalid_argument(
            "[EsdfEnvironmentCollisionConstraint] defaultMinimumDistance "
            "must be finite and non-negative.");
    }

    sphereKinematics_ = std::make_unique<PinocchioSphereKinematics>(
        environment_->getSphereInterface(), mapping);
}

EsdfEnvironmentCollisionConstraint::EsdfEnvironmentCollisionConstraint(
    const EsdfEnvironmentCollisionConstraint& other)
    : StateConstraint(other),
      environment_(other.environment_),
      sphereKinematics_(other.sphereKinematics_->clone()),
      defaultMinimumDistance_(other.defaultMinimumDistance_)
{
}

EsdfEnvironmentCollisionConstraint*
EsdfEnvironmentCollisionConstraint::clone() const
{
    return new EsdfEnvironmentCollisionConstraint(*this);
}

std::size_t EsdfEnvironmentCollisionConstraint::getNumConstraints(
    scalar_t /*time*/) const
{
    return environment_->getNumSpheres();
}

vector_t EsdfEnvironmentCollisionConstraint::getValue(
    scalar_t /*time*/, const vector_t& /*state*/,
    const PreComputation& preComputation) const
{
    const auto& pinocchioInterface = getPinocchioInterface(preComputation);
    const auto distances = environment_->computeDistances(pinocchioInterface);

    vector_t constraints(static_cast<Eigen::Index>(distances.size()));
    for (std::size_t i = 0; i < distances.size(); ++i) {
        constraints(static_cast<Eigen::Index>(i)) =
            distances[i].distance - distances[i].radius -
            distances[i].minimumDistance;
    }
    return constraints;
}

VectorFunctionLinearApproximation
EsdfEnvironmentCollisionConstraint::getLinearApproximation(
    scalar_t /*time*/, const vector_t& state,
    const PreComputation& preComputation) const
{
    const auto& pinocchioInterface = getPinocchioInterface(preComputation);
    sphereKinematics_->setPinocchioInterface(pinocchioInterface);

    const auto sphereCenters =
        sphereKinematics_->getPositionLinearApproximation(state);
    const auto distances = environment_->computeDistances(pinocchioInterface);

    if (sphereCenters.size() != distances.size()) {
        throw std::runtime_error(
            "[EsdfEnvironmentCollisionConstraint] Sphere center and distance "
            "metadata size mismatch.");
    }

    VectorFunctionLinearApproximation approximation(
        static_cast<Eigen::Index>(distances.size()),
        static_cast<Eigen::Index>(state.rows()), 0);

    for (std::size_t i = 0; i < distances.size(); ++i) {
        const Eigen::Index row = static_cast<Eigen::Index>(i);

        approximation.f(row) =
            distances[i].distance - distances[i].radius -
            distances[i].minimumDistance;

        if (distances[i].gradientValid) {
            approximation.dfdx.row(row) =
                distances[i].gradient.transpose() *
                sphereCenters[i].dfdx;
        } else {
            approximation.dfdx.row(row).setZero();
        }
    }

    return approximation;
}

const PinocchioInterface&
EsdfEnvironmentCollisionConstraint::getPinocchioInterface(
    const PreComputation& preComputation) const
{
    return cast<WbmmPreComputation>(preComputation).getPinocchioInterface();
}

}  // namespace wbmm_ocs2
