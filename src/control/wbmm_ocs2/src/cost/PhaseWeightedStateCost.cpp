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

#include "wbmm_ocs2/cost/PhaseWeightedStateCost.h"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace wbmm_ocs2
{

using namespace ocs2;

PhaseWeightedStateCost::PhaseWeightedStateCost(
    std::unique_ptr<StateCost> cost,
    std::shared_ptr<const WbmmReferenceManager> referenceManager,
    TargetKind targetKind,
    std::array<scalar_t, 4> phaseWeights)
    : cost_(std::move(cost)),
      referenceManager_(std::move(referenceManager)),
      targetKind_(targetKind),
      phaseWeights_(phaseWeights)
{
    if (cost_ == nullptr) {
        throw std::invalid_argument(
            "[PhaseWeightedStateCost] cost must not be null.");
    }
    if (referenceManager_ == nullptr) {
        throw std::invalid_argument(
            "[PhaseWeightedStateCost] referenceManager must not be null.");
    }
    for (const scalar_t weight : phaseWeights_) {
        if (!std::isfinite(weight) || weight < 0.0) {
            throw std::invalid_argument(
                "[PhaseWeightedStateCost] phase weights must be finite and "
                "non-negative.");
        }
    }
}

PhaseWeightedStateCost::PhaseWeightedStateCost(
    const PhaseWeightedStateCost& other)
    : StateCost(other),
      cost_(other.cost_->clone()),
      referenceManager_(other.referenceManager_),
      targetKind_(other.targetKind_),
      phaseWeights_(other.phaseWeights_)
{
}

PhaseWeightedStateCost* PhaseWeightedStateCost::clone() const
{
    return new PhaseWeightedStateCost(*this);
}

bool PhaseWeightedStateCost::isActive(scalar_t time) const
{
    if (weight() <= 0.0 || !cost_->isActive(time))
    {
        return false;
    }

    // An end-effector cost must not become active from a stale buffer when a
    // new navigation goal has cleared the previous tracking target.
    if (targetKind_ == TargetKind::kEndEffector &&
        referenceManager_->getEndEffectorTarget().empty())
    {
        return false;
    }

    return true;
}

scalar_t PhaseWeightedStateCost::getValue(
    scalar_t time, const vector_t& state,
    const TargetTrajectories& /*targetTrajectories*/,
    const PreComputation& preComputation) const
{
    return weight() * cost_->getValue(
        time, state, targetForPhase(), preComputation);
}

ScalarFunctionQuadraticApproximation
PhaseWeightedStateCost::getQuadraticApproximation(
    scalar_t time, const vector_t& state,
    const TargetTrajectories& /*targetTrajectories*/,
    const PreComputation& preComputation) const
{
    auto approximation = cost_->getQuadraticApproximation(
        time, state, targetForPhase(), preComputation);
    const scalar_t w = weight();
    approximation.f *= w;
    approximation.dfdx *= w;
    approximation.dfdxx *= w;
    return approximation;
}

const TargetTrajectories& PhaseWeightedStateCost::targetForPhase() const
{
    return targetKind_ == TargetKind::kWholeBody
        ? referenceManager_->getWholeBodyTarget()
        : referenceManager_->getEndEffectorTarget();
}

scalar_t PhaseWeightedStateCost::weight() const
{
    const auto phase = static_cast<std::size_t>(
        referenceManager_->getTaskPhase());
    if (phase >= phaseWeights_.size()) {
        throw std::out_of_range(
            "[PhaseWeightedStateCost] task phase is out of range.");
    }
    return phaseWeights_[phase];
}

}  // namespace wbmm_ocs2
