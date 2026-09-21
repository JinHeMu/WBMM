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

#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>
#include <ocs2_sphere_approximation/PinocchioSphereKinematics.h>

#include "wbmm_ocs2/collision/EsdfEnvironmentInterface.h"

namespace wbmm_ocs2
{

/**
 * ESDF 环境碰撞软约束。
 *
 * 约束值：
 *   h_i = d_i - r_i - margin_i
 *
 * 其中 d_i 是第 i 个碰撞球中心处的 ESDF 距离。
 *
 * 线性近似：
 *   dh_i/dx = J_{p_i}(x)^T grad(d_i)
 */
class EsdfEnvironmentCollisionConstraint final
    : public ocs2::StateConstraint
{
public:
    EsdfEnvironmentCollisionConstraint(
        const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
        std::shared_ptr<const EsdfEnvironmentInterface> environment,
        ocs2::scalar_t defaultMinimumDistance);

    ~EsdfEnvironmentCollisionConstraint() override = default;

    EsdfEnvironmentCollisionConstraint* clone() const override;

    std::size_t getNumConstraints(ocs2::scalar_t time) const override;

    ocs2::vector_t getValue(
        ocs2::scalar_t time, const ocs2::vector_t& state,
        const ocs2::PreComputation& preComputation) const override;

    ocs2::VectorFunctionLinearApproximation getLinearApproximation(
        ocs2::scalar_t time, const ocs2::vector_t& state,
        const ocs2::PreComputation& preComputation) const override;

    std::shared_ptr<const EsdfEnvironmentInterface> getEnvironment() const
    {
        return environment_;
    }

private:
    EsdfEnvironmentCollisionConstraint(
        const EsdfEnvironmentCollisionConstraint& other);

    const ocs2::PinocchioInterface& getPinocchioInterface(
        const ocs2::PreComputation& preComputation) const;

    std::shared_ptr<const EsdfEnvironmentInterface> environment_;
    std::unique_ptr<ocs2::PinocchioSphereKinematics> sphereKinematics_;
    ocs2::scalar_t defaultMinimumDistance_;
};

}  // namespace wbmm_ocs2
