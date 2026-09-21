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

#include <cstddef>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "wbmm_ocs2/WbmmModelInfo.h"

namespace wbmm_ocs2
{

/**
 * 机械臂/全身可操作度代价的配置。
 *
 * 所有 hinge 项都是“低于参考值才惩罚”的一侧二次：
 *   L = 0.5 * weight * max(0, reference - metric)^2
 *
 * 另外提供 inverseManipulability 和 conditionNumber 两个可选诊断/抑制项。
 */
struct ArmManipulabilitySettings
{
    enum class Scope
    {
        kArmOnly,
        kWholeBody,
    };

    Scope scope{Scope::kArmOnly};
    std::string frameName;              // 为空时使用 WbmmModelInfo::eeFrame
    std::size_t armStartIndex{3};       // state 中 q1 的下标
    std::size_t armDim{6};              // 机械臂关节数

    bool useMinSingularValue{true};
    ocs2::scalar_t minSingularWeight{1.0};
    ocs2::scalar_t minSingularRef{0.05};

    bool useYoshikawa{false};
    ocs2::scalar_t yoshikawaWeight{0.1};
    ocs2::scalar_t yoshikawaRef{0.1};

    bool useTaskDirection{false};
    ocs2::vector_t taskDirection{ocs2::vector_t::Zero(6)};
    ocs2::scalar_t taskDirectionWeight{1.0};
    ocs2::scalar_t taskDirectionRef{0.1};

    bool useInverseManipulability{false};
    ocs2::scalar_t inverseManipulabilityWeight{1e-3};

    bool useConditionNumber{false};
    ocs2::scalar_t conditionWeight{1e-3};
    ocs2::scalar_t conditionMax{50.0};

    ocs2::scalar_t regularization{1e-6};
    ocs2::scalar_t finiteDiffStep{1e-6};
    ocs2::scalar_t hessianRegularization{1e-6};
};

/**
 * 基于 Pinocchio 末端 frame Jacobian 的可操作度 state cost。
 *
 * 支持两种 scope：
 *   - kArmOnly: 取末端 frame Jacobian 的机械臂列，得到 6xarmDim；
 *   - kWholeBody: 将底盘输入映射到 [v, omega] 后，得到 6xinputDim。
 *
 * 指标：
 *   - sigmaMin / sigmaMax / conditionNumber
 *   - yoshikawa = prod(singularValues) = sqrt(det(J J^T))
 *   - taskDirection = sqrt(d^T J J^T d)
 *   - inverseManipulability = trace((J J^T + eps I)^-1)
 *
 * 梯度由中心差分得到；Hessian 使用 Gauss-Newton/对角正则近似，保证 PSD。
 */
class ArmManipulabilityCost final : public ocs2::StateCost
{
public:
    struct Metrics
    {
        ocs2::scalar_t yoshikawa{0.0};
        ocs2::scalar_t sigmaMin{0.0};
        ocs2::scalar_t sigmaMax{0.0};
        ocs2::scalar_t taskDirection{0.0};
        ocs2::scalar_t inverseManipulability{0.0};
        ocs2::scalar_t condition{0.0};
        bool valid{false};
    };

    ArmManipulabilityCost(const ocs2::PinocchioInterface& pinocchioInterface,
                          WbmmModelInfo modelInfo,
                          ArmManipulabilitySettings settings);

    ~ArmManipulabilityCost() override = default;

    ArmManipulabilityCost* clone() const override;

    ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
                            const ocs2::TargetTrajectories& targetTrajectories,
                            const ocs2::PreComputation& preComputation) const override;

    ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
        ocs2::scalar_t time, const ocs2::vector_t& state,
        const ocs2::TargetTrajectories& targetTrajectories,
        const ocs2::PreComputation& preComputation) const override;

    /** 暴露指标计算，供日志、可视化和单元测试使用。 */
    Metrics computeMetrics(const ocs2::vector_t& state) const;

    const ArmManipulabilitySettings& settings() const noexcept {return settings_;}

private:
    ArmManipulabilityCost(const ArmManipulabilityCost& other) = default;

    ocs2::scalar_t costFromMetrics(const Metrics& metrics) const;
    std::vector<int> activeStateIndices() const;
    ocs2::matrix_t buildTaskJacobian(
        const ocs2::matrix_t& frameJacobian,
        const ocs2::vector_t& state) const;

    ocs2::PinocchioInterface pinocchioInterface_;
    WbmmModelInfo modelInfo_;
    ArmManipulabilitySettings settings_;
    std::size_t eeFrameId_{0};
};

}  // namespace wbmm_ocs2
