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

#include <wbmm_core/robot_model.hpp>
#include <wbmm_robot_metrics/arm_metrics.hpp>

#include "wbmm_ocs2/WbmmModelInfo.h"

namespace wbmm_ocs2
{

/**
 * 机械臂/全身可操作度代价的配置。
 *
 * 指标计算（Jacobian 选择、SVD、奇异值、条件数、操作度、逆操作度、
 * task-direction 操作度、关节限位裕量）全部委托给 wbmm_robot_metrics；
 * 本结构只保存 OCS2 代价语义：启用开关、参考阈值、权重和数值差分参数。
 *
 * 所有 hinge 项都是“低于参考值才惩罚”的一侧二次：
 *   L = 0.5 * weight * max(0, reference - metric)^2
 *
 * 另外提供 inverseManipulability 和 conditionNumber 两个可选诊断/抑制项。
 */
struct ArmManipulabilitySettings
{
    std::string frameName;  // 为空时使用 WbmmModelInfo::eeFrame
    // OCS2 x/y/yaw 所在的规划坐标系；不是 URDF base link。
    std::string stateFrame{"odom"};

    // Jacobian 选择、任务维度、量纲缩放和数值正则统一由
    // wbmm_robot_metrics 定义，OCS2 不再复制这些接口。
    wbmm::metrics::ArmMetricsOptions metricsOptions;

    bool useMinSingularValue{true};
    ocs2::scalar_t minSingularWeight{1.0};
    ocs2::scalar_t minSingularRef{0.05};

    bool useYoshikawa{false};
    ocs2::scalar_t yoshikawaWeight{0.1};
    ocs2::scalar_t yoshikawaRef{0.1};

    ocs2::scalar_t taskDirectionWeight{1.0};
    ocs2::scalar_t taskDirectionRef{0.1};

    bool useInverseManipulability{false};
    ocs2::scalar_t inverseManipulabilityWeight{1e-3};

    bool useConditionNumber{false};
    ocs2::scalar_t conditionWeight{1e-3};
    ocs2::scalar_t conditionMax{50.0};

    ocs2::scalar_t finiteDiffStep{1e-6};
    ocs2::scalar_t hessianRegularization{1e-6};
    // 评价失败时不得默认变成“最优的零代价”。
    ocs2::scalar_t invalidMetricsPenalty{1e6};
};

/**
 * 基于 wbmm_robot_metrics 构型评价的 OCS2 可操作度 state cost。
 *
 * Pinocchio 运动学统一通过 OCS2 PinocchioInterface 与
 * WbmmPinocchioMapping 完成；指标公式由 wbmm_robot_metrics 完成。本类只负责：
 *   - 从 OCS2 PinocchioInterface 计算并映射 6x8 Jacobian；
 *   - 将 settings.metricsOptions 原样交给公共评价包；
 *   - 把 metrics 组装成 hinge / linear cost；
 *   - 用中心差分组装 PSD 近似梯度。
 */
class ArmManipulabilityCost final : public ocs2::StateCost
{
public:
    using Metrics = wbmm::metrics::ArmKinematicMetrics;

    /** OCS2 内部唯一的模型入口，避免再经 RobotModel 重复计算运动学。 */
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

    /** 暴露当前构型的评价结果，供日志、可视化和单元测试使用。 */
    Metrics computeMetrics(const ocs2::vector_t& state) const;

    const ArmManipulabilitySettings& settings() const noexcept {return settings_;}

private:
    ArmManipulabilityCost(const ArmManipulabilityCost& other) = default;

    wbmm::core::JointState toJointState(
        const ocs2::vector_t& state) const;
    ocs2::scalar_t costFromMetrics(const Metrics& metrics) const;
    std::vector<int> activeStateIndices() const;
    std::size_t armStateStartIndex() const noexcept;

    ocs2::PinocchioInterface pinocchioInterface_;
    WbmmModelInfo modelInfo_;
    ArmManipulabilitySettings settings_;
    wbmm::core::RobotLimits limits_;
    std::size_t endEffectorFrameId_{0U};
};

}  // namespace wbmm_ocs2
