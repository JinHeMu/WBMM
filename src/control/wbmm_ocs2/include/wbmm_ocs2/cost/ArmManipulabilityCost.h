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
#include <mutex>
#include <string>
#include <vector>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <pinocchio/math/tensor.hpp>

#include <wbmm_core/robot_model.hpp>
#include <wbmm_robot_metrics/arm_metrics.hpp>

#include "wbmm_ocs2/WbmmModelInfo.h"

namespace wbmm_ocs2
{

/**
 * 机械臂/全身可操作度代价的配置。
 *
 * 公共指标类型/配置使用 wbmm_robot_metrics；求解器省去无关诊断和矩阵求逆，
 * computeMetrics() 提供包含关节限位裕量的完整诊断结果。
 *
 * 在 TCP 原点、stateFrame 轴向表达的 J=[linear; angular] 中，arm scope
 * 只取最后六列 qdot，不能用底盘冗余掩盖机械臂奇异。pose task 使用六行，
 * translation task 使用前三行。characteristic_length 模式对角速度行乘以
 * 固定长度 ell（m），即 Jbar=[Jv; ell*Jw]；raw pose 混合量纲，仅供旧配置。
 *
 * Jbar=U diag(sigma) V^T；sigma_min 是最弱方向的速度能力，
 * Yoshikawa 操作度 w=prod(sigma)=sqrt(det(Jbar*Jbar^T)) 是椭球体积尺度。
 * 两者不是同一指标，translation 与 pose、不同 ell 的阈值不能混用。
 * conditionNumber=sigma_max/max(sigma_min,floor)；inverseManipulability
 * 是 trace((Jbar*Jbar^T+regularization*I)^-1)，不是 1/w。
 * task-direction 指标 sqrt(d^T Jbar Jbar^T d) 是投影幅值，不能代替
 * sigma_min 或解释成严格沿 d 运动的最大速度。
 *
 * 所有下界 hinge 项都是“低于参考值才惩罚”的一侧二次：
 *   L = 0.5 * weight * max(0, reference - metric)^2
 * normalizeMargins=true 时除以 reference^2，使残差无量纲：
 *   L = 0.5 * weight * max(0, 1 - metric/reference)^2
 * 只归一化 sigma_min / Yoshikawa / task-direction 下界项，启用项的
 * reference 必须为正。weightScale 对整体代价及其导数应用同一缩放。
 * WbmmInterface 将独立实例注册为过程项和终端项；本类不选择任务模式。
 * 这是软偏好，不是奇异性硬约束，也不保证从完全奇异构型脱离。
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

    bool normalizeMargins{false};  // false preserves existing task.info semantics
    ocs2::scalar_t weightScale{1.0};

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

    // 仅在奇异值重合/为零等不可微位置，对线性化 Jacobian 做中心差分。
    ocs2::scalar_t finiteDiffStep{1e-6};
    ocs2::scalar_t hessianRegularization{1e-6};
    // 评价失败时不得默认变成“最优的零代价”。
    ocs2::scalar_t invalidMetricsPenalty{1e6};
};

/**
 * 基于 wbmm_robot_metrics 构型评价的 OCS2 可操作度 state cost。
 *
 * Pinocchio 运动学统一通过 OCS2 PinocchioInterface 与
 * [v,omega,qdot] 映射完成。每次近似只计算一次运动学和一次 SVD：
 * Pinocchio WORLD kinematic Hessian 给出 dJ/dq，再换到 TCP 原点、世界轴向。
 * d(sigma_i)/dq = u_i^T (dJbar/dq) v_i；Yoshikawa 用乘积求导，避免除以零。
 * inverseManipulability = sum_i 1/(sigma_i^2+regularization)。
 * hinge 项沿用 Gauss-Newton PSD 近似 weight*grad(metric)*grad(metric)^T，
 * 并保留原 hessianRegularization；没有改成代价的精确二阶 Hessian。
 * 重合/零奇异值处以 Jbar +/- finiteDiffStep*dJbar 做指标中心差分，
 * 不再扰动关节并重新计算运动学。完整诊断仍使用公共评价入口。
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
    // Carry the metric's frame/point/task/scaling contract into ROS diagnostics.
    std::string metricContract() const;

private:
    ArmManipulabilityCost(const ArmManipulabilityCost& other);

    wbmm::core::JointState toJointState(
        const ocs2::vector_t& state) const;
    ocs2::scalar_t costFromMetrics(const Metrics& metrics) const;
    ocs2::scalar_t marginWeight(ocs2::scalar_t weight,
                               ocs2::scalar_t reference) const;
    std::vector<int> activeStateIndices() const;
    std::size_t armStateStartIndex() const noexcept;
    bool hasActiveTerms(const Metrics& metrics) const;
    Metrics evaluateCostMetrics(const ocs2::vector_t& state,
                               ocs2::matrix_t* gradients = nullptr) const;
    ocs2::matrix_t taskJacobian(const ocs2::matrix_t& frameJacobian,
                               const ocs2::vector_t& state) const;

    // Each cost clone owns a workspace. Reuse it instead of copying the full
    // Pinocchio Data at every knot; serialize callers using the same instance.
    mutable ocs2::PinocchioInterface pinocchioInterface_;
    mutable pinocchio::Tensor<ocs2::scalar_t, 3> kinematicHessian_;
    mutable std::mutex workspaceMutex_;
    WbmmModelInfo modelInfo_;
    ArmManipulabilitySettings settings_;
    wbmm::core::RobotLimits limits_;
    std::size_t endEffectorFrameId_{0U};
};

}  // namespace wbmm_ocs2
