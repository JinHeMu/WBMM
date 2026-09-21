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

#include <atomic>
#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace wbmm_ocs2
{

/**
 * 末端位姿跟踪 state cost。
 *
 *   L = 0.5 * (p - p_d)^T Qp (p - p_d)
 *     + 0.5 * e_o^T Qo e_o
 *
 * 其中 e_o 是 ocs2::EndEffectorKinematics::getOrientationError() 返回的
 * 3 维姿态误差。
 *
 * 目标轨迹来自 ocs2::TargetTrajectories.stateTrajectory：
 *   - 单臂模式必须是 7D: [x, y, z, qx, qy, qz, qw]；
 *   - 空轨迹、维度不匹配、时间非法、四元数非法时，本 cost 返回 0，
 *     不产生梯度，避免模式切换瞬态把 9D 全身参考误解析成末端位姿。
 */
class EndEffectorTrackingCost final : public ocs2::StateCost
{
public:
    using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;
    using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;

    EndEffectorTrackingCost(const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
                            ocs2::matrix_t positionWeight,
                            ocs2::matrix_t orientationWeight);

    ~EndEffectorTrackingCost() override = default;

    EndEffectorTrackingCost* clone() const override;

    ocs2::scalar_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
                            const ocs2::TargetTrajectories& targetTrajectories,
                            const ocs2::PreComputation& preComputation) const override;

    ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
        ocs2::scalar_t time, const ocs2::vector_t& state,
        const ocs2::TargetTrajectories& targetTrajectories,
        const ocs2::PreComputation& preComputation) const override;

private:
    struct TargetPose
    {
        vector3_t position{vector3_t::Zero()};
        quaternion_t orientation{quaternion_t::Identity()};
        bool valid{false};
    };

    EndEffectorTrackingCost(const EndEffectorTrackingCost& other);

    TargetPose getTargetPose(ocs2::scalar_t time,
                             const ocs2::TargetTrajectories& targetTrajectories) const;

    void setPinocchioInterfaceIfNeeded(const ocs2::PreComputation& preComputation) const;

    std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> kinematicsPtr_;
    ocs2::PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;

    ocs2::matrix_t Qp_;
    ocs2::matrix_t Qo_;

    mutable std::atomic<bool> invalidReferenceWarned_{false};
};

}  // namespace wbmm_ocs2
