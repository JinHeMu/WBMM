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

#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include "wbmm_ocs2/WbmmModelInfo.h"

namespace wbmm_ocs2
{

// ============================================================================
// WbmmPinocchioMapping —— 9D/8D OCS2 状态-输入 与 Pinocchio 配置-速度 的映射。
//
//   状态:  x = [x, y, yaw, q_arm]
//   输入:  u = [v, omega, qdot_arm]
//
//   q_pinocchio = x
//   v_pinocchio = [v*cos(yaw), v*sin(yaw), omega, qdot_arm]
//
// 差速底盘没有侧滑自由度，因此 Pinocchio 的 base 线速度是航向系 v 旋转到
// 世界系后的结果。
// ============================================================================
template <typename SCALAR>
class WbmmPinocchioMappingTpl;

using WbmmPinocchioMapping = WbmmPinocchioMappingTpl<ocs2::scalar_t>;
using WbmmPinocchioMappingCppAd = WbmmPinocchioMappingTpl<ocs2::ad_scalar_t>;

template <typename SCALAR>
class WbmmPinocchioMappingTpl final : public ocs2::PinocchioStateInputMapping<SCALAR>
{
public:
    using Base = ocs2::PinocchioStateInputMapping<SCALAR>;
    using typename Base::matrix_t;
    using typename Base::vector_t;

    explicit WbmmPinocchioMappingTpl(WbmmModelInfo info);

    ~WbmmPinocchioMappingTpl() override = default;
    WbmmPinocchioMappingTpl<SCALAR>* clone() const override;

    vector_t getPinocchioJointPosition(const vector_t& state) const override;
    vector_t getPinocchioJointVelocity(const vector_t& state, const vector_t& input) const override;
    std::pair<matrix_t, matrix_t> getOcs2Jacobian(
        const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const override;

    const WbmmModelInfo& getWbmmModelInfo() const { return modelInfo_; }

private:
    WbmmPinocchioMappingTpl(const WbmmPinocchioMappingTpl& rhs) = default;

    const WbmmModelInfo modelInfo_;
};

}  // namespace wbmm_ocs2
