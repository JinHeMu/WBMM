/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#include "wbmm_ocs2/PinocchioMapping.h"

#include <stdexcept>
#include <utility>

namespace wbmm_ocs2
{

// 迁移期：原 OCS2 mobile_manipulator 代码位于 ocs2 命名空间内，
// 这里用文件级 using-directive 保持上游类型的可见性，
// 不污染被 include 的头文件。
using namespace ocs2;


template <typename SCALAR>
WbmmPinocchioMappingTpl<SCALAR>::WbmmPinocchioMappingTpl(WbmmModelInfo info)
    : modelInfo_(std::move(info))
{
}

template <typename SCALAR>
WbmmPinocchioMappingTpl<SCALAR>* WbmmPinocchioMappingTpl<SCALAR>::clone() const
{
    return new WbmmPinocchioMappingTpl(*this);
}

template <typename SCALAR>
auto WbmmPinocchioMappingTpl<SCALAR>::getPinocchioJointPosition(
    const vector_t& state) const -> vector_t
{
    return state.head(modelInfo_.configurationDim());
}

template <typename SCALAR>
auto WbmmPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(
    const vector_t& state, const vector_t& input) const -> vector_t
{
    // 差速底盘只有航向线速度 v 和 yaw 角速度 omega。
    vector_t vPinocchio = vector_t::Zero(modelInfo_.configurationDim());
    const SCALAR theta = state(2);
    const SCALAR v = modelInfo_.baseResponse.enabled ? state(modelInfo_.baseVelocityIndex()) : input(0);
    const SCALAR w = modelInfo_.baseResponse.enabled ? state(modelInfo_.baseVelocityIndex() + 1) : input(1);
    vPinocchio << cos(theta) * v,
        sin(theta) * v,
        w,
        input.tail(static_cast<Eigen::Index>(modelInfo_.armDim));
    return vPinocchio;
}

template <typename SCALAR>
auto WbmmPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(
    const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t>
{
    matrix_t dfdx = matrix_t::Zero(Jq.rows(), modelInfo_.stateDim);
    dfdx.leftCols(modelInfo_.configurationDim()) = Jq;
    matrix_t dfdu = matrix_t::Zero(Jv.rows(), modelInfo_.inputDim);
    Eigen::Matrix<SCALAR, 3, 2> dvdu_base;
    const SCALAR theta = state(2);
    // clang-format off
    dvdu_base << cos(theta), SCALAR(0),
        sin(theta), SCALAR(0),
        SCALAR(0), SCALAR(1.0);
    // clang-format on
    if (modelInfo_.baseResponse.enabled) {
        dfdx.middleCols(modelInfo_.baseVelocityIndex(), 2) = Jv.template leftCols<3>() * dvdu_base;
    } else {
        dfdu.template leftCols<2>() = Jv.template leftCols<3>() * dvdu_base;
    }
    // qdot's world-frame translation rotates with yaw. Jq is the partial
    // derivative at fixed Pinocchio velocity; include the velocity-map term.
    if (modelInfo_.baseResponse.enabled) {
        const SCALAR v = state(modelInfo_.baseVelocityIndex());
        dfdx.col(2) += v * (-sin(theta) * Jv.col(0) + cos(theta) * Jv.col(1));
    }
    dfdu.template rightCols(static_cast<Eigen::Index>(modelInfo_.armDim)) =
        Jv.template rightCols(static_cast<Eigen::Index>(modelInfo_.armDim));
    return {dfdx, dfdu};
}

// explicit template instantiation
template class WbmmPinocchioMappingTpl<scalar_t>;
template class WbmmPinocchioMappingTpl<ad_scalar_t>;

}  // namespace wbmm_ocs2
