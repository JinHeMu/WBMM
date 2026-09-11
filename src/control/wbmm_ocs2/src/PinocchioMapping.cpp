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
    // 差速底盘 + 机械臂：OCS2 state 与 Pinocchio q 一一对应。
    return state;
}

template <typename SCALAR>
auto WbmmPinocchioMappingTpl<SCALAR>::getPinocchioJointVelocity(
    const vector_t& state, const vector_t& input) const -> vector_t
{
    // 差速底盘只有航向线速度 v 和 yaw 角速度 omega。
    vector_t vPinocchio = vector_t::Zero(modelInfo_.stateDim);
    const SCALAR theta = state(2);
    const SCALAR v = input(0);
    vPinocchio << cos(theta) * v,
        sin(theta) * v,
        input(1),
        input.tail(static_cast<Eigen::Index>(modelInfo_.armDim));
    return vPinocchio;
}

template <typename SCALAR>
auto WbmmPinocchioMappingTpl<SCALAR>::getOcs2Jacobian(
    const vector_t& state, const matrix_t& Jq, const matrix_t& Jv) const
    -> std::pair<matrix_t, matrix_t>
{
    matrix_t dfdu(Jv.rows(), static_cast<Eigen::Index>(modelInfo_.inputDim));
    Eigen::Matrix<SCALAR, 3, 2> dvdu_base;
    const SCALAR theta = state(2);
    // clang-format off
    dvdu_base << cos(theta), SCALAR(0),
        sin(theta), SCALAR(0),
        SCALAR(0), SCALAR(1.0);
    // clang-format on
    dfdu.template leftCols<2>() = Jv.template leftCols<3>() * dvdu_base;
    dfdu.template rightCols(static_cast<Eigen::Index>(modelInfo_.armDim)) =
        Jv.template rightCols(static_cast<Eigen::Index>(modelInfo_.armDim));
    return {Jq, dfdu};
}

// explicit template instantiation
template class WbmmPinocchioMappingTpl<scalar_t>;
template class WbmmPinocchioMappingTpl<ad_scalar_t>;

}  // namespace wbmm_ocs2
