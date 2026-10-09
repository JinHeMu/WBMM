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

#include "wbmm_ocs2/Dynamics.h"
#include <cstdint>
#include <cstring>
#include <sstream>

namespace wbmm_ocs2
{

// 迁移期：原 OCS2 mobile_manipulator 代码位于 ocs2 命名空间内，
// 这里用文件级 using-directive 保持上游类型的可见性，
// 不污染被 include 的头文件。
using namespace ocs2;

    WbmmDynamics::WbmmDynamics(
        WbmmModelInfo info, const std::string& modelName,
        const std::string& modelFolder /*= "/tmp/ocs2"*/,
        bool recompileLibraries /*= true*/, bool verbose /*= true*/)
        : info_(std::move(info))
    {
        // Constants are baked into CppAD code: segregate cache by dynamics settings.
        std::string cacheName = modelName;
        if (info_.baseResponse.enabled) {
            const auto& b = info_.baseResponse;
            std::ostringstream signature;
            signature << "_response_v1" << std::hex;
            for (const double value : {b.linearTimeConstant, b.angularTimeConstant,
                                       b.linearGain, b.angularGain}) {
                std::uint64_t bits;
                static_assert(sizeof(bits) == sizeof(value));
                std::memcpy(&bits, &value, sizeof(bits));
                signature << '_' << bits;
            }
            cacheName += signature.str();
        }
        this->initialize(info_.stateDim, info_.inputDim, cacheName, modelFolder, recompileLibraries, verbose);
    }


    ad_vector_t WbmmDynamics::systemFlowMap(ad_scalar_t time, const ad_vector_t& state,
                                                                   const ad_vector_t& input,
                                                                   const ad_vector_t&) const
    {
        (void)time;
        ad_vector_t dxdt(info_.stateDim);
        const auto theta = state(2);
        const auto velocityIndex = info_.baseVelocityIndex();
        const ad_scalar_t v = info_.baseResponse.enabled ? state(velocityIndex) : input(0);
        const ad_scalar_t w = info_.baseResponse.enabled ? state(velocityIndex + 1) : input(1);
        dxdt.head(3) << cos(theta) * v, sin(theta) * v, w;
        dxdt.segment(3, info_.armDim) = input.tail(info_.armDim);
        if (info_.baseResponse.enabled) {
            const auto& b = info_.baseResponse;
            dxdt(velocityIndex) = (b.linearGain * input(0) - v) / b.linearTimeConstant;
            dxdt(velocityIndex + 1) = (b.angularGain * input(1) - w) / b.angularTimeConstant;
        }
        return dxdt;
    }
}
