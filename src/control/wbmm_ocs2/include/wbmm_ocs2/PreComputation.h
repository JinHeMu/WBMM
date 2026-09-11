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

#pragma once

#include <memory>
#include <string>

#include <ocs2_core/PreComputation.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include <wbmm_ocs2/WbmmModelInfo.h>
#include <wbmm_ocs2/PinocchioMapping.h>

namespace wbmm_ocs2
{
    /** Callback for caching and reference update */
    class WbmmPreComputation : public ocs2::PreComputation
    {
    public:
        WbmmPreComputation(ocs2::PinocchioInterface pinocchioInterface, const WbmmModelInfo& info);

        ~WbmmPreComputation() override = default;

        WbmmPreComputation(const WbmmPreComputation& rhs) = delete;
        WbmmPreComputation* clone() const override;

        void request(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x, const ocs2::vector_t& u) override;
        void requestFinal(ocs2::RequestSet request, ocs2::scalar_t t, const ocs2::vector_t& x) override;

        ocs2::PinocchioInterface& getPinocchioInterface() { return pinocchioInterface_; }
        const ocs2::PinocchioInterface& getPinocchioInterface() const { return pinocchioInterface_; }

    private:
        ocs2::PinocchioInterface pinocchioInterface_;
        WbmmPinocchioMapping pinocchioMapping_;
    };
}
