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

#include <ocs2_core/cost/QuadraticStateInputCost.h>

namespace wbmm_ocs2
{
        class QuadraticInputCost final : public ocs2::QuadraticStateInputCost
        {
        public:
            QuadraticInputCost(ocs2::matrix_t R, size_t stateDim)
                : ocs2::QuadraticStateInputCost(ocs2::matrix_t::Zero(stateDim, stateDim), std::move(R)), stateDim_(stateDim)
            {
            }

            ~QuadraticInputCost() override = default;

            QuadraticInputCost(const QuadraticInputCost& rhs) = default;
            QuadraticInputCost* clone() const override { return new QuadraticInputCost(*this); }

            std::pair<ocs2::vector_t, ocs2::vector_t> getStateInputDeviation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::vector_t& input,
                                                                 const ocs2::TargetTrajectories& targetTrajectories)
            const override
            {
                (void)state;
                const ocs2::vector_t inputDeviation = input - targetTrajectories.getDesiredInput(time);
                return {ocs2::vector_t::Zero(stateDim_), inputDeviation};
            }

        private:
            const size_t stateDim_;
        };
    }  // namespace wbmm_ocs2
