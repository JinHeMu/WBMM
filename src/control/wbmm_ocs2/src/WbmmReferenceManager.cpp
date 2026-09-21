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

#include "wbmm_ocs2/WbmmReferenceManager.h"

#include <stdexcept>

namespace wbmm_ocs2
{

using namespace ocs2;

WbmmReferenceManager::WbmmReferenceManager(TaskPhase initialPhase)
    : phase_(static_cast<std::size_t>(initialPhase)),
      requestedPhase_(static_cast<std::size_t>(initialPhase))
{
    setModeSchedule(ModeSchedule(
        {}, {static_cast<std::size_t>(initialPhase)}));
}

void WbmmReferenceManager::setWholeBodyTarget(
    const TargetTrajectories& target)
{
    wholeBodyTarget_.setBuffer(target);
}

void WbmmReferenceManager::setEndEffectorTarget(
    const TargetTrajectories& target)
{
    endEffectorTarget_.setBuffer(target);
}

void WbmmReferenceManager::setTaskPhase(TaskPhase phase)
{
    const auto value = static_cast<std::size_t>(phase);
    if (value > static_cast<std::size_t>(TaskPhase::kRetract)) {
        throw std::invalid_argument(
            "[WbmmReferenceManager] Unknown task phase value.");
    }

    requestedPhase_.store(value, std::memory_order_relaxed);
    phase_.setBuffer(value);

    // policy mode = task phase, used by MRT for phase consistency checks.
    setModeSchedule(ModeSchedule({}, {value}));
}

TaskPhase WbmmReferenceManager::getTaskPhase() const
{
    return static_cast<TaskPhase>(phase_.get());
}

TaskPhase WbmmReferenceManager::getRequestedTaskPhase() const
{
    return static_cast<TaskPhase>(
        requestedPhase_.load(std::memory_order_relaxed));
}

const TargetTrajectories& WbmmReferenceManager::getWholeBodyTarget() const
{
    return wholeBodyTarget_.get();
}

const TargetTrajectories& WbmmReferenceManager::getEndEffectorTarget() const
{
    return endEffectorTarget_.get();
}

bool WbmmReferenceManager::isEndEffectorDominant() const
{
    return getTaskPhase() == TaskPhase::kExecution;
}

const TargetTrajectories& WbmmReferenceManager::getTargetTrajectories() const
{
    return isEndEffectorDominant()
        ? endEffectorTarget_.get()
        : wholeBodyTarget_.get();
}

void WbmmReferenceManager::setTargetTrajectories(
    const TargetTrajectories& target)
{
    if (isEndEffectorDominant()) {
        endEffectorTarget_.setBuffer(target);
    } else {
        wholeBodyTarget_.setBuffer(target);
    }
}

void WbmmReferenceManager::preSolverRun(
    scalar_t initTime, scalar_t finalTime, const vector_t& initState)
{
    wholeBodyTarget_.updateFromBuffer();
    endEffectorTarget_.updateFromBuffer();
    phase_.updateFromBuffer();

    ReferenceManager::preSolverRun(initTime, finalTime, initState);
}

}  // namespace wbmm_ocs2
