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
#include <cstddef>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_core/thread_support/BufferedValue.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace wbmm_ocs2
{

enum class TaskPhase : std::size_t
{
    kNavigation = 0,
    kTransition = 1,
    kExecution = 2,
    kRetract = 3,
};

/**
 * 双参考 OCS2 ReferenceManager。
 *
 * 同时维护：
 *   - whole-body target: 9D [x, y, yaw, q1..q6]
 *   - end-effector target: 7D [x, y, z, qx, qy, qz, qw]
 *   - task phase: Navigation / Transition / Execution / Retract
 *
 * setTaskPhase() 同时把 OCS2 ModeSchedule 设置为当前 phase，方便 MRT
 * 通过 evaluatePolicy() 返回的 mode 检查 policy 与当前阶段是否一致。
 */
class WbmmReferenceManager final : public ocs2::ReferenceManager
{
public:
    explicit WbmmReferenceManager(
        TaskPhase initialPhase = TaskPhase::kNavigation);

    void setWholeBodyTarget(const ocs2::TargetTrajectories& target);
    void setEndEffectorTarget(const ocs2::TargetTrajectories& target);
    void setTaskPhase(TaskPhase phase);

    TaskPhase getTaskPhase() const;
    TaskPhase getRequestedTaskPhase() const;

    const ocs2::TargetTrajectories& getWholeBodyTarget() const;
    const ocs2::TargetTrajectories& getEndEffectorTarget() const;

    const ocs2::TargetTrajectories& getTargetTrajectories() const override;
    void setTargetTrajectories(
        const ocs2::TargetTrajectories& target) override;

    void preSolverRun(ocs2::scalar_t initTime,
                      ocs2::scalar_t finalTime,
                      const ocs2::vector_t& initState) override;

private:
    bool isEndEffectorDominant() const;

    ocs2::BufferedValue<ocs2::TargetTrajectories>
        wholeBodyTarget_{ocs2::TargetTrajectories()};
    ocs2::BufferedValue<ocs2::TargetTrajectories>
        endEffectorTarget_{ocs2::TargetTrajectories()};
    ocs2::BufferedValue<std::size_t> phase_;
    std::atomic<std::size_t> requestedPhase_;
};

}  // namespace wbmm_ocs2
