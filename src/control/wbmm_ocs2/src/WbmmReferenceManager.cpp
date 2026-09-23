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

#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace wbmm_ocs2
{

    using namespace ocs2;

    WbmmReferenceManager::WbmmReferenceManager(
        TaskPhase initialPhase,
        std::size_t wholeBodyStateDim,
        std::size_t endEffectorStateDim)
        : phase_(static_cast<std::size_t>(initialPhase)),
          requestedPhase_(static_cast<std::size_t>(initialPhase)),
          wholeBodyStateDim_(wholeBodyStateDim),
          endEffectorStateDim_(endEffectorStateDim)
    {
        setModeSchedule(ModeSchedule(
            {}, {static_cast<std::size_t>(initialPhase)}));
    }

    void WbmmReferenceManager::setWholeBodyTarget(
        const TargetTrajectories &target)
    {
        wholeBodyTarget_.setBuffer(target);
    }

    void WbmmReferenceManager::setEndEffectorTarget(
        const TargetTrajectories &target)
    {
        endEffectorTarget_.setBuffer(target);
    }

    void WbmmReferenceManager::clearEndEffectorTarget()
    {
        // Drop the previous tracking target. This is required when a new
        // navigation goal starts: otherwise the stale EE target would become
        // active again as soon as the phase reaches Transition/Execution.
        endEffectorTarget_.setBuffer(TargetTrajectories{});
    }

    void WbmmReferenceManager::setTaskPhase(TaskPhase phase)
    {
        const auto value = static_cast<std::size_t>(phase);
        if (value > static_cast<std::size_t>(TaskPhase::kRetract))
        {
            throw std::invalid_argument(
                "[WbmmReferenceManager] Unknown task phase value.");
        }

        if (phase == TaskPhase::kNavigation)
        {
            clearEndEffectorTarget();
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

    const TargetTrajectories &WbmmReferenceManager::getWholeBodyTarget() const
    {
        return wholeBodyTarget_.get();
    }

    const TargetTrajectories &WbmmReferenceManager::getEndEffectorTarget() const
    {
        return endEffectorTarget_.get();
    }

    bool WbmmReferenceManager::isEndEffectorDominant() const
    {
        return getTaskPhase() == TaskPhase::kExecution;
    }

    const TargetTrajectories &WbmmReferenceManager::getTargetTrajectories() const
    {
        // OCS2 copies this into the MPC/MRT command for visualization and
        // generic reference consumers. In Execution, expose the 7D EE target
        // so RViz does not keep drawing the old REMANI whole-body plan. If no
        // EE target has arrived yet, keep the 9D whole-body target as a
        // dimensionally safe fallback instead of returning an empty trajectory.
        if (isEndEffectorDominant())
        {
            const auto &endEffectorTarget = endEffectorTarget_.get();
            if (!endEffectorTarget.empty())
            {
                return endEffectorTarget;
            }
        }
        return wholeBodyTarget_.get();
    }

    bool WbmmReferenceManager::hasStateDim(
        const TargetTrajectories &target, std::size_t stateDim)
    {
        if (stateDim == 0 || target.stateTrajectory.empty())
        {
            return false;
        }
        return std::all_of(
            target.stateTrajectory.begin(), target.stateTrajectory.end(),
            [stateDim](const vector_t &state)
            {
                return static_cast<std::size_t>(state.size()) == stateDim;
            });
    }

    WbmmReferenceManager::TargetRoute
    WbmmReferenceManager::routeTargetTrajectory(
        const TargetTrajectories &target) const
    {
        // 优先按状态维度判断；MPC reset service 与 MRT reset 都只有一个
        // TargetTrajectories，必须靠 9D / 7D 区分两路参考。
        if (hasStateDim(target, wholeBodyStateDim_))
        {
            return TargetRoute::kWholeBody;
        }
        if (hasStateDim(target, endEffectorStateDim_))
        {
            return TargetRoute::kEndEffector;
        }

        // 空 target 或维度未知时，使用 requested phase，而不是 active phase。
        // requested phase 在 service / callback 中立即写入，不受 preSolverRun
        // 锁存时序影响。
        const bool useEndEffector =
            getRequestedTaskPhase() == TaskPhase::kExecution;
        if (!target.stateTrajectory.empty())
        {
            std::cerr << "[WbmmReferenceManager] WARNING: target state dim "
                      << target.stateTrajectory.front().size()
                      << " does not match whole-body(" << wholeBodyStateDim_
                      << ") or end-effector(" << endEffectorStateDim_
                      << "); routing by requested phase." << std::endl;
        }
        return useEndEffector ? TargetRoute::kEndEffector
                              : TargetRoute::kWholeBody;
    }

    void WbmmReferenceManager::setTargetTrajectory(
        const TargetTrajectories &target)
    {
        if (routeTargetTrajectory(target) == TargetRoute::kEndEffector)
        {
            endEffectorTarget_.setBuffer(target);
        }
        else
        {
            wholeBodyTarget_.setBuffer(target);
        }
    }

    void WbmmReferenceManager::setTargetTrajectory(
        TargetTrajectories &&target)
    {
        const TargetRoute route = routeTargetTrajectory(target);
        if (route == TargetRoute::kEndEffector)
        {
            endEffectorTarget_.setBuffer(std::move(target));
        }
        else
        {
            wholeBodyTarget_.setBuffer(std::move(target));
        }
    }

    void WbmmReferenceManager::setTargetTrajectories(
        const TargetTrajectories &target)
    {
        setTargetTrajectory(target);
    }

    void WbmmReferenceManager::setTargetTrajectories(
        TargetTrajectories &&target)
    {
        setTargetTrajectory(std::move(target));
    }

    void WbmmReferenceManager::preSolverRun(
        scalar_t initTime, scalar_t finalTime, const vector_t &initState)
    {
        wholeBodyTarget_.updateFromBuffer();
        endEffectorTarget_.updateFromBuffer();
        phase_.updateFromBuffer();

        ReferenceManager::preSolverRun(initTime, finalTime, initState);
    }

} // namespace wbmm_ocs2
