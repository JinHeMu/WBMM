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

#include "wbmm_ocs2/cost/EndEffectorTrackingCost.h"

#include "wbmm_ocs2/PreComputation.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

#include <ocs2_core/misc/LinearInterpolation.h>

namespace wbmm_ocs2
{

// 迁移期：原 OCS2 mobile_manipulator 代码位于 ocs2 命名空间内，
// 这里用文件级 using-directive 保持上游类型的可见性，
// 不污染被 include 的头文件。
using namespace ocs2;

namespace
{

matrix_t makeSymmetricWeight(const matrix_t& weight, const char* name)
{
    if (weight.rows() != 3 || weight.cols() != 3 || !weight.allFinite()) {
        throw std::runtime_error(
            std::string("[EndEffectorTrackingCost] ") + name +
            " must be a finite 3x3 matrix.");
    }
    return 0.5 * (weight + weight.transpose());
}

bool isFiniteQuaternion(const Eigen::Quaternion<scalar_t>& q)
{
    return std::isfinite(q.w()) && std::isfinite(q.x()) &&
           std::isfinite(q.y()) && std::isfinite(q.z());
}

bool normalizeQuaternion(const Eigen::Quaternion<scalar_t>& input, Eigen::Quaternion<scalar_t>& output)
{
    if (!isFiniteQuaternion(input)) {
        return false;
    }

    const scalar_t norm = input.norm();
    if (!std::isfinite(norm) || norm < 1e-9) {
        return false;
    }

    output = input;
    output.normalize();
    return isFiniteQuaternion(output);
}

}  // namespace

EndEffectorTrackingCost::EndEffectorTrackingCost(
    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
    matrix_t positionWeight,
    matrix_t orientationWeight)
    : kinematicsPtr_(endEffectorKinematics.clone()),
      Qp_(makeSymmetricWeight(positionWeight, "Qp")),
      Qo_(makeSymmetricWeight(orientationWeight, "Qo"))
{
    if (endEffectorKinematics.getIds().size() != 1U) {
        throw std::runtime_error(
            "[EndEffectorTrackingCost] Single-arm mode requires exactly one "
            "end effector ID.");
    }

    pinocchioEEKinPtr_ =
        dynamic_cast<PinocchioEndEffectorKinematics*>(kinematicsPtr_.get());
}

EndEffectorTrackingCost::EndEffectorTrackingCost(
    const EndEffectorTrackingCost& other)
    : StateCost(other),
      kinematicsPtr_(other.kinematicsPtr_->clone()),
      pinocchioEEKinPtr_(
          dynamic_cast<PinocchioEndEffectorKinematics*>(kinematicsPtr_.get())),
      Qp_(other.Qp_),
      Qo_(other.Qo_),
      invalidReferenceWarned_(false)
{
}

EndEffectorTrackingCost* EndEffectorTrackingCost::clone() const
{
    return new EndEffectorTrackingCost(*this);
}

EndEffectorTrackingCost::TargetPose EndEffectorTrackingCost::getTargetPose(
    scalar_t time, const TargetTrajectories& targetTrajectories) const
{
    TargetPose target;

    const auto& timeTrajectory = targetTrajectories.timeTrajectory;
    const auto& stateTrajectory = targetTrajectories.stateTrajectory;

    const auto invalidate = [this](const std::string& reason) {
        if (!invalidReferenceWarned_.exchange(true)) {
            std::cerr << "[EndEffectorTrackingCost] Invalid end-effector "
                         "reference: " << reason
                      << ". Cost is treated as zero until a valid 7D target "
                         "arrives.\n";
        }
        return TargetPose{};
    };

    if (stateTrajectory.empty()) {
        return invalidate("stateTrajectory is empty");
    }
    if (timeTrajectory.size() != stateTrajectory.size()) {
        return invalidate("timeTrajectory and stateTrajectory sizes differ");
    }

    for (const auto& state : stateTrajectory) {
        if (state.size() != 7 || !state.allFinite()) {
            return invalidate(
                "expected each target state to be finite 7D "
                "[x, y, z, qx, qy, qz, qw]");
        }
    }

    if (timeTrajectory.size() > 1U) {
        const bool strictlyIncreasing = std::adjacent_find(
            timeTrajectory.begin(), timeTrajectory.end(),
            [](scalar_t lhs, scalar_t rhs) { return !(lhs < rhs); }) ==
            timeTrajectory.end();
        if (!strictlyIncreasing) {
            return invalidate("timeTrajectory must be strictly increasing");
        }
    }

    const auto makePose = [](const vector_t& state, TargetPose& pose) {
        pose.position = state.head<3>();
        quaternion_t orientation;
        if (!normalizeQuaternion(quaternion_t(state.tail<4>()), orientation)) {
            return false;
        }
        pose.orientation = orientation;
        pose.valid = true;
        return true;
    };

    if (stateTrajectory.size() == 1U) {
        if (!makePose(stateTrajectory.front(), target)) {
            return invalidate("target quaternion is invalid");
        }
        return target;
    }

    if (time <= timeTrajectory.front()) {
        if (!makePose(stateTrajectory.front(), target)) {
            return invalidate("target quaternion is invalid");
        }
        return target;
    }
    if (time >= timeTrajectory.back()) {
        if (!makePose(stateTrajectory.back(), target)) {
            return invalidate("target quaternion is invalid");
        }
        return target;
    }

    int index = 0;
    scalar_t alpha = 0.0;
    std::tie(index, alpha) = LinearInterpolation::timeSegment(time, timeTrajectory);

    if (index < 0 ||
        static_cast<std::size_t>(index) + 1U >= stateTrajectory.size()) {
        return invalidate("interpolation index is out of range");
    }

    const auto& lhs = stateTrajectory[static_cast<std::size_t>(index)];
    const auto& rhs = stateTrajectory[static_cast<std::size_t>(index) + 1U];

    quaternion_t lhsOrientation;
    quaternion_t rhsOrientation;
    if (!normalizeQuaternion(quaternion_t(lhs.tail<4>()), lhsOrientation) ||
        !normalizeQuaternion(quaternion_t(rhs.tail<4>()), rhsOrientation)) {
        return invalidate("target quaternion is invalid");
    }

    target.position =
        alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
    target.orientation = lhsOrientation.slerp(1.0 - alpha, rhsOrientation);
    target.orientation.normalize();

    if (!target.position.allFinite() || !isFiniteQuaternion(target.orientation)) {
        return invalidate("interpolated target pose is non-finite");
    }

    target.valid = true;
    return target;
}

void EndEffectorTrackingCost::setPinocchioInterfaceIfNeeded(
    const PreComputation& preComputation) const
{
    if (pinocchioEEKinPtr_ != nullptr) {
        const auto& preComp = cast<WbmmPreComputation>(preComputation);
        pinocchioEEKinPtr_->setPinocchioInterface(
            preComp.getPinocchioInterface());
    }
}

scalar_t EndEffectorTrackingCost::getValue(
    scalar_t time, const vector_t& state,
    const TargetTrajectories& targetTrajectories,
    const PreComputation& preComputation) const
{
    const TargetPose target = getTargetPose(time, targetTrajectories);
    if (!target.valid) {
        return 0.0;
    }

    setPinocchioInterfaceIfNeeded(preComputation);

    const auto positions = kinematicsPtr_->getPosition(state);
    const auto orientationErrors = kinematicsPtr_->getOrientationError(
        state, {target.orientation});

    if (positions.size() != 1U || orientationErrors.size() != 1U) {
        return 0.0;
    }

    const vector3_t positionError = positions.front() - target.position;
    const vector3_t orientationError = orientationErrors.front();

    if (!positionError.allFinite() || !orientationError.allFinite()) {
        return 0.0;
    }

    return 0.5 * positionError.dot(Qp_ * positionError) +
           0.5 * orientationError.dot(Qo_ * orientationError);
}

ScalarFunctionQuadraticApproximation
EndEffectorTrackingCost::getQuadraticApproximation(
    scalar_t time, const vector_t& state,
    const TargetTrajectories& targetTrajectories,
    const PreComputation& preComputation) const
{
    ScalarFunctionQuadraticApproximation cost;
    cost.f = 0.0;
    cost.dfdx = vector_t::Zero(state.rows());
    cost.dfdxx = matrix_t::Zero(state.rows(), state.rows());

    const TargetPose target = getTargetPose(time, targetTrajectories);
    if (!target.valid) {
        return cost;
    }

    setPinocchioInterfaceIfNeeded(preComputation);

    const auto positionApproximations =
        kinematicsPtr_->getPositionLinearApproximation(state);
    const auto orientationApproximations =
        kinematicsPtr_->getOrientationErrorLinearApproximation(
            state, {target.orientation});

    if (positionApproximations.size() != 1U ||
        orientationApproximations.size() != 1U) {
        return cost;
    }

    const auto& position = positionApproximations.front();
    const auto& orientation = orientationApproximations.front();

    const vector3_t positionError = position.f - target.position;
    const vector3_t orientationError = orientation.f;

    if (!positionError.allFinite() || !orientationError.allFinite() ||
        !position.dfdx.allFinite() || !orientation.dfdx.allFinite()) {
        return cost;
    }

    cost.f = 0.5 * positionError.dot(Qp_ * positionError) +
             0.5 * orientationError.dot(Qo_ * orientationError);
    cost.dfdx = position.dfdx.transpose() * Qp_ * positionError +
                orientation.dfdx.transpose() * Qo_ * orientationError;
    cost.dfdxx = position.dfdx.transpose() * Qp_ * position.dfdx +
                 orientation.dfdx.transpose() * Qo_ * orientation.dfdx;

    return cost;
}

}  // namespace wbmm_ocs2
