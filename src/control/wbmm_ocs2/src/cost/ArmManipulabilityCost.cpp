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

#include "wbmm_ocs2/cost/ArmManipulabilityCost.h"

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

namespace wbmm_ocs2
{

// 迁移期：原 OCS2 mobile_manipulator 代码位于 ocs2 命名空间内，
// 这里用 file-level using-directive 保持上游类型可见。
using namespace ocs2;

namespace
{

void validateWeight(scalar_t weight, const char* name)
{
    if (!std::isfinite(weight) || weight < 0.0) {
        throw std::runtime_error(
            std::string("[ArmManipulabilityCost] ") + name +
            " must be finite and non-negative.");
    }
}

}  // namespace

ArmManipulabilityCost::ArmManipulabilityCost(
    const PinocchioInterface& pinocchioInterface,
    WbmmModelInfo modelInfo,
    ArmManipulabilitySettings settings)
    : pinocchioInterface_(pinocchioInterface),
      modelInfo_(std::move(modelInfo)),
      settings_(std::move(settings))
{
    if (settings_.frameName.empty()) {
        settings_.frameName = modelInfo_.eeFrame;
    }

    const auto& model = pinocchioInterface_.getModel();
    if (modelInfo_.stateDim != static_cast<std::size_t>(model.nq) ||
        modelInfo_.stateDim != static_cast<std::size_t>(model.nv)) {
        throw std::runtime_error(
            "[ArmManipulabilityCost] WbmmModelInfo state dimension does not "
            "match the Pinocchio model.");
    }
    if (settings_.armDim == 0U ||
        settings_.armStartIndex + settings_.armDim > modelInfo_.stateDim) {
        throw std::runtime_error(
            "[ArmManipulabilityCost] Invalid armStartIndex/armDim.");
    }
    if (settings_.scope == ArmManipulabilitySettings::Scope::kArmOnly &&
        settings_.armDim != modelInfo_.armDim) {
        throw std::runtime_error(
            "[ArmManipulabilityCost] arm-only scope requires armDim == "
            "WbmmModelInfo::armDim.");
    }
    if (settings_.scope == ArmManipulabilitySettings::Scope::kWholeBody &&
        modelInfo_.inputDim != settings_.armDim + 2U) {
        throw std::runtime_error(
            "[ArmManipulabilityCost] whole-body scope requires inputDim == "
            "armDim + 2.");
    }
    if (!(settings_.finiteDiffStep > 0.0) ||
        !std::isfinite(settings_.finiteDiffStep)) {
        throw std::runtime_error(
            "[ArmManipulabilityCost] finiteDiffStep must be positive and "
            "finite.");
    }
    if (!(settings_.regularization > 0.0) ||
        !std::isfinite(settings_.regularization)) {
        throw std::runtime_error(
            "[ArmManipulabilityCost] regularization must be positive and "
            "finite.");
    }
    if (!std::isfinite(settings_.hessianRegularization) ||
        settings_.hessianRegularization < 0.0) {
        throw std::runtime_error(
            "[ArmManipulabilityCost] hessianRegularization must be finite and "
            "non-negative.");
    }
    if (settings_.useTaskDirection &&
        settings_.taskDirection.size() != 6) {
        throw std::runtime_error(
            "[ArmManipulabilityCost] taskDirection must be 6D when "
            "useTaskDirection is true.");
    }

    validateWeight(settings_.minSingularWeight, "minSingularWeight");
    validateWeight(settings_.yoshikawaWeight, "yoshikawaWeight");
    validateWeight(settings_.taskDirectionWeight, "taskDirectionWeight");
    validateWeight(
        settings_.inverseManipulabilityWeight,
        "inverseManipulabilityWeight");
    validateWeight(settings_.conditionWeight, "conditionWeight");
    if (!std::isfinite(settings_.minSingularRef) ||
        !std::isfinite(settings_.yoshikawaRef) ||
        !std::isfinite(settings_.taskDirectionRef) ||
        !std::isfinite(settings_.conditionMax)) {
        throw std::runtime_error(
            "[ArmManipulabilityCost] References and conditionMax must be "
            "finite.");
    }

    eeFrameId_ = model.getBodyId(settings_.frameName);
}

ArmManipulabilityCost* ArmManipulabilityCost::clone() const
{
    return new ArmManipulabilityCost(*this);
}

matrix_t ArmManipulabilityCost::buildTaskJacobian(
    const matrix_t& frameJacobian, const vector_t& state) const
{
    if (settings_.scope == ArmManipulabilitySettings::Scope::kArmOnly) {
        return frameJacobian.middleCols(
            static_cast<Eigen::Index>(settings_.armStartIndex),
            static_cast<Eigen::Index>(settings_.armDim));
    }

    matrix_t taskJacobian(
        6, static_cast<Eigen::Index>(modelInfo_.inputDim));
    Eigen::Matrix<scalar_t, 3, 2> baseInputMap;
    // state = [x, y, yaw, q1..qN]
    // Pinocchio generalized velocity = [v cos(yaw), v sin(yaw), omega, qdot]
    baseInputMap << std::cos(state(2)), 0.0,
        std::sin(state(2)), 0.0,
        0.0, 1.0;

    taskJacobian.leftCols<2>() =
        frameJacobian.leftCols<3>() * baseInputMap;
    taskJacobian.rightCols(
        static_cast<Eigen::Index>(settings_.armDim)) =
        frameJacobian.middleCols(
            static_cast<Eigen::Index>(settings_.armStartIndex),
            static_cast<Eigen::Index>(settings_.armDim));
    return taskJacobian;
}

ArmManipulabilityCost::Metrics ArmManipulabilityCost::computeMetrics(
    const vector_t& state) const
{
    Metrics metrics;

    if (state.size() != static_cast<Eigen::Index>(modelInfo_.stateDim) ||
        !state.allFinite()) {
        return metrics;
    }

    const auto& model = pinocchioInterface_.getModel();
    auto data = pinocchioInterface_.getData();  // copy: thread-safe

    const vector_t q = state;
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data, q);

    matrix_t frameJacobian = matrix_t::Zero(6, model.nv);
    pinocchio::getFrameJacobian(
        model, data, eeFrameId_,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
        frameJacobian);

    const matrix_t taskJacobian = buildTaskJacobian(frameJacobian, state);
    if (!taskJacobian.allFinite() || taskJacobian.cols() == 0) {
        return metrics;
    }

    Eigen::JacobiSVD<matrix_t> svd(
        taskJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const vector_t singularValues = svd.singularValues();
    if (singularValues.size() == 0 || !singularValues.allFinite()) {
        return metrics;
    }

    metrics.sigmaMin = singularValues.minCoeff();
    metrics.sigmaMax = singularValues.maxCoeff();
    metrics.condition =
        metrics.sigmaMax / std::max<scalar_t>(metrics.sigmaMin, 1e-9);
    metrics.yoshikawa = singularValues.prod();

    const matrix_t JJt = taskJacobian * taskJacobian.transpose();
    const matrix_t regularizedJJt =
        JJt + settings_.regularization *
        matrix_t::Identity(JJt.rows(), JJt.cols());
    metrics.inverseManipulability = regularizedJJt.inverse().trace();

    if (settings_.taskDirection.size() == 6) {
        const scalar_t directionalValue =
            settings_.taskDirection.dot(JJt * settings_.taskDirection);
        metrics.taskDirection =
            std::sqrt(std::max<scalar_t>(0.0, directionalValue));
    }

    metrics.valid =
        std::isfinite(metrics.sigmaMin) &&
        std::isfinite(metrics.sigmaMax) &&
        std::isfinite(metrics.yoshikawa) &&
        std::isfinite(metrics.inverseManipulability) &&
        std::isfinite(metrics.condition) &&
        std::isfinite(metrics.taskDirection);
    return metrics;
}

scalar_t ArmManipulabilityCost::costFromMetrics(const Metrics& metrics) const
{
    if (!metrics.valid) {
        return 0.0;
    }

    scalar_t cost = 0.0;

    if (settings_.useMinSingularValue) {
        const scalar_t deficit =
            std::max<scalar_t>(0.0, settings_.minSingularRef - metrics.sigmaMin);
        cost += 0.5 * settings_.minSingularWeight * deficit * deficit;
    }

    if (settings_.useYoshikawa) {
        const scalar_t deficit =
            std::max<scalar_t>(0.0, settings_.yoshikawaRef - metrics.yoshikawa);
        cost += 0.5 * settings_.yoshikawaWeight * deficit * deficit;
    }

    if (settings_.useTaskDirection) {
        const scalar_t deficit = std::max<scalar_t>(
            0.0, settings_.taskDirectionRef - metrics.taskDirection);
        cost += 0.5 * settings_.taskDirectionWeight * deficit * deficit;
    }

    if (settings_.useInverseManipulability) {
        cost += settings_.inverseManipulabilityWeight *
            metrics.inverseManipulability;
    }

    if (settings_.useConditionNumber) {
        const scalar_t excess = std::max<scalar_t>(
            0.0, metrics.condition - settings_.conditionMax);
        cost += 0.5 * settings_.conditionWeight * excess * excess;
    }

    return cost;
}

std::vector<int> ArmManipulabilityCost::activeStateIndices() const
{
    std::vector<int> indices;
    if (settings_.scope == ArmManipulabilitySettings::Scope::kArmOnly) {
        indices.reserve(settings_.armDim);
        for (std::size_t i = 0; i < settings_.armDim; ++i) {
            indices.push_back(
                static_cast<int>(settings_.armStartIndex + i));
        }
    } else {
        // Whole-body task Jacobian depends on yaw and arm joints, not on x/y.
        indices.reserve(modelInfo_.stateDim - 2U);
        for (std::size_t i = 2U; i < modelInfo_.stateDim; ++i) {
            indices.push_back(static_cast<int>(i));
        }
    }
    return indices;
}

scalar_t ArmManipulabilityCost::getValue(
    scalar_t /*time*/, const vector_t& state,
    const TargetTrajectories& /*targetTrajectories*/,
    const PreComputation& /*preComputation*/) const
{
    return costFromMetrics(computeMetrics(state));
}

ScalarFunctionQuadraticApproximation
ArmManipulabilityCost::getQuadraticApproximation(
    scalar_t /*time*/, const vector_t& state,
    const TargetTrajectories& /*targetTrajectories*/,
    const PreComputation& /*preComputation*/) const
{
    ScalarFunctionQuadraticApproximation cost;
    cost.f = 0.0;
    cost.dfdx = vector_t::Zero(state.rows());
    cost.dfdxx = matrix_t::Zero(state.rows(), state.rows());

    const Metrics current = computeMetrics(state);
    if (!current.valid) {
        return cost;
    }

    cost.f = costFromMetrics(current);

    const auto indices = activeStateIndices();
    const scalar_t step = settings_.finiteDiffStep;

    vector_t gradSigmaMin = vector_t::Zero(state.rows());
    vector_t gradSigmaMax = vector_t::Zero(state.rows());
    vector_t gradYoshikawa = vector_t::Zero(state.rows());
    vector_t gradTaskDirection = vector_t::Zero(state.rows());
    vector_t gradInverseManipulability = vector_t::Zero(state.rows());
    vector_t gradCondition = vector_t::Zero(state.rows());

    for (const int index : indices) {
        vector_t statePlus = state;
        vector_t stateMinus = state;
        statePlus(index) += step;
        stateMinus(index) -= step;

        const Metrics plus = computeMetrics(statePlus);
        const Metrics minus = computeMetrics(stateMinus);
        if (!plus.valid || !minus.valid) {
            cost.dfdx.setZero();
            cost.dfdxx =
                settings_.hessianRegularization *
                matrix_t::Identity(state.rows(), state.rows());
            return cost;
        }

        const scalar_t denominator = 2.0 * step;
        gradSigmaMin(index) =
            (plus.sigmaMin - minus.sigmaMin) / denominator;
        gradSigmaMax(index) =
            (plus.sigmaMax - minus.sigmaMax) / denominator;
        gradYoshikawa(index) =
            (plus.yoshikawa - minus.yoshikawa) / denominator;
        gradTaskDirection(index) =
            (plus.taskDirection - minus.taskDirection) / denominator;
        gradInverseManipulability(index) =
            (plus.inverseManipulability -
             minus.inverseManipulability) /
            denominator;
        gradCondition(index) =
            (plus.condition - minus.condition) / denominator;
    }

    vector_t gradient = vector_t::Zero(state.rows());
    matrix_t hessian =
        settings_.hessianRegularization *
        matrix_t::Identity(state.rows(), state.rows());

    const auto addHinge = [&gradient, &hessian](
                              const vector_t& metricGradient,
                              scalar_t metricValue,
                              scalar_t reference,
                              scalar_t weight) {
        if (metricValue < reference) {
            const scalar_t deficit = reference - metricValue;
            gradient.noalias() += -weight * deficit * metricGradient;
            hessian.noalias() +=
                weight * metricGradient * metricGradient.transpose();
        }
    };

    if (settings_.useMinSingularValue) {
        addHinge(
            gradSigmaMin, current.sigmaMin,
            settings_.minSingularRef, settings_.minSingularWeight);
    }
    if (settings_.useYoshikawa) {
        addHinge(
            gradYoshikawa, current.yoshikawa,
            settings_.yoshikawaRef, settings_.yoshikawaWeight);
    }
    if (settings_.useTaskDirection) {
        addHinge(
            gradTaskDirection, current.taskDirection,
            settings_.taskDirectionRef, settings_.taskDirectionWeight);
    }
    if (settings_.useConditionNumber &&
        current.condition > settings_.conditionMax) {
        const scalar_t excess = current.condition - settings_.conditionMax;
        gradient.noalias() +=
            settings_.conditionWeight * excess * gradCondition;
        hessian.noalias() += settings_.conditionWeight *
            gradCondition * gradCondition.transpose();
    }
    if (settings_.useInverseManipulability) {
        gradient.noalias() +=
            settings_.inverseManipulabilityWeight *
            gradInverseManipulability;
        // Linear term: no PSD curvature information; keep the diagonal
        // regularization added above.
    }

    cost.dfdx = gradient;
    cost.dfdxx = hessian;
    return cost;
}

}  // namespace wbmm_ocs2
