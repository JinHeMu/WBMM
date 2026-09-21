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

#include "wbmm_ocs2/PinocchioMapping.h"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>

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

        void validateWeight(scalar_t weight, const char *name)
        {
            if (!std::isfinite(weight) || weight < 0.0)
            {
                throw std::runtime_error(
                    std::string("[ArmManipulabilityCost] ") + name +
                    " must be finite and non-negative.");
            }
        }

    } // namespace

    ArmManipulabilityCost::ArmManipulabilityCost(
        const PinocchioInterface &pinocchioInterface,
        WbmmModelInfo modelInfo,
        ArmManipulabilitySettings settings)
        : pinocchioInterface_(pinocchioInterface),
          modelInfo_(std::move(modelInfo)),
          settings_(std::move(settings))
    {
        const auto &model = pinocchioInterface_.getModel();
        if (static_cast<std::size_t>(model.nq) != modelInfo_.stateDim ||
            static_cast<std::size_t>(model.nv) != modelInfo_.stateDim)
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] Pinocchio nq/nv must match stateDim.");
        }
        if (settings_.frameName.empty())
        {
            settings_.frameName = modelInfo_.eeFrame;
        }
        if (settings_.frameName.empty())
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] frameName must not be empty.");
        }
        if (settings_.stateFrame.empty())
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] stateFrame must not be empty.");
        }
        if (modelInfo_.armDim == 0U || modelInfo_.stateDim < modelInfo_.armDim ||
            modelInfo_.inputDim != modelInfo_.armDim + 2U)
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] expected the WBMM 9D/8D-style "
                "state/input layout.");
        }
        if (modelInfo_.dofNames.size() != modelInfo_.armDim)
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] dofNames must match armDim.");
        }
        if (!model.existFrame(settings_.frameName))
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] frame '" + settings_.frameName +
                "' is not in the Pinocchio model.");
        }
        endEffectorFrameId_ = model.getFrameId(settings_.frameName);

        limits_.joint_min.reserve(modelInfo_.armDim);
        limits_.joint_max.reserve(modelInfo_.armDim);
        limits_.max_joint_speed.reserve(modelInfo_.armDim);
        for (const auto &name : modelInfo_.dofNames)
        {
            const auto jointId = model.getJointId(name);
            if (jointId >= static_cast<pinocchio::JointIndex>(model.njoints))
            {
                throw std::runtime_error(
                    "[ArmManipulabilityCost] joint '" + name +
                    "' is not in the Pinocchio model.");
            }
            const auto &joint = model.joints[jointId];
            if (joint.nq() != 1 || joint.nv() != 1)
            {
                throw std::runtime_error(
                    "[ArmManipulabilityCost] joint '" + name +
                    "' must be 1-DoF.");
            }
            limits_.joint_min.push_back(model.lowerPositionLimit[joint.idx_q()]);
            limits_.joint_max.push_back(model.upperPositionLimit[joint.idx_q()]);
            limits_.max_joint_speed.push_back(model.velocityLimit[joint.idx_v()]);
        }
        if (!(settings_.finiteDiffStep > 0.0) ||
            !std::isfinite(settings_.finiteDiffStep))
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] finiteDiffStep must be positive and "
                "finite.");
        }
        if (!std::isfinite(settings_.hessianRegularization) ||
            settings_.hessianRegularization < 0.0)
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] hessianRegularization must be finite and "
                "non-negative.");
        }
        if (!std::isfinite(settings_.invalidMetricsPenalty) ||
            settings_.invalidMetricsPenalty < 0.0)
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] invalidMetricsPenalty must be finite "
                "and non-negative.");
        }
        std::string metricsOptionsMessage;
        if (!wbmm::metrics::ArmMetrics::validateOptions(
                settings_.metricsOptions, &metricsOptionsMessage))
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] invalid metricsOptions: " +
                metricsOptionsMessage);
        }

        validateWeight(settings_.minSingularWeight, "minSingularWeight");
        validateWeight(settings_.yoshikawaWeight, "yoshikawaWeight");
        validateWeight(settings_.taskDirectionWeight, "taskDirectionWeight");
        validateWeight(
            settings_.inverseManipulabilityWeight,
            "inverseManipulabilityWeight");
        validateWeight(settings_.conditionWeight, "conditionWeight");
        if (!std::isfinite(settings_.minSingularRef) ||
            settings_.minSingularRef < 0.0 ||
            !std::isfinite(settings_.yoshikawaRef) ||
            settings_.yoshikawaRef < 0.0 ||
            !std::isfinite(settings_.taskDirectionRef) ||
            settings_.taskDirectionRef < 0.0 ||
            !std::isfinite(settings_.conditionMax) ||
            !(settings_.conditionMax > 0.0))
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] metric references must be finite and "
                "non-negative; conditionMax must be positive.");
        }
    }

    ArmManipulabilityCost *ArmManipulabilityCost::clone() const
    {
        return new ArmManipulabilityCost(*this);
    }

    wbmm::core::JointState ArmManipulabilityCost::toJointState(
        const vector_t &state) const
    {
        wbmm::core::JointState joints;
        joints.names = modelInfo_.dofNames;
        joints.positions.resize(
            static_cast<std::size_t>(modelInfo_.armDim));
        const std::size_t armStartIndex = armStateStartIndex();
        for (std::size_t joint = 0; joint < modelInfo_.armDim; ++joint)
        {
            joints.positions[joint] = state(
                static_cast<Eigen::Index>(armStartIndex + joint));
        }
        return joints;
    }

    std::size_t ArmManipulabilityCost::armStateStartIndex() const noexcept
    {
        return modelInfo_.stateDim - modelInfo_.armDim;
    }

    ArmManipulabilityCost::Metrics ArmManipulabilityCost::computeMetrics(
        const vector_t &state) const
    {
        if (state.size() != static_cast<Eigen::Index>(modelInfo_.stateDim) ||
            !state.allFinite())
        {
            Metrics metrics;
            metrics.status = wbmm::metrics::MetricsStatus::kInvalidInput;
            metrics.message = "OCS2 state must be finite and match stateDim";
            return metrics;
        }
        try
        {
            const auto &model = pinocchioInterface_.getModel();
            auto data = pinocchioInterface_.getData();
            const WbmmPinocchioMapping mapping(modelInfo_);
            const vector_t configuration =
                mapping.getPinocchioJointPosition(state);

            pinocchio::forwardKinematics(model, data, configuration);
            pinocchio::updateFramePlacements(model, data);
            pinocchio::computeJointJacobians(model, data, configuration);

            matrix_t pinocchioJacobian = matrix_t::Zero(6, model.nv);
            pinocchio::getFrameJacobian(
                model, data, endEffectorFrameId_,
                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                pinocchioJacobian);

            const matrix_t unusedStateJacobian =
                matrix_t::Zero(6, model.nq);
            const auto mappedJacobians = mapping.getOcs2Jacobian(
                state, unusedStateJacobian, pinocchioJacobian);

            wbmm::core::Header header;
            header.frame_id = settings_.stateFrame;
            return wbmm::metrics::ArmMetrics::evaluate(
                mappedJacobians.second, modelInfo_.armDim, toJointState(state),
                limits_, header, settings_.frameName, settings_.metricsOptions);
        }
        catch (const std::exception &error)
        {
            Metrics metrics;
            metrics.status = wbmm::metrics::MetricsStatus::kModelError;
            metrics.header.frame_id = settings_.stateFrame;
            metrics.link_name = settings_.frameName;
            metrics.options = settings_.metricsOptions;
            metrics.message =
                std::string("OCS2 Pinocchio evaluation failed: ") + error.what();
            return metrics;
        }
    }

    scalar_t ArmManipulabilityCost::costFromMetrics(const Metrics &metrics) const
    {
        if (metrics.status != wbmm::metrics::MetricsStatus::kSuccess ||
            !std::isfinite(metrics.sigma_min) ||
            !std::isfinite(metrics.manipulability) ||
            !std::isfinite(metrics.inverse_manipulability) ||
            !std::isfinite(metrics.condition_number) ||
            (settings_.metricsOptions.use_task_direction &&
             !std::isfinite(metrics.task_direction_manipulability)))
        {
            return settings_.invalidMetricsPenalty;
        }

        scalar_t cost = 0.0;

        if (settings_.useMinSingularValue)
        {
            const scalar_t deficit =
                std::max<scalar_t>(0.0, settings_.minSingularRef - metrics.sigma_min);
            cost += 0.5 * settings_.minSingularWeight * deficit * deficit;
        }

        if (settings_.useYoshikawa)
        {
            const scalar_t deficit = std::max<scalar_t>(
                0.0, settings_.yoshikawaRef - metrics.manipulability);
            cost += 0.5 * settings_.yoshikawaWeight * deficit * deficit;
        }

        if (settings_.metricsOptions.use_task_direction)
        {
            const scalar_t deficit = std::max<scalar_t>(
                0.0, settings_.taskDirectionRef -
                         metrics.task_direction_manipulability);
            cost += 0.5 * settings_.taskDirectionWeight * deficit * deficit;
        }

        if (settings_.useInverseManipulability)
        {
            cost += settings_.inverseManipulabilityWeight *
                    metrics.inverse_manipulability;
        }

        if (settings_.useConditionNumber)
        {
            const scalar_t excess = std::max<scalar_t>(
                0.0, metrics.condition_number - settings_.conditionMax);
            cost += 0.5 * settings_.conditionWeight * excess * excess;
        }

        return cost;
    }

    std::vector<int> ArmManipulabilityCost::activeStateIndices() const
    {
        std::vector<int> indices;
        if (settings_.metricsOptions.scope ==
            wbmm::metrics::JacobianScope::kArmColumns)
        {
            indices.reserve(
                modelInfo_.armDim +
                (settings_.metricsOptions.use_task_direction ? 1U : 0U));
            // A direction expressed in the world/planning frame rotates relative
            // to an arm Jacobian when base yaw changes.
            if (settings_.metricsOptions.use_task_direction)
            {
                indices.push_back(2);
            }
            const std::size_t armStartIndex = armStateStartIndex();
            for (std::size_t i = 0; i < modelInfo_.armDim; ++i)
            {
                indices.push_back(
                    static_cast<int>(armStartIndex + i));
            }
        }
        else
        {
            // Whole-body task Jacobian depends on yaw and arm joints, not on x/y.
            indices.reserve(modelInfo_.stateDim - 2U);
            for (std::size_t i = 2U; i < modelInfo_.stateDim; ++i)
            {
                indices.push_back(static_cast<int>(i));
            }
        }
        return indices;
    }

    scalar_t ArmManipulabilityCost::getValue(
        scalar_t /*time*/, const vector_t &state,
        const TargetTrajectories & /*targetTrajectories*/,
        const PreComputation & /*preComputation*/) const
    {
        return costFromMetrics(computeMetrics(state));
    }

    ScalarFunctionQuadraticApproximation
    ArmManipulabilityCost::getQuadraticApproximation(
        scalar_t /*time*/, const vector_t &state,
        const TargetTrajectories & /*targetTrajectories*/,
        const PreComputation & /*preComputation*/) const
    {
        ScalarFunctionQuadraticApproximation cost;
        cost.f = 0.0;
        cost.dfdx = vector_t::Zero(state.rows());
        cost.dfdxx = matrix_t::Zero(state.rows(), state.rows());

        const Metrics current = computeMetrics(state);
        if (current.status != wbmm::metrics::MetricsStatus::kSuccess)
        {
            cost.f = settings_.invalidMetricsPenalty;
            cost.dfdxx = settings_.hessianRegularization *
                         matrix_t::Identity(state.rows(), state.rows());
            return cost;
        }

        cost.f = costFromMetrics(current);

        const auto indices = activeStateIndices();
        const scalar_t step = settings_.finiteDiffStep;

        vector_t gradSigmaMin = vector_t::Zero(state.rows());
        vector_t gradYoshikawa = vector_t::Zero(state.rows());
        vector_t gradTaskDirection = vector_t::Zero(state.rows());
        vector_t gradInverseManipulability = vector_t::Zero(state.rows());
        vector_t gradCondition = vector_t::Zero(state.rows());

        for (const int index : indices)
        {
            vector_t statePlus = state;
            vector_t stateMinus = state;
            statePlus(index) += step;
            stateMinus(index) -= step;

            const Metrics plus = computeMetrics(statePlus);
            const Metrics minus = computeMetrics(stateMinus);
            if (plus.status != wbmm::metrics::MetricsStatus::kSuccess ||
                minus.status != wbmm::metrics::MetricsStatus::kSuccess)
            {
                cost.dfdx.setZero();
                cost.dfdxx =
                    settings_.hessianRegularization *
                    matrix_t::Identity(state.rows(), state.rows());
                return cost;
            }

            const scalar_t denominator = 2.0 * step;
            gradSigmaMin(index) =
                (plus.sigma_min - minus.sigma_min) / denominator;
            gradYoshikawa(index) =
                (plus.manipulability - minus.manipulability) / denominator;
            if (settings_.metricsOptions.use_task_direction)
            {
                gradTaskDirection(index) =
                    (plus.task_direction_manipulability -
                     minus.task_direction_manipulability) /
                    denominator;
            }
            gradInverseManipulability(index) =
                (plus.inverse_manipulability -
                 minus.inverse_manipulability) /
                denominator;
            gradCondition(index) =
                (plus.condition_number - minus.condition_number) / denominator;
        }

        vector_t gradient = vector_t::Zero(state.rows());
        matrix_t hessian =
            settings_.hessianRegularization *
            matrix_t::Identity(state.rows(), state.rows());

        const auto addHinge = [&gradient, &hessian](
                                  const vector_t &metricGradient,
                                  scalar_t metricValue,
                                  scalar_t reference,
                                  scalar_t weight)
        {
            if (metricValue < reference)
            {
                const scalar_t deficit = reference - metricValue;
                gradient.noalias() += -weight * deficit * metricGradient;
                hessian.noalias() +=
                    weight * metricGradient * metricGradient.transpose();
            }
        };

        if (settings_.useMinSingularValue)
        {
            addHinge(
                gradSigmaMin, current.sigma_min,
                settings_.minSingularRef, settings_.minSingularWeight);
        }
        if (settings_.useYoshikawa)
        {
            addHinge(
                gradYoshikawa, current.manipulability,
                settings_.yoshikawaRef, settings_.yoshikawaWeight);
        }
        if (settings_.metricsOptions.use_task_direction)
        {
            addHinge(
                gradTaskDirection, current.task_direction_manipulability,
                settings_.taskDirectionRef, settings_.taskDirectionWeight);
        }
        if (settings_.useConditionNumber &&
            current.condition_number > settings_.conditionMax)
        {
            const scalar_t excess = current.condition_number - settings_.conditionMax;
            gradient.noalias() +=
                settings_.conditionWeight * excess * gradCondition;
            hessian.noalias() += settings_.conditionWeight *
                                 gradCondition * gradCondition.transpose();
        }
        if (settings_.useInverseManipulability)
        {
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

} // namespace wbmm_ocs2
