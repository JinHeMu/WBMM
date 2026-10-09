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
#include <pinocchio/algorithm/kinematics-derivatives.hpp>
#include <Eigen/SVD>
#include <limits>
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
        enum MetricRow { Sigma, Yoshikawa, Direction, Inverse, Condition, MetricCount };

        // The hot path needs kinematic metrics, not joint-limit diagnostics.
        // The spectral inverse equals trace((J J^T + regularization I)^-1).
        ArmManipulabilityCost::Metrics spectralMetrics(
            const matrix_t& jacobian, const vector_t& singular,
            const ArmManipulabilitySettings& settings)
        {
            ArmManipulabilityCost::Metrics metrics;
            const auto& options = settings.metricsOptions;
            metrics.status = wbmm::metrics::MetricsStatus::kSuccess;
            metrics.sigma_min = singular.tail(1)(0);
            metrics.sigma_max = singular(0);
            metrics.manipulability = singular.prod();
            metrics.condition_number = metrics.sigma_max /
                std::max(metrics.sigma_min, options.singular_value_floor);
            if (settings.useInverseManipulability) {
                metrics.inverse_manipulability =
                    (singular.array().square() + options.regularization).inverse().sum() +
                    (jacobian.rows() - singular.size()) / options.regularization;
            }
            if (options.use_task_direction) {
                metrics.task_direction_manipulability =
                    (jacobian.transpose() * options.task_direction).norm();
            }
            return metrics;
        }

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
        if (static_cast<std::size_t>(model.nq) != modelInfo_.configurationDim() ||
            static_cast<std::size_t>(model.nv) != modelInfo_.configurationDim())
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] Pinocchio nq/nv must match configurationDim.");
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
        kinematicHessian_.resize(6, model.nv, model.nv);
        kinematicHessian_.setZero();

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

        if (settings_.metricsOptions.use_task_direction) {
            settings_.metricsOptions.task_direction.normalize();
        }

        validateWeight(settings_.minSingularWeight, "minSingularWeight");
        validateWeight(settings_.weightScale, "weightScale");
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
        if (settings_.normalizeMargins &&
            ((settings_.useMinSingularValue && !(settings_.minSingularRef > 0.0)) ||
             (settings_.useYoshikawa && !(settings_.yoshikawaRef > 0.0)) ||
             (settings_.metricsOptions.use_task_direction && !(settings_.taskDirectionRef > 0.0))))
        {
            throw std::runtime_error(
                "[ArmManipulabilityCost] normalized enabled margins require positive references.");
        }
        if (settings_.normalizeMargins)
        {
            const auto validateNormalizedWeight = [this](bool enabled, scalar_t weight, scalar_t reference)
            {
                if (enabled && !std::isfinite(settings_.weightScale * marginWeight(weight, reference)))
                {
                    throw std::runtime_error("[ArmManipulabilityCost] normalized margin weight overflows; check reference and weightScale.");
                }
            };
            validateNormalizedWeight(settings_.useMinSingularValue, settings_.minSingularWeight, settings_.minSingularRef);
            validateNormalizedWeight(settings_.useYoshikawa, settings_.yoshikawaWeight, settings_.yoshikawaRef);
            validateNormalizedWeight(settings_.metricsOptions.use_task_direction, settings_.taskDirectionWeight, settings_.taskDirectionRef);
        }
    }

    ArmManipulabilityCost::ArmManipulabilityCost(const ArmManipulabilityCost &other)
        : ArmManipulabilityCost(other.pinocchioInterface_, other.modelInfo_, other.settings_)
    {
    }

    ArmManipulabilityCost *ArmManipulabilityCost::clone() const
    {
        std::lock_guard<std::mutex> lock(workspaceMutex_);
        return new ArmManipulabilityCost(*this);
    }

    std::string ArmManipulabilityCost::metricContract() const
    {
        const auto &options = settings_.metricsOptions;
        return "frame=" + settings_.stateFrame + ";point=" + settings_.frameName +
            ";scope=" + (options.scope == wbmm::metrics::JacobianScope::kArmColumns ? "arm" : "whole_body") +
            ";task=" + (options.task == wbmm::metrics::JacobianTask::kPose ? "pose" : "translation") +
            ";scaling=" + (options.scaling == wbmm::metrics::JacobianScaling::kCharacteristicLength
                ? "characteristic_length;ell_m=" + std::to_string(options.characteristic_length) : "raw") + ";";
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
        return 3;
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
            std::lock_guard<std::mutex> lock(workspaceMutex_);
            const auto &model = pinocchioInterface_.getModel();
            auto &data = pinocchioInterface_.getData();
            const WbmmPinocchioMapping mapping(modelInfo_);
            const vector_t configuration =
                mapping.getPinocchioJointPosition(state);

            // computeJointJacobians(q) already updates joint placements.
            pinocchio::computeJointJacobians(model, data, configuration);
            pinocchio::updateFramePlacement(model, data, endEffectorFrameId_);

            matrix_t pinocchioJacobian = matrix_t::Zero(6, model.nv);
            pinocchio::getFrameJacobian(
                model, data, endEffectorFrameId_,
                pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                pinocchioJacobian);

            auto mobilityInfo = modelInfo_;
            mobilityInfo.baseResponse.enabled = false;
            const WbmmPinocchioMapping mobilityMapping(mobilityInfo);
            const auto mappedJacobians = mobilityMapping.getOcs2Jacobian(
                state, matrix_t::Zero(6, model.nq), pinocchioJacobian);

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

    matrix_t ArmManipulabilityCost::taskJacobian(
        const matrix_t& frameJacobian, const vector_t& state) const
    {
        const auto& options = settings_.metricsOptions;
        const Eigen::Index rows = options.task == wbmm::metrics::JacobianTask::kTranslation ? 3 : 6;
        matrix_t result;
        if (options.scope == wbmm::metrics::JacobianScope::kArmColumns) {
            result = frameJacobian.topRows(rows).rightCols(modelInfo_.armDim);
        } else {
            // Same input map as WbmmPinocchioMapping: qdot=[cos(yaw)*v,sin(yaw)*v,omega,qdot_arm].
            result.resize(rows, modelInfo_.inputDim);
            result.col(0) = std::cos(state(2)) * frameJacobian.topRows(rows).col(0) +
                            std::sin(state(2)) * frameJacobian.topRows(rows).col(1);
            result.col(1) = frameJacobian.topRows(rows).col(2);
            result.rightCols(modelInfo_.armDim) = frameJacobian.topRows(rows).rightCols(modelInfo_.armDim);
        }
        if (rows == 6 && options.scaling == wbmm::metrics::JacobianScaling::kCharacteristicLength) {
            result.bottomRows(3) *= options.characteristic_length;
        }
        return result;
    }

    bool ArmManipulabilityCost::hasActiveTerms(const Metrics& current) const
    {
        return settings_.weightScale > 0.0 &&
            ((settings_.useMinSingularValue && settings_.minSingularWeight > 0.0 && current.sigma_min < settings_.minSingularRef) ||
             (settings_.useYoshikawa && settings_.yoshikawaWeight > 0.0 && current.manipulability < settings_.yoshikawaRef) ||
             (settings_.metricsOptions.use_task_direction && settings_.taskDirectionWeight > 0.0 && current.task_direction_manipulability < settings_.taskDirectionRef) ||
             (settings_.useConditionNumber && settings_.conditionWeight > 0.0 && current.condition_number > settings_.conditionMax) ||
             (settings_.useInverseManipulability && settings_.inverseManipulabilityWeight > 0.0));
    }

    ArmManipulabilityCost::Metrics ArmManipulabilityCost::evaluateCostMetrics(
        const vector_t& state, matrix_t* gradients) const
    {
        Metrics current;
        if (state.size() != static_cast<Eigen::Index>(modelInfo_.stateDim) || !state.allFinite()) {
            current.status = wbmm::metrics::MetricsStatus::kInvalidInput;
            return current;
        }
        std::lock_guard<std::mutex> lock(workspaceMutex_);
        const auto& model = pinocchioInterface_.getModel();
        auto& data = pinocchioInterface_.getData();
        pinocchio::computeJointJacobians(model, data, vector_t(state.head(modelInfo_.configurationDim())));
        pinocchio::updateFramePlacement(model, data, endEffectorFrameId_);
        matrix_t frameJacobian = matrix_t::Zero(6, model.nv);
        pinocchio::getFrameJacobian(model, data, endEffectorFrameId_,
            pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, frameJacobian);
        const matrix_t jacobian = taskJacobian(frameJacobian, state);
        const auto& options = settings_.metricsOptions;
        if (!jacobian.allFinite()) {
            current.status = wbmm::metrics::MetricsStatus::kModelError;
            return current;
        }
        Eigen::JacobiSVD<matrix_t> svd(jacobian,
            gradients ? Eigen::ComputeThinU | Eigen::ComputeThinV : 0);
        const vector_t singular = svd.singularValues();
        if (!singular.allFinite()) {
            current.status = wbmm::metrics::MetricsStatus::kModelError;
            return current;
        }
        current = spectralMetrics(jacobian, singular, settings_);
        if (!gradients || !hasActiveTerms(current)) { return current; }

        // Native WORLD Hessian uses tensor(row, Jacobian column, differentiation coordinate).
        // WORLD linear velocity is at the world origin. Shift BOTH J and dJ to the TCP.
        pinocchio::computeJointKinematicHessians(model, data);
        kinematicHessian_.setZero();
        pinocchio::getJointKinematicHessian(model, data,
            model.frames[endEffectorFrameId_].parentJoint, pinocchio::WORLD, kinematicHessian_);
        const Eigen::Vector3d point = data.oMf[endEffectorFrameId_].translation();
        const scalar_t tolerance = std::sqrt(std::numeric_limits<scalar_t>::epsilon()) *
            std::max<scalar_t>(1.0, singular(0));
        bool spectralDifference = singular.tail(1)(0) <= tolerance;
        for (Eigen::Index i = 1; i < singular.size(); ++i) {
            spectralDifference = spectralDifference || singular(i - 1) - singular(i) <= tolerance;
        }
        if (settings_.useConditionNumber) {
            spectralDifference = spectralDifference ||
                std::abs(current.sigma_min - options.singular_value_floor) <= tolerance;
        }
        matrix_t derivative(6, model.nv);
        for (const int index : activeStateIndices()) {
            derivative = Eigen::Map<const matrix_t>(
                kinematicHessian_.data() + index * 6 * model.nv, 6, model.nv);
            const Eigen::Vector3d pointDerivative = frameJacobian.col(index).head<3>();
            for (Eigen::Index column = 0; column < model.nv; ++column) {
                derivative.col(column).head<3>() +=
                    derivative.col(column).tail<3>().cross(point) +
                    frameJacobian.col(column).tail<3>().cross(pointDerivative);
            }
            // WBMM root is composite [PX,PY,RZ]. PX/PY are world coordinates:
            // their LWA Jacobian columns are constant world axes. Pinocchio's
            // geometric Hessian includes intra-composite Lie brackets, which
            // must not become coordinate derivatives of these two columns.
            derivative.leftCols(2).setZero();
            matrix_t dJ = taskJacobian(derivative, state);
            if (index == 2 && options.scope == wbmm::metrics::JacobianScope::kWholeBodyInput) {
                vector_t mapDerivative = -std::sin(state(2)) * frameJacobian.col(0) +
                                         std::cos(state(2)) * frameJacobian.col(1);
                if (options.task == wbmm::metrics::JacobianTask::kPose &&
                    options.scaling == wbmm::metrics::JacobianScaling::kCharacteristicLength) {
                    mapDerivative.tail(3) *= options.characteristic_length;
                }
                dJ.col(0) += mapDerivative.head(jacobian.rows());
            }
            if (spectralDifference) {
                // Repeated/zero singular values have no unique SVD-vector derivative.
                // Central differences on the linearized matrix keep the symmetric convention
                // of the old implementation, without another kinematics call.
                const scalar_t step = settings_.finiteDiffStep;
                const matrix_t plusJ = jacobian + step * dJ, minusJ = jacobian - step * dJ;
                const Eigen::JacobiSVD<matrix_t> plusSvd(plusJ), minusSvd(minusJ);
                const auto plus = spectralMetrics(plusJ, plusSvd.singularValues(), settings_);
                const auto minus = spectralMetrics(minusJ, minusSvd.singularValues(), settings_);
                (*gradients)(Sigma, index) = (plus.sigma_min - minus.sigma_min) / (2 * step);
                (*gradients)(Yoshikawa, index) = (plus.manipulability - minus.manipulability) / (2 * step);
                (*gradients)(Condition, index) = (plus.condition_number - minus.condition_number) / (2 * step);
                if (settings_.useInverseManipulability) {
                    (*gradients)(Inverse, index) = (plus.inverse_manipulability - minus.inverse_manipulability) / (2 * step);
                }
            } else {
                vector_t dSigma(singular.size());
                for (Eigen::Index i = 0; i < singular.size(); ++i) {
                    dSigma(i) = svd.matrixU().col(i).dot(dJ * svd.matrixV().col(i));
                }
                (*gradients)(Sigma, index) = dSigma.tail(1)(0);
                if (settings_.useYoshikawa) {
                    scalar_t dw = 0.0;
                    for (Eigen::Index i = 0; i < singular.size(); ++i) {
                        scalar_t product = 1.0;
                        for (Eigen::Index j = 0; j < singular.size(); ++j) {
                            if (j != i) { product *= singular(j); }
                        }
                        dw += product * dSigma(i);
                    }
                    (*gradients)(Yoshikawa, index) = dw;
                }
                if (settings_.useInverseManipulability) {
                    (*gradients)(Inverse, index) =
                        (-2 * singular.array() * dSigma.array() /
                         (singular.array().square() + options.regularization).square()).sum();
                }
                if (settings_.useConditionNumber) {
                    const scalar_t denominator = std::max(current.sigma_min, options.singular_value_floor);
                    (*gradients)(Condition, index) = dSigma(0) / denominator -
                        (current.sigma_min > options.singular_value_floor ?
                         current.sigma_max * dSigma.tail(1)(0) / (denominator * denominator) : 0.0);
                }
            }
            if (options.use_task_direction && current.task_direction_manipulability > 0.0) {
                (*gradients)(Direction, index) =
                    (jacobian.transpose() * options.task_direction).dot(dJ.transpose() * options.task_direction) /
                    current.task_direction_manipulability;
            }
        }
        return current;
    }

    scalar_t ArmManipulabilityCost::marginWeight(scalar_t weight, scalar_t reference) const
    {
        return settings_.normalizeMargins ? weight / (reference * reference) : weight;
    }

    scalar_t ArmManipulabilityCost::costFromMetrics(const Metrics &metrics) const
    {
        if (metrics.status != wbmm::metrics::MetricsStatus::kSuccess ||
            !std::isfinite(metrics.sigma_min) ||
            !std::isfinite(metrics.manipulability) ||
            (settings_.useInverseManipulability && !std::isfinite(metrics.inverse_manipulability)) ||
            (settings_.useConditionNumber && !std::isfinite(metrics.condition_number)) ||
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
            cost += 0.5 * marginWeight(settings_.minSingularWeight, settings_.minSingularRef) * deficit * deficit;
        }

        if (settings_.useYoshikawa)
        {
            const scalar_t deficit = std::max<scalar_t>(
                0.0, settings_.yoshikawaRef - metrics.manipulability);
            cost += 0.5 * marginWeight(settings_.yoshikawaWeight, settings_.yoshikawaRef) * deficit * deficit;
        }

        if (settings_.metricsOptions.use_task_direction)
        {
            const scalar_t deficit = std::max<scalar_t>(
                0.0, settings_.taskDirectionRef -
                         metrics.task_direction_manipulability);
            cost += 0.5 * marginWeight(settings_.taskDirectionWeight, settings_.taskDirectionRef) * deficit * deficit;
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

        return settings_.weightScale * cost;
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
            indices.reserve(modelInfo_.configurationDim() - 2U);
            for (std::size_t i = 2U; i < modelInfo_.configurationDim(); ++i)
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
        return costFromMetrics(evaluateCostMetrics(state));
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

        matrix_t metricGradients = matrix_t::Zero(MetricCount, state.rows());
        const Metrics current = evaluateCostMetrics(state, &metricGradients);
        if (current.status != wbmm::metrics::MetricsStatus::kSuccess)
        {
            cost.f = settings_.invalidMetricsPenalty;
            cost.dfdxx = settings_.hessianRegularization *
                         matrix_t::Identity(state.rows(), state.rows());
            return cost;
        }

        cost.f = costFromMetrics(current);

        // Above all hinge thresholds there is no metric derivative to compute.
        // The analytic metric path skips derivative assembly in the healthy zone.
        const bool active = hasActiveTerms(current);
        if (!active)
        {
            cost.dfdxx = settings_.weightScale * settings_.hessianRegularization *
                         matrix_t::Identity(state.rows(), state.rows());
            return cost;
        }

        const vector_t gradSigmaMin = metricGradients.row(Sigma).transpose();
        const vector_t gradYoshikawa = metricGradients.row(Yoshikawa).transpose();
        const vector_t gradTaskDirection = metricGradients.row(Direction).transpose();
        const vector_t gradInverseManipulability = metricGradients.row(Inverse).transpose();
        const vector_t gradCondition = metricGradients.row(Condition).transpose();

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
                settings_.minSingularRef, marginWeight(settings_.minSingularWeight, settings_.minSingularRef));
        }
        if (settings_.useYoshikawa)
        {
            addHinge(
                gradYoshikawa, current.manipulability,
                settings_.yoshikawaRef, marginWeight(settings_.yoshikawaWeight, settings_.yoshikawaRef));
        }
        if (settings_.metricsOptions.use_task_direction)
        {
            addHinge(
                gradTaskDirection, current.task_direction_manipulability,
                settings_.taskDirectionRef, marginWeight(settings_.taskDirectionWeight, settings_.taskDirectionRef));
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

        cost.dfdx = settings_.weightScale * gradient;
        cost.dfdxx = settings_.weightScale * hessian;
        return cost;
    }

} // namespace wbmm_ocs2
