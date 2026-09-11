#include "wbmm_ocs2/constraint/BodyRelativeConstraint.h"

#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/Numerics.h>
#include <ocs2_core/constraint/ConstraintOrder.h>
#include <wbmm_ocs2/PreComputation.h>

namespace wbmm_ocs2
{

// 迁移期：原 OCS2 mobile_manipulator 代码位于 ocs2 命名空间内，
// 这里用文件级 using-directive 保持上游类型的可见性，
// 不污染被 include 的头文件。
using namespace ocs2;

    BodyRelativeConstraint::BodyRelativeConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                   const std::string& bodyLinkName,
                                                   scalar_t rollTolerance,
                                                   scalar_t pitchTolerance)
        : StateConstraint(ConstraintOrder::Linear)
          , bodyLinkName_(bodyLinkName)
          , rollTolerance_(rollTolerance)
          , pitchTolerance_(pitchTolerance)
          , endEffectorKinematicsPtr_(endEffectorKinematics.clone())
    {
        // 初始化目标姿态为单位四元数（无旋转）
        targetOrientation_ = quaternion_t::Identity();
        targetPosition_ = vector3_t::Zero();

        // Cache PinocchioEndEffectorKinematics pointer
        pinocchioEEKinPtr_ = dynamic_cast<PinocchioEndEffectorKinematics*>(endEffectorKinematicsPtr_.get());

        // Verify if bodyLinkName_ exists in the kinematics interface ID list
        const auto& availableIds = endEffectorKinematicsPtr_->getIds();
        bool found = false;
        for (const auto& id : availableIds)
        {
            if (id == bodyLinkName_)
            {
                found = true;
                break;
            }
        }

        if (!found)
        {
            std::cerr << "[BodyRelativeConstraint] Warning: bodyLinkName '" << bodyLinkName_
                << "' not found in available IDs: ";
            for (const auto& id : availableIds)
            {
                std::cerr << "'" << id << "' ";
            }
            std::cerr << std::endl;
        }
    }

    size_t BodyRelativeConstraint::getNumConstraints(scalar_t time) const
    {
        (void)time;
        // Attitude constraints: roll and pitch + Position constraints: x, y
        return 4;
    }

    vector_t BodyRelativeConstraint::getValue(scalar_t time, const vector_t& state,
                                              const PreComputation& preComputation) const
    {
        // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface
        if (pinocchioEEKinPtr_ != nullptr)
        {
            const auto& preCompMM = cast<WbmmPreComputation>(preComputation);
            pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
        }

        // Update targetPosition based on model type
        updateTargetPosition(state);

        // Get orientation and position errors
        const auto orientationErrors = endEffectorKinematicsPtr_->getOrientationError(state, {targetOrientation_});
        const auto positions = endEffectorKinematicsPtr_->getPosition(state);

        // Calculate position error (relative to base frame)
        vector3_t positionError = positions[0] - targetPosition_;

        // Calculate constraint values: first 2 are attitude constraints, last 2 are position constraints (X and Y)
        vector_t constraints(getNumConstraints(time));

        // Attitude constraints: roll and pitch
        constraints.head<2>() = orientationErrors[0].head<2>();

        // Position constraints: x, y (Z direction unconstrained, allowing free vertical movement)
        constraints.tail<2>() = positionError.head<2>();

        return constraints;
    }

    VectorFunctionLinearApproximation BodyRelativeConstraint::getLinearApproximation(
        scalar_t time, const vector_t& state,
        const PreComputation& preComputation) const
    {
        // PinocchioEndEffectorKinematics requires pre-computation with shared PinocchioInterface
        if (pinocchioEEKinPtr_ != nullptr)
        {
            const auto& preCompMM = cast<WbmmPreComputation>(preComputation);
            pinocchioEEKinPtr_->setPinocchioInterface(preCompMM.getPinocchioInterface());
        }

        // Update targetPosition based on model type (same logic as getValue)
        updateTargetPosition(state);

        // Get linear approximation of orientation and position errors
        const auto orientationErrors = endEffectorKinematicsPtr_->getOrientationErrorLinearApproximation(
            state, {targetOrientation_});
        const auto positions = endEffectorKinematicsPtr_->getPositionLinearApproximation(state);

        // Calculate linear approximation of constraints: first 2 are attitude constraints, last 2 are position constraints
        auto approximation = VectorFunctionLinearApproximation(getNumConstraints(time), state.rows(), 0);

        // Attitude constraints: roll and pitch
        approximation.f.head<2>() = orientationErrors[0].f.head<2>();
        approximation.dfdx.topRows<2>() = orientationErrors[0].dfdx.topRows<2>();

        // Position constraints: x, y (Z direction unconstrained, allowing free vertical movement)
        // Position error = current position - target position
        approximation.f.tail<2>() = (positions[0].f - targetPosition_).head<2>();
        approximation.dfdx.bottomRows<2>() = positions[0].dfdx.topRows<2>();

        return approximation;
    }

    void BodyRelativeConstraint::updateTargetPosition(const vector_t& state) const
    {
        if (endEffectorKinematicsPtr_ == nullptr) {
            return;
        }
        // positions[0] = body link，positions[1] = base frame。
        // 用 base frame 的 FK 位置作为目标，使 body link 相对基座保持
        // XY 位置约束；Z/高度方向不在此约束内。
        try {
            const auto positions = endEffectorKinematicsPtr_->getPosition(state);
            if (positions.size() >= 2) {
                targetPosition_ = positions[1];
            }
        } catch (const std::exception&) {
            // 参考不可用时保持 targetPosition_ 不变（fail-safe）。
        }
    }

}  // namespace wbmm_ocs2
