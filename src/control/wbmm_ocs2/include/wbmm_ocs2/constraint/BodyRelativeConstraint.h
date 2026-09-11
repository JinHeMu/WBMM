#pragma once

#include <memory>
#include <string>

#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

#include <ocs2_core/constraint/StateConstraint.h>

namespace wbmm_ocs2
{
    class BodyRelativeConstraint final : public ocs2::StateConstraint
    {
    public:
        using vector2_t = Eigen::Matrix<ocs2::scalar_t, 2, 1>;
        using vector3_t = Eigen::Matrix<ocs2::scalar_t, 3, 1>;
        using quaternion_t = Eigen::Quaternion<ocs2::scalar_t>;
        using matrix3_t = Eigen::Matrix<ocs2::scalar_t, 3, 3>;

        // WBMM 轮式底盘：base frame 与 body link 的目标位置关系由
        // Pinocchio FK 直接给出，不再需要模型类型分支。
        BodyRelativeConstraint(const ocs2::EndEffectorKinematics<ocs2::scalar_t>& endEffectorKinematics,
                               const std::string& bodyLinkName,
                               ocs2::scalar_t rollTolerance,
                               ocs2::scalar_t pitchTolerance);

        ~BodyRelativeConstraint() override = default;

        BodyRelativeConstraint* clone() const override
        {
            return new BodyRelativeConstraint(*endEffectorKinematicsPtr_, bodyLinkName_,
                                              rollTolerance_, pitchTolerance_);
        }

        size_t getNumConstraints(ocs2::scalar_t time) const override;
        ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state, const ocs2::PreComputation& preComputation) const override;
        ocs2::VectorFunctionLinearApproximation getLinearApproximation(ocs2::scalar_t time, const ocs2::vector_t& state,
                                                                 const ocs2::PreComputation& preComputation) const override;

    private:
        BodyRelativeConstraint(const BodyRelativeConstraint& other) = default;

        // Calculate attitude constraints relative to base frame
        ocs2::vector_t computeOrientationConstraints(const quaternion_t& bodyOrientation) const;

        // 由 base frame 的 FK 位置更新 targetPosition。
        void updateTargetPosition(const ocs2::vector_t& state) const;


        // Constraint parameters
        std::string bodyLinkName_;
        ocs2::scalar_t rollTolerance_;
        ocs2::scalar_t pitchTolerance_;

        // Kinematics interface
        std::unique_ptr<ocs2::EndEffectorKinematics<ocs2::scalar_t>> endEffectorKinematicsPtr_;

        /** Cached pointer to the pinocchio end effector kinematics. Is set to nullptr if not used. */
        ocs2::PinocchioEndEffectorKinematics* pinocchioEEKinPtr_ = nullptr;

        // Cached target orientation (relative to base frame)
        quaternion_t targetOrientation_;
        mutable vector3_t targetPosition_;
    };
}
