#pragma once

#include <ocs2_core/constraint/StateConstraint.h>
#include <ocs2_pinocchio_interface/PinocchioStateInputMapping.h>

#include "wbmm_ocs2/collision/EnvironmentGeometryInterface.h"

namespace wbmm_ocs2 {

/**
 * @brief Environment Collision Constraint
 *
 * This class implements a state constraint for avoiding collisions between
 * the robot and environment obstacles. The constraint value is:
 *   h = distance - minimumDistance
 * where h > 0 means safe (no collision).
 *
 * The constraint uses coal for distance computation and provides
 * analytical Jacobians based on the nearest points between robot links
 * and obstacles.
 */
class EnvironmentCollisionConstraint : public ocs2::StateConstraint {
public:
    /**
     * @brief Constructor
     * @param mapping Pinocchio state-input mapping
     * @param envGeomInterface Environment geometry interface (shared)
     * @param minimumDistance Minimum allowed distance between robot and obstacles
     */
    EnvironmentCollisionConstraint(
        const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
        std::shared_ptr<EnvironmentGeometryInterface> envGeomInterface,
        ocs2::scalar_t minimumDistance);

    ~EnvironmentCollisionConstraint() override = default;

    EnvironmentCollisionConstraint* clone() const override = 0;

    /**
     * @brief Get the number of constraint dimensions
     * @note Returns 0 if no obstacles are present
     */
    size_t getNumConstraints(ocs2::scalar_t time) const override;

    /**
     * @brief Evaluate the constraint value
     * @param time Current time
     * @param state Current state
     * @param preComputation Pre-computation cache
     * @return Constraint values (h = distance - minimumDistance)
     *
     * @note Requires pinocchio::forwardKinematics() to be called in pre-computation
     */
    ocs2::vector_t getValue(ocs2::scalar_t time, const ocs2::vector_t& state,
                      const ocs2::PreComputation& preComputation) const override;

    /**
     * @brief Get the linear approximation of the constraint
     * @param time Current time
     * @param state Current state
     * @param preComputation Pre-computation cache
     * @return Linear approximation (f, dfdx)
     *
     * @note Requires pinocchio::forwardKinematics(), updateGlobalPlacements(),
     *       and computeJointJacobians() to be called in pre-computation
     */
    ocs2::VectorFunctionLinearApproximation getLinearApproximation(
        ocs2::scalar_t time, const ocs2::vector_t& state,
        const ocs2::PreComputation& preComputation) const override;

    /**
     * @brief Get the environment geometry interface
     */
    std::shared_ptr<EnvironmentGeometryInterface> getEnvironmentGeometryInterface() const {
        return envGeomInterface_;
    }

    /**
     * @brief Get the minimum distance setting
     */
    ocs2::scalar_t getMinimumDistance() const { return minimumDistance_; }

protected:
    /**
     * @brief Get the ocs2::PinocchioInterface from pre-computation
     * @note To be implemented by derived class for specific robot types
     */
    virtual const ocs2::PinocchioInterface& getPinocchioInterface(
        const ocs2::PreComputation& preComputation) const = 0;

    EnvironmentCollisionConstraint(const EnvironmentCollisionConstraint& rhs);

    std::shared_ptr<EnvironmentGeometryInterface> envGeomInterface_;
    ocs2::scalar_t minimumDistance_;
    std::unique_ptr<ocs2::PinocchioStateInputMapping<ocs2::scalar_t>> mappingPtr_;
};

/**
 * @brief Mobile Manipulator specific environment collision constraint
 *
 * This class provides the getPinocchioInterface() implementation for
 * the WbmmPreComputation class.
 */
class WbmmEnvironmentCollisionConstraint final
    : public EnvironmentCollisionConstraint {
public:
    WbmmEnvironmentCollisionConstraint(
        const ocs2::PinocchioStateInputMapping<ocs2::scalar_t>& mapping,
        std::shared_ptr<EnvironmentGeometryInterface> envGeomInterface,
        ocs2::scalar_t minimumDistance);

    ~WbmmEnvironmentCollisionConstraint() override = default;

    WbmmEnvironmentCollisionConstraint* clone() const override;

protected:
    const ocs2::PinocchioInterface& getPinocchioInterface(
        const ocs2::PreComputation& preComputation) const override;

private:
    WbmmEnvironmentCollisionConstraint(
        const WbmmEnvironmentCollisionConstraint& rhs) = default;
};

} // namespace wbmm_ocs2
