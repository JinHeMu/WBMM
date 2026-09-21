#pragma once

#include "wbmm_core/robot_model.hpp"
#include "wbmm_robot_metrics/types.hpp"

namespace wbmm::metrics
{

class ArmMetrics
{
public:
  explicit ArmMetrics(wbmm::core::RobotModelPtr robot_model);

  // Uses the arm columns of RobotModel's [v, omega, qdot1..qdot6] Jacobian.
  // No planner cost or control decision is made by this module.
  [[nodiscard]] ArmKinematicMetrics evaluate(
    const wbmm::core::WholeBodyState & state,
    const std::string & link_name,
    ArmMetricsOptions options = {}) const;

private:
  wbmm::core::RobotModelPtr robot_model_;
};

}  // namespace wbmm::metrics
