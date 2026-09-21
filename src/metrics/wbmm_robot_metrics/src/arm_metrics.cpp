#include "wbmm_robot_metrics/arm_metrics.hpp"

#include <utility>

namespace wbmm::metrics
{

ArmMetrics::ArmMetrics(wbmm::core::RobotModelPtr robot_model)
: robot_model_(std::move(robot_model)) {}

ArmKinematicMetrics ArmMetrics::evaluate(
  const wbmm::core::WholeBodyState & state,
  const std::string & link_name,
  ArmMetricsOptions options) const
{
  // TBD: validate, obtain arm Jacobian, select task/scaling, SVD and joint margins.
  ArmKinematicMetrics result;
  result.header = state.header;
  result.link_name = link_name;
  result.options = options;
  return result;
}

}  // namespace wbmm::metrics
