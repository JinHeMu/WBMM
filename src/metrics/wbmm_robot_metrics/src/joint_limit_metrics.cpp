#include "wbmm_robot_metrics/joint_limit_metrics.hpp"

namespace wbmm::metrics
{

JointLimitMetricsResult JointLimitMetrics::evaluate(
  const wbmm::core::JointState &, const wbmm::core::RobotLimits &)
{
  // TBD: validate matching sizes/order and finite limits, then compute normalized margins.
  return {};
}

}  // namespace wbmm::metrics
