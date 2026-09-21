#include "wbmm_robot_metrics/joint_limit_metrics.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace wbmm::metrics
{
namespace
{

JointLimitMetricsResult invalid(const std::string & message)
{
  JointLimitMetricsResult result;
  result.status = MetricsStatus::kInvalidInput;
  result.message = message;
  return result;
}

}  // namespace

JointLimitMetricsResult JointLimitMetrics::evaluate(
  const wbmm::core::JointState & joints, const wbmm::core::RobotLimits & limits)
{
  const auto joint_count = limits.joint_min.size();
  if (joint_count == 0) {
    return invalid("RobotLimits.joint_min must not be empty");
  }
  if (limits.joint_max.size() != joint_count) {
    return invalid("RobotLimits.joint_min and joint_max sizes must match");
  }
  if (joints.positions.size() != joint_count) {
    return invalid("JointState.positions size must match RobotLimits");
  }
  if (!joints.names.empty() && joints.names.size() != joint_count) {
    return invalid("JointState.names size must match RobotLimits when non-empty");
  }
  if (!limits.max_joint_speed.empty() &&
    limits.max_joint_speed.size() != joint_count)
  {
    return invalid("RobotLimits.max_joint_speed size must match joint limits");
  }

  Eigen::VectorXd margins(static_cast<Eigen::Index>(joint_count));
  for (std::size_t joint = 0; joint < joint_count; ++joint) {
    const double lower = limits.joint_min[joint];
    const double upper = limits.joint_max[joint];
    const double position = joints.positions[joint];
    if (!std::isfinite(lower) || !std::isfinite(upper) ||
      !std::isfinite(position))
    {
      return invalid("joint positions and limits must be finite");
    }
    if (!(upper > lower)) {
      return invalid("every joint upper limit must be greater than lower limit");
    }

    const double midpoint = 0.5 * (upper + lower);
    const double half_range = 0.5 * (upper - lower);
    // midpoint=1, boundary=0, out of range<0.
    margins(static_cast<Eigen::Index>(joint)) =
      1.0 - std::abs(position - midpoint) / half_range;
  }

  JointLimitMetricsResult result;
  result.status = MetricsStatus::kSuccess;
  result.normalized_margin = std::move(margins);
  result.min_normalized_margin = result.normalized_margin.minCoeff();
  result.message = "ok";
  return result;
}

}  // namespace wbmm::metrics
