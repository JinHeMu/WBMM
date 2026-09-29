#pragma once

#include "wbmm_core/robot_model.hpp"
#include "wbmm_robot_metrics/types.hpp"

namespace wbmm::metrics
{

class JointLimitMetrics
{
public:
  // Caller must use RobotModel joint order for both joints and limits.
  [[nodiscard]] static JointLimitMetricsResult evaluate(
    const wbmm::core::JointState & joints, const wbmm::core::RobotLimits & limits);
};

}  // namespace wbmm::metrics
