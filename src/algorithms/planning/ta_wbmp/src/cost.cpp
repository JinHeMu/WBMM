#include "ta_wbmp/cost.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ta_wbmp
{

WeightedCandidateCost::WeightedCandidateCost(CandidateCostWeights weights)
: weights_(std::move(weights))
{
}

double WeightedCandidateCost::evaluate(const CandidateMetrics & metrics) const
{
  if (!metrics.feasible) {
    return std::numeric_limits<double>::infinity();
  }
  return weights_.position_error * metrics.max_position_error +
    weights_.axis_error * metrics.max_axis_error +
    weights_.arm_path * metrics.arm_path_length +
    weights_.base_path * metrics.base_path_length +
    weights_.inverse_joint_margin /
    std::max(1.0e-6, metrics.min_joint_margin + 0.02) +
    weights_.inverse_manipulability /
    std::max(1.0e-8, metrics.min_manipulability + 1.0e-5) +
    weights_.inverse_min_sigma /
    std::max(1.0e-5, metrics.min_sigma + 1.0e-4) +
    weights_.standoff_deviation * std::abs(
    metrics.standoff - weights_.preferred_standoff) +
    weights_.longitudinal_offset * std::abs(
    metrics.longitudinal_offset) +
    weights_.navigation_cost * metrics.navigation_cost_estimate;
}

}  // namespace ta_wbmp
