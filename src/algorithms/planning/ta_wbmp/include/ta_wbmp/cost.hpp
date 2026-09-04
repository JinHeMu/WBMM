#pragma once

#include <limits>
#include <memory>
#include <string>

namespace ta_wbmp
{

struct CandidateMetrics
{
  int candidate_id{-1};
  double standoff{0.0};
  double longitudinal_offset{0.0};
  double yaw_offset{0.0};
  bool feasible{false};
  std::string failure_reason;
  double max_position_error{0.0};
  double max_axis_error{0.0};
  double min_joint_margin{std::numeric_limits<double>::infinity()};
  double min_manipulability{std::numeric_limits<double>::infinity()};
  double min_sigma{std::numeric_limits<double>::infinity()};
  double base_path_length{0.0};
  double arm_path_length{0.0};
  double navigation_cost_estimate{0.0};
  double score{std::numeric_limits<double>::infinity()};
};

struct CandidateCostWeights
{
  double position_error{20.0};
  double axis_error{3.0};
  double arm_path{0.06};
  double base_path{0.08};
  double inverse_joint_margin{0.20};
  double inverse_manipulability{0.002};
  double inverse_min_sigma{0.01};
  double standoff_deviation{0.40};
  double longitudinal_offset{0.20};
  double navigation_cost{0.15};
  double preferred_standoff{0.82};
};

class CandidateCostEvaluator
{
public:
  virtual ~CandidateCostEvaluator() = default;
  virtual double evaluate(const CandidateMetrics & metrics) const = 0;
};

class WeightedCandidateCost final : public CandidateCostEvaluator
{
public:
  explicit WeightedCandidateCost(CandidateCostWeights weights = {});
  double evaluate(const CandidateMetrics & metrics) const override;
  const CandidateCostWeights & weights() const {return weights_;}

private:
  CandidateCostWeights weights_;
};

}  // namespace ta_wbmp
