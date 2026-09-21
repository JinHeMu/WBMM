#pragma once

#include "wbmm_core/types.hpp"

#include <Eigen/Core>

#include <limits>
#include <string>

namespace wbmm::metrics
{

enum class MetricsStatus {kNotImplemented = 0, kSuccess, kInvalidInput, kModelError};
enum class JacobianTask {kTranslation, kPose};
enum class JacobianScaling {kRaw, kCharacteristicLength};

struct ArmMetricsOptions
{
  JacobianTask task{JacobianTask::kPose};
  JacobianScaling scaling{JacobianScaling::kRaw};
  // In m; kCharacteristicLength multiplies angular rows by this caller-supplied length.
  double characteristic_length{std::numeric_limits<double>::quiet_NaN()};
};

struct JointLimitMetricsResult
{
  MetricsStatus status{MetricsStatus::kNotImplemented};
  // Model joint order: midpoint=1, limit=0, out of range <0.
  Eigen::VectorXd normalized_margin;
  double min_normalized_margin{std::numeric_limits<double>::quiet_NaN()};
  std::string message{"TBD: joint-limit metrics are not implemented"};
};

struct ArmKinematicMetrics
{
  MetricsStatus status{MetricsStatus::kNotImplemented};
  wbmm::core::Header header;
  // Reference point is the origin of this link; expressing frame is header.frame_id.
  std::string link_name;
  ArmMetricsOptions options;
  Eigen::VectorXd singular_values;
  double sigma_min{std::numeric_limits<double>::quiet_NaN()};
  double sigma_max{std::numeric_limits<double>::quiet_NaN()};
  double condition_number{std::numeric_limits<double>::quiet_NaN()};
  double manipulability{std::numeric_limits<double>::quiet_NaN()};
  int numerical_rank{-1};
  JointLimitMetricsResult joint_limits;
  std::string message{"TBD: arm kinematic metrics are not implemented"};
};

}  // namespace wbmm::metrics
