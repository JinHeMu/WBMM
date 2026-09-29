#pragma once

#include "wbmm_core/types.hpp"

#include <Eigen/Core>

#include <limits>
#include <string>

namespace wbmm::metrics
{

enum class MetricsStatus
{
  kNotEvaluated = 0,
  // Backward-compatible alias retained while the former skeleton API is migrated.
  kNotImplemented = kNotEvaluated,
  kSuccess,
  kInvalidInput,
  kModelError
};
enum class JacobianTask {kTranslation, kPose};
enum class JacobianScaling {kRaw, kCharacteristicLength};

// Column selection applied to the 6 x inputDimension frame Jacobian returned by
// wbmm::core::RobotModel::frameJacobian():
//   kArmColumns:      only the joint columns of the arm (the last joint_names
//                     columns in the generic contract);
//   kWholeBodyInput:  all [v, omega, qdot...] columns.
enum class JacobianScope {kArmColumns, kWholeBodyInput};

struct ArmMetricsOptions
{
  JacobianScope scope{JacobianScope::kArmColumns};
  JacobianTask task{JacobianTask::kPose};
  JacobianScaling scaling{JacobianScaling::kRaw};
  // In m; kCharacteristicLength multiplies angular rows by this caller-supplied length.
  double characteristic_length{std::numeric_limits<double>::quiet_NaN()};

  // Numerical regularization and condition-number floor for diagnostics.
  double regularization{1.0e-6};
  double singular_value_floor{1.0e-9};

  // Optional directional manipulability sqrt(d^T J J^T d).
  // Size must be 3 for kTranslation and 6 for kPose when enabled. The input
  // direction must be non-zero and is normalized internally before evaluation.
  bool use_task_direction{false};
  Eigen::VectorXd task_direction;
};

struct JointLimitMetricsResult
{
  MetricsStatus status{MetricsStatus::kNotEvaluated};
  // Model joint order: midpoint=1, limit=0, out of range <0.
  Eigen::VectorXd normalized_margin;
  double min_normalized_margin{std::numeric_limits<double>::quiet_NaN()};
  std::string message{"joint-limit metrics were not evaluated"};
};

struct ArmKinematicMetrics
{
  MetricsStatus status{MetricsStatus::kNotEvaluated};
  wbmm::core::Header header;
  // Reference point is the origin of this link; expressing frame is header.frame_id.
  std::string link_name;
  ArmMetricsOptions options;
  Eigen::VectorXd singular_values;
  double sigma_min{std::numeric_limits<double>::quiet_NaN()};
  double sigma_max{std::numeric_limits<double>::quiet_NaN()};
  double condition_number{std::numeric_limits<double>::quiet_NaN()};
  // Yoshikawa manipulability = product of singular values = sqrt(det(J J^T)).
  double manipulability{std::numeric_limits<double>::quiet_NaN()};
  // trace((J J^T + regularization I)^-1).
  double inverse_manipulability{std::numeric_limits<double>::quiet_NaN()};
  // sqrt(d^T J J^T d); NaN when no valid task direction was requested.
  double task_direction_manipulability{
    std::numeric_limits<double>::quiet_NaN()};
  int numerical_rank{-1};
  JointLimitMetricsResult joint_limits;
  std::string message{"arm kinematic metrics were not evaluated"};
};

}  // namespace wbmm::metrics
