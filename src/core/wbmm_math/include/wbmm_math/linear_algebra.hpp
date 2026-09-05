#pragma once

#include "wbmm_core/status.hpp"

#include <Eigen/Core>

namespace wbmm::math
{

core::Result<Eigen::Matrix3d> skewSymmetric(const Eigen::Vector3d & vector);

core::Result<Eigen::MatrixXd> dampedPseudoInverse(
  const Eigen::MatrixXd & matrix,
  double damping,
  double singular_value_tolerance = 1.0e-12);

core::Result<Eigen::VectorXd> solveDampedLeastSquares(
  const Eigen::MatrixXd & matrix,
  const Eigen::VectorXd & target,
  double damping,
  double singular_value_tolerance = 1.0e-12);

}  // namespace wbmm::math
