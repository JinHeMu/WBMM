#include "wbmm_math/linear_algebra.hpp"

#include <Eigen/SVD>

#include <cmath>
#include <utility>

namespace wbmm::math
{

core::Result<Eigen::Matrix3d> skewSymmetric(const Eigen::Vector3d & vector)
{
  if (!vector.allFinite()) {
    return core::Result<Eigen::Matrix3d>::Failure(
      {core::ErrorCode::kNonFiniteValue, "skew input contains a non-finite value"});
  }
  Eigen::Matrix3d result;
  result << 0.0, -vector.z(), vector.y(),
    vector.z(), 0.0, -vector.x(),
    -vector.y(), vector.x(), 0.0;
  return core::Result<Eigen::Matrix3d>::Success(std::move(result));
}

core::Result<Eigen::MatrixXd> dampedPseudoInverse(
  const Eigen::MatrixXd & matrix,
  const double damping,
  const double singular_value_tolerance)
{
  if (matrix.rows() <= 0 || matrix.cols() <= 0) {
    return core::Result<Eigen::MatrixXd>::Failure(
      {core::ErrorCode::kInvalidDimension, "matrix dimensions must be positive"});
  }
  if (!matrix.allFinite()) {
    return core::Result<Eigen::MatrixXd>::Failure(
      {core::ErrorCode::kNonFiniteValue, "matrix contains a non-finite value"});
  }
  if (!std::isfinite(damping) || damping < 0.0) {
    return core::Result<Eigen::MatrixXd>::Failure(
      {core::ErrorCode::kInvalidArgument, "damping must be finite and non-negative"});
  }
  if (!std::isfinite(singular_value_tolerance) || singular_value_tolerance < 0.0) {
    return core::Result<Eigen::MatrixXd>::Failure(
      {core::ErrorCode::kInvalidArgument,
        "singular value tolerance must be finite and non-negative"});
  }

  const Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  Eigen::VectorXd filtered = svd.singularValues();
  const double damping_squared = damping * damping;
  for (Eigen::Index index = 0; index < filtered.size(); ++index) {
    const double singular_value = filtered[index];
    if (singular_value <= singular_value_tolerance) {
      filtered[index] = 0.0;
    } else if (damping == 0.0) {
      filtered[index] = 1.0 / singular_value;
    } else {
      filtered[index] = singular_value /
        (singular_value * singular_value + damping_squared);
    }
  }

  Eigen::MatrixXd result =
    svd.matrixV() * filtered.asDiagonal() * svd.matrixU().transpose();
  return core::Result<Eigen::MatrixXd>::Success(std::move(result));
}

core::Result<Eigen::VectorXd> solveDampedLeastSquares(
  const Eigen::MatrixXd & matrix,
  const Eigen::VectorXd & target,
  const double damping,
  const double singular_value_tolerance)
{
  if (target.size() != matrix.rows()) {
    return core::Result<Eigen::VectorXd>::Failure(
      {core::ErrorCode::kInvalidDimension,
        "least-squares target dimension must equal matrix rows"});
  }
  if (!target.allFinite()) {
    return core::Result<Eigen::VectorXd>::Failure(
      {core::ErrorCode::kNonFiniteValue,
        "least-squares target contains a non-finite value"});
  }
  auto pseudo_inverse = dampedPseudoInverse(matrix, damping, singular_value_tolerance);
  if (!pseudo_inverse.ok()) {
    return core::Result<Eigen::VectorXd>::Failure(pseudo_inverse.status());
  }
  Eigen::VectorXd result = pseudo_inverse.value() * target;
  return core::Result<Eigen::VectorXd>::Success(std::move(result));
}

}  // namespace wbmm::math
