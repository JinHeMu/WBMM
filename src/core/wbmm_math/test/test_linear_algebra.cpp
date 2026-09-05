#include "wbmm_math/linear_algebra.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>

namespace wbmm::math
{
namespace
{

TEST(LinearAlgebraTest, SkewMatrixImplementsCrossProduct)
{
  const Eigen::Vector3d first(1.0, 2.0, 3.0);
  const Eigen::Vector3d second(-2.0, 4.0, 0.5);
  const auto skew = skewSymmetric(first);
  ASSERT_TRUE(skew.ok());
  EXPECT_TRUE((skew.value() * second).isApprox(first.cross(second), 1.0e-12));
  EXPECT_TRUE(skew.value().transpose().isApprox(-skew.value(), 1.0e-12));
}

TEST(LinearAlgebraTest, MoorePenroseInverseReconstructsRectangularMatrix)
{
  Eigen::MatrixXd matrix(2, 3);
  matrix << 1.0, 2.0, 0.0,
    0.0, 1.0, 1.0;
  const auto pseudo_inverse = dampedPseudoInverse(matrix, 0.0);
  ASSERT_TRUE(pseudo_inverse.ok());
  EXPECT_TRUE(
    (matrix * pseudo_inverse.value() * matrix).isApprox(matrix, 1.0e-10));
}

TEST(LinearAlgebraTest, DampingKeepsSingularProblemFinite)
{
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(2, 2);
  matrix(0, 0) = 1.0;
  const auto pseudo_inverse = dampedPseudoInverse(matrix, 0.1);
  ASSERT_TRUE(pseudo_inverse.ok());
  EXPECT_TRUE(pseudo_inverse.value().allFinite());
  EXPECT_NEAR(pseudo_inverse.value()(0, 0), 1.0 / 1.01, 1.0e-12);
  EXPECT_DOUBLE_EQ(pseudo_inverse.value()(1, 1), 0.0);
}

TEST(LinearAlgebraTest, SolvesDampedLeastSquaresWithDeclaredConvention)
{
  const Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(2, 2);
  const Eigen::VectorXd target = (Eigen::VectorXd(2) << 2.0, -4.0).finished();
  const auto solution = solveDampedLeastSquares(matrix, target, 1.0);
  ASSERT_TRUE(solution.ok());
  EXPECT_TRUE(solution.value().isApprox(target * 0.5, 1.0e-12));
}

TEST(LinearAlgebraTest, RejectsInvalidDampingAndTargetDimension)
{
  const Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(2, 2);
  const auto invalid_damping = dampedPseudoInverse(matrix, -0.1);
  EXPECT_EQ(invalid_damping.status().code(), core::ErrorCode::kInvalidArgument);

  const auto invalid_target =
    solveDampedLeastSquares(matrix, Eigen::VectorXd::Zero(3), 0.1);
  EXPECT_EQ(invalid_target.status().code(), core::ErrorCode::kInvalidDimension);
}

TEST(LinearAlgebraTest, RejectsNonFiniteMatrix)
{
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(2, 2);
  matrix(0, 0) = std::numeric_limits<double>::quiet_NaN();
  const auto result = dampedPseudoInverse(matrix, 0.1);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), core::ErrorCode::kNonFiniteValue);
}

}  // namespace
}  // namespace wbmm::math
