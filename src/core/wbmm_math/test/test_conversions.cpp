#include "wbmm_math/conversions.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <limits>

namespace wbmm::math
{
namespace
{

core::Header validHeader()
{
  return {"map", core::Timestamp{100, core::ClockType::kSimulation}};
}

TEST(ConversionsTest, PoseRoundTripPreservesTransformAndHeader)
{
  core::Pose pose;
  pose.header = validHeader();
  pose.position = {1.0, -2.0, 0.5};
  const Eigen::Quaterniond expected_quaternion(
    Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitZ()));
  pose.orientation = fromEigen(expected_quaternion);

  const auto transform = toEigenTransform(pose);
  ASSERT_TRUE(transform.ok());
  EXPECT_TRUE(transform.value().translation().isApprox(Eigen::Vector3d(1.0, -2.0, 0.5)));

  const auto round_trip = fromEigenTransform(transform.value(), validHeader());
  ASSERT_TRUE(round_trip.ok());
  EXPECT_EQ(round_trip.value().header.frame_id, "map");
  const auto round_trip_transform = toEigenTransform(round_trip.value());
  ASSERT_TRUE(round_trip_transform.ok());
  EXPECT_TRUE(round_trip_transform.value().matrix().isApprox(transform.value().matrix(), 1.0e-12));
}

TEST(ConversionsTest, RejectsTransformOutsideSO3)
{
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear()(0, 0) = 2.0;
  const auto result = fromEigenTransform(transform, validHeader());
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), core::ErrorCode::kInvalidQuaternion);
}

TEST(ConversionsTest, WrenchOrderIsForceThenTorque)
{
  core::Wrench wrench;
  wrench.header = validHeader();
  wrench.force = {1.0, 2.0, 3.0};
  wrench.torque = {4.0, 5.0, 6.0};

  const auto eigen_wrench = toEigenWrench(wrench);
  ASSERT_TRUE(eigen_wrench.ok());
  const Vector6d expected = (Vector6d() << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished();
  EXPECT_TRUE(eigen_wrench.value().isApprox(expected));

  const auto round_trip = fromEigenWrench(eigen_wrench.value(), validHeader());
  ASSERT_TRUE(round_trip.ok());
  EXPECT_DOUBLE_EQ(round_trip.value().torque.z, 6.0);
}

TEST(ConversionsTest, MatrixRoundTripPreservesRowMajorMeaning)
{
  core::Matrix matrix;
  matrix.rows = 2;
  matrix.cols = 3;
  matrix.row_major_data = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0};

  const auto eigen_matrix = toEigenMatrix(matrix);
  ASSERT_TRUE(eigen_matrix.ok());
  EXPECT_DOUBLE_EQ(eigen_matrix.value()(0, 2), 3.0);
  EXPECT_DOUBLE_EQ(eigen_matrix.value()(1, 0), 4.0);

  const auto round_trip = fromEigenMatrix(eigen_matrix.value());
  ASSERT_TRUE(round_trip.ok());
  EXPECT_EQ(round_trip.value().row_major_data, matrix.row_major_data);
}

TEST(ConversionsTest, MatrixConversionRejectsNonFiniteEigenData)
{
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(2, 2);
  matrix(1, 1) = std::numeric_limits<double>::infinity();
  const auto result = fromEigenMatrix(matrix);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), core::ErrorCode::kNonFiniteValue);
}

}  // namespace
}  // namespace wbmm::math
