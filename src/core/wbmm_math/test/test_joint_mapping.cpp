#include "wbmm_math/conversions.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <limits>
#include <string>
#include <vector>

namespace wbmm::math
{
namespace
{

const std::vector<std::string> kExpectedOrder{
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

core::Timestamp validStamp()
{
  return {10, core::ClockType::kSteady};
}

core::WholeBodyState reorderedState()
{
  core::WholeBodyState state;
  state.header = {"map", validStamp()};
  state.base_model = core::BaseModel::kDifferentialDrive;
  state.base.x_m = 1.0;
  state.base.y_m = 2.0;
  state.base.yaw_rad = 3.0;
  state.joints.names = {
    "joint_3", "joint_1", "joint_6", "joint_2", "joint_5", "joint_4"};
  state.joints.positions_rad = {30.0, 10.0, 60.0, 20.0, 50.0, 40.0};
  return state;
}

core::WholeBodyInput reorderedInput()
{
  core::WholeBodyInput input;
  input.stamp = validStamp();
  input.base_model = core::BaseModel::kDifferentialDrive;
  input.base_command = {0.5, -0.2};
  input.joint_names = {
    "joint_3", "joint_1", "joint_6", "joint_2", "joint_5", "joint_4"};
  input.joint_velocities_radps = {3.0, 1.0, 6.0, 2.0, 5.0, 4.0};
  return input;
}

TEST(JointMappingTest, PositionsAreMappedByName)
{
  const auto result = jointPositions(reorderedState().joints, kExpectedOrder);
  ASSERT_TRUE(result.ok());
  const Eigen::VectorXd expected =
    (Eigen::VectorXd(6) << 10.0, 20.0, 30.0, 40.0, 50.0, 60.0).finished();
  EXPECT_TRUE(result.value().isApprox(expected));
}

TEST(JointMappingTest, DifferentialDriveStateUsesExplicitNineDimensionalContract)
{
  const auto result = differentialDriveStateVector(reorderedState(), kExpectedOrder);
  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.value().size(), 9);
  const Eigen::VectorXd expected =
    (Eigen::VectorXd(9) << 1.0, 2.0, 3.0, 10.0, 20.0, 30.0, 40.0, 50.0, 60.0)
    .finished();
  EXPECT_TRUE(result.value().isApprox(expected));
}

TEST(JointMappingTest, DifferentialDriveInputUsesExplicitEightDimensionalContract)
{
  const auto result = differentialDriveInputVector(reorderedInput(), kExpectedOrder);
  ASSERT_TRUE(result.ok());
  ASSERT_EQ(result.value().size(), 8);
  const Eigen::VectorXd expected =
    (Eigen::VectorXd(8) << 0.5, -0.2, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished();
  EXPECT_TRUE(result.value().isApprox(expected));
}

TEST(JointMappingTest, EigenInputRoundTripPreservesExplicitOrder)
{
  const Eigen::VectorXd vector =
    (Eigen::VectorXd(8) << 0.5, -0.2, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0).finished();
  const auto domain_input =
    differentialDriveInputFromVector(vector, validStamp(), kExpectedOrder);
  ASSERT_TRUE(domain_input.ok());
  EXPECT_EQ(domain_input.value().joint_names, kExpectedOrder);

  const auto round_trip = differentialDriveInputVector(domain_input.value(), kExpectedOrder);
  ASSERT_TRUE(round_trip.ok());
  EXPECT_TRUE(round_trip.value().isApprox(vector));
}

TEST(JointMappingTest, MissingJointFailsClosed)
{
  auto state = reorderedState();
  state.joints.names[0] = "unexpected_joint";
  const auto result = differentialDriveStateVector(state, kExpectedOrder);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), core::ErrorCode::kJointNameMismatch);
}

TEST(JointMappingTest, NonFiniteInputFailsClosed)
{
  auto input = reorderedInput();
  input.joint_velocities_radps[0] = std::numeric_limits<double>::quiet_NaN();
  const auto result = differentialDriveInputVector(input, kExpectedOrder);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), core::ErrorCode::kNonFiniteValue);
}

TEST(JointMappingTest, WrongEigenDimensionFailsClosed)
{
  const Eigen::VectorXd vector = Eigen::VectorXd::Zero(7);
  const auto result = differentialDriveInputFromVector(vector, validStamp(), kExpectedOrder);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), core::ErrorCode::kInvalidDimension);
}

}  // namespace
}  // namespace wbmm::math
