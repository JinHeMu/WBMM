#include "wbmm_core/validation.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>
#include <vector>

namespace wbmm::core
{
namespace
{

const std::vector<std::string> kSixJointOrder{
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

Timestamp validStamp()
{
  return Timestamp{1'000'000, ClockType::kSimulation};
}

WholeBodyState validState()
{
  WholeBodyState state;
  state.header = Header{"map", validStamp()};
  state.base_model = BaseModel::kDifferentialDrive;
  state.base = BaseState{1.0, 2.0, 0.3, 0.1, 0.0, 0.2};
  state.joints.names = kSixJointOrder;
  state.joints.positions_rad = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  state.joints.velocities_radps = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return state;
}

WholeBodyInput validInput()
{
  WholeBodyInput input;
  input.stamp = validStamp();
  input.base_model = BaseModel::kDifferentialDrive;
  input.base_command = {0.1, 0.2};
  input.joint_names = kSixJointOrder;
  input.joint_velocities_radps = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5};
  return input;
}

TEST(ValidationTest, AcceptsValidGenericWholeBodyStateAndInput)
{
  EXPECT_TRUE(validateWholeBodyState(validState()).ok());
  EXPECT_TRUE(validateWholeBodyInput(validInput()).ok());
}

TEST(ValidationTest, CurrentSixJointDifferentialDriveDimensionsRemainNineAndEight)
{
  EXPECT_EQ(DifferentialDriveContract::stateDimension(kSixJointOrder.size()), 9U);
  EXPECT_EQ(DifferentialDriveContract::inputDimension(kSixJointOrder.size()), 8U);
  EXPECT_TRUE(DifferentialDriveContract::validateState(validState(), kSixJointOrder).ok());
  EXPECT_TRUE(DifferentialDriveContract::validateInput(validInput(), kSixJointOrder).ok());
}

TEST(ValidationTest, RejectsEmptyFrame)
{
  auto state = validState();
  state.header.frame_id.clear();
  const auto status = validateWholeBodyState(state);
  EXPECT_EQ(status.code(), ErrorCode::kEmptyFrame);
}

TEST(ValidationTest, RejectsNonFiniteValues)
{
  auto state = validState();
  state.joints.positions_rad[2] = std::numeric_limits<double>::quiet_NaN();
  const auto status = validateWholeBodyState(state);
  EXPECT_EQ(status.code(), ErrorCode::kNonFiniteValue);
}

TEST(ValidationTest, RejectsWrongDimension)
{
  auto input = validInput();
  input.joint_velocities_radps.pop_back();
  const auto status = validateWholeBodyInput(input);
  EXPECT_EQ(status.code(), ErrorCode::kInvalidDimension);
}

TEST(ValidationTest, RejectsDuplicateJointNames)
{
  auto state = validState();
  state.joints.names[5] = state.joints.names[4];
  const auto status = validateWholeBodyState(state);
  EXPECT_EQ(status.code(), ErrorCode::kDuplicateJointName);
}

TEST(ValidationTest, RejectsImplicitJointReordering)
{
  auto actual_order = kSixJointOrder;
  std::swap(actual_order[1], actual_order[2]);
  const auto status = validateJointOrder(actual_order, kSixJointOrder);
  EXPECT_EQ(status.code(), ErrorCode::kJointNameMismatch);
}

TEST(ValidationTest, RejectsDifferentialDriveLateralVelocity)
{
  auto state = validState();
  state.base.lateral_velocity_mps = 0.01;
  const auto status = DifferentialDriveContract::validateState(state, kSixJointOrder);
  EXPECT_EQ(status.code(), ErrorCode::kSafetyLimitExceeded);
}

TEST(ValidationTest, RejectsWrongBaseCommandDimension)
{
  auto input = validInput();
  input.base_command.push_back(0.0);
  const auto status = DifferentialDriveContract::validateInput(input, kSixJointOrder);
  EXPECT_EQ(status.code(), ErrorCode::kInvalidDimension);
}

TEST(ValidationTest, RejectsUnnormalisedQuaternion)
{
  Pose pose;
  pose.header = Header{"tool", validStamp()};
  pose.orientation = Quaternion{2.0, 0.0, 0.0, 0.0};
  const auto status = validatePose(pose);
  EXPECT_EQ(status.code(), ErrorCode::kInvalidQuaternion);
}

TEST(ValidationTest, RejectsNonMonotonicTrajectoryTime)
{
  WholeBodyTrajectory trajectory;
  trajectory.trajectory_id = "plan-1";
  trajectory.environment_revision = 7;
  trajectory.collision_model_revision = 3;
  trajectory.points = {
    WholeBodyTrajectoryPoint{0.0, validState(), validInput()},
    WholeBodyTrajectoryPoint{0.0, validState(), validInput()}};
  const auto status = validateWholeBodyTrajectory(trajectory);
  EXPECT_EQ(status.code(), ErrorCode::kInvalidTimestamp);
}

TEST(ValidationTest, RejectsEnvironmentWithoutVersionedSemantics)
{
  EnvironmentSnapshot snapshot;
  snapshot.header = Header{"map", validStamp()};
  snapshot.revision = 4;
  snapshot.collision_model_revision = 2;
  snapshot.source = "test_map";
  snapshot.validity_horizon_s = 0.5;
  const auto status = validateEnvironmentSnapshot(snapshot);
  EXPECT_EQ(status.code(), ErrorCode::kInvalidArgument);
}

}  // namespace
}  // namespace wbmm::core
