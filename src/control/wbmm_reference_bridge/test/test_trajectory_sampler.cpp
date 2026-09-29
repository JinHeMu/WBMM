#include "wbmm_reference_bridge/trajectory_sampler.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <string>

namespace
{

using wbmm_planner_ros::msg::WholeBodyTrajectory;

// Two samples, two joints. Deliberately small so the expected interpolated
// values can be written out by hand.
WholeBodyTrajectory makeValid()
{
  WholeBodyTrajectory trajectory;
  trajectory.header.frame_id = "odom";
  trajectory.trajectory_id = "test";
  trajectory.environment_revision = 7U;
  trajectory.collision_model_revision = 9U;

  trajectory.time_from_start = {0.0, 1.0};
  trajectory.joint_names = {"joint_1", "joint_2"};

  trajectory.base_x = {0.0, 1.0};
  trajectory.base_y = {0.0, 2.0};
  trajectory.base_yaw = {0.0, 0.5};
  trajectory.base_linear_velocity = {0.1, 0.3};
  trajectory.base_yaw_rate = {0.0, 0.2};

  // sample 0 -> (0, 0), sample 1 -> (1, 1)
  trajectory.joint_positions = {0.0, 0.0, 1.0, 1.0};
  trajectory.joint_velocities = {0.0, 0.0, 2.0, 2.0};
  trajectory.phase = {0U, 3U};
  return trajectory;
}

std::string validateReason(const WholeBodyTrajectory & trajectory)
{
  std::string reason;
  EXPECT_FALSE(wbmm::reference_bridge::TrajectorySampler::validate(trajectory, &reason));
  return reason;
}

TEST(TrajectorySamplerValidation, AcceptsValidTrajectory)
{
  std::string reason;
  EXPECT_TRUE(
    wbmm::reference_bridge::TrajectorySampler::validate(makeValid(), &reason))
    << reason;
}

TEST(TrajectorySamplerValidation, RejectsEmptyFrame)
{
  auto trajectory = makeValid();
  trajectory.header.frame_id.clear();
  EXPECT_FALSE(validateReason(trajectory).empty());
}

TEST(TrajectorySamplerValidation, RejectsEmptySamples)
{
  auto trajectory = makeValid();
  trajectory.time_from_start.clear();
  EXPECT_FALSE(validateReason(trajectory).empty());
}

TEST(TrajectorySamplerValidation, RejectsEmptyJointNames)
{
  auto trajectory = makeValid();
  trajectory.joint_names.clear();
  EXPECT_FALSE(validateReason(trajectory).empty());
}

TEST(TrajectorySamplerValidation, RejectsNonMonotonicTime)
{
  auto trajectory = makeValid();
  trajectory.time_from_start = {0.0, 0.0};
  EXPECT_FALSE(validateReason(trajectory).empty());

  auto decreasing = makeValid();
  decreasing.time_from_start = {1.0, 0.0};
  EXPECT_FALSE(validateReason(decreasing).empty());
}

TEST(TrajectorySamplerValidation, RejectsPerSampleArrayLengthMismatch)
{
  auto trajectory = makeValid();
  trajectory.base_y.pop_back();
  EXPECT_FALSE(validateReason(trajectory).empty());

  auto phase = makeValid();
  phase.phase.pop_back();
  EXPECT_FALSE(validateReason(phase).empty());
}

TEST(TrajectorySamplerValidation, RejectsFlattenedJointLengthMismatch)
{
  auto trajectory = makeValid();
  trajectory.joint_positions.pop_back();
  EXPECT_FALSE(validateReason(trajectory).empty());

  auto velocities = makeValid();
  velocities.joint_velocities.push_back(0.0);
  EXPECT_FALSE(validateReason(velocities).empty());
}

TEST(TrajectorySamplerValidation, RejectsNonFiniteSamples)
{
  auto trajectory = makeValid();
  trajectory.base_x[1] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validateReason(trajectory).empty());

  auto inf = makeValid();
  inf.time_from_start[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(validateReason(inf).empty());
}

TEST(TrajectorySamplerSampling, ReadsMetadataAndNodesExactly)
{
  const wbmm::reference_bridge::TrajectorySampler sampler(makeValid());

  EXPECT_FALSE(sampler.empty());
  EXPECT_DOUBLE_EQ(sampler.duration(), 1.0);
  EXPECT_EQ(sampler.frameId(), "odom");
  EXPECT_EQ(sampler.trajectoryId(), "test");
  EXPECT_EQ(sampler.environmentRevision(), 7U);
  EXPECT_EQ(sampler.collisionModelRevision(), 9U);
  EXPECT_EQ(sampler.jointCount(), 2U);
  EXPECT_EQ(sampler.sampleCount(), 2U);

  const auto first = sampler.sample(0.0);
  EXPECT_DOUBLE_EQ(first.base_x, 0.0);
  EXPECT_DOUBLE_EQ(first.base_y, 0.0);
  EXPECT_DOUBLE_EQ(first.base_yaw, 0.0);
  EXPECT_DOUBLE_EQ(first.base_linear_velocity, 0.1);
  EXPECT_DOUBLE_EQ(first.base_yaw_rate, 0.0);
  EXPECT_DOUBLE_EQ(first.joint_positions[0], 0.0);
  EXPECT_DOUBLE_EQ(first.joint_velocities[1], 0.0);
  EXPECT_EQ(first.phase, 0U);

  const auto last = sampler.sample(1.0);
  EXPECT_DOUBLE_EQ(last.base_x, 1.0);
  EXPECT_DOUBLE_EQ(last.base_y, 2.0);
  EXPECT_DOUBLE_EQ(last.base_yaw, 0.5);
  EXPECT_DOUBLE_EQ(last.base_linear_velocity, 0.3);
  EXPECT_DOUBLE_EQ(last.base_yaw_rate, 0.2);
  EXPECT_DOUBLE_EQ(last.joint_positions[0], 1.0);
  EXPECT_DOUBLE_EQ(last.joint_velocities[0], 2.0);
  EXPECT_EQ(last.phase, 3U);
}

TEST(TrajectorySamplerSampling, InterpolatesMidpointLinearly)
{
  const wbmm::reference_bridge::TrajectorySampler sampler(makeValid());
  const auto mid = sampler.sample(0.5);

  EXPECT_DOUBLE_EQ(mid.time_from_start, 0.5);
  EXPECT_DOUBLE_EQ(mid.base_x, 0.5);
  EXPECT_DOUBLE_EQ(mid.base_y, 1.0);
  EXPECT_DOUBLE_EQ(mid.base_yaw, 0.25);
  EXPECT_NEAR(mid.base_linear_velocity, 0.2, 1e-12);
  EXPECT_NEAR(mid.base_yaw_rate, 0.1, 1e-12);
  EXPECT_DOUBLE_EQ(mid.joint_positions[0], 0.5);
  EXPECT_DOUBLE_EQ(mid.joint_positions[1], 0.5);
  EXPECT_DOUBLE_EQ(mid.joint_velocities[0], 1.0);
  EXPECT_DOUBLE_EQ(mid.joint_velocities[1], 1.0);

  // Phase is a discrete label and must never be interpolated.
  EXPECT_EQ(mid.phase, 0U);
}

TEST(TrajectorySamplerSampling, ClampsOutsideTheTrajectory)
{
  const wbmm::reference_bridge::TrajectorySampler sampler(makeValid());

  const auto before = sampler.sample(-5.0);
  EXPECT_DOUBLE_EQ(before.time_from_start, 0.0);
  EXPECT_DOUBLE_EQ(before.base_x, 0.0);

  // Past the end the terminal pose is held; no extrapolation.
  const auto after = sampler.sample(10.0);
  EXPECT_DOUBLE_EQ(after.time_from_start, 1.0);
  EXPECT_DOUBLE_EQ(after.base_x, 1.0);
  EXPECT_DOUBLE_EQ(after.base_y, 2.0);
  EXPECT_DOUBLE_EQ(after.base_linear_velocity, 0.0);
  EXPECT_DOUBLE_EQ(after.base_yaw_rate, 0.0);
  EXPECT_DOUBLE_EQ(after.joint_velocities[0], 0.0);
}

TEST(TrajectorySamplerSampling, DefaultSamplerIsEmptyAndSafe)
{
  const wbmm::reference_bridge::TrajectorySampler sampler;
  EXPECT_TRUE(sampler.empty());
  EXPECT_DOUBLE_EQ(sampler.duration(), 0.0);

  const auto sample = sampler.sample(1.0);
  EXPECT_DOUBLE_EQ(sample.base_x, 0.0);
  EXPECT_TRUE(sample.joint_positions.empty());
}

TEST(TrajectorySamplerValidation, RejectsAmbiguousJointNamesAndNonzeroOrigin)
{
  auto trajectory = makeValid();
  trajectory.joint_names[1] = trajectory.joint_names[0];
  EXPECT_FALSE(validateReason(trajectory).empty());
  trajectory = makeValid();
  trajectory.joint_names[0] = "";
  EXPECT_FALSE(validateReason(trajectory).empty());
  trajectory = makeValid();
  trajectory.time_from_start = {-1.0, 1.0};
  EXPECT_FALSE(validateReason(trajectory).empty());
}

}  // namespace
