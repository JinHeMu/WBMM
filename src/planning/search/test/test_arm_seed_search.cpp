#include "wbmm_search/arm_seed_search.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace
{

constexpr std::size_t kJoints = 6;

wbmm::core::Header makeHeader()
{
  wbmm::core::Header header;
  header.frame_id = "map";
  header.stamp = 0.0;
  return header;
}

// A straight 4 m base path along +x, sampled coarsely on purpose so the
// resampling step has something to do.
std::vector<wbmm::core::BaseState> makeBasePath()
{
  std::vector<wbmm::core::BaseState> path;
  for (int i = 0; i <= 4; ++i) {
    wbmm::core::BaseState state;
    state.x = static_cast<double>(i);
    state.y = 0.0;
    state.yaw = 0.0;
    path.push_back(state);
  }
  return path;
}

wbmm::core::JointState makeJoints(double value = 0.0)
{
  wbmm::core::JointState joints;
  for (std::size_t j = 0U; j < kJoints; ++j) {
    joints.names.push_back("joint_" + std::to_string(j + 1U));
    joints.positions.push_back(value);
    joints.velocities.push_back(0.0);
    joints.efforts.push_back(0.0);
  }
  return joints;
}

wbmm::core::RobotLimits makeLimits()
{
  wbmm::core::RobotLimits limits;
  limits.joint_min.assign(kJoints, -2.0);
  limits.joint_max.assign(kJoints, 2.0);
  limits.max_joint_speed.assign(kJoints, 2.0);
  limits.max_base_speed = 0.5;
  limits.max_base_yaw_rate = 1.0;
  return limits;
}

wbmm::search::WholeBodyCollisionChecker alwaysFree()
{
  return [](const wbmm::core::Header &, const wbmm::core::WholeBodyState &)
         {return true;};
}

TEST(ArmSeedSearch, ProducesOneConfigurationPerResampledWaypoint)
{
  const auto result = wbmm::search::searchArmSeed(
    makeHeader(), makeBasePath(), makeJoints(), makeLimits(), alwaysFree());

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.status, wbmm::search::ArmSeedStatus::kSuccess);
  EXPECT_EQ(result.arm_seed.size(), result.base_path.size());
  EXPECT_GT(result.waypoints, 5U);  // resampled finer than the 1 m input
  EXPECT_EQ(result.base_path.front().x, 0.0);
  EXPECT_NEAR(result.base_path.back().x, 4.0, 1e-9);
  for (const auto & joints : result.arm_seed) {
    EXPECT_EQ(joints.names.size(), kJoints);
    EXPECT_EQ(joints.positions.size(), kJoints);
  }
}

// When nothing collides the arm must not be re-shuffled: the carry-over
// candidate is tried first precisely so the seed stays continuous.
TEST(ArmSeedSearch, KeepsTheInitialConfigurationWhenItIsFree)
{
  const auto initial = makeJoints(0.3);
  const auto result = wbmm::search::searchArmSeed(
    makeHeader(), makeBasePath(), initial, makeLimits(), alwaysFree());

  ASSERT_TRUE(result.success) << result.message;
  for (const auto & joints : result.arm_seed) {
    for (std::size_t j = 0U; j < kJoints; ++j) {
      EXPECT_DOUBLE_EQ(joints.positions[j], 0.3);
    }
  }
  // Only the carry-over candidate should have been needed.
  EXPECT_EQ(result.checked_candidates, result.waypoints);
}

// The base path enters a region that the current arm posture cannot serve, so
// the arm must reconfigure. The constraint is written in terms of the base
// position, which is the realistic shape of the problem.
namespace
{

wbmm::search::WholeBodyCollisionChecker needsReconfiguration(double x_limit,
                                                             double q0_min)
{
  return [x_limit, q0_min](
           const wbmm::core::Header &, const wbmm::core::WholeBodyState & state)
  {
    if (state.base.x > x_limit) {
      return std::abs(state.joints.positions[0]) > q0_min;
    }
    return true;
  };
}

}  // namespace

TEST(ArmSeedSearch, ReconfiguresBeforeTheBaseEntersARestrictedRegion)
{
  const auto result = wbmm::search::searchArmSeed(
    makeHeader(), makeBasePath(), makeJoints(0.0), makeLimits(),
    needsReconfiguration(2.0, 0.7));

  ASSERT_TRUE(result.success) << result.message;
  for (std::size_t i = 0U; i < result.base_path.size(); ++i) {
    if (result.base_path[i].x > 2.0) {
      EXPECT_GT(std::abs(result.arm_seed[i].positions[0]), 0.7)
        << "waypoint " << i << " at x=" << result.base_path[i].x;
    }
  }
}

// A candidate can be free while the motion through it is not. If only endpoints
// were checked, the search would succeed by jumping straight to a free
// endpoint; the forbidden band in between must make it fail instead.
TEST(ArmSeedSearch, ChecksTheSweptTransitionNotJustEndpoints)
{
  const auto endpoint_only = wbmm::search::searchArmSeed(
    makeHeader(), makeBasePath(), makeJoints(0.0), makeLimits(),
    needsReconfiguration(2.0, 0.7));
  ASSERT_TRUE(endpoint_only.success) << endpoint_only.message;

  const auto with_band = [](const wbmm::core::Header &,
                            const wbmm::core::WholeBodyState & state)
  {
    const double q0 = state.joints.positions[0];
    if (std::abs(q0) > 0.3 && std::abs(q0) < 0.9) {
      return false;  // a band that any escape must cross
    }
    if (state.base.x > 2.0) {
      return std::abs(q0) > 0.7;
    }
    return true;
  };

  const auto result = wbmm::search::searchArmSeed(
    makeHeader(), makeBasePath(), makeJoints(0.0), makeLimits(), with_band);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, wbmm::search::ArmSeedStatus::kNoFeasibleSeed);
}

TEST(ArmSeedSearch, IsDeterministicAcrossRuns)
{
  const auto checker = needsReconfiguration(2.0, 0.7);

  const auto first = wbmm::search::searchArmSeed(
    makeHeader(), makeBasePath(), makeJoints(0.0), makeLimits(), checker);
  const auto second = wbmm::search::searchArmSeed(
    makeHeader(), makeBasePath(), makeJoints(0.0), makeLimits(), checker);

  ASSERT_TRUE(first.success) << first.message;
  ASSERT_TRUE(second.success) << second.message;
  ASSERT_EQ(first.arm_seed.size(), second.arm_seed.size());
  for (std::size_t i = 0U; i < first.arm_seed.size(); ++i) {
    EXPECT_EQ(first.arm_seed[i].positions, second.arm_seed[i].positions)
      << "waypoint " << i;
  }
}

// Documented boundary: if the robot starts inside a forbidden region there is
// no swept transition out of it, because every candidate's transition begins at
// the colliding configuration. The search must say so rather than return a seed
// that starts in collision.
TEST(ArmSeedSearch, FailsCleanlyWhenTheStartItselfIsInCollision)
{
  const auto checker = [](const wbmm::core::Header &,
                          const wbmm::core::WholeBodyState & state)
  {
    return std::abs(state.joints.positions[0]) > 0.15;
  };

  const auto result = wbmm::search::searchArmSeed(
    makeHeader(), makeBasePath(), makeJoints(0.0), makeLimits(), checker);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, wbmm::search::ArmSeedStatus::kNoFeasibleSeed);
  EXPECT_TRUE(result.arm_seed.empty());
}

TEST(ArmSeedSearch, ReportsNoFeasibleSeedInsteadOfReturningGarbage)
{
  const auto checker = [](const wbmm::core::Header &,
                          const wbmm::core::WholeBodyState &) {return false;};

  const auto result = wbmm::search::searchArmSeed(
    makeHeader(), makeBasePath(), makeJoints(), makeLimits(), checker);

  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, wbmm::search::ArmSeedStatus::kNoFeasibleSeed);
  EXPECT_TRUE(result.arm_seed.empty());
  EXPECT_NE(result.message.find("waypoint"), std::string::npos);
}

TEST(ArmSeedSearch, RejectsInvalidInputs)
{
  const auto header = makeHeader();
  const auto path = makeBasePath();
  const auto joints = makeJoints();
  const auto limits = makeLimits();
  const auto checker = alwaysFree();

  // Missing collision checker.
  const auto no_checker = wbmm::search::searchArmSeed(
    header, path, joints, limits, {});
  EXPECT_EQ(
    no_checker.status, wbmm::search::ArmSeedStatus::kMissingCollisionChecker);

  // Degenerate base path.
  const auto short_path = wbmm::search::searchArmSeed(
    header, {joints.positions.empty() ? std::vector<wbmm::core::BaseState>{}
                                      : std::vector<wbmm::core::BaseState>{wbmm::core::BaseState{}}},
    joints, limits, checker);
  EXPECT_EQ(short_path.status, wbmm::search::ArmSeedStatus::kInvalidInput);

  // Limit vector size mismatch.
  auto bad_limits = limits;
  bad_limits.joint_max.pop_back();
  const auto mismatched = wbmm::search::searchArmSeed(
    header, path, joints, bad_limits, checker);
  EXPECT_EQ(mismatched.status, wbmm::search::ArmSeedStatus::kInvalidInput);

  // Initial configuration outside the limits must be rejected, not clamped.
  const auto outside = wbmm::search::searchArmSeed(
    header, path, makeJoints(5.0), limits, checker);
  EXPECT_EQ(outside.status, wbmm::search::ArmSeedStatus::kInvalidInput);

  // Non-positive spacing.
  wbmm::search::ArmSeedConfig bad_config;
  bad_config.waypoint_spacing = 0.0;
  const auto bad_spacing = wbmm::search::searchArmSeed(
    header, path, joints, limits, checker, bad_config);
  EXPECT_EQ(bad_spacing.status, wbmm::search::ArmSeedStatus::kInvalidInput);
}

TEST(ArmSeedSearch, FillsTheWholeBodySearchResultContract)
{
  const auto result = wbmm::search::searchArmSeed(
    makeHeader(), makeBasePath(), makeJoints(), makeLimits(), alwaysFree());
  ASSERT_TRUE(result.success) << result.message;

  wbmm::core::SearchResult base;
  base.solve_time = 0.25;  // pretend a base search ran first
  const auto filled = result.toSearchResult(base);

  EXPECT_TRUE(filled.success);
  EXPECT_EQ(filled.arm_seed.size(), result.arm_seed.size());
  EXPECT_EQ(filled.base_path.size(), result.base_path.size());
  EXPECT_EQ(filled.phases.size(), result.base_path.size());
  EXPECT_NEAR(filled.path_length, 4.0, 0.2);
  EXPECT_EQ(filled.solve_time, result.solve_time);
}

TEST(ArmSeedSearch, LeavesTheSearchResultUntouchedOnFailure)
{
  const auto checker = [](const wbmm::core::Header &,
                          const wbmm::core::WholeBodyState &) {return false;};
  const auto result = wbmm::search::searchArmSeed(
    makeHeader(), makeBasePath(), makeJoints(), makeLimits(), checker);
  ASSERT_FALSE(result.success);

  wbmm::core::SearchResult base;
  base.success = false;
  const auto filled = result.toSearchResult(base);
  EXPECT_FALSE(filled.success);
  EXPECT_TRUE(filled.arm_seed.empty());
}

}  // namespace
