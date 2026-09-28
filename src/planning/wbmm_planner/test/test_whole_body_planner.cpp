#include "wbmm_planner/whole_body_planner.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <string>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{

constexpr std::size_t kJoints = 6;

wbmm::core::Header makeHeader()
{
  wbmm::core::Header header;
  header.frame_id = "map";
  return header;
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
  limits.max_joint_speed.assign(kJoints, 1.57);
  limits.max_base_speed = 0.5;
  limits.max_base_yaw_rate = 1.0;
  return limits;
}

wbmm::planning::PlanRequest makeRequest(double goal_x = 3.0)
{
  wbmm::planning::PlanRequest request;
  request.header = makeHeader();
  request.start.x = 0.0;
  request.start.y = 0.0;
  request.start.yaw = 0.0;
  request.start_joints = makeJoints();
  request.goal.x = goal_x;
  request.goal.y = 0.0;
  request.goal.yaw = 0.0;
  request.limits = makeLimits();
  request.environment_revision = 7U;
  request.collision_model_revision = 9U;
  return request;
}

wbmm::search::BaseCollisionChecker freeBase()
{
  return [](const wbmm::core::Header &, const wbmm::core::BaseState &)
         {return true;};
}

wbmm::search::WholeBodyCollisionChecker freeWholeBody()
{
  return [](const wbmm::core::Header &, const wbmm::core::WholeBodyState &)
         {return true;};
}

// A planner profile that produces a slow, trackable reference so the tests
// exercise the success path rather than the envelope rejection.
wbmm::planning::PlannerConfig makeConfig()
{
  wbmm::planning::PlannerConfig config;
  config.cruise_speed = 0.30;
  config.min_segment_duration = 0.6;
  config.base_search.position_resolution = 0.25;
  config.base_search.position_tolerance = 0.25;
  config.base_search.yaw_tolerance = 0.35;
  config.base_search.max_search_time = 5.0;
  config.arm_seed.waypoint_spacing = 0.5;
  return config;
}

TEST(WholeBodyPlanner, PlansAPublishableTrajectory)
{
  wbmm::planning::WholeBodyPlanner planner(makeConfig());
  const auto result = planner.plan(makeRequest(), freeBase(), freeWholeBody());

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_FALSE(result.trajectory.points.empty());
  EXPECT_GE(result.trajectory.points.size(), 2U);

  // The published trajectory must carry the requested revisions and frame, so a
  // consumer can tell it was planned against the map it holds.
  EXPECT_EQ(result.trajectory.environment_revision, 7U);
  EXPECT_EQ(result.trajectory.collision_model_revision, 9U);
  for (const auto & point : result.trajectory.points) {
    EXPECT_EQ(point.state.header.frame_id, "map");
    EXPECT_EQ(point.state.joints.names.size(), kJoints);
    ASSERT_TRUE(point.feedforward_input.has_value());
    EXPECT_EQ(point.feedforward_input->base_command.size(), 2U);
  }

  // Rest to rest at both ends. The endpoint lands within the base search's own
  // goal tolerance (0.25 m here), not exactly on the requested goal.
  EXPECT_NEAR(result.trajectory.points.front().state.base.x, 0.0, 1e-6);
  EXPECT_NEAR(result.trajectory.points.front().state.base.linear_velocity, 0.0, 1e-9);
  EXPECT_NEAR(result.trajectory.points.back().state.base.linear_velocity, 0.0, 1e-9);
  EXPECT_GT(result.trajectory.points.back().state.base.x, 3.0 - 0.25);

  // The search payload must be filled too.
  EXPECT_TRUE(result.search.success);
  EXPECT_EQ(result.search.arm_seed.size(), result.search.base_path.size());
  EXPECT_GT(result.search.path_length, 2.5);
  EXPECT_EQ(result.gear, 1);
}

TEST(WholeBodyPlanner, OutputIsTrackableByConstruction)
{
  wbmm::planning::WholeBodyPlanner planner(makeConfig());
  const auto result = planner.plan(makeRequest(), freeBase(), freeWholeBody());
  ASSERT_TRUE(result.success) << result.message;

  // The planner rejects untrackable references instead of publishing them, so
  // a successful plan must respect the envelope it was configured with.
  const auto & points = result.trajectory.points;
  for (std::size_t i = 0U; i < points.size(); ++i) {
    EXPECT_LE(std::abs(points[i].state.base.linear_velocity), 0.5 + 1e-9);
    EXPECT_LE(std::abs(points[i].state.base.yaw_rate), 1.0 + 1e-9);
  }

  // Times strictly increasing, which the OCS2 reference requires.
  for (std::size_t i = 1U; i < points.size(); ++i) {
    EXPECT_GT(points[i].time_from_start, points[i - 1U].time_from_start);
  }

  // Yaw must be continuous: the whole point of shaping it here rather than
  // re-deriving it downstream.
  for (std::size_t i = 1U; i < points.size(); ++i) {
    EXPECT_LT(
      std::abs(points[i].state.base.yaw - points[i - 1U].state.base.yaw), 0.5)
      << "yaw stepped at sample " << i;
  }
}

// Time scaling can always make a reference trackable by stretching it, so a
// tiny envelope on its own is not an error. The duration budget is what turns
// an inconsistent configuration into a reported failure instead of a
// several-minute trajectory.
TEST(WholeBodyPlanner, AdaptsToATightEnvelopeWithinTheDurationBudget)
{
  auto config = makeConfig();
  config.builder.max_linear_velocity = 0.15;  // slower than the profile asks
  wbmm::planning::WholeBodyPlanner planner(config);

  const auto result = planner.plan(makeRequest(), freeBase(), freeWholeBody());

  ASSERT_TRUE(result.success) << result.message;
  for (const auto & point : result.trajectory.points) {
    EXPECT_LE(std::abs(point.state.base.linear_velocity), 0.15 + 1e-9);
  }
  // It must have taken longer than the nominal profile to stay inside.
  EXPECT_GT(result.trajectory.points.back().time_from_start, 10.0);
}

TEST(WholeBodyPlanner, RejectsWhenTheEnvelopeNeedsAnUnreasonableDuration)
{
  auto config = makeConfig();
  config.builder.max_linear_velocity = 0.01;  // needs a very long trajectory
  config.max_trajectory_duration = 20.0;
  wbmm::planning::WholeBodyPlanner planner(config);

  const auto result = planner.plan(makeRequest(), freeBase(), freeWholeBody());

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("budget"), std::string::npos);
  EXPECT_TRUE(result.trajectory.points.empty());
}

TEST(WholeBodyPlanner, FailsWhenArmSeedingCannotFindAConfiguration)
{
  const auto blocked = [](const wbmm::core::Header &,
                          const wbmm::core::WholeBodyState &) {return false;};

  wbmm::planning::WholeBodyPlanner planner(makeConfig());
  const auto result = planner.plan(makeRequest(), freeBase(), blocked);

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("Arm seeding failed"), std::string::npos);
}

TEST(WholeBodyPlanner, FailsWhenTheBaseSearchFindsNoPath)
{
  const auto blocked = [](const wbmm::core::Header &,
                          const wbmm::core::BaseState &) {return false;};

  wbmm::planning::WholeBodyPlanner planner(makeConfig());
  const auto result = planner.plan(makeRequest(), blocked, freeWholeBody());

  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("Base search failed"), std::string::npos);
}

TEST(WholeBodyPlanner, RejectsMalformedRequests)
{
  wbmm::planning::WholeBodyPlanner planner(makeConfig());

  auto no_frame = makeRequest();
  no_frame.header.frame_id.clear();
  EXPECT_FALSE(
    planner.plan(no_frame, freeBase(), freeWholeBody()).success);

  auto no_joints = makeRequest();
  no_joints.start_joints = wbmm::core::JointState{};
  EXPECT_FALSE(
    planner.plan(no_joints, freeBase(), freeWholeBody()).success);

  auto mismatched = makeRequest();
  mismatched.limits.joint_max.pop_back();
  EXPECT_FALSE(
    planner.plan(mismatched, freeBase(), freeWholeBody()).success);

  // Missing seams must be reported, not dereferenced.
  EXPECT_FALSE(
    planner.plan(makeRequest(), {}, freeWholeBody()).success);
  EXPECT_FALSE(
    planner.plan(makeRequest(), freeBase(), {}).success);
}

TEST(WholeBodyPlanner, ReportsTimings)
{
  wbmm::planning::WholeBodyPlanner planner(makeConfig());
  const auto result = planner.plan(makeRequest(), freeBase(), freeWholeBody());
  ASSERT_TRUE(result.success) << result.message;

  EXPECT_GE(result.base_search_time, 0.0);
  EXPECT_GE(result.arm_seed_time, 0.0);
  EXPECT_GE(result.build_time, 0.0);
}

TEST(WholeBodyPlanner, RejectsNonSixJointRequestWithoutEigenAssertion)
{
  auto request = makeRequest();
  request.start_joints.names.pop_back();
  request.start_joints.positions.pop_back();
  request.start_joints.velocities.pop_back();
  request.limits.joint_min.pop_back();
  request.limits.joint_max.pop_back();
  EXPECT_FALSE(wbmm::planning::WholeBodyPlanner(makeConfig()).plan(
    request, freeBase(), freeWholeBody()).success);
}

TEST(WholeBodyPlanner, RejectsInvalidTimingBeforeShaping)
{
  auto config = makeConfig();
  config.min_segment_duration = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(wbmm::planning::WholeBodyPlanner(config).plan(
    makeRequest(), freeBase(), freeWholeBody()).success);
}

TEST(WholeBodyPlanner, ChecksFinalReferenceNotOnlySeed)
{
  const auto baseline = wbmm::planning::WholeBodyPlanner(makeConfig()).plan(
    makeRequest(), freeBase(), freeWholeBody());
  ASSERT_TRUE(baseline.success) << baseline.message;
  // A small obstacle between arm seed interpolation checks. Choose its centre
  // from the shaped output but away from all seed transition sample positions.
  double obstacle = -1.0;
  for (const auto & point : baseline.trajectory.points) {
    const double x = point.state.base.x;
    double nearest = 1e9;
    const auto & path = baseline.search.base_path;
    for (std::size_t i = 1; i < path.size(); ++i) {
      for (int k = 0; k <= 8; ++k) {
        nearest = std::min(nearest, std::abs(x -
          (path[i-1].x + k / 8.0 * (path[i].x - path[i-1].x))));
      }
    }
    if (nearest > 0.002) { obstacle = x; break; }
  }
  ASSERT_GE(obstacle, 0.0);
  const auto checker = [obstacle](const auto &, const auto & state) {
    return std::abs(state.base.x - obstacle) > 0.001;
  };
  const auto result = wbmm::planning::WholeBodyPlanner(makeConfig()).plan(
    makeRequest(), freeBase(), checker);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("Final trajectory"), std::string::npos) << result.message;
  EXPECT_TRUE(result.trajectory.points.empty());
}

TEST(WholeBodyPlanner, StraightPathWithShortTailDoesNotReverseAfterShaping)
{
  auto config = makeConfig();
  config.base_search.position_resolution = 0.15;
  config.base_search.position_tolerance = 0.2;
  config.base_search.yaw_tolerance = 0.3;
  config.arm_seed.waypoint_spacing = 0.25;
  config.min_segment_duration = 0.4;
  config.cruise_speed = 0.35;
  auto request = makeRequest(0.8);
  request.start.x = -4.44e-16; request.start.y = 2.22e-16;
  request.start.yaw = -2.22e-16;
  const auto result = wbmm::planning::WholeBodyPlanner(config).plan(
    request, freeBase(), freeWholeBody());
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LT(result.max_yaw_rate, 1e-6);
  for (const auto & point : result.trajectory.points) {
    EXPECT_GE(point.state.base.linear_velocity, -1e-10);
  }
}

TEST(WholeBodyPlanner, RotationAtFixedBaseUsesSegmentedTrajectory) {
  auto request = makeRequest(0);
  request.goal.yaw = 1.2;
  auto result =
      wbmm::planning::WholeBodyPlanner(makeConfig()).plan(request, freeBase(), freeWholeBody());
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.trajectory_backend, "time_scaled_primitives");
  EXPECT_NEAR(result.trajectory.points.front().state.base.yaw, 0, 1e-9);
  EXPECT_NEAR(result.trajectory.points.back().state.base.yaw, 1.2, 0.35);
}
TEST(WholeBodyPlanner, ExplicitArmGoalPreservesMeasuredStart) {
  auto request = makeRequest(0);
  request.goal_joints = makeJoints(0.8);
  auto result =
      wbmm::planning::WholeBodyPlanner(makeConfig()).plan(request, freeBase(), freeWholeBody());
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.trajectory_backend, "time_scaled_primitives");
  EXPECT_NEAR(result.trajectory.points.front().state.joints.positions[0], 0, 1e-9);
  EXPECT_NEAR(result.trajectory.points.back().state.joints.positions[0], 0.8, 1e-8);
  EXPECT_NEAR(result.trajectory.points.back().state.base.x, 0, 1e-9);
}
TEST(WholeBodyPlanner, FallsBackAfterAstarBudgetExhaustion) {
  auto config = makeConfig();
  config.base_search.max_nodes = 2;
  auto result =
      wbmm::planning::WholeBodyPlanner(config).plan(makeRequest(1), freeBase(), freeWholeBody());
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_TRUE(result.whole_body_rrt_attempted);
  EXPECT_EQ(result.search_backend, "whole_body_rrt");
}
TEST(WholeBodyPlanner, SegmentedReferenceObeysDifferentialDrive) {
  auto config = makeConfig();
  config.base_search.max_nodes = 2;
  auto request = makeRequest(0.8);
  request.goal.y = 0.8;
  request.goal.yaw = 1.5;
  auto result = wbmm::planning::WholeBodyPlanner(config).plan(request, freeBase(), freeWholeBody());
  ASSERT_TRUE(result.success) << result.message;
  const auto &p = result.trajectory.points;
  for (std::size_t i = 1; i < p.size(); ++i) {
    const auto &a = p[i - 1].state.base;
    const auto &b = p[i].state.base;
    const double dt = p[i].time_from_start - p[i - 1].time_from_start;
    const double yaw = (a.yaw + b.yaw) / 2;
    EXPECT_NEAR(-(b.x - a.x) * std::sin(yaw) + (b.y - a.y) * std::cos(yaw), 0, 1e-7);
    EXPECT_LE(std::hypot(b.x - a.x, b.y - a.y) / dt, 0.5 + 1e-6);
    EXPECT_LE(std::abs(b.yaw - a.yaw) / dt, 1.0 + 1e-6);
  }
}

TEST(WholeBodyPlanner, FallsBackWhenArmCannotUseTheAstarCorridor) {
  auto config = makeConfig();
  config.sample_rrt.max_iterations = 128;
  config.base_search.min_x = -0.5;
  config.base_search.max_x = 1.5;
  config.base_search.min_y = -1;
  config.base_search.max_y = 1;
  auto checker = [](const auto &, const auto &state) {
    return !(state.base.x > 0.35 && state.base.x < 0.65 && std::abs(state.base.y) < 0.22);
  };
  auto result = wbmm::planning::WholeBodyPlanner(config).plan(makeRequest(1), freeBase(), checker);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_EQ(result.search_backend, "whole_body_rrt");
  EXPECT_TRUE(result.whole_body_rrt_attempted);
  EXPECT_EQ(result.trajectory_backend, "time_scaled_primitives");
}

}  // namespace
