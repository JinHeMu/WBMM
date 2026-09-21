#include "wbmm_search/kino_astar.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace
{
using wbmm::search::CollisionMode;
using wbmm::search::KinoAstar;
using wbmm::search::KinoAstarConfig;
using wbmm::search::SearchStatus;
constexpr double kPi = 3.14159265358979323846;

wbmm::core::Header header()
{
  return {"odom", 10.0};
}

wbmm::core::RobotLimits limits()
{
  wbmm::core::RobotLimits value;
  value.max_base_speed = 0.5;
  value.max_base_yaw_rate = 1.0;
  return value;
}

KinoAstarConfig offlineConfig()
{
  KinoAstarConfig config;
  config.collision_mode = CollisionMode::kDisabled;
  return config;
}

void expectFailure(const wbmm::search::BaseSearchResult & result, SearchStatus status)
{
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.status, status) << result.message;
  EXPECT_TRUE(result.path.empty());
  EXPECT_TRUE(result.primitives.empty());
  EXPECT_FALSE(result.message.empty());
  EXPECT_TRUE(std::isfinite(result.solve_time));
  EXPECT_GE(result.solve_time, 0.0);
}

void expectReplay(
  const wbmm::search::BaseSearchResult & result, const KinoAstarConfig & config = {},
  const wbmm::core::RobotLimits & robot_limits = limits())
{
  ASSERT_TRUE(result.success) << result.message;
  ASSERT_EQ(result.path.size(), result.primitives.size() + 1);
  double length = 0.0;
  double cost = 0.0;
  int last_direction = 0;
  for (std::size_t i = 0; i < result.primitives.size(); ++i) {
    const auto & input = result.primitives[i];
    const auto replay = wbmm::search::propagate(result.path[i], input, input.duration);
    EXPECT_NEAR(replay.x, result.path[i + 1].x, 1.0e-12);
    EXPECT_NEAR(replay.y, result.path[i + 1].y, 1.0e-12);
    EXPECT_NEAR(replay.yaw, result.path[i + 1].yaw, 1.0e-12);
    EXPECT_DOUBLE_EQ(result.path[i + 1].linear_velocity, input.v);
    EXPECT_DOUBLE_EQ(result.path[i + 1].yaw_rate, input.omega);
    EXPECT_DOUBLE_EQ(result.path[i + 1].lateral_velocity, 0.0);
    EXPECT_LE(std::abs(input.v), robot_limits.max_base_speed);
    EXPECT_LE(std::abs(input.omega), robot_limits.max_base_yaw_rate);
    length += std::abs(input.v) * input.duration;
    cost += input.duration + config.reverse_weight * std::max(0.0, -input.v) * input.duration +
      config.rotation_weight * std::abs(input.omega) * input.duration;
    if (i > 0) {
      cost += config.speed_change_weight * std::abs(input.v - result.primitives[i - 1].v) +
        config.yaw_rate_change_weight * std::abs(input.omega - result.primitives[i - 1].omega);
    }
    const int direction = input.v > 0.0 ? 1 : (input.v < 0.0 ? -1 : 0);
    if (direction != 0) {
      if (last_direction != 0 && direction != last_direction) {cost += config.gear_switch_weight;}
      last_direction = direction;
    }
  }
  EXPECT_NEAR(result.path_length, length, 1.0e-12);
  EXPECT_NEAR(result.total_cost, cost, 1.0e-12);
}
}  // namespace

TEST(DifferentialDriveRollout, ForwardAndReverseRespectHeading)
{
  const wbmm::core::BaseState start{1.0, 2.0, kPi / 2.0};
  const auto forward = wbmm::search::propagate(start, {0.5, 0.0, 2.0}, 2.0);
  const auto reverse = wbmm::search::propagate(start, {-0.5, 0.0, 2.0}, 2.0);
  EXPECT_NEAR(forward.x, 1.0, 1.0e-12);
  EXPECT_NEAR(forward.y, 3.0, 1.0e-12);
  EXPECT_NEAR(reverse.y, 1.0, 1.0e-12);
  EXPECT_DOUBLE_EQ(forward.lateral_velocity, 0.0);
}

TEST(DifferentialDriveRollout, CircularArcUsesExactIntegration)
{
  const auto state = wbmm::search::propagate({}, {1.0, 1.0, kPi / 2.0}, kPi / 2.0);
  EXPECT_NEAR(state.x, 1.0, 1.0e-12);
  EXPECT_NEAR(state.y, 1.0, 1.0e-12);
  EXPECT_NEAR(state.yaw, kPi / 2.0, 1.0e-12);
  const auto near_straight = wbmm::search::propagate({}, {1.0, 1.0e-12, 1.0}, 1.0);
  EXPECT_NEAR(near_straight.x, 1.0, 1.0e-12);
  EXPECT_NEAR(near_straight.y, 0.5e-12, 1.0e-15);
}

TEST(DifferentialDriveRollout, RotationKeepsPositionAndWrapsYaw)
{
  const wbmm::core::BaseState start{1.0, 2.0, kPi - 0.1};
  const auto state = wbmm::search::propagate(start, {0.0, 1.0, 0.4}, 0.4);
  EXPECT_DOUBLE_EQ(state.x, start.x);
  EXPECT_DOUBLE_EQ(state.y, start.y);
  EXPECT_NEAR(state.yaw, -kPi + 0.3, 1.0e-12);
}

TEST(DifferentialDriveRollout, InvalidArgumentsAndOverflowThrow)
{
  EXPECT_THROW((void)wbmm::search::propagate({}, {0.5, 0.0, 0.4}, -0.1), std::invalid_argument);
  EXPECT_THROW((void)wbmm::search::propagate({}, {0.5, 0.0, 0.4}, 0.5), std::invalid_argument);
  EXPECT_THROW((void)wbmm::search::propagate({}, {0.5, 0.0, 0.0}, 0.0), std::invalid_argument);
  EXPECT_THROW((void)wbmm::search::propagate({}, {std::numeric_limits<double>::infinity(), 0.0, 1.0}, 1.0),
    std::invalid_argument);
  EXPECT_THROW((void)wbmm::search::propagate({}, {std::numeric_limits<double>::max(), 0.0, 2.0}, 2.0),
    std::invalid_argument);
}

TEST(KinoAstarSearch, ForwardPathReplaysAndEndsInsideGoalRegion)
{
  const auto result = KinoAstar(offlineConfig()).search(header(), {}, {2.03, 0.0, 0.0}, limits());
  expectReplay(result);
  ASSERT_TRUE(result.success);
  EXPECT_LE(std::hypot(result.path.back().x - 2.03, result.path.back().y), 0.15);
  EXPECT_NE(result.path.back().x, 2.03);  // Retain the real endpoint, not a goal snap.
  for (const auto & input : result.primitives) {EXPECT_DOUBLE_EQ(input.omega, 0.0);}
  EXPECT_GT(result.generated_nodes, result.path.size());
  EXPECT_EQ(result.header.frame_id, "odom");
  EXPECT_DOUBLE_EQ(result.header.stamp, 10.0);
  EXPECT_FALSE(result.collision_checked);
}

TEST(KinoAstarSearch, ReverseIsAvailableWithoutTurningAround)
{
  const auto result = KinoAstar(offlineConfig()).search(header(), {}, {-2.0, 0.0, 0.0}, limits());
  expectReplay(result);
  ASSERT_TRUE(result.success);
  bool reversed = false;
  for (const auto & input : result.primitives) {reversed = reversed || input.v < 0.0;}
  EXPECT_TRUE(reversed);
  EXPECT_NEAR(result.path.back().yaw, 0.0, 1.0e-12);
}

TEST(KinoAstarSearch, SamePositionCanHaveDifferentGoalYaw)
{
  const auto result = KinoAstar(offlineConfig()).search(header(), {}, {0.0, 0.0, kPi / 2.0}, limits());
  expectReplay(result);
  ASSERT_TRUE(result.success);
  ASSERT_FALSE(result.primitives.empty());
  EXPECT_NEAR(result.path_length, 0.0, 1.0e-12);
  EXPECT_LE(std::abs(result.path.back().yaw - kPi / 2.0), 0.1);
}

TEST(KinoAstarSearch, YawGoalAcrossPiUsesShortestAngle)
{
  const auto result = KinoAstar(offlineConfig()).search(
    header(), {0.0, 0.0, kPi - 0.05}, {0.0, 0.0, -kPi + 0.05}, limits());
  expectReplay(result);
  ASSERT_TRUE(result.success);
  EXPECT_LT(result.total_cost, 1.0);
}

TEST(KinoAstarSearch, IdenticalGoalReturnsOneNodeAndNoControls)
{
  const auto result = KinoAstar(offlineConfig()).search(header(), {}, {}, limits());
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.path.size(), 1U);
  EXPECT_TRUE(result.primitives.empty());
  EXPECT_DOUBLE_EQ(result.total_cost, 0.0);
  EXPECT_DOUBLE_EQ(result.path_length, 0.0);
}

TEST(KinoAstarSearch, VelocityFieldsAreNotGoalConstraints)
{
  wbmm::core::BaseState start;
  start.linear_velocity = 0.2;
  wbmm::core::BaseState goal;
  goal.linear_velocity = -0.3;
  goal.yaw_rate = 0.7;
  const auto result = KinoAstar(offlineConfig()).search(header(), start, goal, limits());
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.primitives.empty());
  EXPECT_DOUBLE_EQ(result.path.front().linear_velocity, 0.0);
}

TEST(KinoAstarValidation, RequiresExplicitCollisionModeOrChecker)
{
  expectFailure(KinoAstar().search(header(), {}, {}, limits()), SearchStatus::kMissingCollisionChecker);
  const auto result = KinoAstar().search(header(), {}, {0.4, 0.0, 0.0}, limits(),
    [](const auto &, const auto &) {return true;});
  expectReplay(result);
  EXPECT_TRUE(result.collision_checked);
}

TEST(KinoAstarValidation, RejectsBadHeaderAndNonfiniteState)
{
  auto bad_header = header();
  bad_header.frame_id.clear();
  expectFailure(KinoAstar(offlineConfig()).search(bad_header, {}, {}, limits()), SearchStatus::kInvalidInput);
  bad_header = header();
  bad_header.stamp = -1.0;
  expectFailure(KinoAstar(offlineConfig()).search(bad_header, {}, {}, limits()), SearchStatus::kInvalidInput);
  auto state = wbmm::core::BaseState{};
  state.x = std::numeric_limits<double>::quiet_NaN();
  expectFailure(KinoAstar(offlineConfig()).search(header(), state, {}, limits()), SearchStatus::kInvalidInput);
  state = {};
  state.lateral_velocity = 0.1;
  expectFailure(KinoAstar(offlineConfig()).search(header(), state, {}, limits()), SearchStatus::kInvalidInput);
  state = {};
  state.yaw = 4.0;
  expectFailure(KinoAstar(offlineConfig()).search(header(), state, {}, limits()), SearchStatus::kInvalidInput);
}

TEST(KinoAstarValidation, RejectsInvalidConfigAndBaseLimits)
{
  std::vector<KinoAstarConfig> invalid;
  auto config = offlineConfig(); config.position_resolution = 0.0; invalid.push_back(config);
  config = offlineConfig(); config.yaw_bins = 0; invalid.push_back(config);
  config = offlineConfig(); config.max_nodes = 0; invalid.push_back(config);
  config = offlineConfig(); config.primitive_duration = -1.0; invalid.push_back(config);
  config = offlineConfig(); config.max_search_time = 0.0; invalid.push_back(config);
  config = offlineConfig(); config.min_x = config.max_x; invalid.push_back(config);
  config = offlineConfig(); config.yaw_tolerance = 4.0; invalid.push_back(config);
  config = offlineConfig(); config.position_tolerance = -0.1; invalid.push_back(config);
  config = offlineConfig(); config.reverse_weight = -0.1; invalid.push_back(config);
  config = offlineConfig(); config.gear_switch_weight = -0.1; invalid.push_back(config);
  config = offlineConfig(); config.speed_change_weight = std::numeric_limits<double>::infinity(); invalid.push_back(config);
  config = offlineConfig(); config.yaw_rate_change_weight = std::numeric_limits<double>::quiet_NaN(); invalid.push_back(config);
  config = offlineConfig(); config.rotation_weight = std::numeric_limits<double>::infinity(); invalid.push_back(config);
  config = offlineConfig(); config.speed_samples.clear(); invalid.push_back(config);
  config = offlineConfig(); config.yaw_rate_samples = {2.0}; invalid.push_back(config);
  config = offlineConfig(); config.speed_samples = {0.0}; config.yaw_rate_samples = {0.0}; invalid.push_back(config);
  config = offlineConfig(); config.max_sample_angle = 0.0; invalid.push_back(config);
  config = offlineConfig(); config.position_resolution = 1.0e-300; invalid.push_back(config);
  for (const auto & value : invalid) {
    expectFailure(KinoAstar(value).search(header(), {}, {}, limits()), SearchStatus::kInvalidInput);
  }
  auto bad_limits = limits();
  bad_limits.max_base_speed = 0.0;
  expectFailure(KinoAstar(offlineConfig()).search(header(), {}, {}, bad_limits), SearchStatus::kInvalidInput);
  bad_limits = limits();
  bad_limits.max_base_yaw_rate = std::numeric_limits<double>::quiet_NaN();
  expectFailure(KinoAstar(offlineConfig()).search(header(), {}, {}, bad_limits), SearchStatus::kInvalidInput);
}

TEST(KinoAstarValidation, DoesNotRequireArmLimits)
{
  auto base_limits = limits();
  base_limits.joint_min = {std::numeric_limits<double>::quiet_NaN()};
  EXPECT_TRUE(KinoAstar(offlineConfig()).search(header(), {}, {}, base_limits).success);
}

TEST(KinoAstarValidation, StartAndGoalOutsideBoundsAreDistinct)
{
  expectFailure(KinoAstar(offlineConfig()).search(header(), {11.0, 0.0, 0.0}, {}, limits()),
    SearchStatus::kInvalidStart);
  expectFailure(KinoAstar(offlineConfig()).search(header(), {}, {11.0, 0.0, 0.0}, limits()),
    SearchStatus::kInvalidGoal);
}

TEST(KinoAstarCollision, RejectsStartAndGoal)
{
  expectFailure(KinoAstar().search(header(), {}, {0.4, 0.0, 0.0}, limits(),
      [](const auto &, const auto & state) {return state.x > 0.1;}), SearchStatus::kInvalidStart);
  expectFailure(KinoAstar().search(header(), {}, {0.4, 0.0, 0.0}, limits(),
      [](const auto &, const auto & state) {return state.x < 0.1;}), SearchStatus::kInvalidGoal);
}

TEST(KinoAstarCollision, ChecksMidPrimitiveObstacleWithFreeEndpoints)
{
  KinoAstarConfig config;
  config.speed_samples = {1.0};
  config.yaw_rate_samples = {0.0};
  bool checked_middle = false;
  const auto result = KinoAstar(config).search(header(), {}, {0.4, 0.0, 0.0}, limits(),
    [&](const auto & metadata, const auto & state) {
      EXPECT_EQ(metadata.frame_id, "odom");
      const bool occupied = state.x >= 0.075 && state.x <= 0.125;
      checked_middle = checked_middle || occupied;
      return !occupied;
    });
  expectFailure(result, SearchStatus::kNoPath);
  EXPECT_TRUE(checked_middle);
}

TEST(KinoAstarCollision, ChecksOrientationDuringInPlaceRotation)
{
  KinoAstarConfig config;
  config.speed_samples = {0.0};
  config.yaw_rate_samples = {1.0};
  bool checked_rotation = false;
  const auto result = KinoAstar(config).search(header(), {}, {0.0, 0.0, 0.4}, limits(),
    [&](const auto &, const auto & state) {
      const bool occupied = state.yaw >= 0.075 && state.yaw <= 0.125;
      checked_rotation = checked_rotation || occupied;
      EXPECT_DOUBLE_EQ(state.x, 0.0);
      EXPECT_DOUBLE_EQ(state.y, 0.0);
      return !occupied;
    });
  expectFailure(result, SearchStatus::kNoPath);
  EXPECT_TRUE(checked_rotation);
}

TEST(KinoAstarCollision, FindsDetourAroundSyntheticObstacle)
{
  KinoAstarConfig config;
  // This sharp-edged synthetic obstacle needs denser checking than the demo
  // defaults. A sampled checker does not certify continuous clearance.
  config.max_sample_time = 0.0025;
  config.min_x = -0.5;
  config.max_x = 2.5;
  config.min_y = -1.0;
  config.max_y = 1.0;
  const wbmm::search::BaseCollisionChecker checker = [](const auto &, const auto & state) {
      return !(state.x >= 0.8 && state.x <= 1.2 && std::abs(state.y) <= 0.3);
    };
  const auto result = KinoAstar(config).search(header(), {}, {2.0, 0.0, 0.0}, limits(), checker);
  expectReplay(result);
  ASSERT_TRUE(result.success);
  EXPECT_TRUE(result.collision_checked);
  bool curved = false;
  for (std::size_t i = 0; i < result.primitives.size(); ++i) {
    const auto & input = result.primitives[i];
    curved = curved || (input.v != 0.0 && input.omega != 0.0);
    // Independently recheck at 5 ms (the planner checks this scene at 2.5 ms).
    for (int sample = 0; sample <= 80; ++sample) {
      EXPECT_TRUE(checker(header(), wbmm::search::propagate(
            result.path[i], input, input.duration * (static_cast<double>(sample) / 80.0))));
    }
  }
  EXPECT_TRUE(curved);
}

TEST(KinoAstarValidation, RejectsInitialHeuristicOverflow)
{
  auto config = offlineConfig();
  config.min_x = config.min_y = -1.0e200;
  config.max_x = config.max_y = 1.0e200;
  config.position_resolution = 1.0e199;
  auto base_limits = limits();
  base_limits.max_base_speed = 1.0e-200;
  expectFailure(KinoAstar(config).search(header(), {}, {1.0e200, 0.0, 0.0}, base_limits),
    SearchStatus::kInvalidInput);
}

TEST(KinoAstarCollision, CallbackExceptionsFailClosed)
{
  expectFailure(KinoAstar().search(header(), {}, {}, limits(),
      [](const auto &, const auto &) -> bool {throw std::runtime_error("Unavailable map");}),
    SearchStatus::kCollisionCheckerError);
  std::size_t calls = 0;
  expectFailure(KinoAstar().search(header(), {}, {0.4, 0.0, 0.0}, limits(),
      [&](const auto &, const auto &) {if (++calls > 2) {throw 42;} return true;}),
    SearchStatus::kCollisionCheckerError);
}

TEST(KinoAstarCollision, DisabledModeDoesNotInvokeProvidedCallback)
{
  const auto result = KinoAstar(offlineConfig()).search(header(), {}, {}, limits(),
    [](const auto &, const auto &) -> bool {throw std::runtime_error("Must not be invoked");});
  EXPECT_TRUE(result.success);
  EXPECT_FALSE(result.collision_checked);
}

TEST(KinoAstarBounds, RejectsArcLeavingBoundsDespiteValidEndpoint)
{
  auto config = offlineConfig();
  config.min_x = config.min_y = -0.1;
  config.max_x = config.max_y = 0.1;
  config.position_resolution = 0.02;
  config.primitive_duration = 1.0;
  config.position_tolerance = config.yaw_tolerance = 0.001;
  config.speed_samples = config.yaw_rate_samples = {1.0};
  auto base_limits = limits();
  base_limits.max_base_speed = 1.0;
  base_limits.max_base_yaw_rate = 7.0;
  const auto goal = wbmm::search::propagate({}, {1.0, 7.0, 1.0}, 1.0);
  ASSERT_LT(goal.x, config.max_x);
  ASSERT_LT(goal.y, config.max_y);
  expectFailure(KinoAstar(config).search(header(), {}, goal, base_limits), SearchStatus::kNoPath);
}

TEST(KinoAstarBudget, NodeBudgetCountsImmutableRecords)
{
  auto config = offlineConfig();
  config.max_nodes = 1;
  expectFailure(KinoAstar(config).search(header(), {}, {2.0, 0.0, 0.0}, limits()), SearchStatus::kNodeLimit);
}

TEST(KinoAstarBudget, MonotonicClockEnforcesTimeout)
{
  auto config = offlineConfig();
  config.max_search_time = 1.0e-12;
  expectFailure(KinoAstar(config).search(header(), {}, {2.0, 0.0, 0.0}, limits()), SearchStatus::kTimeout);
}

TEST(KinoAstarSearch, RepeatedCallsHaveIndependentState)
{
  const KinoAstar planner(offlineConfig());
  const auto first = planner.search(header(), {}, {2.0, 0.0, 0.0}, limits());
  expectFailure(planner.search(header(), {}, {11.0, 0.0, 0.0}, limits()), SearchStatus::kInvalidGoal);
  const auto second = planner.search(header(), {}, {2.0, 0.0, 0.0}, limits());
  expectReplay(second);
  EXPECT_EQ(first.generated_nodes, second.generated_nodes);
  EXPECT_DOUBLE_EQ(first.total_cost, second.total_cost);
}

TEST(KinoAstarCost, ForcedForwardReverseHasSeparateReverseGearAndChangeCosts)
{
  auto config = offlineConfig();
  config.min_x = -0.11;
  config.max_x = 0.11;
  config.position_resolution = 0.05;
  config.position_tolerance = 1.0e-8;
  config.speed_samples = {-1.0, 0.5};
  config.yaw_rate_samples = {0.0};
  config.reverse_weight = 2.0;
  config.gear_switch_weight = 3.0;
  config.speed_change_weight = 4.0;
  config.yaw_rate_change_weight = 0.0;
  const auto result = KinoAstar(config).search(header(), {}, {-0.1, 0.0, 0.0}, limits());
  expectReplay(result, config);
  ASSERT_EQ(result.primitives.size(), 2U);
  EXPECT_DOUBLE_EQ(result.primitives[0].v, 0.25);
  EXPECT_DOUBLE_EQ(result.primitives[1].v, -0.5);
  // 0.8 time + 2*0.2 reverse distance + 3 gear + 4*0.75 delta-v.
  EXPECT_NEAR(result.total_cost, 7.2, 1.0e-12);
}

TEST(KinoAstarCost, RotationBetweenForwardAndReverseDoesNotEraseDirection)
{
  KinoAstarConfig config;
  config.speed_samples = {-1.0, 0.0, 1.0};
  config.yaw_rate_samples = {0.0, 1.0};
  config.position_resolution = 0.02;
  config.position_tolerance = config.yaw_tolerance = 1.0e-8;
  config.speed_change_weight = config.yaw_rate_change_weight = 0.0;
  config.gear_switch_weight = 2.0;
  auto base_limits = limits();
  base_limits.max_base_speed = 0.25;
  const auto goal = wbmm::search::propagate({0.1, 0.0, 0.4}, {-0.25, 0.0, 0.4}, 0.4);
  // Test-only corridors in pose space force forward -> rotate -> reverse.
  const wbmm::search::BaseCollisionChecker corridor = [goal](const auto &, const auto & state) {
      constexpr double eps = 1.0e-8;
      const bool forward = std::abs(state.yaw) < eps && std::abs(state.y) < eps &&
        state.x >= -eps && state.x <= 0.1 + eps;
      const bool rotate = std::abs(state.x - 0.1) < eps && std::abs(state.y) < eps &&
        state.yaw >= -eps && state.yaw <= 0.4 + eps;
      const bool reverse = std::abs(state.yaw - 0.4) < eps &&
        state.x >= goal.x - eps && state.x <= 0.1 + eps &&
        std::abs(state.y - std::tan(0.4) * (state.x - 0.1)) < eps;
      return forward || rotate || reverse;
    };
  const auto result = KinoAstar(config).search(header(), {}, goal, base_limits, corridor);
  expectReplay(result, config);
  ASSERT_EQ(result.primitives.size(), 3U);
  EXPECT_GT(result.primitives[0].v, 0.0);
  EXPECT_DOUBLE_EQ(result.primitives[1].v, 0.0);
  EXPECT_LT(result.primitives[2].v, 0.0);
  // 1.2 time + 0.05 reverse + 0.04 rotation + 2 gear, counted once.
  EXPECT_NEAR(result.total_cost, 3.29, 1.0e-12);
}

TEST(KinoAstarKey, KeepsDifferentPreviousSpeedsInTheSamePoseBin)
{
  auto config = offlineConfig();
  config.min_x = 0.0;
  config.max_x = 0.31;
  config.position_resolution = 0.15;
  config.position_tolerance = 1.0e-8;
  config.speed_samples = {0.5, 1.0};
  config.yaw_rate_samples = {0.0};
  config.speed_change_weight = 4.0;
  config.yaw_rate_change_weight = 0.0;
  const auto smooth = KinoAstar(config).search(header(), {}, {0.3, 0.0, 0.0}, limits());
  expectReplay(smooth, config);
  ASSERT_EQ(smooth.primitives.size(), 3U);
  for (const auto & input : smooth.primitives) {EXPECT_DOUBLE_EQ(input.v, 0.25);}
  // At x=0.2, g=0.8 at v=0.25 must survive alongside g=0.4 at v=0.5.
  // Three constant slow controls cost 1.2, two differing controls cost 1.8.
  EXPECT_NEAR(smooth.total_cost, 1.2, 1.0e-12);
  config.speed_change_weight = 0.0;
  const auto fast = KinoAstar(config).search(header(), {}, {0.3, 0.0, 0.0}, limits());
  expectReplay(fast, config);
  ASSERT_EQ(fast.primitives.size(), 2U);
  EXPECT_NEAR(fast.total_cost, 0.8, 1.0e-12);
}

TEST(KinoAstarKey, KeepsDifferentPreviousYawRatesInTheSamePoseBin)
{
  auto config = offlineConfig();
  config.speed_samples = {0.0};
  config.yaw_rate_samples = {0.5, 1.0};
  config.yaw_bins = 21;
  config.yaw_tolerance = 1.0e-8;
  config.rotation_weight = 0.0;
  config.speed_change_weight = 0.0;
  config.yaw_rate_change_weight = 4.0;
  const auto result = KinoAstar(config).search(header(), {}, {0.0, 0.0, 0.6}, limits());
  expectReplay(result, config);
  ASSERT_EQ(result.primitives.size(), 3U);
  for (const auto & input : result.primitives) {EXPECT_DOUBLE_EQ(input.omega, 0.5);}
  EXPECT_NEAR(result.total_cost, 1.2, 1.0e-12);
}

TEST(KinoAstarKey, KeepsForwardAndReverseArrivalsForFutureGearCost)
{
  KinoAstarConfig config;
  config.min_x = -0.21;
  config.max_x = 0.01;
  config.min_y = -0.1;
  config.max_y = 0.1;
  config.position_resolution = 0.02;
  config.position_tolerance = config.yaw_tolerance = 1.0e-8;
  config.speed_samples = {-1.0, 0.0, 0.5};
  config.yaw_rate_samples = {-1.0, 0.0, 1.0};
  config.gear_switch_weight = 10.0;
  config.speed_change_weight = config.yaw_rate_change_weight = 0.0;
  auto base_limits = limits();
  base_limits.max_base_yaw_rate = kPi / config.primitive_duration;
  // A straight corridor permits rotations only at x=0 and x=-0.2.
  const wbmm::search::BaseCollisionChecker corridor = [](const auto &, const auto & state) {
      constexpr double eps = 1.0e-8;
      const bool straight = std::abs(state.y) < eps &&
        (std::abs(state.yaw) < eps || std::abs(std::abs(state.yaw) - kPi) < eps);
      const bool rotate = std::abs(state.y) < eps &&
        (std::abs(state.x) < eps || std::abs(state.x + 0.2) < eps);
      return straight || rotate;
    };
  const auto result = KinoAstar(config).search(
    header(), {}, {-0.1, 0.0, 0.0}, base_limits, corridor);
  expectReplay(result, config, base_limits);
  ASSERT_EQ(result.primitives.size(), 5U);
  for (const auto & input : result.primitives) {EXPECT_GE(input.v, 0.0);}
  // At (-0.2,0,0), cheap reverse arrival costs 0.5 but then pays gear=10.
  // More costly forward arrival survives because its direction key differs.
  // Two pi rotations and three forward controls cost 2 + 0.1*2*pi, no gear.
  EXPECT_NEAR(result.total_cost, 2.0 + 0.2 * kPi, 1.0e-12);
}

TEST(KinoAstarCost, FirstPrimitiveDoesNotUseMeasuredStartVelocity)
{
  auto config = offlineConfig();
  config.speed_change_weight = config.yaw_rate_change_weight = 100.0;
  const auto result = KinoAstar(config).search(
    header(), {0.0, 0.0, 0.0, -0.5, 0.0, -1.0}, {0.2, 0.0, 0.0}, limits());
  expectReplay(result, config);
  ASSERT_EQ(result.primitives.size(), 1U);
  EXPECT_DOUBLE_EQ(result.total_cost, 0.4);
}

TEST(KinoAstarValidation, AcceptsTinyLateralNoiseButRejectsActualLateralMotion)
{
  const KinoAstar planner(offlineConfig());
  EXPECT_TRUE(planner.search(header(), {0.0, 0.0, 0.0, 0.0, 1.0e-9}, {}, limits()).success);
  EXPECT_TRUE(planner.search(header(), {}, {0.0, 0.0, 0.0, 0.0, -1.0e-8}, limits()).success);
  expectFailure(planner.search(header(), {0.0, 0.0, 0.0, 0.0, 1.0e-7}, {}, limits()),
    SearchStatus::kInvalidInput);
}

TEST(KinoAstarKey, DuplicateControlSamplesHaveTheSameHistoryIdentity)
{
  auto config = offlineConfig();
  const auto normal = KinoAstar(config).search(header(), {}, {0.4, 0.0, 0.0}, limits());
  config.speed_samples.push_back(1.0);
  config.yaw_rate_samples.push_back(0.0);
  const auto duplicate = KinoAstar(config).search(header(), {}, {0.4, 0.0, 0.0}, limits());
  expectReplay(duplicate, config);
  EXPECT_EQ(normal.generated_nodes, duplicate.generated_nodes);
  EXPECT_EQ(normal.expanded_nodes, duplicate.expanded_nodes);
  EXPECT_DOUBLE_EQ(normal.total_cost, duplicate.total_cost);
}
