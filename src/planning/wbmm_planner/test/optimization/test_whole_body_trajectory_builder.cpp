// Contract tests for the MINCO -> WholeBodyTrajectory builder.
//
// The regression that matters most here is the yaw-rate one. REMANI derived the
// reference yaw rate analytically as (v x a)/|v|^2, which grows as 1/t when the
// base starts from rest. On a rest-to-rest straight line that formula reports a
// large spurious yaw rate near t = 0 even though the path is perfectly straight
// and the heading never changes. The builder instead differentiates the heading
// it actually emits, so a straight line must come out with a yaw rate of ~0.

#include <wbmm_planner/optimization/whole_body_trajectory_builder.hpp>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <string>
#include <vector>

namespace
{

constexpr int kDim = 8;

using MinSnap = wbmm::traj_opt::MinSnapOpt<kDim>;

const std::vector<std::string> kJointNames = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

// Rest-to-rest straight line along +x with the arm held still.
wbmm::traj_opt::Trajectory<7> straightLine(
  const Eigen::Vector2d & start, const Eigen::Vector2d & end, double duration)
{
  Eigen::MatrixXd head(kDim, 4);
  Eigen::MatrixXd tail(kDim, 4);
  head.setZero();
  tail.setZero();
  head(0, 0) = start.x();
  head(1, 0) = start.y();
  tail(0, 0) = end.x();
  tail(1, 0) = end.y();

  Eigen::MatrixXd inner(kDim, 1);
  inner.setZero();
  inner(0, 0) = 0.5 * (start.x() + end.x());
  inner(1, 0) = 0.5 * (start.y() + end.y());

  MinSnap solver;
  solver.reset(head, tail, 2);
  solver.generate(inner, Eigen::VectorXd::Constant(2, duration));
  return solver.getTraj(1);
}

TEST(TrajectoryBuilder, ProducesAConsistentSampledTrajectory)
{
  const auto minco = straightLine({0.0, 0.0}, {3.0, 0.0}, 3.0);
  const auto result = wbmm::traj_opt::buildWholeBodyTrajectory(
    minco, kJointNames, "map");

  ASSERT_TRUE(result.success) << result.message;
  ASSERT_GT(result.trajectory.points.size(), 10U);
  EXPECT_DOUBLE_EQ(result.trajectory.points.front().time_from_start, 0.0);
  EXPECT_NEAR(
    result.trajectory.points.back().time_from_start,
    minco.getTotalDuration(), 1e-9);

  // Times must be strictly increasing for the OCS2 reference to be valid.
  for (std::size_t i = 1U; i < result.trajectory.points.size(); ++i) {
    EXPECT_GT(
      result.trajectory.points[i].time_from_start,
      result.trajectory.points[i - 1U].time_from_start);
  }

  for (const auto & point : result.trajectory.points) {
    EXPECT_EQ(point.state.header.frame_id, "map");
    EXPECT_EQ(point.state.base_model, wbmm::core::BaseModel::kDifferentialDrive);
    EXPECT_EQ(point.state.joints.names, kJointNames);
    EXPECT_EQ(point.state.joints.positions.size(), kJointNames.size());
    ASSERT_TRUE(point.feedforward_input.has_value());
    // Differential drive: the base command is exactly [v, omega].
    ASSERT_EQ(point.feedforward_input->base_command.size(), 2U);
    EXPECT_DOUBLE_EQ(
      point.feedforward_input->base_command[0], point.state.base.linear_velocity);
    EXPECT_DOUBLE_EQ(
      point.feedforward_input->base_command[1], point.state.base.yaw_rate);
  }

  // Straight line along +x: the heading stays at zero.
  for (const auto & point : result.trajectory.points) {
    EXPECT_NEAR(point.state.base.yaw, 0.0, 1e-9);
  }
  EXPECT_NEAR(result.trajectory.points.back().state.base.x, 3.0, 1e-6);
}

// The regression this builder exists for.
TEST(TrajectoryBuilder, StraightLineHasNoSpuriousYawRateNearRest)
{
  // 3 m over 15 s peaks at 2.1875 * L / T = 0.44 m/s, inside the 0.5 envelope.
  const auto minco = straightLine({0.0, 0.0}, {3.0, 0.0}, 15.0);
  const auto result = wbmm::traj_opt::buildWholeBodyTrajectory(
    minco, kJointNames, "map");
  ASSERT_TRUE(result.success) << result.message;

  // REMANI's analytic formula on this trajectory reports several rad/s close to
  // t = 0 because |v| -> 0 while the lateral acceleration term does not vanish.
  // Differentiating the emitted heading gives essentially zero everywhere.
  EXPECT_LT(result.max_yaw_rate, 1e-6);
  EXPECT_TRUE(result.within_limits);
  for (const auto & point : result.trajectory.points) {
    EXPECT_NEAR(point.state.base.yaw_rate, 0.0, 1e-6);
  }
}

TEST(TrajectoryBuilder, YawRateMatchesTheDerivativeOfTheEmittedYaw)
{
  // A diagonal line, so the heading is non-trivial and must stay continuous.
  const auto minco = straightLine({0.0, 0.0}, {2.0, 2.0}, 2.5);
  const auto result = wbmm::traj_opt::buildWholeBodyTrajectory(
    minco, kJointNames, "map");
  ASSERT_TRUE(result.success) << result.message;

  const auto & points = result.trajectory.points;
  for (std::size_t i = 0U; i + 1U < points.size(); ++i) {
    const double dt =
      points[i + 1U].time_from_start - points[i].time_from_start;
    const double delta = std::atan2(
      std::sin(points[i + 1U].state.base.yaw - points[i].state.base.yaw),
      std::cos(points[i + 1U].state.base.yaw - points[i].state.base.yaw));
    // This is the consistency guarantee the MPC relies on: the feedforward yaw
    // rate is the derivative of the yaw reference, not an independent estimate.
    EXPECT_NEAR(points[i].state.base.yaw_rate, delta / dt, 1e-9) << "sample " << i;
  }

  // Heading must be continuous, i.e. never jump across the +-pi branch cut.
  for (std::size_t i = 1U; i < points.size(); ++i) {
    const double step =
      std::abs(points[i].state.base.yaw - points[i - 1U].state.base.yaw);
    EXPECT_LT(step, 1.0) << "yaw jumped at sample " << i;
  }
  EXPECT_NEAR(points.back().state.base.yaw, M_PI / 4.0, 1e-6);
}

TEST(TrajectoryBuilder, ReverseGearFlipsVelocityAndHeading)
{
  const auto minco = straightLine({0.0, 0.0}, {3.0, 0.0}, 15.0);

  wbmm::traj_opt::TrajectoryBuilderConfig config;
  config.gear = -1;
  // Reversing along +x means the body faces -x, so the caller seeds the
  // builder with the heading the robot will actually have.
  config.initial_yaw = M_PI;
  const auto result = wbmm::traj_opt::buildWholeBodyTrajectory(
    minco, kJointNames, "map", config);
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_TRUE(result.within_limits) << result.message;

  for (const auto & point : result.trajectory.points) {
    EXPECT_NEAR(std::abs(point.state.base.yaw), M_PI, 1e-6);
    EXPECT_LE(point.state.base.linear_velocity, 1e-12);
  }
}

// A gear flip with no planned turn is not a feasible reference: the heading
// would step by pi. The builder must report that instead of emitting it.
TEST(TrajectoryBuilder, ReportsGearFlipWithoutAPlannedTurn)
{
  const auto minco = straightLine({0.0, 0.0}, {3.0, 0.0}, 15.0);

  wbmm::traj_opt::TrajectoryBuilderConfig config;
  config.gear = -1;
  config.initial_yaw = 0.0;  // robot still faces +x, plan wants -x
  const auto result = wbmm::traj_opt::buildWholeBodyTrajectory(
    minco, kJointNames, "map", config);

  ASSERT_TRUE(result.success) << result.message;
  EXPECT_FALSE(result.within_limits);
  EXPECT_GT(result.max_heading_step, config.max_heading_step);
  EXPECT_NE(result.message.find("gear flips"), std::string::npos);
}

TEST(TrajectoryBuilder, ReportsEnvelopeViolationsInsteadOfClamping)
{
  const auto minco = straightLine({0.0, 0.0}, {3.0, 0.0}, 15.0);

  wbmm::traj_opt::TrajectoryBuilderConfig config;
  config.max_linear_velocity = 0.05;  // deliberately impossible
  const auto result = wbmm::traj_opt::buildWholeBodyTrajectory(
    minco, kJointNames, "map", config);

  // The builder must never silently clamp: it returns the trajectory together
  // with the observed extremes so the caller can reject or re-time it.
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_FALSE(result.within_limits);
  EXPECT_GT(result.max_linear_velocity, config.max_linear_velocity);
  EXPECT_NE(result.message.find("exceeds"), std::string::npos);
}

TEST(TrajectoryBuilder, RejectsMismatchedJointCountAndBadConfig)
{
  const auto minco = straightLine({0.0, 0.0}, {1.0, 0.0}, 1.0);

  const auto too_few = wbmm::traj_opt::buildWholeBodyTrajectory(
    minco, {"joint_1"}, "map");
  EXPECT_FALSE(too_few.success);
  EXPECT_NE(too_few.message.find("does not match"), std::string::npos);

  const auto no_frame = wbmm::traj_opt::buildWholeBodyTrajectory(
    minco, kJointNames, "");
  EXPECT_FALSE(no_frame.success);

  wbmm::traj_opt::TrajectoryBuilderConfig bad;
  bad.gear = 0;
  const auto bad_gear = wbmm::traj_opt::buildWholeBodyTrajectory(
    minco, kJointNames, "map", bad);
  EXPECT_FALSE(bad_gear.success);
}

TEST(TrajectoryBuilder, CurvedReferenceSatisfiesDifferentialDriveTranslation)
{
  Eigen::MatrixXd head = Eigen::MatrixXd::Zero(kDim, 4);
  Eigen::MatrixXd tail = head;
  tail(0, 0) = 2.0;
  Eigen::MatrixXd inner = Eigen::MatrixXd::Zero(kDim, 1);
  inner(0, 0) = 0.8; inner(1, 0) = 0.6;
  MinSnap solver;
  solver.reset(head, tail, 2);
  solver.generate(inner, Eigen::VectorXd::Constant(2, 4.0));
  const auto minco = solver.getTraj(1);
  const auto result = wbmm::traj_opt::buildWholeBodyTrajectory(minco, kJointNames, "map");
  ASSERT_TRUE(result.success) << result.message;
  for (const auto & point : result.trajectory.points) {
    const auto v = minco.getVel(point.time_from_start);
    EXPECT_NEAR(v(0), point.state.base.linear_velocity * std::cos(point.state.base.yaw), 1e-8);
    EXPECT_NEAR(v(1), point.state.base.linear_velocity * std::sin(point.state.base.yaw), 1e-8);
  }
}

}  // namespace
