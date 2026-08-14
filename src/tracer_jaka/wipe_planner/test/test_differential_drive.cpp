#include "wipe_planner/planner.hpp"
#include <gtest/gtest.h>
#include <algorithm>

TEST(DifferentialDrive, StraightMotionHasNoLateralSlip)
{
  const Eigen::Vector3d start(1.0, 2.0, 0.0);
  const auto end = wipe_planner::Planner::propagateDifferentialDrive(start, -0.2, 0.0, 2.0);
  EXPECT_NEAR(end.x(), 0.6, 1.0e-12);
  EXPECT_NEAR(end.y(), 2.0, 1.0e-12);
  EXPECT_NEAR(end.z(), 0.0, 1.0e-12);
}

TEST(DifferentialDrive, ArcMatchesExactUnicycleSolution)
{
  const Eigen::Vector3d start(0.0, 0.0, 0.0);
  const auto end = wipe_planner::Planner::propagateDifferentialDrive(start, 1.0, 1.0, 0.5);
  EXPECT_NEAR(end.x(), std::sin(0.5), 1.0e-12);
  EXPECT_NEAR(end.y(), 1.0 - std::cos(0.5), 1.0e-12);
  EXPECT_NEAR(end.z(), 0.5, 1.0e-12);
}

TEST(Angles, WrapsAcrossPi)
{
  EXPECT_NEAR(wipe_planner::wrapAngle(3.5), 3.5 - 2.0 * M_PI, 1.0e-12);
}

TEST(WholeBodyHandoff, UsesWrappedNineDimensionalSquaredError)
{
  Eigen::VectorXd current = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd goal = Eigen::VectorXd::Zero(9);
  current[0] = 0.10;
  current[1] = -0.10;
  current[2] = M_PI - 0.05;
  goal[2] = -M_PI + 0.05;
  current[3] = 0.10;
  EXPECT_NEAR(
    wipe_planner::wholeBodySquaredError(current, goal), 0.04, 1.0e-12);
}

TEST(VirtualProgress, SlowsForLagAndStopsForLargeTrackingError)
{
  const auto rate = [](double error, double lag) {
      return wipe_planner::adaptiveProgressRate(
        error, lag, 1.0, 0.0, 1.15, 1.0, 0.7, 0.2, 0.7, 1.8);
    };
  EXPECT_NEAR(rate(0.0, 0.0), 1.0, 1.0e-12);
  EXPECT_LT(rate(0.0, 0.8), 1.0);
  EXPECT_GT(rate(0.0, -0.8), 1.0);
  EXPECT_DOUBLE_EQ(rate(2.0, 0.0), 0.0);
}

TEST(WholeBodyPlan, ContactTrajectoryIsReachableAndNonholonomic)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace + "/src/tracer_jaka/tracer_jaka_mujoco/urdf/tracer_jaka_zu5.urdf",
    "tool0", source + "/config/wipe_task.yaml");
  Eigen::VectorXd seed(9);
  // The seed is used to select a deterministic collision-free IK branch.
  seed << 2.0, 2.0, M_PI_2, 0.0, 1.5707, 0.0, 1.5707, 3.14159, 0.7854;
  wipe_planner::PlanReport report;
  const auto trajectory = planner.plan(seed, report);
  ASSERT_FALSE(trajectory.empty());
  EXPECT_EQ(report.ik_failures, 0);
  EXPECT_GT(report.hybrid_expanded_nodes, 0);
  EXPECT_LT(report.max_position_error, 0.0121);
  EXPECT_LT(report.max_axis_error, 0.121);
  EXPECT_LT(report.max_lateral_velocity, 1.0e-6);
  EXPECT_LE(report.max_wheel_speed, 10.0001);
  EXPECT_TRUE(std::any_of(trajectory.begin(), trajectory.end(),
    [](const auto & point) {return point.in_contact && point.input[0] < -1.0e-4;}));
  const Eigen::Vector3d wall_direction = -planner.surfaceNormal();
  for (const auto & point : trajectory) {
    if (!point.in_contact) {
      continue;
    }
    const Eigen::Vector3d tool_z = planner.frameRotation(point.state, "tool0").col(2);
    EXPECT_GT(tool_z.dot(wall_direction), std::cos(0.121));
    EXPECT_NEAR(point.state[2], 0.0, 1.0e-8);
  }
}

TEST(WholeBodyPlan, StartsAtWallNormalPrecontactAndStaysOffWall)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace + "/src/tracer_jaka/tracer_jaka_mujoco/urdf/tracer_jaka_zu5.urdf",
    "tool0", source + "/config/wipe_task.yaml");
  Eigen::VectorXd seed(9);
  // The measured seed selects the IK basin; execution starts at pre-contact.
  seed << 0.75, 2.06, 0.0, 0.004, 1.565, -0.003, 1.576, 3.137, 0.789;
  wipe_planner::PlanReport report;
  const auto trajectory = planner.plan(seed, report);
  ASSERT_GT(trajectory.size(), 2U);
  EXPECT_DOUBLE_EQ(trajectory.front().time, 0.0);
  EXPECT_FALSE(trajectory.front().in_contact);
  const auto first_contact = std::find_if(trajectory.begin(), trajectory.end(),
    [](const auto & point) {return point.in_contact;});
  ASSERT_NE(first_contact, trajectory.end());
  ASSERT_NE(first_contact, trajectory.begin());
  EXPECT_TRUE(std::none_of(trajectory.begin(), first_contact,
    [](const auto & point) {return point.in_contact;}));
  EXPECT_TRUE(std::all_of(first_contact, trajectory.end(),
    [](const auto & point) {return point.in_contact;}));
  EXPECT_NEAR(first_contact->contact_target.x(), 0.75, 1.0e-9);
  // The physical wall is at y=2.90 m. This fast non-contact profile keeps the
  // contact-tagged reference plane 5 cm on the room side of the wall.
  EXPECT_NEAR(first_contact->contact_target.y(), 2.85, 1.0e-9);
  EXPECT_NEAR(first_contact->contact_target.z(), 0.60, 1.0e-9);
  EXPECT_GE(first_contact->time, 3.49);
  const Eigen::Vector3d precontact_ee =
    planner.framePosition(trajectory.front().state, "tool0");
  EXPECT_NEAR(precontact_ee.y(), 2.73, 0.0121);
  double max_arm_speed = 0.0;
  double max_arm_step = 0.0;
  for (auto point = trajectory.begin(); std::next(point) != trajectory.end(); ++point) {
    const double dt = std::next(point)->time - point->time;
    const Eigen::VectorXd delta =
      std::next(point)->state.tail(6) - point->state.tail(6);
    max_arm_step = std::max(max_arm_step, delta.cwiseAbs().maxCoeff());
    max_arm_speed = std::max(max_arm_speed, delta.cwiseAbs().maxCoeff() / dt);
  }
  // Large geometric raster intervals are allowed, but their timestamps must
  // stretch them into a continuous velocity-limited reference.
  EXPECT_LT(max_arm_step, 0.70);
  EXPECT_LE(max_arm_speed, 0.0801);
}

TEST(WholeBodyPlan, FirstFrameIsACompleteWallNormalPrecontactGoal)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace + "/src/tracer_jaka/tracer_jaka_mujoco/urdf/tracer_jaka_zu5.urdf",
    "tool0", source + "/config/wipe_task.yaml");
  Eigen::VectorXd nominal(9);
  nominal << 0.75, 2.06, 0.0, 0.0, 1.5707, 0.0, 1.5707, 3.14159, 0.7854;
  wipe_planner::PlanReport report;
  const auto planned = planner.plan(nominal, report);
  ASSERT_FALSE(planned.empty());
  EXPECT_EQ(planned.front().state.size(), 9);
  EXPECT_FALSE(planned.front().in_contact);
  EXPECT_DOUBLE_EQ(planned.front().time, 0.0);
  const Eigen::Vector3d tool_z =
    planner.frameRotation(planned.front().state, "tool0").col(2);
  EXPECT_GT(tool_z.dot(-planner.surfaceNormal()), std::cos(0.121));
}

TEST(WholeBodyPlan, MeasuredArmAlignmentHoldsBaseAndRespectsJointSpeed)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace + "/src/tracer_jaka/tracer_jaka_mujoco/urdf/tracer_jaka_zu5.urdf",
    "tool0", source + "/config/wipe_task.yaml");
  Eigen::VectorXd measured(9);
  measured << 0.75, 2.06, 0.0, 0.0, 1.5707, 0.0, 1.5707, 3.14159, 0.7854;
  wipe_planner::PlanReport report;
  const auto planned = planner.plan(measured, report);
  ASSERT_FALSE(planned.empty());
  measured.head<3>() = planned.front().state.head<3>();
  const auto aligned = planner.prependMeasuredAlignment(planned, measured, 0.18, report);
  ASSERT_GT(aligned.size(), planned.size());
  for (const auto & point : aligned) {
    if (point.time > planned.front().time + 0.5 && point.in_contact) {
      break;
    }
    EXPECT_TRUE(point.state.head<3>().isApprox(planned.front().state.head<3>(), 1.0e-12));
  }
  double max_speed = 0.0;
  for (auto point = aligned.begin(); std::next(point) != aligned.end(); ++point) {
    const double dt = std::next(point)->time - point->time;
    if (dt <= 0.0) {
      continue;
    }
    max_speed = std::max(max_speed,
      (std::next(point)->state.tail(6) - point->state.tail(6))
      .cwiseAbs().maxCoeff() / dt);
  }
  EXPECT_LE(max_speed, 0.181);
}
