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

TEST(SafetyRetreat, IsRateLimitedAndDoesNotJump)
{
  double retreat = 0.0;
  retreat = wipe_planner::rateLimitedStep(retreat, 0.020, 0.010, 0.008);
  EXPECT_NEAR(retreat, 0.00008, 1.0e-12);
  for (int sample = 0; sample < 249; ++sample) {
    retreat = wipe_planner::rateLimitedStep(retreat, 0.020, 0.010, 0.008);
  }
  EXPECT_NEAR(retreat, 0.020, 1.0e-12);
  EXPECT_NEAR(
    wipe_planner::rateLimitedStep(retreat, 0.0, 0.005, 0.10),
    0.0195, 1.0e-12);
}

TEST(ForceProgress, UsesContinuousThrottleBeforePauseBand)
{
  EXPECT_DOUBLE_EQ(wipe_planner::forceProgressScale(0.0, 3.0, 8.0, 0.15), 1.0);
  EXPECT_DOUBLE_EQ(wipe_planner::forceProgressScale(3.0, 3.0, 8.0, 0.15), 1.0);
  EXPECT_NEAR(
    wipe_planner::forceProgressScale(5.5, 3.0, 8.0, 0.15), 0.575, 1.0e-12);
  EXPECT_DOUBLE_EQ(wipe_planner::forceProgressScale(8.0, 3.0, 8.0, 0.15), 0.15);
  EXPECT_DOUBLE_EQ(wipe_planner::forceProgressScale(20.0, 3.0, 8.0, 0.15), 0.15);
}

TEST(WholeBodyPlan, ContactTrajectoryIsReachableAndNonholonomic)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace + "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf",
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
  const auto metrics = planner.evaluateTrajectory(trajectory);
  EXPECT_EQ(metrics.evaluated_points, trajectory.size());
  EXPECT_GE(metrics.min_joint_margin, 0.0);
  EXPECT_GE(metrics.min_normalized_joint_margin, 0.0);
  EXPECT_GT(metrics.min_manipulability, 0.0);
  EXPECT_GE(metrics.min_self_collision_clearance, 0.0);
  EXPECT_TRUE(std::any_of(trajectory.begin(), trajectory.end(),
    [](const auto & point) {return point.in_contact && point.input[0] < -1.0e-4;}));
  const Eigen::Vector3d contact_direction = -planner.surfaceNormal();
  for (const auto & point : trajectory) {
    if (!point.in_contact) {
      continue;
    }
    const Eigen::Vector3d tool_z = planner.frameRotation(point.state, "tool0").col(2);
    EXPECT_GT(tool_z.dot(contact_direction), std::cos(0.121));
    // Contact waypoints keep the configured cleaning yaw (wipe_task.yaml:
    // cleaning_yaw = pi/2); the terminal snap makes it exact.
    EXPECT_NEAR(point.state[2], M_PI_2, 1.0e-8);
  }
}

TEST(CoverageGeometry, RasStrokeIsContinuousOnVerticalBoardPlane)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace + "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf",
    "tool0", source + "/config/wipe_task.yaml");
  EXPECT_EQ(planner.surfaceType(), "vertical");
  EXPECT_EQ(planner.coveragePattern(), "ras");
  const auto targets = planner.coverageTargets();
  ASSERT_GT(targets.size(), 30U);

  // The single-stroke RAS character box is centered on the board center:
  // 0.80 m along the wall (odom Y) x 0.52 m tall (odom Z), all on the plane
  // X = 2.486 (contact.offset 0 traces the board plane itself).
  double x_min = targets.front().x();
  double x_max = targets.front().x();
  double y_min = targets.front().y();
  double y_max = targets.front().y();
  double z_min = targets.front().z();
  double z_max = targets.front().z();
  double max_step = 0.0;
  bool z_increasing = false;
  bool z_decreasing = false;
  for (std::size_t index = 0; index < targets.size(); ++index) {
    x_min = std::min(x_min, targets[index].x());
    x_max = std::max(x_max, targets[index].x());
    y_min = std::min(y_min, targets[index].y());
    y_max = std::max(y_max, targets[index].y());
    z_min = std::min(z_min, targets[index].z());
    z_max = std::max(z_max, targets[index].z());
    EXPECT_NEAR(targets[index].x(), 2.486, 1.0e-9);
    if (index > 0) {
      const double step = (targets[index] - targets[index - 1]).norm();
      EXPECT_GT(step, 1.0e-9);
      max_step = std::max(max_step, step);
      // The R's and A's vertical legs run both up and down the board.
      z_increasing |= targets[index].z() > targets[index - 1].z();
      z_decreasing |= targets[index].z() < targets[index - 1].z();
    }
  }
  EXPECT_NEAR(x_max - x_min, 0.0, 1.0e-9);
  EXPECT_NEAR(y_max - y_min, 0.80, 1.0e-12);
  EXPECT_NEAR(z_max - z_min, 0.52, 1.0e-12);
  EXPECT_LE(max_step, 0.030001);
  EXPECT_TRUE(z_increasing);
  EXPECT_TRUE(z_decreasing);
}

TEST(WholeBodyPlan, StartsAtSurfaceNormalPrecontactAndReachesSurface)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace + "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf",
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
  // The first coverage keypoint is the bottom-left corner of the RAS
  // character box, on the board plane X = 2.486 (contact.offset 0). Force
  // admittance supplies bounded, velocity-limited normal correction online.
  EXPECT_NEAR(first_contact->contact_target.x(), 2.486, 1.0e-9);
  EXPECT_NEAR(first_contact->contact_target.y(), 0.465, 1.0e-9);
  // RAS box: center.z 0.853 minus half the 0.52 m box height.
  EXPECT_NEAR(first_contact->contact_target.z(), 0.593, 1.0e-9);
  EXPECT_GE(first_contact->time, 3.49);
  const Eigen::Vector3d precontact_ee =
    planner.framePosition(trajectory.front().state, "tool0");
  // The pre-contact tool sits 12 cm off the plane along the surface normal,
  // at the same height as the first RAS keypoint.
  EXPECT_NEAR(precontact_ee.z(), 0.593, 1.0e-4);
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

TEST(WholeBodyPlan, FirstFrameIsACompleteSurfaceNormalPrecontactGoal)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace + "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf",
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

TEST(WholeBodyPlan, ConditionedRolloutPreservesTheGivenPrecontactFirstFrame)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace + "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf",
    "tool0", source + "/config/wipe_task.yaml");
  const auto candidates = planner.precontactCandidates();
  ASSERT_GE(candidates.size(), 50U);
  EXPECT_TRUE(std::any_of(candidates.begin(), candidates.end(),
    [&candidates](const Eigen::VectorXd & candidate) {
      return (candidate.head<3>() - candidates.front().head<3>()).norm() > 0.05;
    }));

  bool found_success = false;
  for (const Eigen::VectorXd & precontact : candidates) {
    try {
      wipe_planner::PlanReport report;
      const auto trajectory = planner.planFromPrecontact(precontact, report);
      ASSERT_FALSE(trajectory.empty());
      EXPECT_TRUE(trajectory.front().state.isApprox(precontact, 1.0e-12));
      EXPECT_FALSE(trajectory.front().in_contact);
      EXPECT_DOUBLE_EQ(trajectory.front().time, 0.0);
      const auto contact = std::find_if(trajectory.begin(), trajectory.end(),
        [](const wipe_planner::Waypoint & point) {return point.in_contact;});
      ASSERT_NE(contact, trajectory.end());
      EXPECT_GT(contact->time, 0.0);
      EXPECT_TRUE(std::all_of(contact, trajectory.end(),
        [](const wipe_planner::Waypoint & point) {return point.in_contact;}));
      found_success = true;
      break;
    } catch (const std::runtime_error &) {
      // Some valid pre-contact states are expected to make the future task
      // infeasible; the batch experiment measures exactly that distinction.
    }
  }
  EXPECT_TRUE(found_success);
}

TEST(WholeBodyPlan, ForceCorrectionMovesOnlyArmAlongSurfaceNormal)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace + "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf",
    "tool0", source + "/config/wipe_task.yaml");
  Eigen::VectorXd seed(9);
  seed << 0.75, 2.06, 0.0, 0.0, 1.5707, 0.0, 1.5707, 3.14159, 0.7854;
  wipe_planner::PlanReport report;
  const auto trajectory = planner.plan(seed, report);
  const auto contact = std::find_if(trajectory.begin(), trajectory.end(),
    [](const auto & point) {return point.in_contact;});
  ASSERT_NE(contact, trajectory.end());

  const Eigen::VectorXd corrected = planner.normalOffsetCorrectedState(
    contact->state, 0.010, 0.10);
  EXPECT_TRUE(corrected.head<3>().isApprox(contact->state.head<3>(), 1.0e-12));
  EXPECT_LE((corrected.tail<6>() - contact->state.tail<6>())
    .cwiseAbs().maxCoeff(), 0.100001);
  const Eigen::Vector3d displacement =
    planner.framePosition(corrected, "tool0") -
    planner.framePosition(contact->state, "tool0");
  EXPECT_NEAR(displacement.dot(planner.surfaceNormal()), 0.010, 5.0e-4);
  EXPECT_LT((displacement - planner.surfaceNormal() *
    displacement.dot(planner.surfaceNormal())).norm(), 5.0e-4);
}

TEST(WholeBodyPlan, MeasuredArmAlignmentHoldsBaseAndRespectsJointSpeed)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace + "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf",
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
