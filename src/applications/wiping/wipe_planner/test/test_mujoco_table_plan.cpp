#include "wipe_planner/planner.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

TEST(MujocoTaskTable, PhysicalGeometryAndCompletePlanPass)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace +
    "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf",
    "tool0", source + "/config/wipe_task_mujoco_table.yaml");

  EXPECT_EQ(planner.surfaceType(), "horizontal");
  EXPECT_TRUE(planner.surfaceCenter().isApprox(
    Eigen::Vector3d(3.45, 0.0, 0.75), 1.0e-12));

  const auto targets = planner.coverageTargets();
  ASSERT_GT(targets.size(), 8U);
  double x_min = targets.front().x();
  double x_max = targets.front().x();
  double y_min = targets.front().y();
  double y_max = targets.front().y();
  double max_step = 0.0;
  for (std::size_t index = 0; index < targets.size(); ++index) {
    x_min = std::min(x_min, targets[index].x());
    x_max = std::max(x_max, targets[index].x());
    y_min = std::min(y_min, targets[index].y());
    y_max = std::max(y_max, targets[index].y());
    EXPECT_NEAR(targets[index].z(), 0.750, 1.0e-12);
    if (index > 0) {
      max_step = std::max(
        max_step, (targets[index] - targets[index - 1]).norm());
    }
  }
  EXPECT_NEAR(x_min, 2.35, 1.0e-12);
  EXPECT_NEAR(x_max, 2.41, 1.0e-12);
  EXPECT_NEAR(y_min, 0.00, 1.0e-12);
  EXPECT_NEAR(y_max, 0.06, 1.0e-12);
  EXPECT_LE(max_step, 0.030001);
  EXPECT_TRUE(planner.precontactTarget().isApprox(
    Eigen::Vector3d(2.35, 0.0, 0.83), 1.0e-12));

  Eigen::VectorXd seed(9);
  seed << 0.0, 0.0, 0.0,
    0.0, 1.5707, 0.0, 1.5707, 3.14159, 0.785398;
  const auto candidates = planner.precontactCandidates();
  ASSERT_FALSE(candidates.empty());
  wipe_planner::PlanReport report;
  std::vector<wipe_planner::Waypoint> trajectory;
  for (const auto & candidate : candidates) {
    try {
      wipe_planner::PlanReport candidate_report;
      auto candidate_trajectory = planner.planFromPrecontact(
        candidate, candidate_report);
      trajectory = std::move(candidate_trajectory);
      report = candidate_report;
      break;
    } catch (const std::exception &) {
      continue;
    }
  }
  ASSERT_FALSE(trajectory.empty());
  EXPECT_GE(trajectory.size(), targets.size());
  EXPECT_LT(report.max_position_error, 0.0121);
  EXPECT_LT(report.max_axis_error, 0.121);
  EXPECT_LT(report.max_lateral_velocity, 1.0e-6);
  EXPECT_LE(report.max_wheel_speed, 1.5001);
  EXPECT_FALSE(trajectory.front().in_contact);
  EXPECT_TRUE(std::any_of(
    trajectory.begin(), trajectory.end(),
    [](const auto & waypoint) {return waypoint.in_contact;}));

  const Eigen::Vector3d precontact_tool =
    planner.framePosition(trajectory.front().state, "tool0");
  EXPECT_GE(precontact_tool.z(), 0.818);
}
