#include "wipe_planner/planner.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

TEST(RealFrontBoard, GeometryAndCompletePlanPassWithTheRealUrdf)
{
  const std::string source = WIPE_PLANNER_SOURCE_DIR;
  const std::string workspace = WIPE_PLANNER_WORKSPACE_DIR;
  wipe_planner::Planner planner(
    workspace +
    "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf",
    "tool0", source + "/config/wipe_task_real_front.yaml");

  EXPECT_EQ(planner.surfaceType(), "vertical");
  EXPECT_EQ(planner.coveragePattern(), "raster");
  EXPECT_TRUE(planner.surfaceCenter().isApprox(
    Eigen::Vector3d(2.0, 0.0, 0.55), 1.0e-12));

  const auto targets = planner.coverageTargets();
  ASSERT_GT(targets.size(), 250U);
  double y_min = targets.front().y();
  double y_max = targets.front().y();
  double z_min = targets.front().z();
  double z_max = targets.front().z();
  double max_step = 0.0;
  for (std::size_t index = 0; index < targets.size(); ++index) {
    EXPECT_NEAR(targets[index].x(), 1.95, 1.0e-12);
    y_min = std::min(y_min, targets[index].y());
    y_max = std::max(y_max, targets[index].y());
    z_min = std::min(z_min, targets[index].z());
    z_max = std::max(z_max, targets[index].z());
    if (index > 0) {
      max_step = std::max(
        max_step, (targets[index] - targets[index - 1]).norm());
    }
  }
  EXPECT_NEAR(y_max - y_min, 0.90, 1.0e-12);
  EXPECT_NEAR(z_max - z_min, 0.40, 1.0e-12);
  EXPECT_LE(max_step, 0.030001);

  Eigen::VectorXd seed(9);
  seed << 0.0, 0.0, 0.0,
    0.0, 1.5707, 0.0, 1.5707, 3.14159, 0.785398;
  wipe_planner::PlanReport report;
  const auto trajectory = planner.plan(seed, report);
  ASSERT_FALSE(trajectory.empty());
  EXPECT_GE(trajectory.size(), targets.size());
  EXPECT_GT(report.hybrid_expanded_nodes, 0);
  EXPECT_LT(report.max_position_error, 0.0121);
  EXPECT_LT(report.max_axis_error, 0.121);
  EXPECT_LT(report.max_lateral_velocity, 1.0e-6);
  EXPECT_LE(report.max_wheel_speed, 1.5001);
  EXPECT_FALSE(trajectory.front().in_contact);
  EXPECT_TRUE(std::any_of(
    trajectory.begin(), trajectory.end(),
    [](const auto & waypoint) {return waypoint.in_contact;}));

  // REMANI receives only this first pre-contact state.  Keep its tool target
  // outside the conservative 0.085 m sphere + 0.10 m manipulator margin used
  // by REMANI against the board ESDF; the later fixed-base approach remains a
  // WipePlanner-owned part of the trajectory.
  const Eigen::Vector3d precontact_tool =
    planner.framePosition(trajectory.front().state, "tool0");
  const double board_clearance = planner.surfaceNormal().dot(
    precontact_tool - planner.surfaceCenter());
  EXPECT_GE(board_clearance, 0.185);

  // Moving only the tool backward folds the arm into the base.  The selected
  // whole-body pre-contact therefore keeps the base at least 0.90 m from the
  // board as configured, and the complete plan above verifies the approach
  // and coverage remain free of URDF self/base collision.
  const double base_standoff = planner.surfaceNormal().head<2>().dot(
    trajectory.front().state.head<2>() - planner.surfaceCenter().head<2>());
  EXPECT_GE(base_standoff, 0.90 - 1.0e-9);
}
