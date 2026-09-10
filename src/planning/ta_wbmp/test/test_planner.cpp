#include "ta_wbmp/planner.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <string>

namespace
{
class RejectMiddleStateChecker final :
  public ta_wbmp::WholeBodyStateValidityChecker
{
public:
  ta_wbmp::StateValidityResult check(
    const Eigen::VectorXd & state) const override
  {
    if (state[0] > 0.45 && state[0] < 0.55) {
      return {false, -0.01, "TEST_MIDPOINT_COLLISION"};
    }
    return {true, 1.0, {}};
  }
};

TEST(StateValidity, InterpolatedMotionRejectsInteriorCollision)
{
  Eigen::VectorXd first = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd second = Eigen::VectorXd::Zero(9);
  second[0] = 1.0;
  const RejectMiddleStateChecker checker;
  const auto result = ta_wbmp::checkInterpolatedMotion(
    checker, first, second, 0.1, 0.1, 0.1);
  EXPECT_FALSE(result.valid);
  EXPECT_EQ(result.reason, "TEST_MIDPOINT_COLLISION");
}

TEST(TaskAwarePlanner, DemoPlanSatisfiesAllConstraints)
{
  const std::filesystem::path source_dir(TA_WBMP_SOURCE_DIR);
  const std::filesystem::path workspace_dir(TA_WBMP_WORKSPACE_DIR);
  const std::string task_file =
    (source_dir / "config" / "wipe_demo.yaml").string();
  const std::string urdf_file =
    (workspace_dir / "src" / "robot" / "tracer_jaka_description" /
    "urdf" / "tracer_jaka_zu5.urdf").string();
  ASSERT_TRUE(std::filesystem::exists(task_file));
  ASSERT_TRUE(std::filesystem::exists(urdf_file));

  ta_wbmp::TaskAwarePlanner planner(urdf_file, "tool0", task_file);
  const ta_wbmp::Plan plan = planner.plan();
  EXPECT_TRUE(plan.report.success);
  EXPECT_FALSE(planner.environmentCollisionChecked());
  EXPECT_EQ(plan.report.state_dimension, 9);
  EXPECT_GT(plan.report.feasible_candidate_count, 0);
  EXPECT_GT(plan.task_targets.size(), 10U);
  EXPECT_TRUE(std::all_of(
    plan.report.constraints.begin(), plan.report.constraints.end(),
    [](const auto & item) {return item.second;}));
  EXPECT_TRUE(std::any_of(
    plan.waypoints.begin(), plan.waypoints.end(),
    [](const ta_wbmp::Waypoint & point) {
      return point.phase == ta_wbmp::kPhaseTask;
    }));
  EXPECT_FALSE(planner.visualGeometry(plan.waypoints.front().state).empty());
  const auto collision_spheres = planner.collisionSpheres(
    plan.waypoints.front().state);
  EXPECT_EQ(collision_spheres.size(), 21U);
  EXPECT_TRUE(std::all_of(
    collision_spheres.begin(), collision_spheres.end(), [](const auto & sphere) {
      return sphere.center.allFinite() && sphere.radius > 0.0;
    }));
}

TEST(TaskAwarePlanner, CylindricalCoverageFollowsLocalSurfaceNormals)
{
  const std::filesystem::path source_dir(TA_WBMP_SOURCE_DIR);
  const std::filesystem::path workspace_dir(TA_WBMP_WORKSPACE_DIR);
  const std::string task_file =
    (source_dir / "config" / "curved_wipe_demo.yaml").string();
  const std::string urdf_file =
    (workspace_dir / "src" / "robot" / "tracer_jaka_description" /
    "urdf" / "tracer_jaka_zu5.urdf").string();

  ta_wbmp::TaskAwarePlanner planner(urdf_file, "tool0", task_file);
  const ta_wbmp::Plan plan = planner.plan();
  ASSERT_TRUE(plan.report.success);
  EXPECT_EQ(plan.surface_type, "cylindrical");
  EXPECT_EQ(plan.task_targets.size(), plan.task_normals.size());
  EXPECT_GT(plan.task_targets.size(), 70U);
  EXPECT_LT(plan.task_normals.front().dot(plan.task_normals.back()), 0.60);

  double minimum_z = std::numeric_limits<double>::infinity();
  double maximum_z = -std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < plan.task_targets.size(); ++index) {
    const Eigen::Vector2d radial =
      (plan.task_targets[index] - plan.surface_center).head<2>();
    EXPECT_NEAR(radial.norm(), plan.surface_radius, 1.0e-9);
    EXPECT_NEAR(plan.task_normals[index].head<2>().norm(), 1.0, 1.0e-9);
    minimum_z = std::min(minimum_z, plan.task_targets[index].z());
    maximum_z = std::max(maximum_z, plan.task_targets[index].z());
  }
  EXPECT_NEAR(minimum_z, plan.z_limits[0], 1.0e-9);
  EXPECT_NEAR(maximum_z, plan.z_limits[1], 1.0e-9);
  EXPECT_LT(plan.report.max_contact_position_error, 0.012);
  EXPECT_LT(plan.report.max_tool_axis_error, 0.12);
  EXPECT_LT(plan.report.max_lateral_velocity, 1.0e-5);
}

TEST(TaskTrajectoryGenerator, GeneratesThreeIndependentTaskScenarios)
{
  const std::filesystem::path source_dir(TA_WBMP_SOURCE_DIR);
  const std::array<std::pair<std::string, ta_wbmp::TaskTrajectoryType>, 3> cases{{
    {"table_wipe.yaml", ta_wbmp::TaskTrajectoryType::SURFACE_RASTER},
    {"blackboard_wipe.yaml", ta_wbmp::TaskTrajectoryType::SURFACE_RASTER},
    {"ras_drawing.yaml", ta_wbmp::TaskTrajectoryType::RAS_DRAWING}}};
  for (const auto & value : cases) {
    const ta_wbmp::TaskTrajectory task =
      ta_wbmp::TaskTrajectoryGenerator(
      (source_dir / "config" / value.first).string()).generate();
    EXPECT_EQ(task.type, value.second) << value.first;
    EXPECT_GT(task.points.size(), 10U) << value.first;
    EXPECT_TRUE(std::all_of(
      task.points.begin(), task.points.end(), [](const auto & point) {
        return point.position.allFinite() &&
               std::abs(point.surface_normal.norm() - 1.0) < 1.0e-9 &&
               point.nominal_speed > 0.0;
      })) << value.first;
  }

  const ta_wbmp::TaskTrajectory ras = ta_wbmp::TaskTrajectoryGenerator(
    (source_dir / "config" / "ras_drawing.yaml").string()).generate();
  Eigen::Vector3d minimum = ras.points.front().position;
  Eigen::Vector3d maximum = ras.points.front().position;
  for (const auto & point : ras.points) {
    minimum = minimum.cwiseMin(point.position);
    maximum = maximum.cwiseMax(point.position);
  }
  EXPECT_NEAR(maximum.x() - minimum.x(), 0.90, 1.0e-9);
  EXPECT_NEAR(maximum.z() - minimum.z(), 0.60, 1.0e-9);
}

TEST(TaskTrajectoryGenerator, LoadsUnifiedMpcAndOptionalForceExecution)
{
  const std::filesystem::path source_dir(TA_WBMP_SOURCE_DIR);
  const ta_wbmp::TaskTrajectory task = ta_wbmp::TaskTrajectoryGenerator(
    (source_dir / "config" / "mujoco_table_task.yaml").string()).generate();
  EXPECT_EQ(task.frame_id, "odom");
  EXPECT_FALSE(task.execution.force.enabled);
  EXPECT_EQ(task.execution.force.mode, "constant_force");
  EXPECT_EQ(task.execution.force.force_axis, "z");
  EXPECT_NEAR(task.execution.force.desired_force, 12.0, 1.0e-12);
  EXPECT_NEAR(task.execution.mpc.reference_rate, 20.0, 1.0e-12);
  EXPECT_TRUE(std::all_of(
    task.points.begin(), task.points.end(), [](const auto & point) {
      return point.contact &&
             point.surface_normal.isApprox(Eigen::Vector3d::UnitZ(), 1.0e-12);
    }));
}

TEST(ExtensionInterfaces, CandidateCostAndNavigationEstimateAreInjectable)
{
  ta_wbmp::CandidateMetrics short_safe;
  short_safe.feasible = true;
  short_safe.standoff = 0.8;
  short_safe.min_joint_margin = 0.2;
  short_safe.min_manipulability = 0.05;
  short_safe.min_sigma = 0.1;
  short_safe.navigation_cost_estimate = 1.0;
  ta_wbmp::CandidateMetrics far = short_safe;
  far.navigation_cost_estimate = 3.0;
  ta_wbmp::CandidateCostWeights weights;
  weights.navigation_cost = 2.0;
  const ta_wbmp::WeightedCandidateCost cost(weights);
  EXPECT_LT(cost.evaluate(short_safe), cost.evaluate(far));

  Eigen::VectorXd start = Eigen::VectorXd::Zero(9);
  Eigen::VectorXd goal = Eigen::VectorXd::Zero(9);
  goal[0] = 3.0;
  goal[2] = 1.0;
  const ta_wbmp::Se2NavigationCostEstimator estimator(0.5);
  EXPECT_NEAR(estimator.estimate(start, goal), 3.5, 1.0e-12);
}

TEST(TaskAwarePlanner, ThreeGenericScenariosProduceExecutionContracts)
{
  const std::filesystem::path source_dir(TA_WBMP_SOURCE_DIR);
  const std::filesystem::path workspace_dir(TA_WBMP_WORKSPACE_DIR);
  const std::string urdf_file =
    (workspace_dir / "src" / "robot" / "tracer_jaka_description" /
    "urdf" / "tracer_jaka_zu5.urdf").string();
  for (const std::string task : {
      "table_wipe.yaml", "blackboard_wipe.yaml", "ras_drawing.yaml"})
  {
    ta_wbmp::TaskAwarePlanner planner(
      urdf_file, "tool0", (source_dir / "config" / task).string());
    const ta_wbmp::Plan plan = planner.plan();
    EXPECT_TRUE(plan.report.success) << task;
    EXPECT_EQ(plan.remani_navigation_goal.size(), 9) << task;
    EXPECT_EQ(plan.task_entry_state.size(), 9) << task;
    EXPECT_LT(plan.task_start_index, plan.waypoints.size()) << task;
    EXPECT_LE(plan.execution_start_index, plan.task_start_index) << task;
    ASSERT_LT(plan.execution_start_index, plan.waypoints.size()) << task;
    EXPECT_TRUE(plan.waypoints[plan.execution_start_index].state.isApprox(
      plan.remani_navigation_goal, 1.0e-9)) << task;
    EXPECT_EQ(plan.task_trajectory.points.size(), plan.task_targets.size()) << task;
    EXPECT_EQ(plan.candidate_evaluations.size(),
      static_cast<std::size_t>(plan.report.candidate_count)) << task;
    EXPECT_GT(plan.report.minimum_sigma, 0.0) << task;
  }
}
}  // namespace
