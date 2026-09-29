// Verifies the whole-body optimizer.
//
// The important test is MatchesFiniteDifferences: the objective is a pure
// function of the decision vector, so its analytic gradient can be compared
// against central differences. Every penalty margin is widened on purpose so
// that the obstacle, ground and self-collision terms are all active at the test
// point; otherwise those gradient paths would silently go untested.

#include <wbmm_traj_opt/whole_body_optimizer.hpp>

#include <wbmm_collision/urdf_collision_model.hpp>
#include <wbmm_environment/esdf_loader.hpp>

#include <Eigen/Core>

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

namespace
{

constexpr int kDim = 8;

constexpr double kStep = 1.0e-5;

const std::vector<std::string> kJointNames = {
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

std::shared_ptr<const wbmm::environment::EsdfGrid> loadEnvironment()
{
  const auto loaded = wbmm::environment::NpzEsdfLoader::load(WBMM_TEST_ESDF);
  EXPECT_EQ(loaded.status, wbmm::environment::LoadStatus::kSuccess)
    << loaded.message;
  return loaded.grid;
}

wbmm::collision::UrdfCollisionModel loadModel()
{
  return wbmm::collision::loadUrdfCollisionModel(
    WBMM_TEST_URDF, kJointNames, "base_link");
}

wbmm::traj_opt::OptimizerConfig baseConfig()
{
  wbmm::traj_opt::OptimizerConfig config;
  config.joint_min =
    (Eigen::VectorXd(6) << -2.0, -1.4, -3.0, -1.4, -2.0, -2.0).finished();
  config.joint_max =
    (Eigen::VectorXd(6) << 2.0, 4.6, 3.0, 4.6, 2.0, 2.0).finished();
  config.max_base_speed = 0.5;
  config.max_joint_speed = 1.57;
  config.max_joint_acceleration = 3.14;
  config.samples_per_piece = 6;
  config.max_iterations = 40;
  return config;
}

// A three-piece rest-to-rest trajectory along +x with the arm held at zero.
wbmm::traj_opt::OptimizerInput makeInput()
{
  wbmm::traj_opt::OptimizerInput input;
  input.header.frame_id = "map";
  input.gear = 1;
  input.joint_names = kJointNames;

  input.head_state = Eigen::MatrixXd::Zero(kDim, 4);
  input.tail_state = Eigen::MatrixXd::Zero(kDim, 4);
  // 1.0 rather than 1.2: at 1.2 the folded arm's joint_3 sphere reaches into an
  // obstacle, so the endpoint itself is in collision and the safety sweep
  // rightly refuses the result. That is a property of the goal, not of the
  // optimizer.
  input.tail_state(0, 0) = 1.0;

  input.inner_points = Eigen::MatrixXd::Zero(kDim, 2);
  input.inner_points(0, 0) = 0.35;
  input.inner_points(0, 1) = 0.70;

  input.durations = (Eigen::VectorXd(3) << 1.5, 1.5, 1.5).finished();
  return input;
}

// Widens every margin so each penalty is active at the test point.
wbmm::traj_opt::OptimizerConfig allTermsActiveConfig()
{
  auto config = baseConfig();
  config.obstacle_margin = 0.6;
  config.ground_margin = 0.5;
  config.self_collision_margin = 0.8;
  return config;
}

// Check every waypoint coordinate AND virtual duration, including transverse
// motion. No skipped components or deliberately loose singularity allowance.
double worstGradientError(wbmm::traj_opt::WholeBodyOptimizer &optimizer,
                          const Eigen::VectorXd &x,
                          const Eigen::VectorXd &analytic, double tolerance,
                          Eigen::Index *worst_index, bool,
                          double step = kStep) {
  double worst = 0;
  *worst_index = 0;
  Eigen::VectorXd scratch;
  for (Eigen::Index i = 0; i < x.size(); ++i) {
    auto plus = x, minus = x;
    plus(i) += step;
    minus(i) -= step;
    const double numeric =
        (optimizer.cost(plus, scratch) - optimizer.cost(minus, scratch)) /
        (2 * step);
    const double relative =
        std::abs(numeric - analytic(i)) /
        std::max({1.0, std::abs(numeric), std::abs(analytic(i))});
    if (relative > worst) {
      worst = relative;
      *worst_index = i;
    }
    if (relative > tolerance)
      std::fprintf(stderr, "[fd] %ld analytic=%g numeric=%g relative=%g\n",
                   long(i), analytic(i), numeric, relative);
  }
  return worst;
}

TEST(WholeBodyOptimizer, AllWaypointAndTimeGradientsMatchFiniteDifferences) {
  auto config = allTermsActiveConfig();
  config.max_yaw_rate = .03;
  config.max_base_acceleration = .05;
  wbmm::traj_opt::WholeBodyOptimizer optimizer(config, loadEnvironment(),
                                               loadModel());
  auto input = makeInput();
  input.inner_points(1, 0) = .08;
  input.inner_points(1, 1) = -.04;
  ASSERT_TRUE(optimizer.prepare(input)) << optimizer.message();
  auto x = optimizer.initialGuess();
  Eigen::VectorXd gradient;
  ASSERT_TRUE(std::isfinite(optimizer.cost(x, gradient)));
  ASSERT_EQ(x.size(), 19);
  Eigen::Index worst;
  EXPECT_LT(worstGradientError(optimizer, x, gradient, 2e-3, &worst, false),
            2e-3)
      << worst;
}

TEST(WholeBodyOptimizer, RestEndpointsAndTransverseGradientsAreStable) {
  wbmm::traj_opt::WholeBodyOptimizer optimizer(allTermsActiveConfig(),
                                               loadEnvironment(), loadModel());
  ASSERT_TRUE(optimizer.prepare(makeInput()));
  auto x = optimizer.initialGuess();
  Eigen::VectorXd g;
  (void)optimizer.cost(x, g);
  Eigen::Index worst;
  EXPECT_LT(worstGradientError(optimizer, x, g, 2e-3, &worst, false), 2e-3)
      << worst;
}

TEST(WholeBodyOptimizer, FiniteDifferencesJointLimitOnly)
{
  const auto environment = loadEnvironment();
  const auto model = loadModel();
  ASSERT_TRUE(model.success) << model.message;

  auto config = baseConfig();
  config.obstacle_weight = 0.0;
  config.ground_weight = 0.0;
  config.self_collision_weight = 0.0;
  config.feasibility_weight = 100.0;
  config.joint_max(0) = 0.2;  // joint_1's upper limit, so the penalty is active

  wbmm::traj_opt::WholeBodyOptimizer optimizer(config, environment, model);
  auto input = makeInput();
  input.inner_points(2, 0) = 0.5;  // joint_1 of the first interior waypoint
  ASSERT_TRUE(optimizer.prepare(input)) << optimizer.message();

  const Eigen::VectorXd x = optimizer.initialGuess();
  Eigen::VectorXd analytic(
    static_cast<Eigen::Index>(optimizer.decisionDimension()));
  (void)optimizer.cost(x, analytic);

  Eigen::Index worst_index = -1;
  const double worst =
      worstGradientError(optimizer, x, analytic, 2.0e-4, &worst_index, false);
  EXPECT_LT(worst, 2.0e-4) << "worst at " << worst_index
                           << " analytic=" << analytic(worst_index);
}

TEST(WholeBodyOptimizer, ReducesTheCost)
{
  const auto environment = loadEnvironment();
  const auto model = loadModel();
  ASSERT_TRUE(model.success) << model.message;

  wbmm::traj_opt::WholeBodyOptimizer optimizer(baseConfig(), environment, model);
  ASSERT_TRUE(optimizer.prepare(makeInput())) << optimizer.message();

  const auto result = optimizer.optimize();
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_LT(result.final_cost, result.initial_cost);
  EXPECT_GT(result.evaluations, 1);
  EXPECT_FALSE(result.trajectory.points.empty());
  EXPECT_NEAR(result.trajectory.points.back().state.base.x, 1.0, 1e-6);
}

TEST(WholeBodyOptimizer, OptimizedTrajectoryIsPublishable)
{
  const auto environment = loadEnvironment();
  const auto model = loadModel();
  ASSERT_TRUE(model.success) << model.message;

  wbmm::traj_opt::WholeBodyOptimizer optimizer(baseConfig(), environment, model);
  ASSERT_TRUE(optimizer.prepare(makeInput())) << optimizer.message();
  const auto result = optimizer.optimize();
  ASSERT_TRUE(result.success) << result.message;

  const auto & points = result.trajectory.points;
  ASSERT_GE(points.size(), 2U);
  for (std::size_t i = 0; i < points.size(); ++i) {
    EXPECT_EQ(points[i].state.header.frame_id, "map");
    EXPECT_EQ(points[i].state.joints.names, kJointNames);
    ASSERT_TRUE(points[i].feedforward_input.has_value());
  }
  for (std::size_t i = 1; i < points.size(); ++i) {
    EXPECT_GT(points[i].time_from_start, points[i - 1U].time_from_start);
  }
  // The safety sweep runs on the optimized trajectory before it is published.
  EXPECT_TRUE(result.safe) << result.message;
}

TEST(WholeBodyOptimizer, RejectsMalformedInput)
{
  const auto environment = loadEnvironment();
  const auto model = loadModel();
  ASSERT_TRUE(model.success) << model.message;

  wbmm::traj_opt::WholeBodyOptimizer optimizer(baseConfig(), environment, model);

  // No environment.
  wbmm::traj_opt::WholeBodyOptimizer no_environment(
    baseConfig(), nullptr, model);
  EXPECT_FALSE(no_environment.prepare(makeInput()));

  // Wrong boundary shape.
  auto bad_state = makeInput();
  bad_state.head_state = Eigen::MatrixXd::Zero(3, 4);
  EXPECT_FALSE(optimizer.prepare(bad_state));

  // inner_points must have one column fewer than the durations.
  auto bad_inner = makeInput();
  bad_inner.inner_points = Eigen::MatrixXd::Zero(kDim, 5);
  EXPECT_FALSE(optimizer.prepare(bad_inner));

  // Non-positive duration.
  auto bad_duration = makeInput();
  bad_duration.durations(1) = 0.0;
  EXPECT_FALSE(optimizer.prepare(bad_duration));

  // Invalid gear.
  auto bad_gear = makeInput();
  bad_gear.gear = 0;
  EXPECT_FALSE(optimizer.prepare(bad_gear));

  // Joint count must match the collision model.
  auto bad_joints = makeInput();
  bad_joints.joint_names = {"joint_1"};
  EXPECT_FALSE(optimizer.prepare(bad_joints));

  // An invalid collision model must be reported, not dereferenced.
  auto invalid_model = model;
  invalid_model.success = false;
  wbmm::traj_opt::WholeBodyOptimizer no_model(
    baseConfig(), environment, invalid_model);
  EXPECT_FALSE(no_model.prepare(makeInput()));
}

TEST(WholeBodyOptimizer, CostIsFiniteAwayFromTheStartPoint)
{
  const auto environment = loadEnvironment();
  const auto model = loadModel();
  ASSERT_TRUE(model.success) << model.message;

  wbmm::traj_opt::WholeBodyOptimizer optimizer(
    allTermsActiveConfig(), environment, model);
  ASSERT_TRUE(optimizer.prepare(makeInput())) << optimizer.message();

  Eigen::VectorXd gradient(
    static_cast<Eigen::Index>(optimizer.decisionDimension()));

  // A perturbed decision vector must still produce a finite cost and gradient,
  // otherwise the line search would chase a NaN.
  Eigen::VectorXd x = optimizer.initialGuess();
  x.array() += 0.3;
  const double value = optimizer.cost(x, gradient);
  EXPECT_TRUE(std::isfinite(value));
  EXPECT_TRUE(gradient.allFinite());
}

TEST(WholeBodyOptimizer, DoesNotReturnCollidingCandidateAsSuccess)
{
  auto data = loadEnvironment()->data();
  std::fill(data.esdf.begin(), data.esdf.end(), -1.0F);
  const auto environment = std::make_shared<wbmm::environment::EsdfGrid>(std::move(data));
  const auto model = loadModel();
  ASSERT_TRUE(model.success);
  auto input = makeInput();
  // Every voxel is occupied, so no waypoint refinement can create a safe path.
  wbmm::traj_opt::WholeBodyOptimizer optimizer(baseConfig(), environment, model);
  ASSERT_TRUE(optimizer.prepare(input));
  const auto result = optimizer.optimize();
  EXPECT_FALSE(result.safe);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.trajectory.points.empty());
}

}  // namespace

TEST(OptimizerTiming, SinglePieceCanOptimizeTimeWithoutWaypoints) {
  auto input = makeInput();
  input.inner_points.resize(8, 0);
  input.durations = Eigen::VectorXd::Constant(1, 12.0);
  auto config = baseConfig();
  config.max_solve_time = 5;
  wbmm::traj_opt::WholeBodyOptimizer optimizer(config, loadEnvironment(),
                                               loadModel());
  ASSERT_TRUE(optimizer.prepare(input));
  auto result = optimizer.optimize();
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LT(result.final_cost, result.initial_cost);
  ASSERT_EQ(result.durations.size(), 1);
  EXPECT_LT(result.durations.sum(), 12.0);
  EXPECT_GT(result.durations(0), config.min_piece_duration);
}
TEST(OptimizerTiming, RejectsWrongJointOrderAndFailedReprepare) {
  wbmm::traj_opt::WholeBodyOptimizer optimizer(baseConfig(), loadEnvironment(),
                                               loadModel());
  auto input = makeInput();
  ASSERT_TRUE(optimizer.prepare(input));
  std::swap(input.joint_names[0], input.joint_names[1]);
  EXPECT_FALSE(optimizer.prepare(input));
  EXPECT_FALSE(optimizer.optimize().success);
}

TEST(OptimizerTiming, RefinesInteriorGeometryAndDurationsTogether) {
  auto input = makeInput();
  input.inner_points(1, 0) = .08;
  input.inner_points(1, 1) = -.04;
  auto config = baseConfig();
  config.max_solve_time = 5;
  config.max_iterations = 100;
  wbmm::traj_opt::WholeBodyOptimizer optimizer(config, loadEnvironment(),
                                               loadModel());
  ASSERT_TRUE(optimizer.prepare(input));
  const auto result = optimizer.optimize();
  ASSERT_TRUE(result.success) << result.message;
  EXPECT_LT(result.final_cost, result.initial_cost);
  EXPECT_GT((result.inner_points - input.inner_points).norm(), 1e-4);
  EXPECT_GT((result.durations - input.durations).norm(), 1e-3);
}
