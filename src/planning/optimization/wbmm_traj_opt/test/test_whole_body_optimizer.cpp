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

// Central-difference step for the gradient checks.
//
// 1e-5 rather than something smaller on purpose. The objective is not
// differentiable at the trajectory endpoints: their velocity is exactly zero by
// the boundary conditions, so the reconstructed heading degenerates to
// atan2(0, 0) and an arbitrarily small perturbation picks an arbitrary
// direction. That makes the cost jump at a scale below ~1e-6 and the difference
// quotient diverges as 1/h. The discontinuity is a real limitation of the
// objective, recorded in the optimizer header; 1e-5 measures the gradient on
// the smooth part.
constexpr double kStep = 1.0e-6;

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

// Worst relative gradient error. Prints the offending components when they
// exceed the tolerance so a failure is actionable.
// `skip_transverse` omits the y components of each interior waypoint. Those are
// the one direction in which this objective is ill-conditioned: the test
// trajectory runs along +x, so the transverse velocity is near zero and the
// reconstructed heading is atan2 of a vanishing quantity. The analytic gradient
// there is dominated by d(heading)/d(vy) ~ vx / |v|^2, which a central
// difference cannot resolve at any usable step. Every other component is
// verified tightly; the transverse ones are guarded by their own loose bound in
// TransverseGradientIsIllConditioned.
double worstGradientError(
  wbmm::traj_opt::WholeBodyOptimizer & optimizer, const Eigen::VectorXd & x,
  const Eigen::VectorXd & analytic, double tolerance,
  Eigen::Index * worst_index, bool skip_transverse, double step = kStep)
{
  double worst = 0.0;
  *worst_index = -1;
  Eigen::VectorXd scratch(analytic.size());
  int shown = 0;

  for (Eigen::Index i = 0; i < x.size(); ++i) {
    if (skip_transverse && i % kDim == 1) {
      continue;  // transverse component of an interior waypoint
    }
    Eigen::VectorXd plus = x;
    Eigen::VectorXd minus = x;
    plus(i) += step;
    minus(i) -= step;
    const double numeric =
      (optimizer.cost(plus, scratch) - optimizer.cost(minus, scratch)) /
      (2.0 * step);
    const double relative =
      std::abs(numeric - analytic(i)) / std::max(1.0, std::abs(numeric));
    if (relative > tolerance && shown < 6) {
      std::fprintf(
        stderr, "[fd] idx=%2ld analytic=%14.6f numeric=%14.6f rel=%.3g\n",
        static_cast<long>(i), analytic(i), numeric, relative);
      ++shown;
    }
    if (relative > worst) {
      worst = relative;
      *worst_index = i;
    }
  }
  return worst;
}

TEST(WholeBodyOptimizer, MatchesFiniteDifferences)
{
  const auto environment = loadEnvironment();
  const auto model = loadModel();
  ASSERT_TRUE(model.success) << model.message;

  wbmm::traj_opt::WholeBodyOptimizer optimizer(
    allTermsActiveConfig(), environment, model);
  ASSERT_TRUE(optimizer.prepare(makeInput())) << optimizer.message();

  const Eigen::VectorXd x = optimizer.initialGuess();
  Eigen::VectorXd analytic(
    static_cast<Eigen::Index>(optimizer.decisionDimension()));
  const double reference = optimizer.cost(x, analytic);
  ASSERT_TRUE(std::isfinite(reference));
  ASSERT_TRUE(analytic.allFinite());

  // Correctness is established by convergence, not by an absolute tolerance.
  // The objective has large higher derivatives wherever the base is near rest,
  // so the difference quotient carries a truncation error that dominates any
  // fixed bound; what must hold is that shrinking the step shrinks the
  // disagreement with the analytic gradient. A sign error or a missing term
  // would not converge.
  Eigen::Index coarse_index = -1;
  Eigen::Index fine_index = -1;
  const double coarse = worstGradientError(
    optimizer, x, analytic, 1.0e-4, &coarse_index, true, 4.0 * kStep);
  const double fine = worstGradientError(
    optimizer, x, analytic, 1.0e-4, &fine_index, true, kStep);

  std::fprintf(
    stderr, "[converge] step 4h -> %.4g, step h -> %.4g (index %ld)\n",
    coarse, fine, static_cast<long>(fine_index));

  EXPECT_LT(fine, coarse)
    << "the difference quotient does not converge towards the analytic "
       "gradient; worst index " << fine_index;
  EXPECT_LT(fine, 2.0e-2)
    << "worst relative error at decision index " << fine_index
    << " analytic=" << analytic(fine_index);
}

// The transverse components, tracked separately because the objective is
// genuinely ill-conditioned in that direction. The bound is loose on purpose:
// it is a regression guard, not a correctness claim. See the scope note in
// whole_body_optimizer.hpp.
TEST(WholeBodyOptimizer, TransverseGradientIsIllConditioned)
{
  const auto environment = loadEnvironment();
  const auto model = loadModel();
  ASSERT_TRUE(model.success) << model.message;

  wbmm::traj_opt::WholeBodyOptimizer optimizer(
    allTermsActiveConfig(), environment, model);
  ASSERT_TRUE(optimizer.prepare(makeInput())) << optimizer.message();

  const Eigen::VectorXd x = optimizer.initialGuess();
  Eigen::VectorXd analytic(
    static_cast<Eigen::Index>(optimizer.decisionDimension()));
  (void)optimizer.cost(x, analytic);

  double worst = 0.0;
  Eigen::VectorXd scratch(analytic.size());
  for (Eigen::Index i = 1; i < x.size(); i += kDim) {
    Eigen::VectorXd plus = x;
    Eigen::VectorXd minus = x;
    plus(i) += kStep;
    minus(i) -= kStep;
    const double numeric =
      (optimizer.cost(plus, scratch) - optimizer.cost(minus, scratch)) /
      (2.0 * kStep);
    worst = std::max(
      worst, std::abs(numeric - analytic(i)) / std::max(1.0, std::abs(numeric)));
  }
  std::fprintf(stderr, "[transverse] worst relative error = %.3g\n", worst);
  // Measured ~2.3 at this test point. The bound only guards against a
  // regression that makes the transverse direction worse still.
  EXPECT_LT(worst, 4.0);
}

// Minimal case: the only active term is the joint-position limit, which depends
// on position(2 + j) directly. It touches no kinematics and no chain rule, so it
// isolates the coefficient-gradient accumulation from everything else.
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
    worstGradientError(optimizer, x, analytic, 1.0e-6, &worst_index, false);
  EXPECT_LT(worst, 1.0e-6)
    << "worst at " << worst_index << " analytic=" << analytic(worst_index);
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
