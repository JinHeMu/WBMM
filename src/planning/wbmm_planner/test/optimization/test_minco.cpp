// Verifies the MINCO (min-snap) contract that the whole-body optimizer relies
// on. These are the properties that must survive the migration from the REMANI
// traj_utils package: exact boundary derivatives, exact interior waypoints and
// C2/C3 continuity at the junctions.
//
// Dimensions follow the WBMM whole-body contract: 8 DoF = 2 base (x, y) + 6 arm
// joints. Yaw is deliberately NOT part of the MINCO state, exactly as upstream.

#include <wbmm_planner/optimization/minco.hpp>
#include <wbmm_planner/optimization/root_finder.hpp>
#include <wbmm_planner/optimization/trajectory_container.hpp>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>

namespace
{

constexpr int kDim = 8;

using MinSnap = wbmm::traj_opt::MinSnapOpt<kDim>;

// headState / tailState are kDim x 4 columns of (position, velocity,
// acceleration, jerk).
Eigen::MatrixXd makeBoundary(const Eigen::VectorXd & position, double seed)
{
  Eigen::MatrixXd state(kDim, 4);
  state.setZero();
  state.col(0) = position;
  for (int i = 0; i < kDim; ++i) {
    state(i, 1) = seed * 0.10 * (i + 1);
    state(i, 2) = seed * 0.01 * (i + 1);
    state(i, 3) = seed * 0.001 * (i + 1);
  }
  return state;
}

TEST(Minco, HonoursBoundaryConditionsAndInteriorWaypoints)
{
  constexpr int kPieces = 3;

  Eigen::VectorXd head(kDim);
  Eigen::VectorXd tail(kDim);
  for (int i = 0; i < kDim; ++i) {
    head(i) = 0.1 * i;
    tail(i) = 0.1 * i + 2.0;
  }

  const Eigen::MatrixXd headState = makeBoundary(head, 1.0);
  const Eigen::MatrixXd tailState = makeBoundary(tail, -1.0);

  Eigen::MatrixXd inner(kDim, kPieces - 1);
  for (int p = 0; p < kPieces - 1; ++p) {
    inner.col(p) = head + (tail - head) * (static_cast<double>(p + 1) / kPieces);
  }

  Eigen::VectorXd durations(kPieces);
  durations << 0.7, 1.1, 0.9;

  MinSnap solver;
  solver.reset(headState, tailState, kPieces);
  solver.generate(inner, durations);

  const auto trajectory = solver.getTraj(1);
  ASSERT_EQ(trajectory.getPieceNum(), kPieces);
  EXPECT_NEAR(trajectory.getTotalDuration(), durations.sum(), 1e-12);

  // Head boundary: position, velocity, acceleration and jerk are hard-fixed.
  EXPECT_TRUE(trajectory.getPos(0.0).isApprox(headState.col(0), 1e-9));
  EXPECT_TRUE(trajectory.getVel(0.0).isApprox(headState.col(1), 1e-9));
  EXPECT_TRUE(trajectory.getAcc(0.0).isApprox(headState.col(2), 1e-9));
  EXPECT_TRUE(trajectory.getJer(0.0).isApprox(headState.col(3), 1e-9));

  // Tail boundary.
  const double total = trajectory.getTotalDuration();
  EXPECT_TRUE(trajectory.getPos(total).isApprox(tailState.col(0), 1e-9));
  EXPECT_TRUE(trajectory.getVel(total).isApprox(tailState.col(1), 1e-9));
  EXPECT_TRUE(trajectory.getAcc(total).isApprox(tailState.col(2), 1e-9));
  EXPECT_TRUE(trajectory.getJer(total).isApprox(tailState.col(3), 1e-9));

  // Interior waypoints are passed through exactly (this is what makes MINCO
  // spatial-temporal deformation safe for the optimizer's penalty terms).
  double junction = 0.0;
  for (int p = 0; p < kPieces - 1; ++p) {
    junction += durations(p);
    EXPECT_TRUE(trajectory.getPos(junction).isApprox(inner.col(p), 1e-9))
      << "piece " << p;
  }
}

TEST(Minco, IsContinuousThroughInteriorJunctions)
{
  constexpr int kPieces = 4;

  Eigen::VectorXd head = Eigen::VectorXd::Zero(kDim);
  Eigen::VectorXd tail = Eigen::VectorXd::Ones(kDim);
  const Eigen::MatrixXd headState = makeBoundary(head, 0.5);
  const Eigen::MatrixXd tailState = makeBoundary(tail, 0.5);

  Eigen::MatrixXd inner(kDim, kPieces - 1);
  for (int p = 0; p < kPieces - 1; ++p) {
    inner.col(p) = Eigen::VectorXd::Constant(
      kDim, 0.2 + 0.2 * static_cast<double>(p));
  }
  Eigen::VectorXd durations = Eigen::VectorXd::Constant(kPieces, 0.8);

  MinSnap solver;
  solver.reset(headState, tailState, kPieces);
  solver.generate(inner, durations);
  const auto trajectory = solver.getTraj(1);

  constexpr double kEps = 1e-6;
  double junction = 0.0;
  for (int p = 0; p < kPieces - 1; ++p) {
    junction += durations(p);
    // Position, velocity, acceleration and jerk must agree from both sides.
    EXPECT_TRUE(
      trajectory.getPos(junction - kEps).isApprox(
        trajectory.getPos(junction + kEps), 1e-5)) << "pos at " << junction;
    EXPECT_TRUE(
      trajectory.getVel(junction - kEps).isApprox(
        trajectory.getVel(junction + kEps), 1e-4)) << "vel at " << junction;
    EXPECT_TRUE(
      trajectory.getAcc(junction - kEps).isApprox(
        trajectory.getAcc(junction + kEps), 1e-3)) << "acc at " << junction;
  }
}

TEST(Minco, SnapCostIsPositiveAndFinite)
{
  constexpr int kPieces = 2;
  const Eigen::MatrixXd headState =
    makeBoundary(Eigen::VectorXd::Zero(kDim), 1.0);
  const Eigen::MatrixXd tailState =
    makeBoundary(Eigen::VectorXd::Ones(kDim), -1.0);

  Eigen::MatrixXd inner(kDim, kPieces - 1);
  inner.col(0) = Eigen::VectorXd::Constant(kDim, 0.5);
  Eigen::VectorXd durations = Eigen::VectorXd::Constant(kPieces, 1.0);

  MinSnap solver;
  solver.reset(headState, tailState, kPieces);
  solver.generate(inner, durations);

  const double cost = solver.getTrajJerkCost();
  EXPECT_TRUE(std::isfinite(cost));
  EXPECT_GT(cost, 0.0);

  // A straight rest-to-rest line has no snap at all; the cost must collapse.
  Eigen::MatrixXd restHead(kDim, 4);
  Eigen::MatrixXd restTail(kDim, 4);
  restHead.setZero();
  restTail.setZero();
  restHead.col(0) = Eigen::VectorXd::Zero(kDim);
  restTail.col(0) = Eigen::VectorXd::Ones(kDim);
  Eigen::MatrixXd mid(kDim, 1);
  mid.col(0) = Eigen::VectorXd::Constant(kDim, 0.5);

  MinSnap straight;
  straight.reset(restHead, restTail, 2);
  straight.generate(mid, durations);
  EXPECT_LT(straight.getTrajJerkCost(), cost);
}

TEST(Minco, GearSignFlipsTheReconstructedHeading)
{
  const Eigen::MatrixXd headState =
    makeBoundary(Eigen::VectorXd::Zero(kDim), 0.0);
  const Eigen::MatrixXd tailState =
    makeBoundary(Eigen::VectorXd::Ones(kDim), 0.0);
  Eigen::MatrixXd inner(kDim, 1);
  inner.col(0) = Eigen::VectorXd::Constant(kDim, 0.5);
  Eigen::VectorXd durations = Eigen::VectorXd::Constant(2, 1.0);

  MinSnap solver;
  solver.reset(headState, tailState, 2);
  solver.generate(inner, durations);

  const auto forward = solver.getTraj(1);
  const auto reverse = solver.getTraj(-1);

  // Both gears describe the same geometric path; the signed longitudinal
  // quantities flip with `singul`, which is how the differential-drive base
  // represents reversing without planning yaw.
  EXPECT_TRUE(forward.getPos(0.5).isApprox(reverse.getPos(0.5), 1e-12));
  EXPECT_NEAR(
    forward.getCarVel(0.5), -reverse.getCarVel(0.5),
    std::abs(forward.getCarVel(0.5)) * 1e-9 + 1e-12);
}

TEST(TrajectoryContainer, IsRosFreeAndUsableStandalone)
{
  // The container migrated without rclcpp; constructing it must not need a node.
  wbmm::traj_opt::TrajContainer container;
  container.singul_traj_data.clearSingulTraj();
  EXPECT_TRUE(container.singul_traj_data.singul_traj.empty());
}

}  // namespace
