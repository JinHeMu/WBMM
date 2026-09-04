#include "wipe_planner/frame_transform.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(FrameTransform, WholeBodyStateRoundTripsAcrossMapAndOdom)
{
  Eigen::VectorXd odom_state(9);
  odom_state << 0.30, -0.40, 2.90, 0.1, -0.2, 0.3, -0.4, 0.5, -0.6;
  const wipe_planner::PlanarTransform map_from_odom{1.20, -0.70, 0.45};
  const Eigen::VectorXd map_state = wipe_planner::transformWholeBodyState(
    odom_state, map_from_odom);
  const Eigen::VectorXd recovered = wipe_planner::transformWholeBodyState(
    map_state, wipe_planner::inversePlanarTransform(map_from_odom));

  EXPECT_NEAR(map_state[0],
    1.20 + std::cos(0.45) * 0.30 + std::sin(0.45) * 0.40, 1.0e-12);
  EXPECT_NEAR(map_state[1],
    -0.70 + std::sin(0.45) * 0.30 - std::cos(0.45) * 0.40, 1.0e-12);
  EXPECT_NEAR(map_state[2], wipe_planner::wrapAngle(2.90 + 0.45), 1.0e-12);
  EXPECT_NEAR(
    wipe_planner::wrapAngle(recovered[2] - odom_state[2]), 0.0, 1.0e-12);
  EXPECT_TRUE(recovered.isApprox(odom_state, 1.0e-12));
  EXPECT_TRUE(map_state.tail<6>().isApprox(odom_state.tail<6>(), 1.0e-12));
}

TEST(FrameTransform, RejectsAStateWithTheWrongContract)
{
  EXPECT_THROW(
    wipe_planner::transformWholeBodyState(
      Eigen::VectorXd::Zero(8), wipe_planner::PlanarTransform{}),
    std::invalid_argument);
}

