#include "whole_body_force_control/wbmm_conversions.hpp"
#include "whole_body_force_control/wbmm_ros_conversions.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace
{

std::vector<std::string> jointNames()
{
  return {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
}

wbmm::core::WholeBodyState makeState(
  double x, double y, double yaw, double first_joint,
  const std::vector<std::string> & names,
  double stamp,
  wbmm::core::ClockDomain clock)
{
  wbmm::core::WholeBodyState state;
  state.header.frame_id = "odom";
  state.header.stamp = stamp;
  state.header.clock = clock;
  state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
  state.base.x = x;
  state.base.y = y;
  state.base.yaw = yaw;
  state.joints.names = names;
  state.joints.positions.resize(names.size());
  for (std::size_t i = 0; i < names.size(); ++i) {
    state.joints.positions[i] = first_joint + static_cast<double>(i);
  }
  return state;
}

}  // namespace

TEST(WbmmRosConversions, QuaternionRoundTripPreservesRosOrder)
{
  geometry_msgs::msg::Quaternion ros;
  ros.x = 0.1;
  ros.y = 0.2;
  ros.z = 0.3;
  ros.w = 0.9;

  const auto core = whole_body_force_control::quaternionFromRos(ros);
  EXPECT_DOUBLE_EQ(core.w, 0.9);
  EXPECT_DOUBLE_EQ(core.x, 0.1);
  EXPECT_DOUBLE_EQ(core.y, 0.2);
  EXPECT_DOUBLE_EQ(core.z, 0.3);

  const auto back = whole_body_force_control::quaternionToRos(core);
  EXPECT_DOUBLE_EQ(back.x, ros.x);
  EXPECT_DOUBLE_EQ(back.y, ros.y);
  EXPECT_DOUBLE_EQ(back.z, ros.z);
  EXPECT_DOUBLE_EQ(back.w, ros.w);
}

TEST(WbmmRosConversions, MpcObservationConvertsToWholeBodyState)
{
  ocs2_msgs::msg::MpcObservation message;
  message.time = 12.5;
  message.state.value = {
    1.0F, 2.0F, 0.5F, 0.1F, 0.2F, 0.3F, 0.4F, 0.5F, 0.6F};

  const auto state =
    whole_body_force_control::wholeBodyStateFromMpcObservation(
    message, jointNames(), "odom", wbmm::core::ClockDomain::kOcs2Mpc);

  ASSERT_TRUE(state.has_value());
  EXPECT_EQ(state->header.frame_id, "odom");
  EXPECT_DOUBLE_EQ(state->header.stamp, 12.5);
  EXPECT_EQ(state->header.clock, wbmm::core::ClockDomain::kOcs2Mpc);
  EXPECT_EQ(state->base_model, wbmm::core::BaseModel::kDifferentialDrive);
  EXPECT_DOUBLE_EQ(state->base.x, 1.0);
  EXPECT_DOUBLE_EQ(state->base.y, 2.0);
  EXPECT_DOUBLE_EQ(state->base.yaw, 0.5);
  EXPECT_EQ(state->joints.names, jointNames());
  EXPECT_NEAR(state->joints.positions[0], 0.1, 1.0e-6);
  EXPECT_NEAR(state->joints.positions[5], 0.6, 1.0e-6);
  EXPECT_TRUE(wbmm::core::validate(*state));
}

TEST(WbmmRosConversions, MpcObservationRejectsWrongSizeOrBadStamp)
{
  ocs2_msgs::msg::MpcObservation message;
  message.time = 1.0;
  message.state.value = {0.0F, 0.0F, 0.0F};
  EXPECT_FALSE(
    whole_body_force_control::wholeBodyStateFromMpcObservation(
      message, jointNames(), "odom",
      wbmm::core::ClockDomain::kOcs2Mpc).has_value());

  message.state.value = {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  message.time = -1.0;
  EXPECT_FALSE(
    whole_body_force_control::wholeBodyStateFromMpcObservation(
      message, jointNames(), "odom",
      wbmm::core::ClockDomain::kOcs2Mpc).has_value());
}

TEST(WbmmRosConversions, NonFiniteObservationFailsCoreValidation)
{
  ocs2_msgs::msg::MpcObservation message;
  message.time = 1.0;
  message.state.value = {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  message.state.value[3] = std::numeric_limits<float>::quiet_NaN();

  const auto state =
    whole_body_force_control::wholeBodyStateFromMpcObservation(
    message, jointNames(), "odom", wbmm::core::ClockDomain::kOcs2Mpc);
  ASSERT_TRUE(state.has_value());
  EXPECT_FALSE(wbmm::core::validate(*state));
}

TEST(WbmmRosConversions, WrenchUsesFallbackFrameAndConvertsUnits)
{
  geometry_msgs::msg::WrenchStamped message;
  message.header.stamp.sec = 3;
  message.header.stamp.nanosec = 500000000U;
  message.wrench.force.x = 1.5;
  message.wrench.force.y = -2.5;
  message.wrench.force.z = 3.5;
  message.wrench.torque.x = 0.1;
  message.wrench.torque.y = 0.2;
  message.wrench.torque.z = 0.3;

  const auto wrench = whole_body_force_control::wrenchFromRos(
    message, wbmm::core::ClockDomain::kSystem, "tool0");
  ASSERT_TRUE(wrench.has_value());
  EXPECT_EQ(wrench->header.frame_id, "tool0");
  EXPECT_DOUBLE_EQ(wrench->header.stamp, 3.5);
  EXPECT_DOUBLE_EQ(wrench->force.x, 1.5);
  EXPECT_DOUBLE_EQ(wrench->force.y, -2.5);
  EXPECT_DOUBLE_EQ(wrench->force.z, 3.5);
  EXPECT_DOUBLE_EQ(wrench->torque.z, 0.3);
  EXPECT_TRUE(wbmm::core::validate(*wrench));

  EXPECT_FALSE(
    whole_body_force_control::wrenchFromRos(
      message, wbmm::core::ClockDomain::kSystem).has_value());
}

TEST(WbmmRosConversions, WholeBodyTrajectoryConvertsToMpcTargetTrajectories)
{
  const auto names = jointNames();
  wbmm::core::WholeBodyTrajectory trajectory;
  trajectory.trajectory_id = "conversion_test";
  trajectory.environment_revision = 1;
  trajectory.collision_model_revision = 1;

  for (double time : {0.1, 0.2}) {
    wbmm::core::WholeBodyTrajectoryPoint point;
    point.time_from_start = time;
    point.phase = wbmm::core::ExecutionPhase::kExecution;
    point.state = makeState(
      1.0, 2.0, 0.5, 0.1, names, 5.0 + time,
      wbmm::core::ClockDomain::kOcs2Mpc);
    point.feedforward_input =
      whole_body_force_control::makeZeroWholeBodyInput(
      names, 5.0 + time, wbmm::core::ClockDomain::kOcs2Mpc);
    trajectory.points.push_back(std::move(point));
  }

  const auto message =
    whole_body_force_control::toMpcTargetTrajectories(trajectory, 5.0, 8);
  ASSERT_EQ(message.time_trajectory.size(), 2U);
  ASSERT_EQ(message.state_trajectory.size(), 2U);
  ASSERT_EQ(message.input_trajectory.size(), 2U);
  EXPECT_DOUBLE_EQ(message.time_trajectory[0], 5.1);
  EXPECT_DOUBLE_EQ(message.time_trajectory[1], 5.2);
  ASSERT_EQ(message.state_trajectory[0].value.size(), 9U);
  EXPECT_FLOAT_EQ(message.state_trajectory[0].value[0], 1.0F);
  EXPECT_FLOAT_EQ(message.state_trajectory[0].value[3], 0.1F);
  EXPECT_FLOAT_EQ(message.state_trajectory[1].value[8], 5.1F);
  ASSERT_EQ(message.input_trajectory[0].value.size(), 8U);
  for (const float value : message.input_trajectory[0].value) {
    EXPECT_FLOAT_EQ(value, 0.0F);
  }
}

TEST(WbmmRosConversions, EigenStateRoundTripMatchesCoreContract)
{
  const auto names = jointNames();
  Eigen::VectorXd state(9);
  state << 1.0, 2.0, 0.5, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  wbmm::core::Header header;
  header.frame_id = "odom";
  header.clock = wbmm::core::ClockDomain::kSystem;

  const auto core = whole_body_force_control::toCoreState(state, names, header);
  ASSERT_TRUE(core.has_value());
  EXPECT_TRUE(whole_body_force_control::toEigenState(*core).isApprox(state));

  Eigen::VectorXd wrong_size(8);
  wrong_size.setZero();
  EXPECT_FALSE(
    whole_body_force_control::toCoreState(
      wrong_size, names, header).has_value());
}
