#include "whole_body_force_control/controllers.hpp"
#include "whole_body_force_control/whole_body_kinematics.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <string>

TEST(AdmittanceController, PassiveAdmittanceIsBoundedAndReturnsToZero)
{
  whole_body_force_control::AdmittanceController controller(
    0.0, 3.0, 45.0, 150.0, 0.08, 0.035, 1.0);
  for (int i = 0; i < 1000; ++i) {
    controller.update(12.0, 0.01);
  }
  EXPECT_NEAR(controller.offset(), 0.08, 1.0e-6);
  for (int i = 0; i < 1000; ++i) {
    controller.update(0.0, 0.01);
  }
  EXPECT_NEAR(controller.offset(), 0.0, 1.0e-4);
}

TEST(AdmittanceController, ConstantForceUsesMeasuredMinusDesiredError)
{
  whole_body_force_control::AdmittanceController controller(
    10.0, 2.0, 30.0, 100.0, 0.05, 0.02, 1.0);
  for (int i = 0; i < 500; ++i) {
    controller.update(10.0, 0.01);
  }
  EXPECT_NEAR(controller.offset(), 0.0, 1.0e-9);
  for (int i = 0; i < 500; ++i) {
    controller.update(5.0, 0.01);
  }
  EXPECT_LT(controller.offset(), -0.01);
  controller.reset(10.0);
  for (int i = 0; i < 500; ++i) {
    controller.update(15.0, 0.01);
  }
  EXPECT_GT(controller.offset(), 0.01);
}

TEST(ForceFollower, TracksForceWithoutDampingTerm)
{
  whole_body_force_control::ForceFollower follower(
    0.0, 150.0, 0.08, 0.035, 1.0);
  for (int i = 0; i < 500; ++i) {
    follower.update(5.0, 0.02);
  }
  EXPECT_NEAR(follower.offset(), 5.0 / 150.0, 1.0e-9);
  for (int i = 0; i < 500; ++i) {
    follower.update(12.0, 0.02);
  }
  EXPECT_NEAR(follower.offset(), 0.08, 1.0e-9);
  for (int i = 0; i < 500; ++i) {
    follower.update(0.0, 0.02);
  }
  EXPECT_NEAR(follower.offset(), 0.0, 1.0e-9);
}

TEST(ForceFollower, PreservesSignedPushAndPullWhenRequested)
{
  whole_body_force_control::ForceFollower follower(
    0.0, 200.0, 0.05, 0.10, 1.0, false);
  for (int i = 0; i < 100; ++i) {
    follower.update(-6.0, 0.01);
  }
  EXPECT_NEAR(follower.offset(), -0.03, 1.0e-9);
  for (int i = 0; i < 100; ++i) {
    follower.update(6.0, 0.01);
  }
  EXPECT_NEAR(follower.offset(), 0.03, 1.0e-9);
}

TEST(ForceFollower, VelocityModeKeepsFollowingWhileForceIsPresent)
{
  whole_body_force_control::ForceFollower follower(
    0.0, 1.0, 1000.0, 0.10, 1.0, false, true, 0.2);
  // A sustained +5 N force should keep increasing offset at max_velocity.
  for (int i = 0; i < 100; ++i) {
    follower.update(5.0, 0.01);
  }
  EXPECT_NEAR(follower.offset(), 0.10, 1.0e-9);
  // Releasing the force stops the robot at the current position (no spring back).
  const double held_offset = follower.offset();
  for (int i = 0; i < 100; ++i) {
    follower.update(0.0, 0.01);
  }
  EXPECT_NEAR(follower.offset(), held_offset, 1.0e-12);
  EXPECT_NEAR(follower.velocity(), 0.0, 1.0e-12);
}

TEST(CartesianComplianceController, SelectsIndependentSixAxisAdmittance)
{
  using whole_body_force_control::AxisMask6d;
  using whole_body_force_control::Vector6d;
  AxisMask6d admittance{true, false, true, false, true, false};
  AxisMask6d constant_force{false, false, true, false, false, false};
  Vector6d desired = Vector6d::Zero();
  desired[2] = 5.0;
  const Vector6d mass = Vector6d::Constant(1.0);
  const Vector6d damping = Vector6d::Constant(20.0);
  const Vector6d stiffness = Vector6d::Constant(100.0);
  const Vector6d max_offset = Vector6d::Constant(0.2);
  const Vector6d max_velocity = Vector6d::Constant(0.5);
  const Vector6d alpha = Vector6d::Ones();
  whole_body_force_control::CartesianComplianceController controller(
    admittance, constant_force, desired, mass, damping, stiffness,
    max_offset, max_velocity, alpha);

  Vector6d wrench;
  wrench << 2.0, 100.0, 5.0, 100.0, -3.0, 100.0;
  for (int i = 0; i < 1000; ++i) {
    controller.update(wrench, 0.01);
  }
  EXPECT_NEAR(controller.offset()[0], 0.02, 1.0e-5);
  EXPECT_DOUBLE_EQ(controller.offset()[1], 0.0);
  EXPECT_NEAR(controller.offset()[2], 0.0, 1.0e-9);
  EXPECT_DOUBLE_EQ(controller.offset()[3], 0.0);
  EXPECT_NEAR(controller.offset()[4], -0.03, 1.0e-5);
  EXPECT_DOUBLE_EQ(controller.offset()[5], 0.0);
}

TEST(CartesianComplianceController, RejectsConstantForceOnRigidAxis)
{
  using whole_body_force_control::AxisMask6d;
  using whole_body_force_control::Vector6d;
  AxisMask6d admittance{true, false, false, false, false, false};
  AxisMask6d constant_force{false, true, false, false, false, false};
  EXPECT_THROW(
    whole_body_force_control::CartesianComplianceController(
      admittance, constant_force, Vector6d::Zero(), Vector6d::Ones(),
      Vector6d::Ones(), Vector6d::Ones(), Vector6d::Ones(),
      Vector6d::Ones(), Vector6d::Ones()),
    std::invalid_argument);
}

TEST(CartesianComplianceController, TransformsForceAndLeverArmTorque)
{
  whole_body_force_control::Vector6d source =
    whole_body_force_control::Vector6d::Zero();
  source[0] = 2.0;
  source[4] = 0.5;
  const Eigen::Matrix3d rotation =
    Eigen::AngleAxisd(
    0.5 * std::acos(-1.0), Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d target_to_source(0.1, 0.0, 0.0);
  const auto target = whole_body_force_control::transformWrench(
    source, rotation, target_to_source);
  EXPECT_TRUE(target.head<3>().isApprox(Eigen::Vector3d(0.0, 2.0, 0.0), 1.0e-12));
  // Rotated source torque is (-0.5, 0, 0); lever arm contributes +0.2 Nm on z.
  EXPECT_TRUE(target.tail<3>().isApprox(Eigen::Vector3d(-0.5, 0.0, 0.2), 1.0e-12));
}

TEST(WholeBodyKinematics, SharesMotionWithoutBaseSideslip)
{
  const std::string workspace = WHOLE_BODY_FORCE_CONTROL_WORKSPACE_DIR;
  whole_body_force_control::WholeBodyKinematics kinematics(
    workspace + "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf",
    "tool0");
  Eigen::VectorXd seed(9);
  seed << 2.027707685, -0.577917895, -0.958716061,
    -0.000000098, 1.900822751, 0.474463874,
    2.337099332, 4.712392653, 0.785416000;
  const Eigen::Vector3d direction(
    std::cos(seed[2]), std::sin(seed[2]), 0.0);
  const Eigen::VectorXd corrected = kinematics.correctedState(
    seed, direction, 0.040, 0.40, 0.030, 0.20);
  const Eigen::Vector2d heading(std::cos(seed[2]), std::sin(seed[2]));
  const Eigen::Vector2d base_delta = corrected.head<2>() - seed.head<2>();
  EXPECT_NEAR(base_delta.dot(heading), 0.016, 1.0e-9);
  EXPECT_NEAR(
    base_delta.x() * heading.y() - base_delta.y() * heading.x(),
    0.0, 1.0e-12);
  const Eigen::Vector3d displacement =
    kinematics.framePosition(corrected) - kinematics.framePosition(seed);
  EXPECT_NEAR(displacement.dot(direction), 0.040, 7.5e-4);
  EXPECT_LT(
    (displacement - direction * displacement.dot(direction)).norm(), 7.5e-4);
}

TEST(WholeBodyKinematics, RealizesSixAxisToolFrameCorrection)
{
  const std::string workspace = WHOLE_BODY_FORCE_CONTROL_WORKSPACE_DIR;
  whole_body_force_control::WholeBodyKinematics kinematics(
    workspace + "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf",
    "tool0");
  Eigen::VectorXd seed(9);
  seed << 2.027707685, -0.577917895, -0.958716061,
    -0.000000098, 1.900822751, 0.474463874,
    2.337099332, 4.712392653, 0.785416000;
  Eigen::Matrix<double, 6, 1> correction;
  correction << 0.004, -0.003, 0.005, 0.008, -0.006, 0.004;
  const Eigen::Vector3d initial_position = kinematics.framePosition(seed);
  const Eigen::Matrix3d initial_rotation = kinematics.frameRotation(seed);
  const Eigen::VectorXd corrected = kinematics.correctedState6D(
    seed, correction, 0.0, 0.03, 0.20);
  const Eigen::Vector3d expected_position =
    initial_position + initial_rotation * correction.head<3>();
  const double angle = correction.tail<3>().norm();
  const Eigen::Matrix3d expected_rotation = initial_rotation *
    Eigen::AngleAxisd(angle, correction.tail<3>() / angle).toRotationMatrix();

  EXPECT_LT(
    (kinematics.framePosition(corrected) - expected_position).norm(),
    1.0e-3);
  EXPECT_LT(
    (kinematics.frameRotation(corrected) - expected_rotation).norm(),
    2.0e-3);
  EXPECT_TRUE(corrected.head<3>().isApprox(seed.head<3>(), 1.0e-12));
}
