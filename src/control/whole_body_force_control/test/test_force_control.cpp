#include "whole_body_force_control/controllers.hpp"
#include "whole_body_force_control/force_processor.hpp"
#include "wbmm_pinocchio/pinocchio_robot_model.hpp"
#include "wbmm_robot_model/wbmm_robot_model.hpp"
#include "wbmm_ros_interfaces/wbmm_conversions.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{

std::string tracerJakaUrdfPath()
{
#ifndef WHOLE_BODY_FORCE_CONTROL_TEST_URDF_FALLBACK
#error "WHOLE_BODY_FORCE_CONTROL_TEST_URDF_FALLBACK must be defined"
#endif
  try {
    const std::string share_path =
      ament_index_cpp::get_package_share_directory("tracer_jaka_description") +
      "/urdf/tracer_jaka_zu5.urdf";
    if (std::ifstream(share_path).good()) {
      return share_path;
    }
  } catch (const std::exception &) {
  }
  return WHOLE_BODY_FORCE_CONTROL_TEST_URDF_FALLBACK;
}

std::shared_ptr<wbmm::pinocchio::PinocchioRobotModel> sharedRobotModel()
{
  static const auto model = []() {
      const auto description =
        wbmm::robot_model::loadRobotDescription(tracerJakaUrdfPath());
      auto config = wbmm::robot_model::RobotModelConfig::defaultsFor(description);
      config.state_base_frame = description.root_link;
      return std::make_shared<wbmm::pinocchio::PinocchioRobotModel>(
        wbmm::pinocchio::KinematicModel::create(
          std::make_shared<const wbmm::robot_model::RobotDescription>(description),
          config));
    }();
  return model;
}

Eigen::VectorXd seedState()
{
  Eigen::VectorXd seed(9);
  seed << 2.027707685, -0.577917895, -0.958716061,
    -0.000000098, 1.900822751, 0.474463874,
    2.337099332, 4.712392653, 0.785416000;
  return seed;
}

wbmm::core::WholeBodyState coreState(
  const Eigen::VectorXd & state, const std::vector<std::string> & joint_names)
{
  wbmm::core::Header header;
  header.frame_id = "odom";
  header.stamp = 0.0;
  const auto converted =
    wbmm::ros_interfaces::toCoreState(state, joint_names, header);
  if (!converted.has_value()) {
    throw std::invalid_argument("test state dimension does not match joint names");
  }
  return *converted;
}

// 对 9D 状态沿 8D 输入做精确一步积分
// (底盘用单轮车闭式解，关节用线性积分)，
// 用于对 frameJacobian 做中心差分校验。
Eigen::VectorXd integrateState(
  const Eigen::VectorXd & state, const Eigen::VectorXd & input, double dt)
{
  Eigen::VectorXd next = state;
  const double linear = input[0];
  const double angular = input[1];
  const double yaw = state[2];
  const double next_yaw = yaw + angular * dt;
  if (std::abs(angular) > 1.0e-12) {
    next[0] += linear / angular * (std::sin(next_yaw) - std::sin(yaw));
    next[1] += -linear / angular * (std::cos(next_yaw) - std::cos(yaw));
  } else {
    next[0] += linear * dt * std::cos(yaw);
    next[1] += linear * dt * std::sin(yaw);
  }
  next[2] = next_yaw;
  next.tail(6) += dt * input.tail(6);
  return next;
}

Eigen::Vector3d log3(const Eigen::Matrix3d & rotation)
{
  const Eigen::AngleAxisd angle_axis(rotation);
  return angle_axis.angle() * angle_axis.axis();
}

Eigen::Matrix3d rotationOf(const wbmm::core::Pose & pose)
{
  const Eigen::Quaterniond quaternion(
    pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  return quaternion.normalized().toRotationMatrix();
}

}  // namespace

TEST(AdmittanceController, PassiveAdmittanceIsBoundedAndReturnsToZero)
{
  whole_body_force_control::AdmittanceController controller(
    3.0, 45.0, 150.0, 0.035);
  for (int i = 0; i < 1000; ++i) {
    controller.update(12.0, 0.01);
  }
  EXPECT_NEAR(controller.offset(), 0.08, 1.0e-6);
  for (int i = 0; i < 1000; ++i) {
    controller.update(0.0, 0.01);
  }
  EXPECT_NEAR(controller.offset(), 0.0, 1.0e-4);
}

TEST(AdmittanceController, ZeroStiffnessFollowsWhileForceIsPresent)
{
  whole_body_force_control::AdmittanceController controller(
    1.0, 10.0, 0.0, 0.5);
  for (int i = 0; i < 200; ++i) {
    controller.update(5.0, 0.01);
  }
  EXPECT_GT(controller.offset(), 0.30);
  EXPECT_NEAR(controller.velocity(), 0.5, 1.0e-9);

  const double held_offset = controller.offset();
  for (int i = 0; i < 200; ++i) {
    controller.update(0.0, 0.01);
  }
  EXPECT_GT(controller.offset(), held_offset);
  EXPECT_NEAR(controller.velocity(), 0.0, 1.0e-6);
}

TEST(AdmittanceController, LimitOffsetStopsIntegratorWindup)
{
  whole_body_force_control::AdmittanceController controller(
    1.0, 10.0, 0.0, 0.5);
  for (int i = 0; i < 200; ++i) {
    controller.update(5.0, 0.01);
  }
  ASSERT_GT(controller.offset(), 0.30);

  // Whole-body kinematics can only realize 0.20 m: shrink the hidden
  // integrator state and stop velocity so a post-release command cannot jump.
  EXPECT_TRUE(controller.limitOffset(0.20));
  EXPECT_NEAR(controller.offset(), 0.20, 1.0e-12);
  EXPECT_NEAR(controller.velocity(), 0.0, 1.0e-12);
  EXPECT_FALSE(controller.limitOffset(0.30));

  // Reversal remains possible through the normal dynamics.
  controller.update(-5.0, 0.01);
  EXPECT_LT(controller.velocity(), 0.0);
}


TEST(CartesianComplianceController, SelectsIndependentSixAxisAdmittance)
{
  using whole_body_force_control::AxisMask6d;
  using whole_body_force_control::Vector6d;
  const AxisMask6d admittance{true, false, true, false, true, false};
  const Vector6d mass = Vector6d::Constant(1.0);
  const Vector6d damping = Vector6d::Constant(20.0);
  const Vector6d stiffness = Vector6d::Constant(100.0);
  const Vector6d max_velocity = Vector6d::Constant(0.5);
  whole_body_force_control::CartesianComplianceController controller(
    admittance, mass, damping, stiffness, max_velocity);

  Vector6d wrench;
  wrench << 2.0, 100.0, 5.0, 100.0, -3.0, 100.0;
  for (int i = 0; i < 1000; ++i) {
    controller.update(wrench, 0.01);
  }
  EXPECT_NEAR(controller.offset()[0], 0.02, 1.0e-5);
  EXPECT_DOUBLE_EQ(controller.offset()[1], 0.0);
  EXPECT_NEAR(controller.offset()[2], 0.05, 1.0e-5);
  EXPECT_DOUBLE_EQ(controller.offset()[3], 0.0);
  EXPECT_NEAR(controller.offset()[4], -0.03, 1.0e-5);
  EXPECT_DOUBLE_EQ(controller.offset()[5], 0.0);
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
  EXPECT_TRUE(target.tail<3>().isApprox(Eigen::Vector3d(-0.5, 0.0, 0.2), 1.0e-12));
}

TEST(EndEffectorPoseTarget, AppliesLocalCorrectionInNominalFrame)
{
  wbmm::core::Pose nominal;
  nominal.header.frame_id = "odom";
  nominal.header.stamp = 0.0;
  nominal.position.x = 1.0;
  nominal.position.y = 2.0;
  nominal.position.z = 0.5;
  const Eigen::Quaterniond nominal_q(
    Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ()));
  nominal.orientation.w = nominal_q.w();
  nominal.orientation.x = nominal_q.x();
  nominal.orientation.y = nominal_q.y();
  nominal.orientation.z = nominal_q.z();

  whole_body_force_control::Vector6d correction =
    whole_body_force_control::Vector6d::Zero();
  correction[0] = 0.10;
  correction[5] = 0.20;

  const auto target = whole_body_force_control::makeEndEffectorPoseTarget(
    nominal, correction, "odom", 0.5);

  const Eigen::Matrix3d nominal_rotation = nominal_q.toRotationMatrix();
  const Eigen::Vector3d expected_position =
    Eigen::Vector3d(1.0, 2.0, 0.5) +
    nominal_rotation * correction.head<3>();
  const Eigen::Matrix3d expected_rotation =
    Eigen::AngleAxisd(
      0.20, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
    nominal_rotation;

  EXPECT_DOUBLE_EQ(target.header.stamp, 0.5);
  EXPECT_EQ(target.header.frame_id, "odom");
  EXPECT_NEAR(target.position.x, expected_position.x(), 1.0e-12);
  EXPECT_NEAR(target.position.y, expected_position.y(), 1.0e-12);
  EXPECT_NEAR(target.position.z, expected_position.z(), 1.0e-12);

  const Eigen::Quaterniond target_q(
    target.orientation.w, target.orientation.x,
    target.orientation.y, target.orientation.z);
  EXPECT_LT(
    (target_q.normalized().toRotationMatrix() - expected_rotation).norm(),
    1.0e-12);
  EXPECT_TRUE(wbmm::core::validate(target).ok);
  EXPECT_THROW(
    whole_body_force_control::makeEndEffectorPoseTarget(
      nominal, correction, "map", 0.5),
    std::invalid_argument);
}

TEST(PinocchioRobotModel, MatchesCoreDimensionAndJointContract)
{
  const auto model = sharedRobotModel();
  EXPECT_EQ(model->stateDimension(), 9U);
  EXPECT_EQ(model->inputDimension(), 8U);
  EXPECT_EQ(model->baseModel(), wbmm::core::BaseModel::kDifferentialDrive);
  EXPECT_EQ(model->jointNames().size(), 6U);
  EXPECT_EQ(model->jointNames().front(), "joint_1");
  EXPECT_EQ(model->jointNames().back(), "joint_6");
  EXPECT_EQ(model->limits().joint_min.size(), 6U);
  EXPECT_EQ(model->limits().joint_max.size(), 6U);
  EXPECT_EQ(model->limits().max_joint_speed.size(), 6U);
  EXPECT_TRUE(model->hasFrame("tool0"));
  EXPECT_FALSE(model->hasFrame("not_a_link"));

  const auto state = coreState(seedState(), model->jointNames());
  std::string reason;
  EXPECT_TRUE(model->validate(state, &reason)) << reason;
}

TEST(PinocchioRobotModel, RejectsJointOutsideLimits)
{
  const auto model = sharedRobotModel();
  auto state = coreState(seedState(), model->jointNames());
  state.joints.positions[1] = 100.0;
  std::string reason;
  EXPECT_FALSE(model->validate(state, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(PinocchioRobotModel, ForwardKinematicsMapsJointsByName)
{
  const auto model = sharedRobotModel();
  const auto seed = seedState();

  Eigen::VectorXd reversed = seed;
  for (int i = 0; i < 6; ++i) {
    reversed[3 + i] = seed[3 + (5 - i)];
  }
  auto reversed_names = model->jointNames();
  std::reverse(reversed_names.begin(), reversed_names.end());

  wbmm::core::Pose ordered_pose;
  wbmm::core::Pose reversed_pose;
  ASSERT_TRUE(
    model->forwardKinematics(
      coreState(seed, model->jointNames()), "tool0", ordered_pose));
  ASSERT_TRUE(
    model->forwardKinematics(
      coreState(reversed, reversed_names), "tool0", reversed_pose));

  EXPECT_TRUE(
    (rotationOf(ordered_pose) - rotationOf(reversed_pose)).norm() < 1.0e-12);
  EXPECT_NEAR(ordered_pose.position.x, reversed_pose.position.x, 1.0e-12);
  EXPECT_NEAR(ordered_pose.position.y, reversed_pose.position.y, 1.0e-12);
  EXPECT_NEAR(ordered_pose.position.z, reversed_pose.position.z, 1.0e-12);

  Eigen::MatrixXd ordered_jacobian(6, 8);
  Eigen::MatrixXd reversed_jacobian(6, 8);
  ASSERT_TRUE(
    model->frameJacobian(
      coreState(seed, model->jointNames()), "tool0", ordered_jacobian));
  ASSERT_TRUE(
    model->frameJacobian(
      coreState(reversed, reversed_names), "tool0", reversed_jacobian));

  EXPECT_TRUE(
    reversed_jacobian.leftCols(2).isApprox(ordered_jacobian.leftCols(2), 1.0e-12));
  const Eigen::MatrixXd reversed_columns =
    ordered_jacobian.rightCols(6).rowwise().reverse();
  EXPECT_TRUE(
    reversed_jacobian.rightCols(6).isApprox(reversed_columns, 1.0e-12));
}

TEST(PinocchioRobotModel, ValidationMapsJointVelocitiesByName)
{
  const auto model = sharedRobotModel();
  auto state = coreState(seedState(), model->jointNames());
  state.joints.velocities.assign(6, 0.0);
  state.joints.velocities[0] = 100.0;  // joint_1 exceeds the URDF speed limit

  auto reversed_names = model->jointNames();
  std::reverse(reversed_names.begin(), reversed_names.end());
  std::reverse(state.joints.positions.begin(), state.joints.positions.end());
  std::reverse(state.joints.velocities.begin(), state.joints.velocities.end());
  state.joints.names = reversed_names;

  std::string reason;
  EXPECT_FALSE(model->validate(state, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(PinocchioRobotModel, JacobianMatchesExactFiniteDifference)
{
  const auto model = sharedRobotModel();
  const auto names = model->jointNames();
  const auto seed = seedState();
  const auto nominal = coreState(seed, names);
  const double step = 1.0e-6;

  std::vector<Eigen::VectorXd> inputs;
  for (int axis = 0; axis < 8; ++axis) {
    Eigen::VectorXd input = Eigen::VectorXd::Zero(8);
    input[axis] = 1.0;
    inputs.push_back(input);
  }
  Eigen::VectorXd combined(8);
  combined << 0.3, -0.2, 0.1, -0.05, 0.04, 0.03, -0.02, 0.01;
  inputs.push_back(combined);

  Eigen::MatrixXd jacobian(6, 8);
  ASSERT_TRUE(model->frameJacobian(nominal, "tool0", jacobian));
  ASSERT_TRUE(jacobian.allFinite());

  for (const auto & input : inputs) {
    wbmm::core::Pose plus_pose;
    wbmm::core::Pose minus_pose;
    ASSERT_TRUE(
      model->forwardKinematics(
        coreState(integrateState(seed, input, step), names), "tool0", plus_pose));
    ASSERT_TRUE(
      model->forwardKinematics(
        coreState(integrateState(seed, input, -step), names), "tool0", minus_pose));

    const Eigen::Vector3d plus_position(
      plus_pose.position.x, plus_pose.position.y, plus_pose.position.z);
    const Eigen::Vector3d minus_position(
      minus_pose.position.x, minus_pose.position.y, minus_pose.position.z);
    const Eigen::Vector3d linear_velocity =
      (plus_position - minus_position) / (2.0 * step);
    const Eigen::Vector3d angular_velocity =
      log3(rotationOf(plus_pose) * rotationOf(minus_pose).transpose()) /
      (2.0 * step);

    const Eigen::Matrix<double, 6, 1> expected = jacobian * input;
    EXPECT_LT((expected.head<3>() - linear_velocity).norm(), 1.0e-7);
    EXPECT_LT((expected.tail<3>() - angular_velocity).norm(), 1.0e-7);
  }
}

TEST(ForceProcessor, AutoTareAndProcessedOutput)
{
  using whole_body_force_control::ForceProcessor;
  using whole_body_force_control::ForceProcessorConfig;
  using whole_body_force_control::Vector6d;

  ForceProcessorConfig config;
  config.tare_samples = 3;
  config.filter_alpha = Vector6d::Ones();
  config.scale = Vector6d::Ones();
  config.hard_wrench_limit = Vector6d::Constant(100.0);
  ForceProcessor processor(config);
  processor.startTare();

  wbmm::core::Wrench raw;
  raw.header.frame_id = "sensor_frame";
  raw.force.x = 10.0;
  const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  auto result = processor.process(raw, "tool0", rotation, translation);
  EXPECT_TRUE(result.taring);
  result = processor.process(raw, "tool0", rotation, translation);
  EXPECT_TRUE(result.taring);
  result = processor.process(raw, "tool0", rotation, translation);
  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.wrench.header.frame_id, "tool0");
  EXPECT_NEAR(result.wrench.force.x, 0.0, 1.0e-12);

  raw.force.x = 12.0;
  result = processor.process(raw, "tool0", rotation, translation);
  ASSERT_TRUE(result.ok);
  EXPECT_NEAR(result.wrench.force.x, 2.0, 1.0e-12);
}

TEST(ForceProcessor, LowPassFilterUsesConfiguredAlpha)
{
  using whole_body_force_control::ForceProcessor;
  using whole_body_force_control::ForceProcessorConfig;
  using whole_body_force_control::Vector6d;

  ForceProcessorConfig config;
  config.filter_alpha = Vector6d::Constant(0.5);
  config.scale = Vector6d::Ones();
  config.hard_wrench_limit = Vector6d::Constant(100.0);
  ForceProcessor processor(config);

  wbmm::core::Wrench raw;
  raw.force.x = 10.0;
  const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  auto result = processor.process(raw, "tool0", rotation, translation);
  ASSERT_TRUE(result.ok);
  EXPECT_NEAR(result.wrench.force.x, 10.0, 1.0e-12);

  raw.force.x = 20.0;
  result = processor.process(raw, "tool0", rotation, translation);
  ASSERT_TRUE(result.ok);
  EXPECT_NEAR(result.wrench.force.x, 15.0, 1.0e-12);
}

TEST(ForceProcessor, DeadbandDoesNotEraseTcpFilterMemory)
{
  using whole_body_force_control::ForceProcessor;
  using whole_body_force_control::ForceProcessorConfig;
  using whole_body_force_control::Vector6d;

  ForceProcessorConfig config;
  config.filter_alpha = Vector6d::Constant(0.5);
  config.hard_wrench_limit = Vector6d::Constant(100.0);
  config.hard_force_norm_limit = 100.0;
  config.force_deadband_n = 1.0;
  ForceProcessor processor(config);

  wbmm::core::Wrench raw;
  raw.header.frame_id = "sensor_frame";
  raw.force.x = 0.5;
  const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  auto result = processor.process(raw, "tool0", rotation, translation);
  ASSERT_TRUE(result.ok);
  EXPECT_DOUBLE_EQ(result.wrench.force.x, 0.0);
  EXPECT_EQ(result.wrench.header.frame_id, "tool0");

  raw.force.x = 2.0;
  result = processor.process(raw, "tool0", rotation, translation);
  ASSERT_TRUE(result.ok);
  EXPECT_NEAR(result.wrench.force.x, 1.25, 1.0e-12);
}

TEST(ForceProcessor, TransformIncludesLeverArmTorque)
{
  using whole_body_force_control::ForceProcessor;
  using whole_body_force_control::ForceProcessorConfig;
  using whole_body_force_control::Vector6d;

  ForceProcessorConfig config;
  config.filter_alpha = Vector6d::Ones();
  config.scale = Vector6d::Ones();
  config.hard_wrench_limit = Vector6d::Constant(100.0);
  ForceProcessor processor(config);

  wbmm::core::Wrench raw;
  raw.header.frame_id = "sensor_frame";
  raw.force.x = 1.0;
  const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d translation(0.0, 0.0, 1.0);

  const auto result = processor.process(raw, "tool0", rotation, translation);
  ASSERT_TRUE(result.ok);
  EXPECT_EQ(result.wrench.header.frame_id, "tool0");
  EXPECT_NEAR(result.wrench.force.x, 1.0, 1.0e-12);
  EXPECT_NEAR(result.wrench.torque.y, 1.0, 1.0e-12);
}

TEST(ForceProcessor, HardLimitIsReported)
{
  using whole_body_force_control::ForceProcessor;
  using whole_body_force_control::ForceProcessorConfig;
  using whole_body_force_control::Vector6d;

  ForceProcessorConfig config;
  config.filter_alpha = Vector6d::Ones();
  config.scale = Vector6d::Ones();
  config.hard_wrench_limit = Vector6d::Constant(5.0);
  ForceProcessor processor(config);

  wbmm::core::Wrench raw;
  raw.force.x = 10.0;
  const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  auto result = processor.process(raw, "tool0", rotation, translation);
  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(result.hard_limit_exceeded);

}

TEST(ForceProcessor, RawForceNormLimitStopsDuringTare)
{
  using whole_body_force_control::ForceProcessor;
  using whole_body_force_control::ForceProcessorConfig;
  using whole_body_force_control::Vector6d;

  ForceProcessorConfig config;
  config.filter_alpha = Vector6d::Ones();
  config.scale = Vector6d::Ones();
  config.hard_wrench_limit = Vector6d::Constant(100.0);
  config.hard_force_norm_limit = 5.0;
  config.tare_samples = 50;
  ForceProcessor processor(config);
  processor.startTare();

  wbmm::core::Wrench raw;
  raw.force.x = 3.0;
  raw.force.y = 4.0;
  const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  auto result = processor.process(raw, "tool0", rotation, translation);
  ASSERT_TRUE(result.taring);
  EXPECT_FALSE(result.hard_limit_exceeded);

  raw.force.x = 3.1;
  raw.force.y = 4.1;
  result = processor.process(raw, "tool0", rotation, translation);
  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(result.hard_limit_exceeded);
  EXPECT_FALSE(result.taring);
}

TEST(ForceProcessor, LoadCompensationSkipsTareAndRemovesGravity)
{
  using whole_body_force_control::ForceProcessor;
  using whole_body_force_control::ForceProcessorConfig;
  using whole_body_force_control::Vector6d;

  ForceProcessorConfig config;
  config.filter_alpha = Vector6d::Ones();
  config.scale = Vector6d::Ones();
  config.hard_wrench_limit = Vector6d::Constant(100.0);
  config.hard_force_norm_limit = 100.0;
  config.load_compensation.enable = true;
  config.load_compensation.gravity_m_s2 = 10.0;
  config.load_compensation.mass_kg = 1.0;
  config.load_compensation.gravity_direction_base =
      Eigen::Vector3d(0.0, 0.0, -1.0);
  config.force_deadband_n = 0.0;
  config.torque_deadband_nm = 0.0;

  ForceProcessor processor(config);
  processor.startTare();  // must be ignored in compensation mode

  wbmm::core::Wrench raw;
  raw.force.z = -10.0;
  const Eigen::Matrix3d target_rotation_source = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d target_to_source = Eigen::Vector3d::Zero();
  const Eigen::Matrix3d source_rotation_base = Eigen::Matrix3d::Identity();

  const auto result = processor.process(
      raw, "tool0", target_rotation_source, target_to_source,
      source_rotation_base);
  ASSERT_TRUE(result.ok);
  EXPECT_FALSE(result.taring);
  EXPECT_NEAR(result.wrench.force.z, 0.0, 1e-12);
}

TEST(ForceProcessor, DeadbandZerosSmallForceAndTorque)
{
  using whole_body_force_control::ForceProcessor;
  using whole_body_force_control::ForceProcessorConfig;
  using whole_body_force_control::Vector6d;

  ForceProcessorConfig config;
  config.filter_alpha = Vector6d::Ones();
  config.scale = Vector6d::Ones();
  config.hard_wrench_limit = Vector6d::Constant(100.0);
  config.hard_force_norm_limit = 100.0;
  config.load_compensation.enable = true;
  config.load_compensation.mass_kg = 0.0;
  config.force_deadband_n = 1.0;
  config.torque_deadband_nm = 0.1;

  ForceProcessor processor(config);

  wbmm::core::Wrench raw;
  raw.force.x = 0.5;
  raw.force.y = 2.0;
  raw.torque.y = 0.05;
  const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  const Eigen::Vector3d translation = Eigen::Vector3d::Zero();

  const auto result = processor.process(raw, "tool0", rotation, translation);
  ASSERT_TRUE(result.ok);
  EXPECT_DOUBLE_EQ(result.wrench.force.x, 0.0);
  EXPECT_DOUBLE_EQ(result.wrench.torque.y, 0.0);
  EXPECT_DOUBLE_EQ(result.wrench.force.y, 2.0);
}
