/******************************************************************************
 * WBMM 自己的 OCS2 问题定义基线测试。
 *
 * 覆盖：
 *   - 轮式移动机械臂的 9D/8D/6 维模型合同；
 *   - Pinocchio 映射（状态、速度、Jacobian 维度与差速底盘旋转）；
 *   - WholeBodyTrajectoryCost 的基本数值与 yaw wrap。
 *
 * 动力学 WbmmDynamics 的 CppAD 代码生成测试放在后续 PR，避免单测触发
 * 运行时编译。
 ******************************************************************************/

#include "wbmm_ocs2/Dynamics.h"
#include "wbmm_ocs2/WbmmInterface.h"
#include "wbmm_ocs2/FactoryFunctions.h"
#include "wbmm_ocs2/PinocchioMapping.h"
#include "wbmm_ocs2/PreComputation.h"
#include "wbmm_ocs2/WbmmModelInfo.h"
#include "wbmm_ocs2/cost/WholeBodyTrajectoryCost.h"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <fstream>
#include <string>

namespace
{

std::string testRobotUrdfPath()
{
#ifndef WBMM_OCS2_TEST_URDF_FALLBACK
#error "WBMM_OCS2_TEST_URDF_FALLBACK must be defined"
#endif
  try {
    const std::string path =
      ament_index_cpp::get_package_share_directory("tracer_jaka_description") +
      "/urdf/tracer_jaka_zu5.urdf";
    if (std::ifstream(path).good()) {
      return path;
    }
  } catch (const std::exception &) {
  }
  return WBMM_OCS2_TEST_URDF_FALLBACK;
}

struct Fixture
{
  ocs2::PinocchioInterface interface =
    wbmm_ocs2::createWbmmPinocchioInterface(testRobotUrdfPath());
  wbmm_ocs2::WbmmModelInfo info =
    wbmm_ocs2::createWbmmModelInfo(interface, "base_footprint", "tool0");
};

ocs2::vector_t sampleState()
{
  ocs2::vector_t state(9);
  state << 1.0, 2.0, 0.4, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  return state;
}

}  // namespace

TEST(WbmmFactory, ResolvesWheelBasedModelDimensions)
{
  Fixture fixture;
  EXPECT_EQ(fixture.info.stateDim, 9U);
  EXPECT_EQ(fixture.info.inputDim, 8U);
  EXPECT_EQ(fixture.info.armDim, 6U);
  EXPECT_EQ(fixture.info.baseFrame, "base_footprint");
  EXPECT_EQ(fixture.info.eeFrame, "tool0");
  ASSERT_EQ(fixture.info.dofNames.size(), 6U);
  EXPECT_EQ(fixture.info.dofNames.front(), "joint_1");
  EXPECT_EQ(fixture.info.dofNames.back(), "joint_6");
}

TEST(WbmmPinocchioMapping, MapsDifferentialDriveVelocity)
{
  Fixture fixture;
  wbmm_ocs2::WbmmPinocchioMapping mapping(fixture.info);
  const auto state = sampleState();
  EXPECT_TRUE(mapping.getPinocchioJointPosition(state).isApprox(state));

  ocs2::vector_t input(8);
  input << 0.5, -0.2, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06;
  const auto velocity = mapping.getPinocchioJointVelocity(state, input);
  ASSERT_EQ(velocity.size(), 9);
  EXPECT_NEAR(velocity[0], 0.5 * std::cos(0.4), 1.0e-12);
  EXPECT_NEAR(velocity[1], 0.5 * std::sin(0.4), 1.0e-12);
  EXPECT_NEAR(velocity[2], -0.2, 1.0e-12);
  EXPECT_TRUE(velocity.tail(6).isApprox(input.tail(6), 1.0e-12));
}

TEST(WbmmPinocchioMapping, MapsOcs2JacobianWithoutSideslip)
{
  Fixture fixture;
  wbmm_ocs2::WbmmPinocchioMapping mapping(fixture.info);
  const auto state = sampleState();

  ocs2::matrix_t Jq = ocs2::matrix_t::Zero(6, 9);
  ocs2::matrix_t Jv = ocs2::matrix_t::Zero(6, 9);
  Jv.leftCols(3).setIdentity();
  Jv.rightCols(6).setIdentity();

  const auto jacobians = mapping.getOcs2Jacobian(state, Jq, Jv);
  EXPECT_TRUE(jacobians.first.isApprox(Jq));
  ASSERT_EQ(jacobians.second.rows(), 6);
  ASSERT_EQ(jacobians.second.cols(), 8);

  const double theta = state(2);
  Eigen::MatrixXd dvdu(3, 2);
  dvdu << std::cos(theta), 0.0,
    std::sin(theta), 0.0,
    0.0, 1.0;
  const ocs2::matrix_t expectedBase = Jv.leftCols(3) * dvdu;
  EXPECT_TRUE(jacobians.second.leftCols(2).isApprox(expectedBase, 1.0e-12));
  EXPECT_TRUE(jacobians.second.rightCols(6).isApprox(Jv.rightCols(6), 1.0e-12));
}

TEST(WbmmDynamics, FlowMapMatchesDifferentialDriveContract)
{
  Fixture fixture;
  wbmm_ocs2::WbmmDynamics dynamics(
    fixture.info, "wbmm_ocs2_test_dynamics", "/tmp/wbmm_ocs2_test_lib",
    true, false);

  const auto state = sampleState();
  ocs2::vector_t input(8);
  input << 0.5, -0.2, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06;
  wbmm_ocs2::WbmmPreComputation preComputation(fixture.interface, fixture.info);

  const ocs2::vector_t dxdt =
    dynamics.computeFlowMap(0.0, state, input, preComputation);
  ASSERT_EQ(dxdt.size(), 9);
  EXPECT_NEAR(dxdt[0], 0.5 * std::cos(0.4), 1.0e-12);
  EXPECT_NEAR(dxdt[1], 0.5 * std::sin(0.4), 1.0e-12);
  EXPECT_NEAR(dxdt[2], -0.2, 1.0e-12);
  EXPECT_TRUE(dxdt.tail(6).isApprox(input.tail(6), 1.0e-12));
}

TEST(WbmmInterface, AssemblesProblemAndRegistersTerms)
{
#ifndef WBMM_OCS2_TEST_TASK_FILE
#error "WBMM_OCS2_TEST_TASK_FILE must be defined"
#endif
  // 使用与 wbmm_ocs2_ros/config/task.info 相同的任务配置快照。
  // 构造过程会生成/编译 CppAD 库并组装 cost/constraint/dynamics。
  wbmm_ocs2::WbmmInterface interface(
    WBMM_OCS2_TEST_TASK_FILE, "/tmp/wbmm_ocs2_interface_test_lib",
    testRobotUrdfPath());

  const auto & modelInfo = interface.getWbmmModelInfo();
  EXPECT_EQ(modelInfo.stateDim, 9U);
  EXPECT_EQ(modelInfo.inputDim, 8U);
  EXPECT_EQ(modelInfo.armDim, 6U);
  EXPECT_EQ(interface.getInitialState().size(), 9);

  const auto & problem = interface.getOptimalControlProblem();
  ASSERT_NE(problem.dynamicsPtr, nullptr);
  ASSERT_NE(problem.preComputationPtr, nullptr);
  ASSERT_NE(problem.costPtr, nullptr);
  ASSERT_NE(problem.softConstraintPtr, nullptr);
  ASSERT_NE(problem.stateSoftConstraintPtr, nullptr);

  std::size_t index = 0;
  EXPECT_TRUE(problem.costPtr->getTermIndex("inputCost", index));
  EXPECT_TRUE(problem.softConstraintPtr->getTermIndex("jointLimits", index));
  EXPECT_TRUE(problem.stateSoftConstraintPtr->getTermIndex("selfCollision", index));

  ASSERT_NE(problem.stateCostPtr, nullptr);
  EXPECT_TRUE(problem.stateCostPtr->getTermIndex("wholeBodyTracking", index));
  ASSERT_NE(problem.finalCostPtr, nullptr);
  EXPECT_TRUE(problem.finalCostPtr->getTermIndex("finalWholeBodyTracking", index));

  // 全身跟踪模式下 EE 目标约束必须关闭，避免把 9D 状态误解析成 7D 位姿。
  if (problem.stateSoftConstraintPtr != nullptr) {
    EXPECT_FALSE(problem.stateSoftConstraintPtr->getTermIndex("endEffector", index));
  }
  ASSERT_NE(problem.finalSoftConstraintPtr, nullptr);
  EXPECT_FALSE(problem.finalSoftConstraintPtr->getTermIndex("finalEndEffector", index));
}

TEST(WholeBodyTrajectoryCost, TracksWholeBodyStateAndWrapsYaw)
{
  Fixture fixture;
  ocs2::matrix_t Q = ocs2::matrix_t::Identity(9, 9);
  const auto reference = sampleState();
  wbmm_ocs2::WholeBodyTrajectoryCost cost(Q, 2, reference);
  wbmm_ocs2::WbmmPreComputation preComputation(fixture.interface, fixture.info);

  ocs2::TargetTrajectories targets;
  targets.timeTrajectory = {0.0, 1.0};
  targets.stateTrajectory = {reference, reference};
  EXPECT_NEAR(cost.getValue(0.5, reference, targets, preComputation), 0.0, 1.0e-15);

  auto shifted = reference;
  shifted(3) += 0.2;
  EXPECT_NEAR(
    cost.getValue(0.5, shifted, targets, preComputation), 0.5 * 0.2 * 0.2,
    1.0e-15);

  auto nearPi = reference;
  nearPi(2) = 3.1415;
  auto nearMinusPi = reference;
  nearMinusPi(2) = -3.1415;
  targets.stateTrajectory = {nearPi, nearPi};
  EXPECT_LT(cost.getValue(0.5, nearMinusPi, targets, preComputation), 1.0e-4);
}

TEST(WholeBodyTrajectoryCost, InvalidReferenceFallsBackToHoldState)
{
  Fixture fixture;
  ocs2::matrix_t Q = ocs2::matrix_t::Identity(9, 9);
  const auto holdState = sampleState();
  wbmm_ocs2::WholeBodyTrajectoryCost cost(Q, 2, holdState);
  wbmm_ocs2::WbmmPreComputation preComputation(fixture.interface, fixture.info);

  ocs2::TargetTrajectories targets;
  targets.timeTrajectory = {0.0};
  targets.stateTrajectory = {ocs2::vector_t::Zero(7)};

  auto displaced = holdState;
  displaced(3) += 0.2;
  EXPECT_NEAR(
    cost.getValue(0.0, displaced, targets, preComputation), 0.5 * 0.2 * 0.2,
    1.0e-15);
}
