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
#include "wbmm_ocs2/cost/EndEffectorTrackingCost.h"
#include "wbmm_ocs2/cost/ArmManipulabilityCost.h"
#include "wbmm_ocs2/cost/PhaseWeightedStateCost.h"
#include "wbmm_ocs2/WbmmReferenceManager.h"
#include "wbmm_ocs2/collision/EsdfEnvironmentInterface.h"
#include "wbmm_ocs2/constraint/EsdfEnvironmentCollisionConstraint.h"
#include <wbmm_environment/esdf_grid.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

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

ocs2::vector_t makeEndEffectorTarget(
  const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation)
{
  ocs2::vector_t target(7);
  target.head<3>() = position;
  target.tail<4>() = orientation.normalized().coeffs();
  return target;
}

class TargetEchoCost final : public ocs2::StateCost
{
public:
  TargetEchoCost() = default;

  TargetEchoCost* clone() const override
  {
    return new TargetEchoCost(*this);
  }

  ocs2::scalar_t getValue(
    ocs2::scalar_t, const ocs2::vector_t &,
    const ocs2::TargetTrajectories & targets,
    const ocs2::PreComputation &) const override
  {
    if (targets.stateTrajectory.empty() ||
        targets.stateTrajectory.front().size() == 0) {
      return 0.0;
    }
    return targets.stateTrajectory.front()(0);
  }

  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
    ocs2::scalar_t time, const ocs2::vector_t & state,
    const ocs2::TargetTrajectories & targets,
    const ocs2::PreComputation & preComputation) const override
  {
    ocs2::ScalarFunctionQuadraticApproximation approximation;
    approximation.f = getValue(time, state, targets, preComputation);
    approximation.dfdx = ocs2::vector_t::Zero(state.rows());
    approximation.dfdxx =
      ocs2::matrix_t::Zero(state.rows(), state.rows());
    return approximation;
  }

private:
  TargetEchoCost(const TargetEchoCost &) = default;
};

std::shared_ptr<const wbmm::environment::EsdfGrid> makeLinearEsdfGrid()
{
  using wbmm::environment::EsdfGrid;
  using wbmm::environment::EsdfGridData;

  constexpr int kSize = 21;
  constexpr double kVoxelSize = 1.0;
  const Eigen::Vector3d origin(-10.0, -10.0, -10.0);

  EsdfGridData data;
  data.info.frame_id = "odom";
  data.info.origin = origin;
  data.info.voxel_size = kVoxelSize;
  data.info.shape = Eigen::Vector3i(kSize, kSize, kSize);

  const std::size_t voxelCount =
    static_cast<std::size_t>(kSize * kSize * kSize);
  data.esdf.resize(voxelCount);
  data.occupancy.assign(voxelCount, 0U);
  data.observed.assign(voxelCount, 1U);

  for (int ix = 0; ix < kSize; ++ix) {
    for (int iy = 0; iy < kSize; ++iy) {
      for (int iz = 0; iz < kSize; ++iz) {
        const std::size_t index = static_cast<std::size_t>(
          (ix * kSize + iy) * kSize + iz);
        data.esdf[index] = static_cast<float>(
          origin.x() + (static_cast<double>(ix) + 0.5) * kVoxelSize);
      }
    }
  }

  return std::make_shared<const EsdfGrid>(std::move(data));
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
  // 使用与 src/bringup/config/real/task.info 相同的任务配置快照。
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

TEST(EndEffectorTrackingCost, SevenDimensionalTargetMatchesFiniteDifference)
{
  Fixture fixture;
  wbmm_ocs2::WbmmPinocchioMapping mapping(fixture.info);
  ocs2::PinocchioEndEffectorKinematics kinematics(
    fixture.interface, mapping, {fixture.info.eeFrame});

  ocs2::matrix_t Qp = ocs2::matrix_t::Identity(3, 3);
  ocs2::matrix_t Qo = 0.5 * ocs2::matrix_t::Identity(3, 3);
  wbmm_ocs2::EndEffectorTrackingCost cost(kinematics, Qp, Qo);

  wbmm_ocs2::WbmmPreComputation preComputation(
    fixture.interface, fixture.info);
  const auto state = sampleState();
  const ocs2::vector_t input =
    ocs2::vector_t::Zero(static_cast<Eigen::Index>(fixture.info.inputDim));
  const auto request = ocs2::Request::Cost + ocs2::Request::Approximation;

  preComputation.request(request, 0.0, state, input);
  kinematics.setPinocchioInterface(preComputation.getPinocchioInterface());
  const Eigen::Vector3d currentPosition =
    kinematics.getPosition(state).front();

  const Eigen::Quaterniond targetOrientation(
    Eigen::AngleAxisd(0.25, Eigen::Vector3d::UnitZ()));
  const auto target = makeEndEffectorTarget(
    currentPosition + Eigen::Vector3d(0.05, -0.03, 0.02),
    targetOrientation);
  const ocs2::TargetTrajectories targets(
    {0.0}, {target}, {ocs2::vector_t::Zero(input.size())});

  const auto approximation =
    cost.getQuadraticApproximation(0.0, state, targets, preComputation);
  EXPECT_GT(cost.getValue(0.0, state, targets, preComputation), 0.0);

  constexpr double kStep = 1.0e-6;
  for (Eigen::Index i = 0; i < state.size(); ++i) {
    auto statePlus = state;
    statePlus(i) += kStep;
    auto stateMinus = state;
    stateMinus(i) -= kStep;

    preComputation.request(request, 0.0, statePlus, input);
    const double valuePlus =
      cost.getValue(0.0, statePlus, targets, preComputation);

    preComputation.request(request, 0.0, stateMinus, input);
    const double valueMinus =
      cost.getValue(0.0, stateMinus, targets, preComputation);

    const double finiteDifference =
      (valuePlus - valueMinus) / (2.0 * kStep);
    EXPECT_NEAR(approximation.dfdx(i), finiteDifference, 1.0e-5)
      << "state index " << i;
  }
}

TEST(EndEffectorTrackingCost, NineDimensionalTargetIsIgnored)
{
  Fixture fixture;
  wbmm_ocs2::WbmmPinocchioMapping mapping(fixture.info);
  ocs2::PinocchioEndEffectorKinematics kinematics(
    fixture.interface, mapping, {fixture.info.eeFrame});

  wbmm_ocs2::EndEffectorTrackingCost cost(
    kinematics,
    ocs2::matrix_t::Identity(3, 3),
    ocs2::matrix_t::Identity(3, 3));

  wbmm_ocs2::WbmmPreComputation preComputation(
    fixture.interface, fixture.info);
  const auto state = sampleState();
  const ocs2::vector_t input =
    ocs2::vector_t::Zero(static_cast<Eigen::Index>(fixture.info.inputDim));
  preComputation.request(
    ocs2::Request::Cost + ocs2::Request::Approximation,
    0.0, state, input);

  // 9D whole-body reference must not be interpreted as a 7D EE pose.
  const ocs2::TargetTrajectories targets(
    {0.0}, {state}, {ocs2::vector_t::Zero(input.size())});

  EXPECT_DOUBLE_EQ(
    cost.getValue(0.0, state, targets, preComputation), 0.0);

  const auto approximation =
    cost.getQuadraticApproximation(0.0, state, targets, preComputation);
  EXPECT_DOUBLE_EQ(approximation.f, 0.0);
  EXPECT_TRUE(approximation.dfdx.isZero(1.0e-12));
  EXPECT_TRUE(approximation.dfdxx.isZero(1.0e-12));
}

TEST(ArmManipulabilityCost, MetricsMatchEigenSvd)
{
  Fixture fixture;
  wbmm_ocs2::ArmManipulabilitySettings settings;
  settings.scope = wbmm_ocs2::ArmManipulabilitySettings::Scope::kArmOnly;
  settings.frameName = fixture.info.eeFrame;
  settings.armStartIndex = 3;
  settings.armDim = 6;

  wbmm_ocs2::ArmManipulabilityCost cost(
    fixture.interface, fixture.info, settings);

  const auto state = sampleState();
  const auto& model = fixture.interface.getModel();
  auto data = fixture.interface.getData();

  pinocchio::forwardKinematics(model, data, state);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::computeJointJacobians(model, data, state);

  ocs2::matrix_t frameJacobian = ocs2::matrix_t::Zero(6, model.nv);
  pinocchio::getFrameJacobian(
    model, data, model.getBodyId(fixture.info.eeFrame),
    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, frameJacobian);

  const ocs2::matrix_t armJacobian = frameJacobian.middleCols(3, 6);
  Eigen::JacobiSVD<ocs2::matrix_t> svd(
    armJacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);

  const ocs2::scalar_t expectedSigmaMin =
    svd.singularValues().minCoeff();
  const ocs2::scalar_t expectedYoshikawa =
    svd.singularValues().prod();

  const auto metrics = cost.computeMetrics(state);
  ASSERT_TRUE(metrics.valid);
  EXPECT_NEAR(metrics.sigmaMin, expectedSigmaMin, 1.0e-9);
  EXPECT_NEAR(metrics.yoshikawa, expectedYoshikawa, 1.0e-9);
}

TEST(ArmManipulabilityCost, GradientMatchesFiniteDifference)
{
  Fixture fixture;
  wbmm_ocs2::ArmManipulabilitySettings settings;
  settings.scope = wbmm_ocs2::ArmManipulabilitySettings::Scope::kArmOnly;
  settings.frameName = fixture.info.eeFrame;
  settings.armStartIndex = 3;
  settings.armDim = 6;

  // Force the hinge terms to be active at the sample configuration.
  settings.useMinSingularValue = true;
  settings.minSingularWeight = 1.0;
  settings.minSingularRef = 1.0;

  settings.useYoshikawa = true;
  settings.yoshikawaWeight = 0.5;
  settings.yoshikawaRef = 1.0;

  settings.useInverseManipulability = true;
  settings.inverseManipulabilityWeight = 1.0e-3;

  settings.finiteDiffStep = 1.0e-6;
  settings.hessianRegularization = 1.0e-6;

  wbmm_ocs2::ArmManipulabilityCost cost(
    fixture.interface, fixture.info, settings);

  const auto state = sampleState();
  const ocs2::TargetTrajectories targets;
  const ocs2::PreComputation preComputation;

  const auto approximation = cost.getQuadraticApproximation(
    0.0, state, targets, preComputation);

  constexpr double kStep = 1.0e-6;
  for (Eigen::Index i = 0; i < state.size(); ++i) {
    auto statePlus = state;
    auto stateMinus = state;
    statePlus(i) += kStep;
    stateMinus(i) -= kStep;

    const double valuePlus =
      cost.getValue(0.0, statePlus, targets, preComputation);
    const double valueMinus =
      cost.getValue(0.0, stateMinus, targets, preComputation);
    const double finiteDifference =
      (valuePlus - valueMinus) / (2.0 * kStep);

    EXPECT_NEAR(approximation.dfdx(i), finiteDifference, 1.0e-4)
      << "state index " << i;
  }
}

TEST(WbmmReferenceManager, LatchesPhaseAndKeepsTargetsSeparate)
{
  const auto state = sampleState();
  const ocs2::PreComputation preComputation;
  const ocs2::TargetTrajectories unusedTargets;

  auto manager = std::make_shared<wbmm_ocs2::WbmmReferenceManager>(
    wbmm_ocs2::TaskPhase::kNavigation);

  ocs2::vector_t wholeBodyValue(1);
  wholeBodyValue << 10.0;
  ocs2::vector_t endEffectorValue(1);
  endEffectorValue << 20.0;

  manager->setWholeBodyTarget(
    ocs2::TargetTrajectories({0.0}, {wholeBodyValue}));
  manager->setEndEffectorTarget(
    ocs2::TargetTrajectories({0.0}, {endEffectorValue}));

  // Latch the initial Navigation phase and both target buffers.
  manager->preSolverRun(0.0, 1.0, state);

  auto wholeBodyCost = std::make_unique<wbmm_ocs2::PhaseWeightedStateCost>(
    std::make_unique<TargetEchoCost>(), manager,
    wbmm_ocs2::TargetKind::kWholeBody,
    std::array<ocs2::scalar_t, 4>{1.0, 0.5, 0.1, 0.5});
  auto endEffectorCost = std::make_unique<wbmm_ocs2::PhaseWeightedStateCost>(
    std::make_unique<TargetEchoCost>(), manager,
    wbmm_ocs2::TargetKind::kEndEffector,
    std::array<ocs2::scalar_t, 4>{0.0, 0.5, 1.0, 0.0});

  // Requested phase is buffered; active phase changes only at preSolverRun().
  manager->setTaskPhase(wbmm_ocs2::TaskPhase::kExecution);
  EXPECT_EQ(manager->getTaskPhase(), wbmm_ocs2::TaskPhase::kNavigation);
  EXPECT_EQ(
    manager->getRequestedTaskPhase(), wbmm_ocs2::TaskPhase::kExecution);

  EXPECT_DOUBLE_EQ(
    wholeBodyCost->getValue(0.0, state, unusedTargets, preComputation),
    10.0);
  EXPECT_DOUBLE_EQ(
    endEffectorCost->getValue(0.0, state, unusedTargets, preComputation),
    0.0);
  EXPECT_EQ(
    manager->getTargetTrajectories().stateTrajectory.front()(0), 10.0);

  manager->preSolverRun(0.0, 1.0, state);

  EXPECT_EQ(manager->getTaskPhase(), wbmm_ocs2::TaskPhase::kExecution);
  EXPECT_DOUBLE_EQ(
    wholeBodyCost->getValue(0.0, state, unusedTargets, preComputation),
    1.0);
  EXPECT_DOUBLE_EQ(
    endEffectorCost->getValue(0.0, state, unusedTargets, preComputation),
    20.0);
  EXPECT_EQ(
    manager->getTargetTrajectories().stateTrajectory.front()(0), 20.0);
  EXPECT_TRUE(wholeBodyCost->isActive(0.0));
  EXPECT_TRUE(endEffectorCost->isActive(0.0));
}

TEST(WbmmInterface, SupportsDualReferenceModeSwitch)
{
#ifndef WBMM_OCS2_TEST_MODE_SWITCH_TASK_FILE
#error "WBMM_OCS2_TEST_MODE_SWITCH_TASK_FILE must be defined"
#endif
  wbmm_ocs2::WbmmInterface interface(
    WBMM_OCS2_TEST_MODE_SWITCH_TASK_FILE,
    "/tmp/wbmm_ocs2_mode_switch_test_lib", testRobotUrdfPath());

  EXPECT_TRUE(interface.isModeSwitchEnabled());
  ASSERT_NE(interface.getWbmmReferenceManagerPtr(), nullptr);

  const auto & problem = interface.getOptimalControlProblem();
  std::size_t index = 0;
  ASSERT_NE(problem.stateCostPtr, nullptr);
  EXPECT_TRUE(problem.stateCostPtr->getTermIndex("wholeBodyTracking", index));
  EXPECT_TRUE(problem.stateCostPtr->getTermIndex("endEffectorTracking", index));

  ASSERT_NE(problem.finalCostPtr, nullptr);
  EXPECT_TRUE(problem.finalCostPtr->getTermIndex("finalWholeBodyTracking", index));
  EXPECT_TRUE(problem.finalCostPtr->getTermIndex("finalEndEffectorTracking", index));

  ASSERT_NE(problem.stateSoftConstraintPtr, nullptr);
  EXPECT_FALSE(problem.stateSoftConstraintPtr->getTermIndex("endEffector", index));

  ocs2::vector_t eeTarget(7);
  eeTarget << 0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0;
  interface.setWholeBodyTarget(
    ocs2::TargetTrajectories({0.0}, {sampleState()}));
  interface.setEndEffectorTarget(
    ocs2::TargetTrajectories({0.0}, {eeTarget}));

  interface.setTaskPhase(wbmm_ocs2::TaskPhase::kExecution);
  EXPECT_EQ(
    interface.getWbmmReferenceManagerPtr()->getRequestedTaskPhase(),
    wbmm_ocs2::TaskPhase::kExecution);

  interface.getWbmmReferenceManagerPtr()->preSolverRun(
    0.0, 1.0, sampleState());
  EXPECT_EQ(interface.getTaskPhase(), wbmm_ocs2::TaskPhase::kExecution);
  ASSERT_EQ(interface.getWholeBodyTarget().stateTrajectory.size(), 1U);
  ASSERT_EQ(interface.getEndEffectorTarget().stateTrajectory.size(), 1U);
  EXPECT_EQ(interface.getWholeBodyTarget().stateTrajectory.front().size(), 9);
  EXPECT_EQ(interface.getEndEffectorTarget().stateTrajectory.front().size(), 7);
}

TEST(EsdfEnvironmentCollisionConstraint, SimpleLinearFieldValueAndGradient)
{
  Fixture fixture;
  const auto grid = makeLinearEsdfGrid();

  constexpr double kMargin = 0.1;
  const std::vector<std::string> collisionLinks{"tool0_and_camera_link"};
  const std::vector<ocs2::scalar_t> maxExcesses{0.01};

  auto environment = std::make_shared<wbmm_ocs2::EsdfEnvironmentInterface>(
    fixture.interface, grid, collisionLinks, maxExcesses, 0.8, kMargin);
  ASSERT_GT(environment->getNumSpheres(), 0U);

  wbmm_ocs2::WbmmPinocchioMapping mapping(fixture.info);
  wbmm_ocs2::EsdfEnvironmentCollisionConstraint constraint(
    mapping, environment, kMargin);

  const auto state = sampleState();
  const ocs2::vector_t input =
    ocs2::vector_t::Zero(static_cast<Eigen::Index>(fixture.info.inputDim));
  wbmm_ocs2::WbmmPreComputation preComputation(
    fixture.interface, fixture.info);
  const auto request =
    ocs2::Request::Constraint + ocs2::Request::Approximation;

  preComputation.request(request, 0.0, state, input);

  const auto distances =
    environment->computeDistances(preComputation.getPinocchioInterface());
  const auto h = constraint.getValue(0.0, state, preComputation);

  ASSERT_EQ(h.size(), distances.size());
  ASSERT_EQ(
    constraint.getNumConstraints(0.0),
    static_cast<std::size_t>(h.size()));

  for (Eigen::Index i = 0; i < h.size(); ++i) {
    EXPECT_NEAR(
      h(i),
      distances[static_cast<std::size_t>(i)].distance -
        distances[static_cast<std::size_t>(i)].radius - kMargin,
      1.0e-12);

    // d = x => gradient is [1, 0, 0].
    EXPECT_NEAR(
      distances[static_cast<std::size_t>(i)].gradient.x(), 1.0, 1.0e-12);
    EXPECT_NEAR(
      distances[static_cast<std::size_t>(i)].gradient.y(), 0.0, 1.0e-12);
    EXPECT_NEAR(
      distances[static_cast<std::size_t>(i)].gradient.z(), 0.0, 1.0e-12);
  }

  const auto linear =
    constraint.getLinearApproximation(0.0, state, preComputation);
  EXPECT_TRUE(linear.f.isApprox(h, 1.0e-12));

  // Moving base x translates every sphere center by +1 in x, so dh/dx = 1.
  for (Eigen::Index i = 0; i < h.size(); ++i) {
    EXPECT_NEAR(linear.dfdx(i, 0), 1.0, 1.0e-9);
  }

  // Compare the full analytic Jacobian against central finite differences.
  constexpr double kStep = 1.0e-6;
  for (Eigen::Index k = 0; k < state.size(); ++k) {
    auto statePlus = state;
    auto stateMinus = state;
    statePlus(k) += kStep;
    stateMinus(k) -= kStep;

    preComputation.request(request, 0.0, statePlus, input);
    const auto hPlus = constraint.getValue(0.0, statePlus, preComputation);

    preComputation.request(request, 0.0, stateMinus, input);
    const auto hMinus = constraint.getValue(0.0, stateMinus, preComputation);

    for (Eigen::Index i = 0; i < h.size(); ++i) {
      const double finiteDifference =
        (hPlus(i) - hMinus(i)) / (2.0 * kStep);
      EXPECT_NEAR(linear.dfdx(i, k), finiteDifference, 1.0e-4)
        << "constraint row " << i << ", state column " << k;
    }
  }
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
