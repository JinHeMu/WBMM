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
#include <boost/property_tree/info_parser.hpp>
#include <gtest/gtest.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <cmath>
#include <fstream>
#include <future>
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
  EXPECT_FALSE(problem.stateCostPtr->getTermIndex("armManipulability", index));
  EXPECT_FALSE(interface.isArmManipulabilityEnabled());
  ASSERT_NE(problem.finalCostPtr, nullptr);
  EXPECT_TRUE(problem.finalCostPtr->getTermIndex("finalWholeBodyTracking", index));

  // 旧 EndEffectorConstraint 已删除，末端目标只通过 tracking cost 接入。
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
  settings.frameName = fixture.info.eeFrame;
  settings.stateFrame = "odom";
  settings.metricsOptions.scope =
    wbmm::metrics::JacobianScope::kArmColumns;

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
  ASSERT_EQ(metrics.status, wbmm::metrics::MetricsStatus::kSuccess);
  EXPECT_EQ(metrics.header.frame_id, "odom");
  EXPECT_EQ(metrics.link_name, fixture.info.eeFrame);
  EXPECT_NEAR(metrics.sigma_min, expectedSigmaMin, 1.0e-9);
  EXPECT_NEAR(metrics.manipulability, expectedYoshikawa, 1.0e-9);
}

TEST(ArmManipulabilityCost, GradientMatchesFiniteDifference)
{
  Fixture fixture;
  wbmm_ocs2::ArmManipulabilitySettings settings;
  settings.frameName = fixture.info.eeFrame;
  settings.metricsOptions.scope =
    wbmm::metrics::JacobianScope::kArmColumns;

  // Force the hinge terms to be active at the sample configuration.
  settings.useMinSingularValue = true;
  settings.minSingularWeight = 1.0;
  settings.minSingularRef = 1.0;

  settings.useYoshikawa = true;
  settings.yoshikawaWeight = 0.5;
  settings.yoshikawaRef = 1.0;

  settings.useInverseManipulability = true;
  settings.inverseManipulabilityWeight = 1.0e-3;

  settings.metricsOptions.use_task_direction = true;
  settings.metricsOptions.task_direction = ocs2::vector_t::Zero(6);
  settings.metricsOptions.task_direction(0) = 1.0;
  settings.taskDirectionWeight = 0.25;
  settings.taskDirectionRef = 1.0;

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

TEST(ArmManipulabilityCost, InvalidStateUsesConfiguredPenalty)
{
  Fixture fixture;
  wbmm_ocs2::ArmManipulabilitySettings settings;
  settings.frameName = fixture.info.eeFrame;
  settings.invalidMetricsPenalty = 1234.0;

  wbmm_ocs2::ArmManipulabilityCost cost(
    fixture.interface, fixture.info, settings);

  const ocs2::TargetTrajectories targets;
  const ocs2::PreComputation preComputation;
  const ocs2::vector_t invalidState = ocs2::vector_t::Zero(8);
  EXPECT_DOUBLE_EQ(
    cost.getValue(0.0, invalidState, targets, preComputation), 1234.0);

  const auto approximation = cost.getQuadraticApproximation(
    0.0, invalidState, targets, preComputation);
  EXPECT_DOUBLE_EQ(approximation.f, 1234.0);
  EXPECT_TRUE(approximation.dfdx.isZero(1.0e-12));
  EXPECT_TRUE(approximation.dfdxx.isApprox(
    settings.hessianRegularization *
      ocs2::matrix_t::Identity(invalidState.size(), invalidState.size()),
    1.0e-12));
}

TEST(ArmManipulabilityCost, AnalyticApproximationMatchesIndependentMetricDifferences)
{
  Fixture fixture;
  const ocs2::TargetTrajectories targets;
  const ocs2::PreComputation pre;
  // Cover rectangular/arm Jacobians, TCP offsets, world direction, scaling,
  // condition-floor saturation, and all five metric derivatives together.
  for (bool wholeBody : {false, true}) {
    for (bool translation : {false, true}) {
      for (bool scaled : {false, true}) {
        wbmm_ocs2::ArmManipulabilitySettings settings;
        settings.frameName = scaled ? fixture.info.eeFrame : "jk_se_vi_200_link";
        settings.metricsOptions.scope = wholeBody ?
            wbmm::metrics::JacobianScope::kWholeBodyInput : wbmm::metrics::JacobianScope::kArmColumns;
        settings.metricsOptions.task = translation ?
            wbmm::metrics::JacobianTask::kTranslation : wbmm::metrics::JacobianTask::kPose;
        settings.metricsOptions.scaling = scaled ?
            wbmm::metrics::JacobianScaling::kCharacteristicLength : wbmm::metrics::JacobianScaling::kRaw;
        settings.metricsOptions.characteristic_length = 0.3;
        settings.metricsOptions.use_task_direction = true;
        settings.metricsOptions.task_direction = ocs2::vector_t::LinSpaced(translation ? 3 : 6, 1.0, 2.0);
        settings.useYoshikawa = settings.useInverseManipulability = settings.useConditionNumber = true;
        settings.minSingularRef = settings.yoshikawaRef = settings.taskDirectionRef = 2.0;
        settings.conditionMax = 1.0;
        settings.conditionWeight = 1e-4;
        settings.inverseManipulabilityWeight = 1e-6;
        settings.weightScale = 0.7;
        settings.normalizeMargins = true;
        settings.metricsOptions.singular_value_floor = scaled ? 0.5 : 1e-9;
        wbmm_ocs2::ArmManipulabilityCost cost(fixture.interface, fixture.info, settings);
        for (int sample = 0; sample < 5; ++sample) {
          auto state = sampleState();
          state.tail(6).array() += 0.13 * sample;
          state(2) -= 0.21 * sample;
          const auto approx = cost.getQuadraticApproximation(0, state, targets, pre);
          EXPECT_TRUE(approx.dfdx.allFinite());
          Eigen::SelfAdjointEigenSolver<ocs2::matrix_t> eigen(approx.dfdxx);
          EXPECT_GE(eigen.eigenvalues().minCoeff(), -1e-9);
          // Independent oracle retains full public metrics and joint-state evaluation.
          auto metricCost = [&](const ocs2::vector_t& x) {
            const auto m = cost.computeMetrics(x);
            EXPECT_EQ(m.status, wbmm::metrics::MetricsStatus::kSuccess);
            auto hinge = [](double metric, double ref, double weight) {
              return 0.5 * weight * std::pow(std::max(0.0, 1.0 - metric / ref), 2);
            };
            return settings.weightScale * (
                hinge(m.sigma_min, settings.minSingularRef, settings.minSingularWeight) +
                hinge(m.manipulability, settings.yoshikawaRef, settings.yoshikawaWeight) +
                hinge(m.task_direction_manipulability, settings.taskDirectionRef, settings.taskDirectionWeight) +
                settings.inverseManipulabilityWeight * m.inverse_manipulability +
                0.5 * settings.conditionWeight * std::pow(std::max(0.0, m.condition_number - settings.conditionMax), 2));
          };
          EXPECT_NEAR(approx.f, metricCost(state), 1e-8);
          EXPECT_NEAR(cost.getValue(0, state, targets, pre), approx.f, 1e-12);
          ocs2::matrix_t expectedHessian = settings.hessianRegularization *
              ocs2::matrix_t::Identity(9, 9);
          ocs2::matrix_t metricGradient = ocs2::matrix_t::Zero(5, 9);
          for (int index = 0; index < 9; ++index) {
            auto plus = state, minus = state;
            constexpr double step = 1e-6;
            plus(index) += step; minus(index) -= step;
            const double gradient = (metricCost(plus) - metricCost(minus)) / (2 * step);
            EXPECT_NEAR(approx.dfdx(index), gradient, 1e-5 * std::max(1.0, std::abs(gradient)))
                << "whole=" << wholeBody << " translation=" << translation << " scaled=" << scaled
                << " sample=" << sample << " index=" << index;
            const auto p = cost.computeMetrics(plus), m = cost.computeMetrics(minus);
            metricGradient.col(index) << p.sigma_min - m.sigma_min, p.manipulability - m.manipulability,
                p.task_direction_manipulability - m.task_direction_manipulability,
                p.inverse_manipulability - m.inverse_manipulability, p.condition_number - m.condition_number;
            metricGradient.col(index) /= 2 * step;
          }
          const auto current = cost.computeMetrics(state);
          const double weights[] = {settings.minSingularWeight / std::pow(settings.minSingularRef, 2),
              settings.yoshikawaWeight / std::pow(settings.yoshikawaRef, 2),
              settings.taskDirectionWeight / std::pow(settings.taskDirectionRef, 2), 0.0, settings.conditionWeight};
          const bool active[] = {current.sigma_min < settings.minSingularRef,
              current.manipulability < settings.yoshikawaRef,
              current.task_direction_manipulability < settings.taskDirectionRef, false,
              current.condition_number > settings.conditionMax};
          for (int row = 0; row < 5; ++row) {
            if (active[row]) {
              expectedHessian += weights[row] * metricGradient.row(row).transpose() * metricGradient.row(row);
            }
          }
          EXPECT_TRUE(approx.dfdxx.isApprox(settings.weightScale * expectedHessian, 2e-5));
        }
      }
    }
  }
}

TEST(ArmManipulabilityCost, RankDeficientStatesRemainFiniteAndSymmetric)
{
  Fixture fixture;
  wbmm_ocs2::ArmManipulabilitySettings settings;
  settings.minSingularRef = 1.0;
  settings.useYoshikawa = true;
  settings.yoshikawaRef = 1.0;
  settings.metricsOptions.scaling = wbmm::metrics::JacobianScaling::kCharacteristicLength;
  settings.metricsOptions.characteristic_length = 0.3;
  wbmm_ocs2::ArmManipulabilityCost cost(fixture.interface, fixture.info, settings);
  const ocs2::TargetTrajectories targets;
  const ocs2::PreComputation pre;
  for (const double offset : {0.0, 1e-10, 1e-7}) {
    const ocs2::vector_t state = ocs2::vector_t::Constant(9, offset);
    const auto approx = cost.getQuadraticApproximation(0, state, targets, pre);
    EXPECT_TRUE(approx.dfdx.allFinite());
    EXPECT_TRUE(approx.dfdxx.allFinite());
    EXPECT_TRUE(approx.dfdxx.isApprox(approx.dfdxx.transpose(), 1e-12));
    for (int index = 0; index < 9; ++index) {
      auto plus = state, minus = state;
      plus(index) += settings.finiteDiffStep; minus(index) -= settings.finiteDiffStep;
      EXPECT_NEAR(approx.dfdx(index),
          (cost.getValue(0, plus, targets, pre) - cost.getValue(0, minus, targets, pre)) /
              (2 * settings.finiteDiffStep), 1e-5);
    }
  }
}

TEST(ArmManipulabilityCost, NormalizedSigmaAndYoshikawaScaleValuesAndDerivatives)
{
  Fixture fixture;
  wbmm_ocs2::ArmManipulabilitySettings settings;
  settings.metricsOptions.scaling = wbmm::metrics::JacobianScaling::kCharacteristicLength;
  settings.metricsOptions.characteristic_length = 0.3;
  settings.normalizeMargins = true;
  settings.weightScale = 2.5;
  settings.minSingularRef = 0.5;
  settings.minSingularWeight = 0.3;
  settings.useYoshikawa = true;
  settings.yoshikawaRef = 0.1;
  settings.yoshikawaWeight = 0.2;
  wbmm_ocs2::ArmManipulabilityCost cost(fixture.interface, fixture.info, settings);
  const auto state = sampleState();
  const auto metrics = cost.computeMetrics(state);
  const ocs2::TargetTrajectories targets;
  const ocs2::PreComputation pre;
  const double expected = settings.weightScale * 0.5 *
      (settings.minSingularWeight * std::pow(std::max(0.0, 1.0 - metrics.sigma_min / settings.minSingularRef), 2) +
       settings.yoshikawaWeight * std::pow(std::max(0.0, 1.0 - metrics.manipulability / settings.yoshikawaRef), 2));
  EXPECT_NEAR(cost.getValue(0.0, state, targets, pre), expected, 1e-12);
  const auto approximation = cost.getQuadraticApproximation(0.0, state, targets, pre);
  EXPECT_NEAR(approximation.f, expected, 1e-12);
  for (Eigen::Index i = 0; i < state.size(); ++i) {
    auto plus = state, minus = state;
    plus(i) += 1e-5; minus(i) -= 1e-5;
    EXPECT_NEAR(approximation.dfdx(i),
        (cost.getValue(0, plus, targets, pre) - cost.getValue(0, minus, targets, pre)) / 2e-5, 1e-5);
  }
  // Changing base placement cannot improve an arm-only singular-value metric.
  auto moved = state; moved.head(3) << -3.0, 5.0, -1.2;
  EXPECT_NEAR(cost.computeMetrics(moved).sigma_min, metrics.sigma_min, 1e-12);
  EXPECT_TRUE(approximation.dfdx.head(3).isZero(1e-12));
  Eigen::SelfAdjointEigenSolver<ocs2::matrix_t> eigen(approximation.dfdxx);
  EXPECT_GE(eigen.eigenvalues().minCoeff(), -1e-12);
  auto healthySettings = settings;
  healthySettings.minSingularRef = metrics.sigma_min * 0.5;
  healthySettings.yoshikawaRef = metrics.manipulability * 0.5;
  wbmm_ocs2::ArmManipulabilityCost healthy(fixture.interface, fixture.info, healthySettings);
  const auto healthyApprox = healthy.getQuadraticApproximation(0, state, targets, pre);
  EXPECT_DOUBLE_EQ(healthyApprox.f, 0.0);
  EXPECT_TRUE(healthyApprox.dfdx.isZero());
}

TEST(ArmManipulabilityCost, ReusedWorkspaceAndConcurrentClonesPreserveMetrics)
{
  Fixture fixture;
  wbmm_ocs2::ArmManipulabilitySettings settings;
  settings.metricsOptions.scaling = wbmm::metrics::JacobianScaling::kCharacteristicLength;
  settings.metricsOptions.characteristic_length = 0.3;
  wbmm_ocs2::ArmManipulabilityCost cost(fixture.interface, fixture.info, settings);
  std::vector<ocs2::vector_t> states;
  std::vector<wbmm_ocs2::ArmManipulabilityCost::Metrics> expected;
  std::vector<ocs2::ScalarFunctionQuadraticApproximation> expectedApprox;
  const ocs2::TargetTrajectories targets;
  const ocs2::PreComputation pre;
  for (int i = 0; i < 4; ++i) {
    auto state = sampleState();
    state.tail(6).array() += i * 0.2;
    states.push_back(state);
    expected.push_back(cost.computeMetrics(state));
    expectedApprox.push_back(cost.getQuadraticApproximation(0, state, targets, pre));
    ASSERT_EQ(expected.back().status, wbmm::metrics::MetricsStatus::kSuccess);
  }
  std::vector<std::future<bool>> workers;
  for (int worker = 0; worker < 4; ++worker) {
    workers.push_back(std::async(std::launch::async, [&, worker] {
      for (int iteration = 0; iteration < 30; ++iteration) {
        const auto index = (iteration + worker) % states.size();
        const auto result = cost.computeMetrics(states[index]);
        const auto approx = cost.getQuadraticApproximation(0, states[index], targets, pre);
        std::unique_ptr<wbmm_ocs2::ArmManipulabilityCost> clone(cost.clone());
        const auto copied = clone->computeMetrics(states[(index + 1) % states.size()]);
        const auto copiedApprox = clone->getQuadraticApproximation(0, states[index], targets, pre);
        if (result.status != wbmm::metrics::MetricsStatus::kSuccess ||
            copied.status != wbmm::metrics::MetricsStatus::kSuccess ||
            !result.singular_values.isApprox(expected[index].singular_values, 1e-12) ||
            !copied.singular_values.isApprox(expected[(index + 1) % states.size()].singular_values, 1e-12) ||
            !approx.dfdx.isApprox(expectedApprox[index].dfdx, 1e-12) ||
            !approx.dfdxx.isApprox(expectedApprox[index].dfdxx, 1e-12) ||
            !copiedApprox.dfdx.isApprox(expectedApprox[index].dfdx, 1e-12)) {
          return false;
        }
      }
      return true;
    }));
  }
  for (auto& worker : workers) { EXPECT_TRUE(worker.get()); }
}

TEST(ArmManipulabilityCost, NormalizedReferencesRejectZero)
{
  Fixture fixture;
  wbmm_ocs2::ArmManipulabilitySettings settings;
  settings.normalizeMargins = true;
  settings.minSingularRef = 0.0;
  EXPECT_THROW(wbmm_ocs2::ArmManipulabilityCost(fixture.interface, fixture.info, settings), std::runtime_error);
  settings.minSingularRef = 0.1;
  settings.useYoshikawa = true;
  settings.yoshikawaRef = 0.0;
  EXPECT_THROW(wbmm_ocs2::ArmManipulabilityCost(fixture.interface, fixture.info, settings), std::runtime_error);
  settings.useYoshikawa = false;
  settings.minSingularRef = 1e-200;
  EXPECT_THROW(wbmm_ocs2::ArmManipulabilityCost(fixture.interface, fixture.info, settings), std::runtime_error);
}

TEST(WbmmInterface, PositionLimitToggleKeepsTaskVelocityConstraints)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(WBMM_OCS2_TEST_TASK_FILE, pt);
  pt.put("jointPositionLimits.activate", false);
  const std::string path = "/tmp/wbmm_position_limits_disabled.info";
  boost::property_tree::write_info(path, pt);
  wbmm_ocs2::WbmmInterface enabled(WBMM_OCS2_TEST_TASK_FILE, "/tmp/wbmm_position_limits_on", testRobotUrdfPath());
  wbmm_ocs2::WbmmInterface disabled(path, "/tmp/wbmm_position_limits_off", testRobotUrdfPath());
  const auto state = enabled.getInitialState();
  ocs2::vector_t outside = state;
  outside[3] = enabled.getPinocchioInterface().getModel().upperPositionLimit.tail(6)[0] + 0.5;
  ocs2::vector_t input = ocs2::vector_t::Zero(8);
  ocs2::TargetTrajectories targets;
  ocs2::PreComputation pre;
  auto &on = enabled.getOptimalControlProblem().softConstraintPtr->get("jointLimits");
  auto &off = disabled.getOptimalControlProblem().softConstraintPtr->get("jointLimits");
  EXPECT_GT(on.getValue(0.0, outside, input, targets, pre) - on.getValue(0.0, state, input, targets, pre), 1.0);
  EXPECT_NEAR(off.getValue(0.0, outside, input, targets, pre), off.getValue(0.0, state, input, targets, pre), 1e-9);
  const double stationary = off.getValue(0.0, state, input, targets, pre);
  input[0] = 0.6;  // task.info base upper bound is 0.5 m/s.
  EXPECT_GT(off.getValue(0.0, state, input, targets, pre) - stationary, 1.0);
}

TEST(WbmmInterface, TcpOverridePreservesTaskVelocityBounds)
{
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(WBMM_OCS2_TEST_TASK_FILE, pt);
  pt.get_child("model_information").erase("eeFrame");
  const std::string path = "/tmp/wbmm_integrated_config_test.info";
  boost::property_tree::write_info(path, pt);
  wbmm_ocs2::WbmmInterface integrated(path, "/tmp/wbmm_integrated_config_test_lib",
                                     testRobotUrdfPath(), "", "odom", "Link_6");
  EXPECT_EQ(integrated.getWbmmModelInfo().eeFrame, "Link_6");
  EXPECT_DOUBLE_EQ(integrated.getInputVelocityUpperBound()[0], 0.5);
  EXPECT_DOUBLE_EQ(integrated.getInputVelocityUpperBound()[1], 1.0);
  EXPECT_DOUBLE_EQ(integrated.getInputVelocityUpperBound()[2], 2.0);
}

TEST(WbmmInterface, ArmTerminalRegistrationIsIndependentAndOverridesReferences)
{
  // Start with the cost-off baseline. A terminal-only arm term is allowed.
  std::ifstream input(WBMM_OCS2_TEST_TASK_FILE);
  const std::string original((std::istreambuf_iterator<char>(input)), {});
  const std::string path = "/tmp/wbmm_arm_terminal_registration.info";
  {
    std::ofstream output(path);
    output << original << R"(
armManipulability
{
  activate false
  normalizeMargins true
  minSingularRef 0.2
  useYoshikawa false
  terminal
  {
    activate true
    weightScale 3.0
    minSingularRef 0.4
    useYoshikawa true
    yoshikawaRef 0.02
    yoshikawaWeight 0.3
  }
  diagnostics { activate true }
}
)";
  }
  wbmm_ocs2::WbmmInterface interface(path, "/tmp/wbmm_arm_terminal_test_lib", testRobotUrdfPath());
  const auto& problem = interface.getOptimalControlProblem();
  std::size_t index = 0;
  EXPECT_FALSE(interface.isArmManipulabilityEnabled());
  EXPECT_FALSE(problem.stateCostPtr->getTermIndex("armManipulability", index));
  EXPECT_TRUE(interface.isFinalArmManipulabilityEnabled());
  ASSERT_TRUE(problem.finalCostPtr->getTermIndex("finalArmManipulability", index));
  const auto& cost = problem.finalCostPtr->get<wbmm_ocs2::ArmManipulabilityCost>("finalArmManipulability");
  EXPECT_DOUBLE_EQ(cost.settings().weightScale, 3.0);
  EXPECT_DOUBLE_EQ(cost.settings().minSingularRef, 0.4);
  EXPECT_TRUE(cost.settings().useYoshikawa);
  ASSERT_NE(interface.getArmMetricsEvaluator(), nullptr);
  EXPECT_FALSE(interface.getArmMetricsEvaluator()->settings().useYoshikawa);
  EXPECT_DOUBLE_EQ(interface.getArmMetricsEvaluator()->settings().minSingularRef, 0.2);
  std::remove(path.c_str());
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
  EXPECT_EQ(
    manager->getEndEffectorTarget().stateTrajectory.front()(0), 20.0);
  EXPECT_TRUE(wholeBodyCost->isActive(0.0));
  EXPECT_TRUE(endEffectorCost->isActive(0.0));
}

TEST(WbmmReferenceManager, RvalueSetRoutesByStateDimension)
{
  // This test calls through ReferenceManagerInterface so it catches a
  // missing rvalue overload: RosReferenceManager / MPC reset use std::move,
  // which must still land in the dual-reference buffers.
  auto manager = std::make_shared<wbmm_ocs2::WbmmReferenceManager>(
    wbmm_ocs2::TaskPhase::kNavigation, 9, 7);
  std::shared_ptr<ocs2::ReferenceManagerInterface> genericManager = manager;

  const ocs2::vector_t wholeBodyTarget = sampleState();
  ocs2::vector_t endEffectorTarget(7);
  endEffectorTarget << 0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0;

  genericManager->setTargetTrajectories(
    ocs2::TargetTrajectories({0.0}, {wholeBodyTarget}));
  genericManager->setTargetTrajectories(
    ocs2::TargetTrajectories({0.0}, {endEffectorTarget}));

  manager->preSolverRun(0.0, 1.0, wholeBodyTarget);

  ASSERT_EQ(manager->getWholeBodyTarget().stateTrajectory.size(), 1U);
  ASSERT_EQ(manager->getEndEffectorTarget().stateTrajectory.size(), 1U);
  EXPECT_EQ(manager->getWholeBodyTarget().stateTrajectory.front().size(), 9);
  EXPECT_EQ(manager->getEndEffectorTarget().stateTrajectory.front().size(), 7);
  EXPECT_TRUE(
    manager->getWholeBodyTarget().stateTrajectory.front().isApprox(
      wholeBodyTarget));
  EXPECT_TRUE(
    manager->getEndEffectorTarget().stateTrajectory.front().isApprox(
      endEffectorTarget));
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
  EXPECT_TRUE(problem.stateCostPtr->getTermIndex("armManipulability", index));
  EXPECT_TRUE(interface.isArmManipulabilityEnabled());

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

TEST(WbmmInterface, LoadsEsdfBackendAndQueriesGrid)
{
#ifndef WBMM_OCS2_TEST_ESDF_TASK_FILE
#error "WBMM_OCS2_TEST_ESDF_TASK_FILE must be defined"
#endif
#ifndef WBMM_OCS2_TEST_ESDF_NPZ
#error "WBMM_OCS2_TEST_ESDF_NPZ must be defined"
#endif
  wbmm_ocs2::WbmmInterface interface(
    WBMM_OCS2_TEST_ESDF_TASK_FILE,
    "/tmp/wbmm_ocs2_esdf_interface_test_lib",
    testRobotUrdfPath(),
    WBMM_OCS2_TEST_ESDF_NPZ,
    "odom");

  EXPECT_TRUE(interface.isEnvironmentCollisionEnabled());
  const auto environment = interface.getEsdfEnvironmentInterface();
  ASSERT_NE(environment, nullptr);
  EXPECT_EQ(environment->getFrameId(), "odom");
  ASSERT_GT(environment->getNumSpheres(), 0U);

  wbmm_ocs2::WbmmPreComputation preComputation(
    interface.getPinocchioInterface(), interface.getWbmmModelInfo());
  const auto state = sampleState();
  const ocs2::vector_t input =
    ocs2::vector_t::Zero(static_cast<Eigen::Index>(interface.getWbmmModelInfo().inputDim));
  preComputation.request(
    ocs2::Request::Constraint + ocs2::Request::Approximation,
    0.0, state, input);

  const auto distances =
    environment->computeDistances(preComputation.getPinocchioInterface());
  ASSERT_EQ(distances.size(), environment->getNumSpheres());
  std::size_t valid_queries = 0U;
  for (const auto & distance : distances) {
    if (distance.gradientValid) {
      ++valid_queries;
    }
  }
  EXPECT_GT(valid_queries, 0U);
}

TEST(WbmmInterface, RejectsEsdfFrameMismatch)
{
#ifndef WBMM_OCS2_TEST_ESDF_TASK_FILE
#error "WBMM_OCS2_TEST_ESDF_TASK_FILE must be defined"
#endif
#ifndef WBMM_OCS2_TEST_ESDF_NPZ
#error "WBMM_OCS2_TEST_ESDF_NPZ must be defined"
#endif
  EXPECT_THROW(
    wbmm_ocs2::WbmmInterface(
      WBMM_OCS2_TEST_ESDF_TASK_FILE,
      "/tmp/wbmm_ocs2_esdf_frame_mismatch_test_lib",
      testRobotUrdfPath(),
      WBMM_OCS2_TEST_ESDF_NPZ,
      "map"),
    std::runtime_error);
}

TEST(WbmmReferenceManager, NavigationClearsStaleEndEffectorTarget)
{
  const auto state = sampleState();
  auto manager = std::make_shared<wbmm_ocs2::WbmmReferenceManager>(
    wbmm_ocs2::TaskPhase::kExecution);

  ocs2::vector_t wholeBodyValue = state;
  ocs2::vector_t endEffectorValue(7);
  endEffectorValue << 0.5, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0;

  manager->setWholeBodyTarget(
    ocs2::TargetTrajectories({0.0}, {wholeBodyValue}));
  manager->setEndEffectorTarget(
    ocs2::TargetTrajectories({0.0}, {endEffectorValue}));
  manager->preSolverRun(0.0, 1.0, state);
  ASSERT_FALSE(manager->getEndEffectorTarget().empty());

  // Starting a new navigation goal must invalidate the old EE target.
  manager->setTaskPhase(wbmm_ocs2::TaskPhase::kNavigation);
  manager->preSolverRun(0.0, 1.0, state);
  EXPECT_TRUE(manager->getEndEffectorTarget().empty());

  auto endEffectorCost = std::make_unique<wbmm_ocs2::PhaseWeightedStateCost>(
    std::make_unique<TargetEchoCost>(), manager,
    wbmm_ocs2::TargetKind::kEndEffector,
    std::array<ocs2::scalar_t, 4>{0.0, 0.5, 1.0, 0.0});
  manager->setTaskPhase(wbmm_ocs2::TaskPhase::kTransition);
  manager->preSolverRun(0.0, 1.0, state);
  EXPECT_FALSE(endEffectorCost->isActive(0.0));
}
