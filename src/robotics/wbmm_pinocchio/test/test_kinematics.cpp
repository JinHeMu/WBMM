#include <wbmm_pinocchio/wbmm_pinocchio.hpp>
#include <wbmm_robot_model/wbmm_robot_model.hpp>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{

#ifdef WBMM_TEST_URDF
wbmm::pinocchio::KinematicModelPtr realModel()
{
  static const auto model = [] {
    auto description = wbmm::robot_model::loadRobotDescription(WBMM_TEST_URDF);
    auto config = wbmm::robot_model::RobotModelConfig::defaultsFor(description);
    return wbmm::pinocchio::KinematicModel::create(
      std::make_shared<const wbmm::robot_model::RobotDescription>(
        std::move(description)), std::move(config));
  }();
  return model;
}

wbmm::core::WholeBodyState referenceState()
{
  wbmm::core::WholeBodyState state;
  state.header.frame_id = "odom";
  state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
  state.base.x = 1.2;
  state.base.y = -0.4;
  state.base.yaw = 0.7;
  state.joints.names = realModel()->controlledJointNames();
  state.joints.positions = {0.1, -0.3, 0.5, 1.2, -0.8, 0.9};
  return state;
}

TEST(KinematicModel, ExposesTheSemanticContract)
{
  const auto model = realModel();
  EXPECT_EQ(model->rootFrame(), "base_footprint");
  EXPECT_EQ(model->stateBaseFrame(), "base_footprint");
  EXPECT_TRUE(model->rootFromStateBase().matrix().isApprox(
    Eigen::Matrix4d::Identity(), 1e-12));
  EXPECT_EQ(model->stateDimension(), 9U);
  EXPECT_EQ(model->inputDimension(), 8U);
  const std::vector<std::string> expected = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  EXPECT_EQ(model->controlledJointNames(), expected);
  EXPECT_TRUE(model->hasFrame("tool0"));
  EXPECT_TRUE(model->hasFrame("base_footprint"));
  EXPECT_FALSE(model->hasFrame("not_a_link"));
}

TEST(KinematicModel, RejectsStatesWithWrongJointSets)
{
  const auto model = realModel();
  auto state = referenceState();

  Eigen::VectorXd configuration;
  std::string message;
  EXPECT_TRUE(model->configurationFromState(state, configuration, &message))
    << message;

  state.joints.names[2] = "joint_9";
  EXPECT_FALSE(model->configurationFromState(state, configuration, &message));
  EXPECT_NE(message.find("unknown joint"), std::string::npos);

  state = referenceState();
  state.joints.positions.pop_back();
  EXPECT_FALSE(model->configurationFromState(state, configuration, &message));
}

// Golden values captured from the pre-refactor PinocchioRobotModel on the
// deployed URDF. The unified kinematics must reproduce them exactly.
TEST(FrameKinematics, ReproducesTheLegacyForwardKinematics)
{
  auto model = realModel();
  wbmm::pinocchio::KinematicsData kinematics_data(model);
  ASSERT_TRUE(kinematics_data.update(referenceState()));

  wbmm::pinocchio::FrameKinematics frames(model);

  Eigen::Isometry3d base_footprint;
  ASSERT_TRUE(frames.framePlacement(kinematics_data, "base_footprint", base_footprint));
  EXPECT_NEAR(base_footprint.translation().x(), 1.2, 1e-12);
  EXPECT_NEAR(base_footprint.translation().y(), -0.4, 1e-12);
  EXPECT_NEAR(base_footprint.translation().z(), 0.0, 1e-12);

  Eigen::Isometry3d base_link;
  ASSERT_TRUE(frames.framePlacement(kinematics_data, "base_link", base_link));
  EXPECT_NEAR(base_link.translation().z(), 0.147, 1e-12);

  Eigen::Isometry3d tool;
  ASSERT_TRUE(frames.framePlacement(kinematics_data, "tool0", tool));
  EXPECT_NEAR(tool.translation().x(), 1.75148210816222, 1e-9);
  EXPECT_NEAR(tool.translation().y(), -1.17239639300588, 1e-9);
  EXPECT_NEAR(tool.translation().z(), 0.703758036626783, 1e-9);
  const Eigen::Quaterniond orientation(tool.linear());
  EXPECT_NEAR(std::abs(orientation.w()), 0.842390607280234, 1e-9);
  EXPECT_NEAR(std::abs(orientation.x()), 0.20041883272547, 1e-9);
  EXPECT_NEAR(std::abs(orientation.y()), 0.326150721828308, 1e-9);
  EXPECT_NEAR(std::abs(orientation.z()), 0.379257251619361, 1e-9);

  const auto placements = frames.linkPlacements(kinematics_data);
  ASSERT_EQ(placements.count("tool0"), 1U);
  EXPECT_TRUE(placements.at("tool0").matrix().isApprox(tool.matrix(), 1e-12));
}

TEST(FrameKinematics, ReproducesTheLegacyFrameJacobian)
{
  auto model = realModel();
  wbmm::pinocchio::KinematicsData kinematics_data(model);
  ASSERT_TRUE(kinematics_data.update(
    referenceState(),
    wbmm::pinocchio::UpdateMode::kJacobians));

  wbmm::pinocchio::FrameKinematics frames(model);
  Eigen::MatrixXd jacobian(6, 8);
  ASSERT_TRUE(frames.frameJacobian(kinematics_data, "tool0", jacobian));

  Eigen::MatrixXd expected(6, 8);
  expected <<
    0.764842187284488, 0.772396393005883, 0.772396393005883, -0.154790524608856, -0.246017031249873, -0.193466492894162, -0.238721779101071, 1.06143175815988e-17,
    0.644217687237691, 0.551482108162221, 0.551482108162221, 0.150090692360606, 0.238552248360677, 0.187597503566716, -0.176702054497479, 8.94031636552566e-18,
    0.0, -0.0, 0.0, 0.933607236902957, 0.522812546581718, 0.16165564848462, -0.280464875015621, 1.38777878078145e-17,
    0.0, 0.0, 0.0, -0.696135238622661, -0.696135238622661, -0.696135238622661, 0.707467867180737, -0.397472017907226,
    0.0, 0.0, 0.0, -0.7179106696061, -0.7179106696061, -0.7179106696061, -0.68600319769403, -0.585051937168559,
    0.0, 1.0, 1.0, -3.67320510363811e-06, -3.67320510363811e-06, -3.67320510363811e-06, -0.169967142884455, 0.706915996279668;
  EXPECT_TRUE(jacobian.isApprox(expected, 1e-9));
}

TEST(FrameKinematics, JacobianMatchesFiniteDifferences)
{
  auto model = realModel();
  wbmm::pinocchio::KinematicsData kinematics_data(model);
  const auto nominal = referenceState();
  ASSERT_TRUE(kinematics_data.update(
    nominal,
    wbmm::pinocchio::UpdateMode::kJacobians));

  wbmm::pinocchio::FrameKinematics frames(model);
  Eigen::MatrixXd jacobian(6, 8);
  ASSERT_TRUE(frames.frameJacobian(kinematics_data, "tool0", jacobian));

  constexpr double kStep = 1.0e-6;
  const auto positionAt = [&](const wbmm::core::WholeBodyState & state) {
      wbmm::pinocchio::KinematicsData local(model);
      EXPECT_TRUE(local.update(state));
      Eigen::Isometry3d placement;
      EXPECT_TRUE(frames.framePlacement(local, "tool0", placement));
      // 显式构造 Vector3d：translation() 返回表达式，会在 placement 析构后悬空。
      return Eigen::Vector3d(placement.translation());
    };
  // 绝对 + 相对混合容差：tool0 与末轴共线时该列本来就接近 0，纯相对比较
  // 会把 1e-16 的舍入噪声判成失败。
  const auto expectClose = [](const Eigen::Vector3d & actual,
                              const Eigen::Vector3d & expected,
                              const std::string & label) {
      const double tolerance = 1.0e-8 + 1.0e-6 * expected.norm();
      EXPECT_LT((actual - expected).norm(), tolerance)
        << label << " d=" << actual.transpose() << " e=" << expected.transpose();
    };

  // 底盘线速度列：沿航向的单位平移，所以扰动必须沿 heading，而不是沿 x。
  {
    auto plus = nominal;
    auto minus = nominal;
    plus.base.x += kStep * std::cos(nominal.base.yaw);
    plus.base.y += kStep * std::sin(nominal.base.yaw);
    minus.base.x -= kStep * std::cos(nominal.base.yaw);
    minus.base.y -= kStep * std::sin(nominal.base.yaw);
    const Eigen::Vector3d derivative =
      (positionAt(plus) - positionAt(minus)) / (2.0 * kStep);
    expectClose(derivative, jacobian.col(0).head<3>(), "base linear");
  }
  // 底盘角速度列：绕 state 系 z。
  {
    auto plus = nominal;
    auto minus = nominal;
    plus.base.yaw += kStep;
    minus.base.yaw -= kStep;
    const Eigen::Vector3d derivative =
      (positionAt(plus) - positionAt(minus)) / (2.0 * kStep);
    expectClose(derivative, jacobian.col(1).head<3>(), "base angular");
  }
  // 关节列：d(position)/d(q_j)。
  for (int j = 0; j < 6; ++j) {
    auto plus = nominal;
    auto minus = nominal;
    plus.joints.positions[static_cast<std::size_t>(j)] += kStep;
    minus.joints.positions[static_cast<std::size_t>(j)] -= kStep;
    const Eigen::Vector3d derivative =
      (positionAt(plus) - positionAt(minus)) / (2.0 * kStep);
    expectClose(derivative, jacobian.col(2 + j).head<3>(),
                "joint column " + std::to_string(j));
  }
}

TEST(PinocchioRobotModel, KeepsTheCoreContract)
{
  auto model = std::make_shared<wbmm::pinocchio::PinocchioRobotModel>(realModel());
  EXPECT_EQ(model->stateDimension(), 9U);
  EXPECT_EQ(model->inputDimension(), 8U);
  EXPECT_EQ(model->baseModel(), wbmm::core::BaseModel::kDifferentialDrive);
  EXPECT_EQ(model->jointNames().size(), 6U);
  EXPECT_EQ(model->jointNames().front(), "joint_1");
  EXPECT_EQ(model->jointNames().back(), "joint_6");
  EXPECT_TRUE(model->hasFrame("tool0"));

  wbmm::core::Pose pose;
  EXPECT_TRUE(model->forwardKinematics(referenceState(), "tool0", pose));
  EXPECT_NEAR(pose.position.z, 0.703758036626783, 1e-9);

  auto state = referenceState();
  state.joints.positions[0] = 100.0;
  std::string reason;
  EXPECT_FALSE(model->validate(state, &reason));
  EXPECT_FALSE(reason.empty());
}

TEST(SphereKinematics, PlacesBaseSpheresThroughTheSharedModel)
{
  const auto description = wbmm::robot_model::loadRobotDescription(WBMM_TEST_URDF);
  auto config = wbmm::robot_model::RobotModelConfig::defaultsFor(description);
  config.base_collision_link = "base_link";
  config.state_base_frame = "base_footprint";
  const auto built =
    wbmm::robot_model::buildRobotModelDescription(description, config);
  ASSERT_TRUE(built.success) << built.message;
  const auto aggregate = std::make_shared<const wbmm::robot_model::RobotModelDescription>(
    built.model);
  auto model = wbmm::pinocchio::KinematicModel::create(aggregate);

  wbmm::pinocchio::KinematicsData kinematics_data(model);
  ASSERT_TRUE(kinematics_data.update(referenceState()));
  wbmm::pinocchio::SphereKinematics spheres(model, aggregate->collision_spheres);

  std::vector<wbmm::pinocchio::SphereSample> samples;
  ASSERT_TRUE(spheres.centers(kinematics_data, samples));
  EXPECT_EQ(samples.size(), aggregate->collision_spheres.sphereCount());

  // 第一颗底盘球在 base_link 系里是 (0.2, -0.17, 0.07)；base_footprint ->
  // base_link 抬高 0.147，因此 z 必须正好是 0.217。
  ASSERT_FALSE(aggregate->collision_spheres.base.spheres.empty());
  EXPECT_NEAR(samples.front().center.z(), 0.217, 1e-9);
}
#endif

}  // namespace
