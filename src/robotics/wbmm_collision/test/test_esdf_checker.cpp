#include "wbmm_collision/esdf_checker.hpp"

#include <wbmm_robot_model/wbmm_robot_model.hpp>

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{

// A small robot with the same relevant geometry as the real one: the base
// collision spheres sit on base_link, and the arm spheres sit one fixed hop
// higher (z = 0.5). Six controlled joints keep the core 9D state contract.
constexpr char kCheckerUrdf[] = R"(<?xml version="1.0"?>
<robot name="checker_test">
  <link name="base_link"/>
  <link name="arm_link"/>
  <link name="link_2"/>
  <link name="link_3"/>
  <link name="link_4"/>
  <link name="link_5"/>
  <link name="link_6"/>
  <link name="link_7"/>
  <joint name="base_to_arm" type="fixed">
    <parent link="base_link"/><child link="arm_link"/>
    <origin xyz="0 0 0.5"/>
  </joint>
  <joint name="joint_1" type="continuous">
    <parent link="arm_link"/><child link="link_2"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="joint_2" type="continuous">
    <parent link="link_2"/><child link="link_3"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="joint_3" type="continuous">
    <parent link="link_3"/><child link="link_4"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="joint_4" type="continuous">
    <parent link="link_4"/><child link="link_5"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="joint_5" type="continuous">
    <parent link="link_5"/><child link="link_6"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="joint_6" type="continuous">
    <parent link="link_6"/><child link="link_7"/><axis xyz="0 0 1"/>
  </joint>
</robot>
)";

wbmm::pinocchio::KinematicModelPtr makeKinematicModel()
{
  static const auto model = []() {
      auto description =
        wbmm::robot_model::parseRobotDescription(kCheckerUrdf, "checker_test");
      auto config = wbmm::robot_model::RobotModelConfig::defaultsFor(description);
      config.state_base_frame = "base_link";
      return wbmm::pinocchio::KinematicModel::create(
        std::make_shared<const wbmm::robot_model::RobotDescription>(description),
        std::move(config));
    }();
  return model;
}

std::shared_ptr<const wbmm::environment::EsdfGrid> makeLinearGrid(
  bool observed = true)
{
  using wbmm::environment::EsdfGrid;
  using wbmm::environment::EsdfGridData;

  constexpr int kSize = 10;
  constexpr double kVoxelSize = 1.0;
  const Eigen::Vector3d origin = Eigen::Vector3d::Zero();

  EsdfGridData data;
  data.info.frame_id = "odom";
  data.info.origin = origin;
  data.info.voxel_size = kVoxelSize;
  data.info.shape = Eigen::Vector3i(kSize, kSize, kSize);

  const std::size_t voxel_count =
    static_cast<std::size_t>(kSize * kSize * kSize);
  data.esdf.resize(voxel_count);
  data.occupancy.assign(voxel_count, 0U);
  data.observed.assign(voxel_count, observed ? 1U : 0U);

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

wbmm::robot_model::CollisionSphereModel makeCollisionModel(
  double base_radius = 0.2, double arm_radius = 0.2)
{
  wbmm::robot_model::CollisionSphereModel model;
  model.success = true;
  model.message = "ok";
  model.root_link = "base_link";
  model.base_collision_link = "base_link";
  model.base.name = "base";
  model.base.owner_link = "base_link";
  model.base.spheres.push_back({
    "base_sphere", "base_link", Eigen::Vector3d::Zero(), base_radius, {"base"},
    "base_link/collision_0"});
  wbmm::robot_model::CollisionSphereGroup arm;
  arm.name = "joint_1";
  arm.owner_link = "arm_link";
  arm.spheres.push_back({
    "arm_sphere", "arm_link", Eigen::Vector3d::Zero(), arm_radius,
    {"arm", "joint_1"}, "arm_link/collision_0"});
  model.arm.push_back(std::move(arm));
  model.arm_joint_names = {"joint_1"};
  return model;
}

wbmm::core::WholeBodyState makeState(
  double x = 0.5, double y = 0.5, const std::string & frame_id = "odom")
{
  wbmm::core::WholeBodyState state;
  state.header.frame_id = frame_id;
  state.header.stamp = 0.0;
  state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
  state.base.x = x;
  state.base.y = y;
  state.base.yaw = 0.0;
  state.joints.names = makeKinematicModel()->controlledJointNames();
  state.joints.positions.assign(state.joints.names.size(), 0.0);
  return state;
}

class EsdfCheckerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    kinematic_ = makeKinematicModel();
    environment_ = makeLinearGrid();
    model_ = makeCollisionModel();
  }

  wbmm::pinocchio::KinematicModelPtr kinematic_;
  std::shared_ptr<const wbmm::environment::EsdfGrid> environment_;
  wbmm::robot_model::CollisionSphereModel model_;
};

}  // namespace

TEST_F(EsdfCheckerTest, FreeWhenAllClearancesArePositive)
{
  wbmm::collision::CollisionCheckOptions options;
  options.safety_margin = 0.1;
  wbmm::collision::EsdfChecker checker(
    kinematic_, environment_, model_, options);

  const auto result = checker.check(makeState(), wbmm::collision::CheckScope::kWholeBody);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kFree);
  EXPECT_TRUE(result.isFree());
  EXPECT_NEAR(result.min_clearance, 0.2, 1.0e-9);
  EXPECT_FALSE(result.closest_sphere.empty());
  EXPECT_EQ(checker.sphereCount(), 2U);
}

TEST_F(EsdfCheckerTest, ReportsCollisionWhenRadiusExceedsDistance)
{
  auto model = makeCollisionModel(0.6, 0.2);
  wbmm::collision::EsdfChecker checker(
    kinematic_, environment_, model);
  const auto result = checker.check(makeState(), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kCollision);
  EXPECT_FALSE(result.isFree());
  EXPECT_LT(result.min_clearance, 0.0);
}

TEST_F(EsdfCheckerTest, BaseScopeIgnoresArmSpheres)
{
  auto model = makeCollisionModel(0.2, 0.6);
  wbmm::collision::EsdfChecker checker(
    kinematic_, environment_, model);

  const auto base_result = checker.check(makeState(), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(base_result.status, wbmm::collision::CollisionStatus::kFree);

  const auto whole_body_result = checker.check(makeState());
  EXPECT_EQ(whole_body_result.status, wbmm::collision::CollisionStatus::kCollision);
}

TEST_F(EsdfCheckerTest, CheckBaseUsesOnlyBaseSpheres)
{
  auto model = makeCollisionModel(0.2, 0.6);
  wbmm::collision::EsdfChecker checker(
    kinematic_, environment_, model);

  const auto result = checker.checkBase(
    wbmm::core::Header{"odom", 0.0}, makeState().base);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kFree);
}

TEST_F(EsdfCheckerTest, RejectsFrameMismatch)
{
  wbmm::collision::EsdfChecker checker(
    kinematic_, environment_, model_);
  const auto result = checker.check(
    makeState(0.5, 0.5, "map"), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kFrameMismatch);
}

TEST_F(EsdfCheckerTest, RejectsOutOfBoundsCenters)
{
  wbmm::collision::EsdfChecker checker(
    kinematic_, environment_, model_);
  const auto result = checker.check(
    makeState(20.0, 0.5), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kOutOfBounds);
}

// Default policy: an unobserved stencil is trusted, matching the deployed
// map1 metadata (unknown_is_occupied = false) and the REMANI behaviour being
// replaced. The clearance itself is unchanged, so this is kFree.
TEST_F(EsdfCheckerTest, TrustsUnobservedSpaceByDefault)
{
  environment_ = makeLinearGrid(false);
  wbmm::collision::EsdfChecker checker(
    kinematic_, environment_, model_);
  const auto result = checker.check(
    makeState(), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kFree);
  EXPECT_TRUE(result.isFree());
}

// Opt-in conservative policy: the same query is rejected.
TEST_F(EsdfCheckerTest, RejectsUnknownSpaceWhenOptedIn)
{
  environment_ = makeLinearGrid(false);
  wbmm::collision::CollisionCheckOptions options;
  options.treat_unknown_as_occupied = true;
  wbmm::collision::EsdfChecker checker(
    kinematic_, environment_, model_, options);
  const auto result = checker.check(
    makeState(), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kUnknownSpace);
}

TEST_F(EsdfCheckerTest, RejectsInvalidModels)
{
  wbmm::collision::EsdfChecker null_model_checker(
    nullptr, environment_, model_);
  EXPECT_EQ(
    null_model_checker.check(makeState()).status,
    wbmm::collision::CollisionStatus::kModelError);
  EXPECT_EQ(null_model_checker.sphereCount(), 0U);

  wbmm::collision::EsdfChecker empty_model_checker(
    kinematic_, environment_, {});
  EXPECT_EQ(
    empty_model_checker.check(makeState()).status,
    wbmm::collision::CollisionStatus::kInvalidInput);
}

TEST_F(EsdfCheckerTest, RejectsInvalidSphereMetadata)
{
  auto model = makeCollisionModel();
  model.base.spheres.front().radius = -1.0;
  wbmm::collision::EsdfChecker checker(
    kinematic_, environment_, model);
  const auto result = checker.check(
    makeState(), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kInvalidInput);
}

TEST_F(EsdfCheckerTest, RejectsStructurallyInvalidWholeBodyState)
{
  wbmm::collision::EsdfChecker checker(
    kinematic_, environment_, model_);
  auto state = makeState();
  state.joints.names.clear();
  state.joints.positions.clear();
  const auto result = checker.check(state);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kInvalidInput);
}
