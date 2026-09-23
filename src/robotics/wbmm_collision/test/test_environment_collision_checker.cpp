#include "wbmm_collision/environment_collision_checker.hpp"

#include <gtest/gtest.h>

#include <Eigen/Geometry>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{

class FakeRobotModel final : public wbmm::core::RobotModel
{
public:
  FakeRobotModel()
  {
    limits_.joint_min.assign(6U, -10.0);
    limits_.joint_max.assign(6U, 10.0);
    limits_.max_joint_speed.assign(6U, 1.0);
    limits_.max_base_speed = 1.0;
    limits_.max_base_yaw_rate = 1.0;
  }

  std::size_t stateDimension() const override {return 9U;}
  std::size_t inputDimension() const override {return 8U;}
  wbmm::core::BaseModel baseModel() const override
  {
    return wbmm::core::BaseModel::kDifferentialDrive;
  }
  const std::vector<std::string> & jointNames() const override
  {
    return joint_names_;
  }
  const wbmm::core::RobotLimits & limits() const override {return limits_;}

  bool forwardKinematics(
    const wbmm::core::WholeBodyState & state, const std::string & link_name,
    wbmm::core::Pose & pose) const override
  {
    if (state.base_model != wbmm::core::BaseModel::kDifferentialDrive) {
      return false;
    }
    pose.header = state.header;
    pose.orientation = quaternionFromYaw(state.base.yaw);
    if (link_name == "base_link") {
      pose.position = {state.base.x, state.base.y, 0.0};
      return true;
    }
    if (link_name == "arm_link") {
      pose.position = {state.base.x, state.base.y, 0.5};
      return true;
    }
    return false;
  }

  bool frameJacobian(
    const wbmm::core::WholeBodyState &, const std::string &,
    Eigen::Ref<Eigen::MatrixXd>) const override
  {
    return false;
  }

  bool validate(
    const wbmm::core::WholeBodyState &, std::string * message = nullptr) const override
  {
    if (message != nullptr) {
      message->clear();
    }
    return true;
  }

  bool validate(
    const wbmm::core::WholeBodyInput &, std::string * message = nullptr) const override
  {
    if (message != nullptr) {
      message->clear();
    }
    return true;
  }

private:
  static wbmm::core::Quaternion quaternionFromYaw(double yaw)
  {
    return {std::cos(0.5 * yaw), 0.0, 0.0, std::sin(0.5 * yaw)};
  }

  std::vector<std::string> joint_names_{
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  wbmm::core::RobotLimits limits_;
};

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

wbmm::collision::CollisionModel makeCollisionModel(
  double base_radius = 0.2, double arm_radius = 0.2)
{
  wbmm::collision::CollisionModel model;
  model.spheres.push_back({
    "base_sphere", "base_link", wbmm::collision::CollisionGroup::kBase,
    Eigen::Vector3d::Zero(), base_radius});
  model.spheres.push_back({
    "arm_sphere", "arm_link", wbmm::collision::CollisionGroup::kArm,
    Eigen::Vector3d::Zero(), arm_radius});
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
  state.joints.names = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  state.joints.positions.assign(6U, 0.0);
  return state;
}

class EnvironmentCollisionCheckerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    robot_model_ = std::make_shared<FakeRobotModel>();
    environment_ = makeLinearGrid();
    model_ = makeCollisionModel();
  }

  std::shared_ptr<FakeRobotModel> robot_model_;
  std::shared_ptr<const wbmm::environment::EsdfGrid> environment_;
  wbmm::collision::CollisionModel model_;
};

}  // namespace

TEST_F(EnvironmentCollisionCheckerTest, FreeWhenAllClearancesArePositive)
{
  wbmm::collision::CollisionCheckOptions options;
  options.safety_margin = 0.1;
  wbmm::collision::EnvironmentCollisionChecker checker(
    robot_model_, environment_, model_, options);

  const auto result = checker.check(makeState(), wbmm::collision::CheckScope::kWholeBody);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kFree);
  EXPECT_TRUE(result.isFree());
  EXPECT_NEAR(result.min_clearance, 0.2, 1.0e-9);
  EXPECT_FALSE(result.closest_sphere.empty());
}

TEST_F(EnvironmentCollisionCheckerTest, ReportsCollisionWhenRadiusExceedsDistance)
{
  auto model = makeCollisionModel(0.6, 0.2);
  wbmm::collision::EnvironmentCollisionChecker checker(
    robot_model_, environment_, std::move(model));
  const auto result = checker.check(makeState(), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kCollision);
  EXPECT_FALSE(result.isFree());
  EXPECT_LT(result.min_clearance, 0.0);
}

TEST_F(EnvironmentCollisionCheckerTest, BaseScopeIgnoresArmSpheres)
{
  auto model = makeCollisionModel(0.2, 0.6);
  wbmm::collision::EnvironmentCollisionChecker checker(
    robot_model_, environment_, std::move(model));

  const auto base_result = checker.check(makeState(), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(base_result.status, wbmm::collision::CollisionStatus::kFree);

  const auto whole_body_result = checker.check(makeState());
  EXPECT_EQ(whole_body_result.status, wbmm::collision::CollisionStatus::kCollision);
}

TEST_F(EnvironmentCollisionCheckerTest, CheckBaseUsesOnlyBaseSpheres)
{
  auto model = makeCollisionModel(0.2, 0.6);
  wbmm::collision::EnvironmentCollisionChecker checker(
    robot_model_, environment_, std::move(model));

  const auto result = checker.checkBase(
    wbmm::core::Header{"odom", 0.0}, makeState().base);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kFree);
}

TEST_F(EnvironmentCollisionCheckerTest, RejectsFrameMismatch)
{
  wbmm::collision::EnvironmentCollisionChecker checker(
    robot_model_, environment_, model_);
  const auto result = checker.check(
    makeState(0.5, 0.5, "map"), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kFrameMismatch);
}

TEST_F(EnvironmentCollisionCheckerTest, RejectsOutOfBoundsCenters)
{
  wbmm::collision::EnvironmentCollisionChecker checker(
    robot_model_, environment_, model_);
  const auto result = checker.check(
    makeState(20.0, 0.5), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kOutOfBounds);
}

TEST_F(EnvironmentCollisionCheckerTest, RejectsUnknownSpace)
{
  environment_ = makeLinearGrid(false);
  wbmm::collision::EnvironmentCollisionChecker checker(
    robot_model_, environment_, model_);
  const auto result = checker.check(
    makeState(), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kUnknownSpace);
}

TEST_F(EnvironmentCollisionCheckerTest, RejectsInvalidModels)
{
  wbmm::collision::EnvironmentCollisionChecker null_model_checker(
    nullptr, environment_, model_);
  EXPECT_EQ(
    null_model_checker.check(makeState()).status,
    wbmm::collision::CollisionStatus::kModelError);

  wbmm::collision::EnvironmentCollisionChecker empty_model_checker(
    robot_model_, environment_, {});
  EXPECT_EQ(
    empty_model_checker.check(makeState()).status,
    wbmm::collision::CollisionStatus::kInvalidInput);
}

TEST_F(EnvironmentCollisionCheckerTest, RejectsInvalidSphereMetadata)
{
  auto model = makeCollisionModel();
  model.spheres.front().radius = -1.0;
  wbmm::collision::EnvironmentCollisionChecker checker(
    robot_model_, environment_, std::move(model));
  const auto result = checker.check(
    makeState(), wbmm::collision::CheckScope::kBase);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kInvalidInput);
}

TEST_F(EnvironmentCollisionCheckerTest, RejectsStructurallyInvalidWholeBodyState)
{
  wbmm::collision::EnvironmentCollisionChecker checker(
    robot_model_, environment_, model_);
  auto state = makeState();
  state.joints.names.clear();
  state.joints.positions.clear();
  const auto result = checker.check(state);
  EXPECT_EQ(result.status, wbmm::collision::CollisionStatus::kInvalidInput);
}
