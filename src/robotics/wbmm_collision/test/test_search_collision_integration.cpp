#include "wbmm_collision/environment_collision_checker.hpp"
#include "wbmm_search/kino_astar.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{

class SearchFakeRobotModel final : public wbmm::core::RobotModel
{
public:
  SearchFakeRobotModel()
  {
    limits_.joint_min.assign(6U, -10.0);
    limits_.joint_max.assign(6U, 10.0);
    limits_.max_joint_speed.assign(6U, 1.0);
    limits_.max_base_speed = 0.5;
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
    if (state.base_model != wbmm::core::BaseModel::kDifferentialDrive ||
      link_name != "base_link")
    {
      return false;
    }
    pose.header = state.header;
    pose.position = {state.base.x, state.base.y, 0.0};
    pose.orientation = {
      std::cos(0.5 * state.base.yaw), 0.0, 0.0,
      std::sin(0.5 * state.base.yaw)};
    return true;
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
  std::vector<std::string> joint_names_{
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  wbmm::core::RobotLimits limits_;
};

std::shared_ptr<const wbmm::environment::EsdfGrid> makeWallGrid()
{
  using wbmm::environment::EsdfGrid;
  using wbmm::environment::EsdfGridData;

  constexpr int kSize = 20;
  constexpr double kVoxelSize = 1.0;

  EsdfGridData data;
  data.info.frame_id = "odom";
  data.info.origin = Eigen::Vector3d::Zero();
  data.info.voxel_size = kVoxelSize;
  data.info.shape = Eigen::Vector3i(kSize, kSize, kSize);

  const std::size_t voxel_count =
    static_cast<std::size_t>(kSize * kSize * kSize);
  data.esdf.resize(voxel_count);
  data.occupancy.assign(voxel_count, 0U);
  data.observed.assign(voxel_count, 1U);

  for (int ix = 0; ix < kSize; ++ix) {
    for (int iy = 0; iy < kSize; ++iy) {
      for (int iz = 0; iz < kSize; ++iz) {
        const std::size_t index = static_cast<std::size_t>(
          (ix * kSize + iy) * kSize + iz);
        data.esdf[index] = static_cast<float>(ix + 0.5);
      }
    }
  }
  return std::make_shared<const EsdfGrid>(std::move(data));
}

wbmm::collision::CollisionModel makeBaseModel()
{
  wbmm::collision::CollisionModel model;
  model.spheres.push_back({
    "base_sphere", "base_link", wbmm::collision::CollisionGroup::kBase,
    Eigen::Vector3d::Zero(), 0.2});
  return model;
}

}  // namespace

TEST(SearchCollisionIntegration, KinoAstarUsesBaseEnvironmentChecker)
{
  auto robot_model = std::make_shared<SearchFakeRobotModel>();
  auto environment = makeWallGrid();

  wbmm::collision::CollisionCheckOptions options;
  options.safety_margin = 0.1;
  wbmm::collision::EnvironmentCollisionChecker collision_checker(
    robot_model, environment, makeBaseModel(), options);

  const wbmm::search::BaseCollisionChecker checker =
    [&collision_checker](
      const wbmm::core::Header & header, const wbmm::core::BaseState & base)
    {
      return collision_checker.checkBase(header, base).isFree();
    };

  wbmm::search::KinoAstarConfig config;
  config.min_x = 0.0;
  config.max_x = 12.0;
  config.min_y = 0.0;
  config.max_y = 12.0;
  config.position_resolution = 0.25;
  config.yaw_bins = 36U;
  config.max_search_time = 5.0;
  config.collision_mode = wbmm::search::CollisionMode::kRequireChecker;

  wbmm::core::RobotLimits limits;
  limits.max_base_speed = 0.5;
  limits.max_base_yaw_rate = 1.0;

  wbmm::search::KinoAstar planner(config);
  const wbmm::core::Header header{"odom", 0.0};
  wbmm::core::BaseState start;
  start.x = 2.0;
  start.y = 6.0;
  wbmm::core::BaseState goal;
  goal.x = 4.0;
  goal.y = 6.0;

  const auto result = planner.search(header, start, goal, limits, checker);
  EXPECT_TRUE(result.success)
    << wbmm::search::statusName(result.status) << ": " << result.message;
  EXPECT_TRUE(result.collision_checked);
  EXPECT_GT(result.path.size(), 0U);
  for (const auto & state : result.path) {
    EXPECT_GE(state.x, 2.0);
  }
}
