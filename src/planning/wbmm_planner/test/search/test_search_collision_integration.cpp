#include "wbmm_collision/esdf_checker.hpp"
#include "wbmm_planner/search/kino_astar.hpp"

#include <wbmm_robot_model/wbmm_robot_model.hpp>

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>

namespace
{

// Minimal robot with the base_link as the URDF root, so the shared
// KinematicModel places base_link exactly at the planning state (x, y, yaw).
constexpr char kSearchUrdf[] = R"(<?xml version="1.0"?>
<robot name="search_fake">
  <link name="base_link"/>
  <link name="tool"/>
  <joint name="joint_1" type="continuous">
    <parent link="base_link"/><child link="tool"/>
    <axis xyz="0 0 1"/>
  </joint>
</robot>
)";

wbmm::pinocchio::KinematicModelPtr makeKinematicModel()
{
  static const auto model = []() {
      auto description =
        wbmm::robot_model::parseRobotDescription(kSearchUrdf, "search_fake");
      auto config = wbmm::robot_model::RobotModelConfig::defaultsFor(description);
      config.state_base_frame = "base_link";
      return wbmm::pinocchio::KinematicModel::create(
        std::make_shared<const wbmm::robot_model::RobotDescription>(description),
        std::move(config));
    }();
  return model;
}

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

wbmm::robot_model::CollisionSphereModel makeBaseModel()
{
  wbmm::robot_model::CollisionSphereModel model;
  model.success = true;
  model.message = "ok";
  model.root_link = "base_link";
  model.base_collision_link = "base_link";
  model.base.name = "base";
  model.base.owner_link = "base_link";
  model.base.spheres.push_back({
    "base_sphere", "base_link", Eigen::Vector3d::Zero(), 0.2, {"base"},
    "base_link/collision_0"});
  return model;
}

}  // namespace

TEST(SearchCollisionIntegration, KinoAstarUsesBaseEnvironmentChecker)
{
  auto kinematic_model = makeKinematicModel();
  auto environment = makeWallGrid();

  wbmm::collision::CollisionCheckOptions options;
  options.safety_margin = 0.1;
  wbmm::collision::EsdfChecker collision_checker(
    kinematic_model, environment, makeBaseModel(), options);

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
