#include "wbmm_environment/esdf_loader.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>

namespace
{

std::size_t address(
  const Eigen::Vector3i & index, const Eigen::Vector3i & shape)
{
  return (static_cast<std::size_t>(index.x()) *
            static_cast<std::size_t>(shape.y()) +
          static_cast<std::size_t>(index.y())) *
           static_cast<std::size_t>(shape.z()) +
         static_cast<std::size_t>(index.z());
}

}  // namespace

TEST(EnvironmentEsdf, LoadsAndQueriesTrilinearDistanceAndGradient)
{
  const auto loaded = wbmm::environment::NpzEsdfLoader::load(WBMM_ENV_TEST_NPZ);
  ASSERT_EQ(loaded.status, wbmm::environment::LoadStatus::kSuccess);
  ASSERT_NE(loaded.grid, nullptr);

  const auto & info = loaded.grid->info();
  EXPECT_EQ(info.frame_id, "odom");
  EXPECT_EQ(info.shape, Eigen::Vector3i(3, 3, 3));
  EXPECT_DOUBLE_EQ(info.voxel_size, 1.0);

  const auto query =
    loaded.grid->query("odom", Eigen::Vector3d(1.25, 1.5, 1.5));
  ASSERT_EQ(query.status, wbmm::environment::QueryStatus::kSuccess);
  EXPECT_TRUE(query.gradient_valid);
  EXPECT_NEAR(query.distance, 1.25, 1e-6);
  EXPECT_NEAR(query.gradient.x(), 1.0, 1e-6);
  EXPECT_NEAR(query.gradient.y(), 0.0, 1e-6);
  EXPECT_NEAR(query.gradient.z(), 0.0, 1e-6);
}

TEST(EnvironmentEsdf, ClampsWithinBoundaryHalfVoxelLayer)
{
  const auto loaded = wbmm::environment::NpzEsdfLoader::load(WBMM_ENV_TEST_NPZ);
  ASSERT_EQ(loaded.status, wbmm::environment::LoadStatus::kSuccess);
  ASSERT_NE(loaded.grid, nullptr);

  // x=2.9 is inside [origin_x, bounds_max_x] but beyond the last voxel
  // center (2.5). The query should clamp to the last voxel value instead of
  // being rejected as out of bounds.
  const auto query =
    loaded.grid->query("odom", Eigen::Vector3d(2.9, 1.5, 1.5));
  ASSERT_EQ(query.status, wbmm::environment::QueryStatus::kSuccess);
  EXPECT_TRUE(query.gradient_valid);
  EXPECT_NEAR(query.distance, 2.5, 1e-6);
}

TEST(EnvironmentEsdf, RejectsFrameMismatchAndOutOfBounds)
{
  const auto loaded = wbmm::environment::NpzEsdfLoader::load(WBMM_ENV_TEST_NPZ);
  ASSERT_EQ(loaded.status, wbmm::environment::LoadStatus::kSuccess);
  ASSERT_NE(loaded.grid, nullptr);

  const auto frame_mismatch =
    loaded.grid->query("map", Eigen::Vector3d(1.0, 1.0, 1.0));
  EXPECT_EQ(
    frame_mismatch.status,
    wbmm::environment::QueryStatus::kFrameMismatch);

  const auto out_of_bounds =
    loaded.grid->query("odom", Eigen::Vector3d(4.0, 1.0, 1.0));
  EXPECT_EQ(
    out_of_bounds.status,
    wbmm::environment::QueryStatus::kOutOfBounds);
  EXPECT_FALSE(out_of_bounds.gradient_valid);
}

TEST(EnvironmentEsdf, MarksUnobservedInterpolationCornersAsUnknown)
{
  const auto loaded = wbmm::environment::NpzEsdfLoader::load(WBMM_ENV_TEST_NPZ);
  ASSERT_EQ(loaded.status, wbmm::environment::LoadStatus::kSuccess);
  ASSERT_NE(loaded.grid, nullptr);

  auto data = loaded.grid->data();
  const Eigen::Vector3i shape = data.info.shape;
  const Eigen::Vector3i corner(1, 1, 1);
  data.observed[address(corner, shape)] = 0U;

  const wbmm::environment::EsdfGrid grid_with_unknown(std::move(data));
  const auto query =
    grid_with_unknown.query("odom", Eigen::Vector3d(1.25, 1.5, 1.5));

  EXPECT_EQ(query.status, wbmm::environment::QueryStatus::kUnknown);
  EXPECT_FALSE(query.gradient_valid);
  EXPECT_FALSE(query.gradient.allFinite());
}

TEST(EnvironmentEsdf, MissingObservedArrayIsTreatedAsFullyObserved)
{
  const auto loaded =
    wbmm::environment::NpzEsdfLoader::load(WBMM_ENV_TEST_NPZ_NO_OBSERVED);
  ASSERT_EQ(loaded.status, wbmm::environment::LoadStatus::kSuccess);
  ASSERT_NE(loaded.grid, nullptr);

  const auto & data = loaded.grid->data();
  EXPECT_EQ(data.observed.size(), data.esdf.size());

  const auto query =
    loaded.grid->query("odom", Eigen::Vector3d(1.25, 1.5, 1.5));
  EXPECT_EQ(query.status, wbmm::environment::QueryStatus::kSuccess);
  EXPECT_TRUE(query.gradient_valid);
  EXPECT_NEAR(query.distance, 1.25, 1e-6);
}

TEST(EnvironmentEsdf, MissingFileIsIoError)
{
  const auto loaded =
    wbmm::environment::NpzEsdfLoader::load("/tmp/wbmm_missing_esdf_file.npz");
  EXPECT_EQ(loaded.status, wbmm::environment::LoadStatus::kIoError);
  EXPECT_EQ(loaded.grid, nullptr);
}
