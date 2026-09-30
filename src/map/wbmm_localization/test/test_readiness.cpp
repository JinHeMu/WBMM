#include "wbmm_localization/readiness.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <limits>

using namespace wbmm_localization;
namespace {
constexpr double pi = 3.14159265358979323846;
Grid wall(Pose2d origin = {0., 0., 0.}) {
  std::vector<int8_t> data(100, 0);
  for (int y = 0; y < 10; ++y)
    data[y * 10 + 5] = 100;
  return Grid(10, 10, 0.1, origin, data, 0.12);
}
} // namespace

TEST(Readiness, WallAlignmentWrongPoseAndRotatedOrigin) {
  std::vector<Point> points, wrong, rotated;
  for (int i = 0; i < 10; ++i) {
    const double y = .05 + i * .1;
    points.push_back({.55, y});
    wrong.push_back({.85, y});
    rotated.push_back({2 - y, -1 + .55});
  }
  EXPECT_DOUBLE_EQ(wall().agreement(points).match_ratio, 1.);
  EXPECT_DOUBLE_EQ(wall().agreement(wrong).match_ratio, 0.);
  EXPECT_DOUBLE_EQ(wall({2., -1., pi / 2}).agreement(rotated).match_ratio, 1.);
}

TEST(Readiness, UnknownAndOutOfBoundsStayInDenominator) {
  std::vector<int8_t> data(100, 0);
  data[0] = -1;
  data[55] = 100;
  const auto score = Grid(10, 10, .1, {0, 0, 0}, data, .12)
                         .agreement({{.55, .55}, {.05, .05}, {-1, .5}});
  EXPECT_EQ(score.endpoints, 3u);
  EXPECT_DOUBLE_EQ(score.known_ratio, 1. / 3);
  EXPECT_DOUBLE_EQ(score.match_ratio, 1. / 3);
}

TEST(Readiness, CellCenterDistanceAndEmptyGrid) {
  EXPECT_DOUBLE_EQ(wall().agreement({{.43, .55}}).match_ratio, 1.);
  EXPECT_DOUBLE_EQ(wall().agreement({{.42, .55}}).match_ratio, 0.);
  EXPECT_DOUBLE_EQ(
      (Grid(10, 10, .1, {0, 0, 0}, std::vector<int8_t>(100, 0), .12)
           .agreement({{.55, .55}})
           .match_ratio),
      0.);
}

TEST(Readiness, RejectInvalidGrid) {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((Grid(1, 1, 0, {0, 0, 0}, {0}, .1)), std::invalid_argument);
  EXPECT_THROW((Grid(1, 1, nan, {0, 0, 0}, {0}, .1)), std::invalid_argument);
  EXPECT_THROW((Grid(1, 1, .1, {0, 0, 0}, {-2}, .1)), std::invalid_argument);
  EXPECT_THROW((Grid(1, 1, .1, {0, 0, 0}, {}, .1)), std::invalid_argument);
}

TEST(Readiness, ScanFilteringOffsetAndRotation) {
  const auto rotation =
      quaternionMatrix({0, 0, std::sin(pi / 4), std::cos(pi / 4)});
  const auto points = scanEndpoints({1, INFINITY, NAN, 0, 30, 1}, 0, pi / 2,
                                    .08, 30, rotation, {2, 3, 0}, 10);
  ASSERT_EQ(points.size(), 2u);
  EXPECT_NEAR(points[0][0], 2, 1e-10);
  EXPECT_NEAR(points[0][1], 4, 1e-10);
  EXPECT_NEAR(points[1][0], 1, 1e-10);
  EXPECT_NEAR(points[1][1], 3, 1e-10);
  EXPECT_THROW((quaternionMatrix({0, 0, 0, 0})), std::invalid_argument);
  EXPECT_LE(scanEndpoints(std::vector<float>(1000, 1), 0, .001, .08, 30,
                          quaternionMatrix({0, 0, 0, 1}), {0, 0, 0}, 180)
                .size(),
            180u);
}

TEST(Readiness, StabilityNeedsDistinctScansAndFullDuration) {
  StableWindow window(3., 5);
  for (int i = 0; i < 20; ++i)
    EXPECT_FALSE(window.update(1, true, {0, 0, 0}));
  for (double t : {1.5, 2., 2.5, 3., 3.5})
    EXPECT_FALSE(window.update(t, true, {0, 0, 0}));
  EXPECT_TRUE(window.update(4, true, {0, 0, 0}));
  EXPECT_TRUE(window.update(4.5, true, {0, 0, 0}));
  EXPECT_FALSE(window.update(5, false, {0, 0, 0}));
}

TEST(Readiness, CorrectionJumpClockResetAndYawWrap) {
  StableWindow window(1., 2);
  window.update(1, true, {0, 0, pi - .01});
  EXPECT_TRUE(window.update(2, true, {0, 0, -pi + .01}));
  EXPECT_FALSE(window.update(2.1, true, {1, 0, 0}));
  EXPECT_FALSE(window.update(1, true, {1, 0, 0}));
  EXPECT_FALSE(window.update(2, true, {NAN, 0, 0}));
}

TEST(Readiness, StatusRejectsMalformedStaleFutureAndWrongBackend) {
  nlohmann::json status = {{"ready", true},
                           {"stamp", 10.},
                           {"backend", "cartographer_localization"}};
  EXPECT_TRUE(statusIsReady(status, 10.2, "cartographer_localization"));
  EXPECT_FALSE(statusIsReady(status, 12., "cartographer_localization"));
  EXPECT_FALSE(statusIsReady(status, 9., "cartographer_localization"));
  EXPECT_FALSE(statusIsReady(status, 10.2, "amcl"));
  status["ready"] = "true";
  EXPECT_FALSE(statusIsReady(status, 10.2));
  status["ready"] = true;
  status["stamp"] = NAN;
  EXPECT_FALSE(statusIsReady(status, 10.2));
  EXPECT_FALSE(statusIsReady(nlohmann::json::array(), 10.));
  EXPECT_FALSE(
      statusIsReady(nlohmann::json::parse("broken", nullptr, false), 10.));
  EXPECT_FALSE(fresh(0, 0, 1));
}
