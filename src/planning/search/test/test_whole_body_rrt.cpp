#include <gtest/gtest.h>
#include <stdexcept>
#include <wbmm_search/whole_body_rrt.hpp>

namespace {
using namespace wbmm::search;
wbmm::core::JointState joints(double q = 0) { return {{"joint"}, {q}, {0}, {0}}; }
wbmm::core::RobotLimits limits() {
  wbmm::core::RobotLimits l;
  l.joint_min = {-2};
  l.joint_max = {2};
  l.max_joint_speed = {1};
  l.max_base_speed = 0.5;
  l.max_base_yaw_rate = 1;
  return l;
}
BaseSearchResult path() {
  BaseSearchResult p;
  p.path.push_back({});
  for (int i = 0; i < 5; ++i) {
    p.primitives.push_back({0.5, 0, 0.4});
    p.path.push_back(propagate(p.path.back(), p.primitives.back(), 0.4));
  }
  p.success = true;
  return p;
}
TEST(WholeBodyRrt, KeepsStartAndEscapesGreedyDeadEnd) {
  // Arm must retract before entering a gate; carrying q=0 reaches a dead end.
  auto checker = [](const auto &, const auto &s) {
    return s.base.x < 0.55 || s.joints.positions[0] < -0.8;
  };
  WholeBodyRrtConfig c;
  c.random_seed = 7;
  c.max_search_time = 2;
  auto r = sampleArmRrt({"map", 0}, path(), joints(), limits(), checker, c);
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_EQ(r.search.arm_seed.front().positions[0], 0);
  EXPECT_LT(r.search.arm_seed.back().positions[0], -0.8);
  EXPECT_EQ(r.primitives.size() + 1, r.search.base_path.size());
  for (std::size_t i = 0; i < r.primitives.size(); ++i)
    for (int k = 0; k <= 50; ++k) {
      wbmm::core::WholeBodyState s;
      s.base =
          propagate(r.search.base_path[i], r.primitives[i], r.primitives[i].duration * k / 50.0);
      s.joints = joints(
          r.search.arm_seed[i].positions[0] +
          k / 50.0 * (r.search.arm_seed[i + 1].positions[0] - r.search.arm_seed[i].positions[0]));
      EXPECT_TRUE(checker(wbmm::core::Header{}, s));
    }
  auto repeated = sampleArmRrt({"map", 0}, path(), joints(), limits(), checker, c);
  ASSERT_TRUE(repeated.success);
  EXPECT_EQ(r.generated_nodes, repeated.generated_nodes);
}
TEST(WholeBodyRrt, FindsDetourWhenFixedBasePathIsBlockedForEveryArm) {
  auto checker = [](const auto &, const auto &s) {
    return !(s.base.x > 0.35 && s.base.x < 0.65 && std::abs(s.base.y) < 0.22);
  };
  WholeBodyRrtConfig c;
  c.max_iterations = 1000;
  c.max_search_time = 3;
  EXPECT_FALSE(sampleArmRrt({"map", 0}, path(), joints(), limits(), checker, c).success);
  KinoAstarConfig b;
  b.min_x = -0.5;
  b.max_x = 1.5;
  b.min_y = -1;
  b.max_y = 1;
  wbmm::core::BaseState goal;
  goal.x = 1;
  auto r = searchWholeBodyRrt(
      {"map", 0}, {}, goal, joints(), limits(), [](const auto &, const auto &) { return true; },
      checker, b, c);
  ASSERT_TRUE(r.success) << r.message;
  bool detour = false;
  for (const auto &p : r.search.base_path)
    detour |= std::abs(p.y) > 0.22;
  EXPECT_TRUE(detour);
}
TEST(WholeBodyRrt, SupportsTerminalArmGoalAndStationaryBase) {
  BaseSearchResult p;
  p.path.push_back({});
  auto r = sampleArmRrt(
      {"map", 0}, p, joints(), limits(), [](const auto &, const auto &) { return true; }, {},
      joints(1.2));
  ASSERT_TRUE(r.success) << r.message;
  EXPECT_DOUBLE_EQ(r.search.arm_seed.front().positions[0], 0);
  EXPECT_NEAR(r.search.arm_seed.back().positions[0], 1.2, 1e-8);
}
TEST(WholeBodyRrt, CollisionErrorsAreFatalAndDoNotEscape) {
  auto r =
      sampleArmRrt({"map", 0}, path(), joints(), limits(), [](const auto &, const auto &) -> bool {
        throw std::runtime_error("map offline");
      });
  EXPECT_FALSE(r.success);
  EXPECT_TRUE(r.fatal);
  EXPECT_NE(r.message.find("map offline"), std::string::npos);
}
TEST(WholeBodyRrt, RejectsMalformedPrimitiveAndLimits) {
  auto p = path();
  p.primitives[0].v = 1;
  auto free = [](const auto &, const auto &) { return true; };
  EXPECT_TRUE(sampleArmRrt({"map", 0}, p, joints(), limits(), free).fatal);
  auto l = limits();
  l.joint_min.clear();
  EXPECT_TRUE(sampleArmRrt({"map", 0}, path(), joints(), l, free).fatal);
  WholeBodyRrtConfig c;
  c.collision_angle_step = 0;
  EXPECT_TRUE(sampleArmRrt({"map", 0}, path(), joints(), limits(), free, c).fatal);
}
} // namespace
