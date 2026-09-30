#include "wbmm_collision/self_collision_checker.hpp"

#include <wbmm_robot_model/wbmm_robot_model.hpp>

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

namespace
{
using namespace wbmm::collision;

SphereSample sphere(const std::string & id, double x, double radius = 0.25)
{
  SphereSample sample;
  sample.id = id;
  sample.position = Eigen::Vector3d(x, 0.0, 0.0);
  sample.radius = radius;
  sample.jacobian = Eigen::MatrixXd::Zero(3, 7);
  return sample;
}

TrajectorySpheres sample()
{
  TrajectorySpheres out;
  out.success = true;
  out.variable_count = 7;
  out.base = {sphere("base", 0.0)};
  out.arm = {sphere("arm", 1.0)};
  out.arm_group = {0};
  return out;
}

TEST(SelfCollisionChecker, ClearanceMarginAndClosestPair)
{
  const auto spheres = sample();
  const auto free = checkSelfCollision(spheres, 0.1);
  ASSERT_TRUE(free.isFree()) << free.message;
  ASSERT_EQ(free.pairs.size(), 1U);
  EXPECT_NEAR(free.min_clearance, 0.4, 1e-12);
  EXPECT_EQ(free.closest_first_sphere, "arm");
  EXPECT_EQ(free.closest_second_sphere, "base");
  EXPECT_EQ(checkSelfCollision(spheres, 0.5).status, CollisionStatus::kCollision);
  EXPECT_EQ(checkSelfCollision(spheres, 0.6).status, CollisionStatus::kCollision);
  auto overlap = spheres;
  overlap.arm[0].position.x() = 0.4;
  EXPECT_NEAR(checkSelfCollision(overlap).min_clearance, -0.1, 1e-12);
}

TEST(SelfCollisionChecker, PreservesExistingGroupPolicyAndPairOrder)
{
  auto spheres = sample();
  spheres.base.push_back(sphere("base2", 0.0));
  spheres.arm = {sphere("a0", 1.0), sphere("b0", 1.0),
    sphere("a1", 1.0), sphere("a2", 1.0)};
  spheres.arm_group = {0, 0, 1, 2};
  const auto checked = checkSelfCollision(spheres);
  ASSERT_TRUE(checked.status == CollisionStatus::kCollision);
  // 4 * 2 arm/base pairs + (a0,a2) + (b0,a2); no base/base pairs.
  ASSERT_EQ(checked.pairs.size(), 10U);
  EXPECT_EQ(checked.pairs[0].first_sphere, "a0");
  EXPECT_EQ(checked.pairs[0].second_sphere, "base");
  EXPECT_EQ(checked.pairs[2].first_sphere, "a0");
  EXPECT_EQ(checked.pairs[2].second_sphere, "a2");
  EXPECT_EQ(checked.pairs[5].first_sphere, "b0");
  EXPECT_EQ(checked.pairs[5].second_sphere, "a2");
}

TEST(SelfCollisionChecker, CoincidentCentersAreCollisionWithFiniteGradient)
{
  auto spheres = sample();
  spheres.arm[0].position.setZero();
  spheres.arm[0].jacobian(0, 4) = 1.0;
  const auto checked = checkSelfCollision(spheres);
  ASSERT_EQ(checked.status, CollisionStatus::kCollision);
  EXPECT_DOUBLE_EQ(checked.min_clearance, -0.5);
  ASSERT_EQ(checked.pairs.size(), 1U);
  EXPECT_TRUE(checked.pairs[0].gradient.isZero());
}

TEST(SelfCollisionChecker, CloseCentersRetainDistanceGradient)
{
  auto spheres = sample();
  spheres.arm[0].position.x() = 1e-10;
  spheres.arm[0].jacobian(0, 4) = 1.0;
  const auto checked = checkSelfCollision(spheres);
  ASSERT_EQ(checked.status, CollisionStatus::kCollision);
  ASSERT_EQ(checked.pairs.size(), 1U);
  EXPECT_DOUBLE_EQ(checked.pairs[0].gradient(4), 1.0);
}

TEST(SelfCollisionChecker, NoEligiblePairsIsFree)
{
  auto spheres = sample();
  spheres.arm.clear();
  spheres.arm_group.clear();
  const auto checked = checkSelfCollision(spheres);
  EXPECT_TRUE(checked.isFree());
  EXPECT_TRUE(checked.pairs.empty());
  EXPECT_EQ(checked.min_clearance, std::numeric_limits<double>::infinity());
  EXPECT_TRUE(checked.closest_first_sphere.empty());
}

TEST(SelfCollisionChecker, RejectsInvalidInputWithoutPartialResults)
{
  const auto expectInvalid = [](const TrajectorySpheres & spheres, double margin = 0.0) {
      const auto checked = checkSelfCollision(spheres, margin);
      EXPECT_EQ(checked.status, CollisionStatus::kInvalidInput);
      EXPECT_FALSE(checked.isFree());
      EXPECT_TRUE(std::isnan(checked.min_clearance));
      EXPECT_TRUE(checked.pairs.empty());
      EXPECT_FALSE(checked.message.empty());
    };
  expectInvalid(sample(), -0.1);
  expectInvalid(sample(), std::numeric_limits<double>::quiet_NaN());
  auto bad = sample();
  bad.success = false;
  expectInvalid(bad);
  bad = sample();
  bad.arm_group.clear();
  expectInvalid(bad);
  bad = sample();
  bad.arm_group[0] = 3;
  expectInvalid(bad);
  bad = sample();
  bad.arm[0].radius = -1.0;
  expectInvalid(bad);
  bad = sample();
  bad.base[0].position.x() = std::numeric_limits<double>::infinity();
  expectInvalid(bad);
  bad = sample();
  bad.arm[0].jacobian.resize(3, 6);
  expectInvalid(bad);
  bad = sample();
  bad.arm[0].jacobian(0, 0) = std::numeric_limits<double>::quiet_NaN();
  expectInvalid(bad);
  bad = sample();
  bad.variable_count = 3;
  expectInvalid(bad);
}

#ifdef WBMM_TEST_URDF
TEST(SelfCollisionChecker, RealModelClearanceGradientsMatchFiniteDifferences)
{
  auto description = wbmm::robot_model::loadRobotDescription(WBMM_TEST_URDF);
  auto config = wbmm::robot_model::RobotModelConfig::defaultsFor(description);
  config.state_base_frame = "base_footprint";
  config.base_collision_link = "base_link";
  const auto built = wbmm::robot_model::buildRobotModelDescription(description, config);
  ASSERT_TRUE(built.success) << built.message;
  Eigen::VectorXd z(10);
  z << 0.5, -0.3, 0.3, 0.2, 0.4, 0.9, -0.6, 1.2, -0.3, 0.7;
  const auto evaluate = [&](const Eigen::VectorXd & variables, int gear) {
      return checkSelfCollision(evaluateTrajectorySpheres(
        built.model.description, built.model.collision_spheres,
        variables.head<2>(), variables.segment<2>(2), gear, variables.tail(6)), 0.1);
    };
  for (int gear : {1, -1}) {
    const auto analytic = evaluate(z, gear);
    ASSERT_FALSE(analytic.pairs.empty());
    ASSERT_NE(analytic.status, CollisionStatus::kInvalidInput) << analytic.message;
    for (Eigen::Index variable = 0; variable < z.size(); ++variable) {
      auto plus = z, minus = z;
      plus(variable) += 1e-6;
      minus(variable) -= 1e-6;
      const auto upper = evaluate(plus, gear);
      const auto lower = evaluate(minus, gear);
      ASSERT_EQ(upper.pairs.size(), analytic.pairs.size());
      ASSERT_EQ(lower.pairs.size(), analytic.pairs.size());
      for (std::size_t pair = 0; pair < analytic.pairs.size(); ++pair) {
        EXPECT_NEAR(analytic.pairs[pair].gradient(variable),
          (upper.pairs[pair].clearance - lower.pairs[pair].clearance) / 2e-6, 1e-7)
          << "pair=" << pair << " variable=" << variable << " gear=" << gear;
      }
    }
    // Self distances are invariant to the common planar base transform.
    for (const auto & pair : analytic.pairs) {
      EXPECT_LT(pair.gradient.head<4>().norm(), 1e-12);
    }
  }
}
#endif

}  // namespace
