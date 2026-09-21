#include "wbmm_collision/environment_collision_checker.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(CollisionSkeleton, OnlyAnExplicitFreeResultAcceptsAState)
{
  using wbmm::collision::CollisionStatus;
  wbmm::collision::CollisionResult result;
  for (const auto status : {
      CollisionStatus::kNotImplemented, CollisionStatus::kCollision,
      CollisionStatus::kInvalidInput, CollisionStatus::kFrameMismatch,
      CollisionStatus::kUnknownSpace, CollisionStatus::kOutOfBounds,
      CollisionStatus::kModelError})
  {
    result.status = status;
    EXPECT_FALSE(result.isFree());
  }
  result.status = CollisionStatus::kFree;
  EXPECT_TRUE(result.isFree());
}

TEST(CollisionSkeleton, BothEntryPointsRejectUnimplementedChecks)
{
  const wbmm::collision::EnvironmentCollisionChecker checker(nullptr, nullptr, {});
  const auto base = checker.checkBase({}, {});
  const auto whole_body = checker.check({});
  EXPECT_EQ(base.status, wbmm::collision::CollisionStatus::kNotImplemented);
  EXPECT_EQ(whole_body.status, wbmm::collision::CollisionStatus::kNotImplemented);
  EXPECT_FALSE(base.isFree());
  EXPECT_FALSE(whole_body.isFree());
  EXPECT_TRUE(std::isnan(base.min_clearance));
  EXPECT_TRUE(std::isnan(whole_body.min_clearance));
}
