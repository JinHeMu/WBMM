#include "ArmPositionCommand.h"
#include <gtest/gtest.h>
#include <limits>

TEST(ArmPositionCommand, ReversalsRespectVelocityInRobotTime) {
  std::vector<double> out;
  ASSERT_TRUE(wbmm::boundedArmPositionCommand({0, 0}, {1, -1}, {0, 0}, .008, .3, .05, out));
  EXPECT_NEAR(out[0], .0024, 1e-12);
  EXPECT_NEAR(out[1], -.0024, 1e-12);
  const auto previous = out;
  ASSERT_TRUE(wbmm::boundedArmPositionCommand({0, 0}, {-1, 1}, previous, .004, .3, .05, out));
  EXPECT_NEAR(out[0], .0012, 1e-12);
  EXPECT_NEAR(out[1], -.0012, 1e-12);
}

TEST(ArmPositionCommand, PausedClockKeepsPreviousCommand) {
  std::vector<double> out;
  ASSERT_TRUE(wbmm::boundedArmPositionCommand({0}, {1}, {.01}, 0, .3, .05, out));
  EXPECT_DOUBLE_EQ(out[0], .01);
}

TEST(ArmPositionCommand, LeadCannotAccumulateWhenActuatorIsBlocked) {
  std::vector<double> previous{0}, out;
  for (int i = 0; i < 1000; ++i) {
    ASSERT_TRUE(wbmm::boundedArmPositionCommand({0}, {1}, previous, .008, .3, .05, out));
    previous = out;
  }
  EXPECT_DOUBLE_EQ(out[0], .05);
}

TEST(ArmPositionCommand, FeedbackJumpRequiresStopRatherThanBreakingVelocityLimit) {
  std::vector<double> out{99};
  EXPECT_FALSE(wbmm::boundedArmPositionCommand({1}, {1}, {0}, .008, .3, .05, out));
  EXPECT_TRUE(out.empty());
}

TEST(ArmPositionCommand, RejectsInvalidInputsAtomically) {
  std::vector<double> out{99};
  EXPECT_FALSE(wbmm::boundedArmPositionCommand({0}, {1}, {0}, -.1, .3, .05, out));
  EXPECT_TRUE(out.empty());
  EXPECT_FALSE(wbmm::boundedArmPositionCommand({0}, {1}, {0}, .1, 0, .05, out));
  EXPECT_FALSE(wbmm::boundedArmPositionCommand({0, 0}, {1}, {0}, .1, .3, .05, out));
  EXPECT_FALSE(wbmm::boundedArmPositionCommand({0, 0}, {1, std::numeric_limits<double>::quiet_NaN()},
                                              {0, 0}, .1, .3, .05, out));
  EXPECT_TRUE(out.empty());
}
