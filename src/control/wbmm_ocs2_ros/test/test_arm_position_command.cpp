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

TEST(ArmHoldCommand, Bag04FaultTransitionCannotBypassVelocityLimit) {
  // Recorded J2: last active command -> measured position at interlock.
  const std::vector<double> target{1.589366}, previous{1.5777620718347975};
  std::vector<double> out;
  ASSERT_TRUE(wbmm::boundedArmHoldCommand(target, previous, .008, .2, out));
  EXPECT_NEAR(out[0] - previous[0], .0016, 1e-12);
  auto command = out;
  for (int i = 0; i < 20; ++i) {
    ASSERT_TRUE(wbmm::boundedArmHoldCommand(target, command, .008, .2, out));
    EXPECT_LE(std::abs(out[0] - command[0]), .0016 + 1e-12);
    EXPECT_LE(out[0], target[0]);
    command = out;
  }
  EXPECT_DOUBLE_EQ(command[0], target[0]);
}

TEST(ArmHoldCommand, StaleOrJumpingFeedbackDoesNotMoveLatchedHoldTarget) {
  std::vector<double> out;
  ASSERT_TRUE(wbmm::boundedArmHoldCommand({0}, {.01}, .008, .2, out));
  EXPECT_NEAR(out[0], .0084, 1e-12);
  ASSERT_TRUE(wbmm::boundedArmHoldCommand({0}, {.0084}, 0, .2, out));
  EXPECT_DOUBLE_EQ(out[0], .0084);
  EXPECT_FALSE(wbmm::boundedArmHoldCommand({0}, {.01}, -.008, .2, out));
  EXPECT_TRUE(out.empty());
  EXPECT_FALSE(wbmm::boundedArmHoldCommand({0}, {std::numeric_limits<double>::infinity()}, .008, .2, out));
  EXPECT_TRUE(out.empty());
}
