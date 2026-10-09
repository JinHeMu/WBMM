#include "ContinuousYaw.h"
#include <gtest/gtest.h>

TEST(ContinuousYaw, KeepsInitialCoordinateAndCrossesBothWrapBoundaries) {
  const double pi = std::acos(-1.0);
  wbmm::ContinuousYaw yaw;
  EXPECT_DOUBLE_EQ(yaw.update(3.13), 3.13);
  EXPECT_NEAR(yaw.update(-3.13), 2.0 * pi - 3.13, 1e-12);
  EXPECT_NEAR(yaw.update(3.12), 3.12, 1e-12);
  wbmm::ContinuousYaw reverse;
  EXPECT_DOUBLE_EQ(reverse.update(-3.13), -3.13);
  EXPECT_NEAR(reverse.update(3.13), -2.0 * pi + 3.13, 1e-12);
}

TEST(ContinuousYaw, RecordedDiag02WrapIsASmallPhysicalStep) {
  wbmm::ContinuousYaw yaw;
  const double before = yaw.update(3.1399095);
  const double after = yaw.update(-3.1399348);
  EXPECT_NEAR(after - before, 0.003341007179586, 1e-12);
}

TEST(ContinuousYaw, SupportsMultipleTurnsInEitherDirection) {
  for (double direction : {-1.0, 1.0}) {
    wbmm::ContinuousYaw yaw;
    for (int i = 0; i < 2000; ++i) {
      const double angle = 0.37 + direction * i * 0.02;
      EXPECT_NEAR(yaw.update(std::atan2(std::sin(angle), std::cos(angle))), angle, 1e-10);
    }
  }
}
