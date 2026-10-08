#include "jaka_hardware_interface/servo_command_stream.hpp"
#include <gtest/gtest.h>
#include <limits>

using jaka_hardware_interface::ServoCommandStream;

TEST(ServoCommandStream, IdenticalHoldCommandsAreSentEveryCycle) {
  ServoCommandStream stream;
  int sends = 0, stops = 0;
  const std::vector<double> q(6, 0), v(6, 0);
  for (int i = 0; i < 500; ++i) {
    ASSERT_TRUE(stream.update(q, q, v, [&] { ++sends; return true; },
                             [&] { ++stops; return true; }));
  }
  EXPECT_EQ(sends, 500);
  EXPECT_EQ(stops, 0);
}

TEST(ServoCommandStream, SendFailureStopsAndLatchesWithoutResuming) {
  ServoCommandStream stream;
  const std::vector<double> q(6, 0), v(6, 0);
  int sends = 0, stops = 0;
  EXPECT_FALSE(stream.update(q, q, v, [&] { ++sends; return false; },
                            [&] { ++stops; return true; }));
  for (int i = 0; i < 10; ++i) {
    EXPECT_FALSE(stream.update(q, q, v, [&] { ++sends; return true; },
                              [&] { ++stops; return true; }));
  }
  EXPECT_EQ(sends, 1);
  EXPECT_EQ(stops, 1);
  EXPECT_TRUE(stream.faulted());
}

TEST(ServoCommandStream, RejectedStopIsRetriedWhileMotionRemainsBlocked) {
  ServoCommandStream stream;
  const std::vector<double> q(6, 0), v(6, 0);
  int sends = 0, stops = 0;
  for (int i = 0; i < 3; ++i) {
    EXPECT_FALSE(stream.update(q, q, v, [&] { ++sends; return false; },
                              [&] { ++stops; return stops == 3; }));
  }
  EXPECT_EQ(sends, 1);
  EXPECT_EQ(stops, 3);
}

TEST(ServoCommandStream, Bag04RunawayTripsBeforeSendingAnotherCommand) {
  ServoCommandStream stream;
  stream.setLimits(.0532, .4);
  const std::vector<double> q(6, 0);
  auto v = q;
  v[1] = .49768;  // Bag 04 J2, approximately 56 ms after the force interlock.
  int sends = 0, stops = 0;
  EXPECT_FALSE(stream.update(q, q, v, [&] { ++sends; return true; },
                            [&] { ++stops; return true; }));
  EXPECT_EQ(sends, 0);
  EXPECT_EQ(stops, 1);
  EXPECT_STREQ(stream.faultReason(), "joint feedback velocity exceeded");
}

TEST(ServoCommandStream, TrackingErrorAndInvalidSamplesTrip) {
  const std::vector<double> q(6, 0), v(6, 0);
  ServoCommandStream tracking;
  tracking.setLimits(.0532, .4);
  auto command = q;
  command[1] = .06;
  EXPECT_FALSE(tracking.update(command, q, v, [] { return true; }, [] { return true; }));
  EXPECT_STREQ(tracking.faultReason(), "joint position tracking error");
  ServoCommandStream invalid;
  command[1] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(invalid.update(command, q, v, [] { return true; }, [] { return true; }));
}
