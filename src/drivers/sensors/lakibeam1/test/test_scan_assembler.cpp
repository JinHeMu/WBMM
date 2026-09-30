#include "scan_assembler.hpp"
#include "msop_fixture.hpp"
#include <array>
#include <cmath>
#include <gtest/gtest.h>

namespace {
using namespace msop_test;
struct Stream {
  std::vector<lakibeam::Scan> scans;
  lakibeam::ScanAssembler assembler{25., [this](const auto &s) { scans.push_back(s); }};
  uint32_t stamp = 100000;
  int64_t receive = 2000000000;
  Packet send(int angle, int valid = 12, int step = 400) {
    auto p = packet(angle, stamp, valid, step);
    EXPECT_TRUE(assembler.packet(p.data(), p.size(), receive));
    stamp += static_cast<uint32_t>(std::llround(valid * step / 36000. * 40000.));
    receive += std::llround(valid * step / 36000. * 40000000.);
    return p;
  }
  void turn(int omit = -1) {
    for (int i = 0; i < 8; ++i) {
      if (i == omit) {
        stamp += 5333;
        receive += 5333333;
      } else send(i * 4800, i == 7 ? 6 : 12);
    }
  }
};
}  // namespace

TEST(ScanAssembler, FullTurnsFirstRayStampAndNoOverlap) {
  Stream s;
  s.turn(); s.turn(); s.turn(); s.send(0);
  ASSERT_EQ(s.scans.size(), 3u);
  for (size_t i = 0; i < s.scans.size(); ++i) {
    const auto &scan = s.scans[i];
    ASSERT_EQ(scan.ranges.size(), 1440u);
    EXPECT_NEAR(scan.duration, .04, 1e-5);
    EXPECT_NEAR(scan.ranges.front(), 1., 1e-6);
    EXPECT_NEAR(scan.ranges.back(), 2.439, 1e-6);
    EXPECT_EQ(scan.intensities.back(), 42.f);
    if (i > 0) {
      const auto &previous = s.scans[i - 1];
      EXPECT_GT(scan.start_ns, previous.start_ns);
      EXPECT_GT(scan.start_ns, previous.start_ns +
                std::llround(previous.duration * 1e9 * 1439 / 1440));
    }
  }
  EXPECT_LT(s.scans.front().start_ns, 2000000000);
}

TEST(ScanAssembler, DropInitialPartialAndPreserveMissingAngles) {
  Stream s;
  s.send(24000); s.send(28800); s.send(33600, 6);
  s.turn(2); s.send(0);
  ASSERT_EQ(s.scans.size(), 1u);
  EXPECT_TRUE(std::isinf(s.scans[0].ranges[384]));
  EXPECT_NEAR(s.scans[0].ranges[576], 1.576, 1e-6);
  EXPECT_EQ(s.scans[0].ranges.size(), 1440u);
}

TEST(ScanAssembler, DuplicateLatePacketAndSmallRollbackNeverCreateTurn) {
  Stream s;
  auto earlier = s.send(0);
  s.send(4800);
  EXPECT_FALSE(s.assembler.packet(earlier.data(), earlier.size(), s.receive));
  auto rollback = packet(4400, s.stamp);
  EXPECT_TRUE(s.assembler.packet(rollback.data(), rollback.size(), s.receive));
  s.stamp += 5333; s.receive += 5333333;
  EXPECT_TRUE(s.scans.empty());
  s.send(9600); s.send(14400); s.send(19200); s.send(24000);
  s.send(28800); s.send(33600, 6); s.send(0);
  ASSERT_EQ(s.scans.size(), 1u);
}

TEST(ScanAssembler, BoundaryInsidePacketAndUint32TimestampRollover) {
  Stream s;
  s.stamp = 0xffffe000u;
  s.send(33600);  // wraps between blocks 5 and 6; initial fragment discarded
  for (int a : {2400, 7200, 12000, 16800, 21600, 26400, 31200}) s.send(a);
  s.send(0);
  ASSERT_EQ(s.scans.size(), 1u);
  EXPECT_NEAR(s.scans[0].duration, .04, 1e-5);
}

TEST(ScanAssembler, InvalidLengthFlagsAnglesAndZeroRange) {
  Stream s;
  auto p = packet(0, s.stamp);
  EXPECT_FALSE(s.assembler.packet(p.data(), p.size() - 1, s.receive));
  put16(p.data(), 0);
  EXPECT_FALSE(s.assembler.packet(p.data(), p.size(), s.receive));
  p = packet(0, s.stamp); put16(p.data() + 2, 36000);
  EXPECT_FALSE(s.assembler.packet(p.data(), p.size(), s.receive));
  p = packet(0, s.stamp); put16(p.data() + 4, 0);
  EXPECT_TRUE(s.assembler.packet(p.data(), p.size(), s.receive));
  s.stamp += 5333; s.receive += 5333333;
  for (int a : {4800, 9600, 14400, 19200, 24000, 28800}) s.send(a);
  s.send(33600, 6); s.send(0);
  ASSERT_EQ(s.scans.size(), 1u);
  EXPECT_TRUE(std::isinf(s.scans[0].ranges.front()));
}

TEST(ScanAssembler, HalfResolutionAndClockRegression) {
  Stream s;
  for (int turn = 0; turn < 2; ++turn)
    for (int a = 0; a < 36000; a += 2400) s.send(a, 12, 200);
  s.send(0, 12, 200);
  ASSERT_EQ(s.scans.size(), 2u);
  EXPECT_EQ(s.scans[0].ranges.size(), 2880u);
  const auto count = s.scans.size();
  s.receive -= 1000000000;
  for (int turn = 0; turn < 3; ++turn)
    for (int a = 0; a < 36000; a += 2400) s.send(a, 12, 200);
  EXPECT_EQ(s.scans.size(), count);
}

TEST(ScanAssembler, MeasuresActualFrequencyWithoutReconfiguringHardware) {
  std::vector<lakibeam::Scan> scans;
  lakibeam::ScanAssembler assembler(30., [&](const auto &scan) { scans.push_back(scan); });
  uint32_t stamp = 100000;
  int64_t receive = 2000000000;
  for (int turn = 0; turn < 3; ++turn) {
    for (int i = 0; i < 15; ++i) {
      auto p = msop_test::packet(i * 2400, stamp, 12, 200);
      ASSERT_TRUE(assembler.packet(p.data(), p.size(), receive));
      stamp += 6667; receive += 6666667;
    }
  }
  ASSERT_EQ(scans.size(), 2u);
  EXPECT_NEAR(scans[0].duration, .1, 1e-4);
  EXPECT_NEAR(scans[1].duration, .1, 1e-4);
}
