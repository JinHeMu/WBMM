#include "ForceExecutionGate.h"
#include <gtest/gtest.h>

using Gate = wbmm::ForceExecutionGate;
using namespace std::chrono_literals;

TEST(ForceExecutionGate, NoMotionBeforeActiveAndDuringTareOrDisable) {
  Gate gate;
  const Gate::Time now{};
  EXPECT_FALSE(gate.allow(now, 0.25));
  gate.update("TARING", now);
  EXPECT_FALSE(gate.allow(now, 0.25));
  gate.update("ACTIVE", now);
  EXPECT_TRUE(gate.allow(now + 100ms, 0.25));
  gate.update("DISABLED", now + 100ms);
  EXPECT_FALSE(gate.allow(now + 100ms, 0.25));
}

TEST(ForceExecutionGate, FaultRemainsLatchedWhenForceDataReturns) {
  Gate gate;
  const Gate::Time now{};
  gate.update("ACTIVE", now);
  gate.update("FAULT_WRENCH_LIMIT", now + 10ms);
  EXPECT_FALSE(gate.allow(now + 10ms, 0.25));
  gate.update("ACTIVE", now + 20ms);
  EXPECT_FALSE(gate.allow(now + 20ms, 0.25));
  EXPECT_TRUE(gate.reset(now + 20ms, 0.25));
  EXPECT_TRUE(gate.allow(now + 20ms, 0.25));
}

TEST(ForceExecutionGate, LostProcessHeartbeatLatchesAndRequiresFreshReset) {
  Gate gate;
  const Gate::Time now{};
  gate.update("ACTIVE", now);
  EXPECT_FALSE(gate.allow(now + 251ms, 0.25));
  EXPECT_TRUE(gate.faulted());
  EXPECT_FALSE(gate.reset(now + 251ms, 0.25));
  gate.update("ACTIVE", now + 300ms);
  EXPECT_FALSE(gate.allow(now + 300ms, 0.25));
  EXPECT_TRUE(gate.reset(now + 300ms, 0.25));
  EXPECT_TRUE(gate.allow(now + 300ms, 0.25));
}
