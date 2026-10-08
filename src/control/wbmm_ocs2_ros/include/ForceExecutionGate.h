#pragma once

#include <chrono>
#include <string>

namespace wbmm {

// Access is denied until an ACTIVE heartbeat arrives. Faults and loss of an
// active heartbeat latch until an explicit reset with fresh ACTIVE input.
// The caller serializes access between callbacks and the execution thread.
class ForceExecutionGate {
 public:
  using Clock = std::chrono::steady_clock;
  using Time = Clock::time_point;

  void update(const std::string& state, Time now) {
    received_ = true;
    active_ = state == "ACTIVE";
    last_ = now;
    if (state.rfind("FAULT_", 0) == 0) {
      fault_ = true;
    }
  }

  bool allow(Time now, double timeout) {
    const bool fresh = received_ &&
        std::chrono::duration<double>(now - last_).count() <= timeout;
    if (active_ && !fresh) {
      fault_ = true;
    }
    return active_ && fresh && !fault_;
  }

  bool reset(Time now, double timeout) {
    if (!received_ || !active_ ||
        std::chrono::duration<double>(now - last_).count() > timeout) {
      return false;
    }
    fault_ = false;
    return true;
  }

  void fault() { fault_ = true; }
  bool faulted() const { return fault_; }

 private:
  bool received_{false};
  bool active_{false};
  bool fault_{false};
  Time last_{};
};

}  // namespace wbmm
