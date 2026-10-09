#pragma once

#include <chrono>
#include <cstdint>

namespace whole_body_force_control {

// Measure between samples that reach the processor, including tare samples.
// The stamped path is validated by the ROS adapter before calling this helper.
// Legacy unstamped inputs retain their steady-clock interval.
class SampleInterval {
public:
  using Clock = std::chrono::steady_clock;

  double next(int64_t stamp_ns, Clock::time_point wall_time,
              bool use_stamp, double initial_dt) {
    double dt = initial_dt;
    if (initialized_) {
      dt = use_stamp ? static_cast<double>(stamp_ns - previous_stamp_ns_) * 1e-9
                     : std::chrono::duration<double>(wall_time - previous_wall_).count();
    }
    previous_stamp_ns_ = stamp_ns;
    previous_wall_ = wall_time;
    initialized_ = true;
    return dt;
  }

  void reset() { initialized_ = false; }

private:
  bool initialized_{false};
  int64_t previous_stamp_ns_{0};
  Clock::time_point previous_wall_{};
};

}  // namespace whole_body_force_control
