#pragma once

#include <cmath>
#include <cstddef>
#include <vector>

namespace jaka_hardware_interface {

// No SDK or ROS dependency: all dispatch is injected, so safety behaviour can
// be tested offline without opening a connection to the robot.
class ServoCommandStream {
 public:
  void setLimits(double tracking_error, double feedback_velocity) {
    max_tracking_error_ = tracking_error;
    max_feedback_velocity_ = feedback_velocity;
  }

  bool faulted() const { return fault_ != nullptr; }
  const char* faultReason() const { return fault_ ? fault_ : "none"; }

  template <typename Stop>
  bool trip(const char* reason, Stop stop) {
    if (!fault_) { fault_ = reason; }
    if (!stop_requested_) { stop_requested_ = stop(); }
    return false;
  }

  template <typename Send, typename Stop>
  bool update(const std::vector<double>& command,
              const std::vector<double>& feedback,
              const std::vector<double>& velocity,
              Send send, Stop stop) {
    if (faulted()) { return trip(fault_, stop); }
    if (command.size() != 6 || feedback.size() != 6 || velocity.size() != 6) {
      return trip("invalid servo sample", stop);
    }
    for (std::size_t i = 0; i < 6; ++i) {
      if (!std::isfinite(command[i]) || !std::isfinite(feedback[i]) ||
          !std::isfinite(velocity[i])) {
        return trip("non-finite servo command/feedback", stop);
      }
      if (max_tracking_error_ > 0.0 &&
          std::abs(command[i] - feedback[i]) > max_tracking_error_) {
        return trip("joint position tracking error", stop);
      }
      if (max_feedback_velocity_ > 0.0 &&
          std::abs(velocity[i]) > max_feedback_velocity_) {
        return trip("joint feedback velocity exceeded", stop);
      }
    }
    // A fixed setpoint is still a cyclic servo command. Suppressing identical
    // packets after a position step can leave the last increment in effect.
    if (!send()) { return trip("servo command send failed", stop); }
    return true;
  }

 private:
  double max_tracking_error_{0.0};
  double max_feedback_velocity_{0.0};
  const char* fault_{nullptr};
  bool stop_requested_{false};
};

}  // namespace jaka_hardware_interface
