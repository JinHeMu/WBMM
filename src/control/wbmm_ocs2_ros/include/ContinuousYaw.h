#pragma once

#include <cmath>

namespace wbmm {

// Odometry quaternions wrap at +/- pi; the MPC dynamics and warm-start
// controller operate on a continuous yaw coordinate. Call under the state lock.
class ContinuousYaw {
public:
  double update(double yaw) {
    if (initialized_) {
      const double delta = yaw - previous_;
      value_ += std::atan2(std::sin(delta), std::cos(delta));
    } else {
      value_ = yaw;
      initialized_ = true;
    }
    previous_ = yaw;
    return value_;
  }

private:
  bool initialized_{false};
  double previous_{0.0};
  double value_{0.0};
};

}  // namespace wbmm
