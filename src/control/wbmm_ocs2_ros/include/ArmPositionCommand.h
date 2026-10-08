#pragma once

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

namespace wbmm {

// Limit successive position commands in robot time, while bounding how far
// the commanded position can lead measured feedback. A disjoint envelope
// means feedback jumped: the caller must stop/hold rather than break either
// limit to catch up. This function does not integrate an MPC velocity input.
inline bool boundedArmPositionCommand(
    const std::vector<double>& measured, const std::vector<double>& target,
    const std::vector<double>& previous, double dt, double maxVelocity,
    double maxLead, std::vector<double>& command) {
  command.clear();
  if (measured.empty() || measured.size() != target.size() ||
      measured.size() != previous.size() || !std::isfinite(dt) || dt < 0.0 ||
      !std::isfinite(maxVelocity) || maxVelocity <= 0.0 ||
      !std::isfinite(maxLead) || maxLead <= 0.0) { return false; }
  const double step = dt * maxVelocity;
  if (!std::isfinite(step)) { return false; }
  std::vector<double> result(measured.size());
  for (std::size_t i = 0; i < measured.size(); ++i) {
    if (!std::isfinite(measured[i]) || !std::isfinite(target[i]) ||
        !std::isfinite(previous[i])) { return false; }
    const double lower = std::max(previous[i] - step, measured[i] - maxLead);
    const double upper = std::min(previous[i] + step, measured[i] + maxLead);
    if (lower > upper) { return false; }
    result[i] = std::clamp(target[i], lower, upper);
  }
  command = std::move(result);
  return true;
}

}  // namespace wbmm
