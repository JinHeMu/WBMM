#include "wbmm_localization/readiness.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace wbmm_localization {
namespace {
template <class Values> bool finite(const Values &values) {
  return std::all_of(values.begin(), values.end(),
                     [](double value) { return std::isfinite(value); });
}
} // namespace

bool fresh(double stamp, double now, double timeout, double future_tolerance) {
  return std::isfinite(stamp) && std::isfinite(now) && stamp > 0.0 &&
         now - stamp >= -future_tolerance && now - stamp <= timeout;
}

Rotation quaternionMatrix(const Quaternion &q) {
  if (!finite(q))
    throw std::invalid_argument("Invalid quaternion");
  const double norm =
      std::hypot(std::hypot(q[0], q[1]), std::hypot(q[2], q[3]));
  if (norm < 1e-9)
    throw std::invalid_argument("Zero quaternion");
  const double x = q[0] / norm, y = q[1] / norm, z = q[2] / norm,
               w = q[3] / norm;
  return {
      {{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
       {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
       {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}}};
}

double yaw(const Quaternion &q) {
  const auto r = quaternionMatrix(q);
  return std::atan2(r[1][0], r[0][0]);
}

Grid::Grid(int width, int height, double resolution, Pose2d origin,
           std::vector<int8_t> data, double match_distance,
           int occupied_threshold)
    : width_(width), height_(height), radius_(0),
      threshold_(occupied_threshold), resolution_(resolution),
      distance_(match_distance), origin_(origin), data_(std::move(data)) {
  if (width <= 0 || height <= 0 ||
      data_.size() != static_cast<std::size_t>(width) * height ||
      !std::isfinite(resolution) || !finite(origin) ||
      !std::isfinite(match_distance) || resolution <= 0 || match_distance < 0 ||
      occupied_threshold < 1 || occupied_threshold > 100 ||
      std::any_of(data_.begin(), data_.end(),
                  [](int value) { return value < -1 || value > 100; })) {
    throw std::invalid_argument("Invalid occupancy grid or match distance");
  }
  // Bound the neighborhood to the grid, including very small resolutions.
  radius_ = static_cast<int>(
      std::min(std::ceil(distance_ / resolution_) + 1.,
               static_cast<double>(std::max(width_, height_))));
}

Agreement Grid::agreement(const std::vector<Point> &points) const {
  std::size_t total = 0, known = 0, matched = 0;
  const double c = std::cos(origin_[2]), s = std::sin(origin_[2]);
  for (const auto &point : points) {
    if (!finite(point))
      continue;
    ++total;
    const double x =
        (c * (point[0] - origin_[0]) + s * (point[1] - origin_[1])) /
        resolution_;
    const double y =
        (-s * (point[0] - origin_[0]) + c * (point[1] - origin_[1])) /
        resolution_;
    if (!(x >= 0 && x < width_ && y >= 0 && y < height_))
      continue;
    const int ix = static_cast<int>(std::floor(x)),
              iy = static_cast<int>(std::floor(y));
    if (data_[static_cast<std::size_t>(iy) * width_ + ix] < 0)
      continue;
    ++known;
    bool found = false;
    for (int gy = std::max(0, iy - radius_);
         gy <= std::min(height_ - 1, iy + radius_) && !found; ++gy) {
      for (int gx = std::max(0, ix - radius_);
           gx <= std::min(width_ - 1, ix + radius_); ++gx) {
        if (data_[static_cast<std::size_t>(gy) * width_ + gx] >= threshold_ &&
            std::hypot(gx + 0.5 - x, gy + 0.5 - y) * resolution_ <=
                distance_ + 1e-12) {
          found = true;
          break;
        }
      }
    }
    matched += found;
  }
  // Unknown/out-of-map endpoints remain in the denominator.
  return {total, total ? static_cast<double>(known) / total : 0.0,
          total ? static_cast<double>(matched) / total : 0.0};
}

std::vector<Point> scanEndpoints(const std::vector<float> &ranges,
                                 double angle_min, double angle_increment,
                                 double range_min, double range_max,
                                 const Rotation &rotation,
                                 const std::array<double, 3> &translation,
                                 int limit, double max_range) {
  if (limit < 1 ||
      !finite(std::array<double, 5>{angle_min, angle_increment, range_min,
                                    range_max, max_range}) ||
      !finite(translation) || angle_increment == 0 || range_min < 0 ||
      range_max <= range_min ||
      !std::all_of(rotation.begin(), rotation.end(),
                   [](const auto &row) { return finite(row); })) {
    throw std::invalid_argument("Invalid laser scan metadata");
  }
  std::vector<std::size_t> valid;
  for (std::size_t i = 0; i < ranges.size(); ++i) {
    if (std::isfinite(ranges[i]) && ranges[i] >= range_min &&
        ranges[i] < std::min(range_max, max_range)) {
      valid.push_back(i);
    }
  }
  const auto stride =
      std::max<std::size_t>(1, (valid.size() + limit - 1) / limit);
  std::vector<Point> points;
  for (std::size_t j = 0; j < valid.size(); j += stride) {
    const auto i = valid[j];
    const double angle = angle_min + i * angle_increment,
                 x = ranges[i] * std::cos(angle),
                 y = ranges[i] * std::sin(angle);
    points.push_back(
        {translation[0] + rotation[0][0] * x + rotation[0][1] * y,
         translation[1] + rotation[1][0] * x + rotation[1][1] * y});
  }
  return points;
}

StableWindow::StableWindow(double duration, int min_samples,
                           double max_translation, double max_yaw)
    : duration_(duration), max_translation_(max_translation), max_yaw_(max_yaw),
      min_samples_(min_samples) {
  if (!finite(std::array<double, 3>{duration, max_translation, max_yaw}) ||
      duration <= 0 || max_translation <= 0 || max_yaw <= 0 ||
      min_samples < 2) {
    throw std::invalid_argument("Invalid readiness stability thresholds");
  }
}

void StableWindow::reset() {
  samples_.clear();
  ready_ = false;
}

bool StableWindow::update(double stamp, bool good, const Pose2d &correction) {
  if (!good || !std::isfinite(stamp) || !finite(correction)) {
    reset();
    return false;
  }
  if (!samples_.empty() && stamp < samples_.back()[0])
    reset();
  if (!samples_.empty() && stamp == samples_.back()[0])
    return ready_;
  while (samples_.size() > 1 && samples_[1][0] <= stamp - duration_)
    samples_.pop_front();
  for (const auto &sample : samples_) {
    const double da = correction[2] - sample[3];
    if (std::hypot(correction[0] - sample[1], correction[1] - sample[2]) >
            max_translation_ ||
        std::abs(std::atan2(std::sin(da), std::cos(da))) > max_yaw_) {
      reset();
      break;
    }
  }
  samples_.push_back({stamp, correction[0], correction[1], correction[2]});
  ready_ = samples_.size() >= static_cast<std::size_t>(min_samples_) &&
           stamp - samples_.front()[0] >= duration_;
  return ready_;
}

bool statusIsReady(const nlohmann::json &status, double now,
                   const std::string &backend, double timeout) {
  if (!status.is_object() || !status.contains("ready") ||
      !status["ready"].is_boolean() || !status["ready"].get<bool>() ||
      !status.contains("stamp") || !status["stamp"].is_number())
    return false;
  return fresh(status["stamp"].get<double>(), now, timeout) &&
         (backend.empty() ||
          (status.contains("backend") && status["backend"].is_string() &&
           status["backend"].get<std::string>() == backend));
}
} // namespace wbmm_localization
