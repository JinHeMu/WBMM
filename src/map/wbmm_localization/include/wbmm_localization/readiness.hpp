#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <nlohmann/json.hpp>
#include <vector>

namespace wbmm_localization {

using Point = std::array<double, 2>;
using Pose2d = std::array<double, 3>;
using Quaternion = std::array<double, 4>; // x, y, z, w
using Rotation = std::array<std::array<double, 3>, 3>;

bool fresh(double stamp, double now, double timeout,
           double future_tolerance = 0.3);
Rotation quaternionMatrix(const Quaternion &q);
double yaw(const Quaternion &q);

struct Agreement {
  std::size_t endpoints = 0;
  double known_ratio = 0.0;
  double match_ratio = 0.0;
};

class Grid {
public:
  Grid(int width, int height, double resolution, Pose2d origin,
       std::vector<int8_t> data, double match_distance,
       int occupied_threshold = 65);
  Agreement agreement(const std::vector<Point> &points) const;

private:
  int width_, height_, radius_, threshold_;
  double resolution_, distance_;
  Pose2d origin_;
  std::vector<int8_t> data_;
};

std::vector<Point> scanEndpoints(const std::vector<float> &ranges,
                                 double angle_min, double angle_increment,
                                 double range_min, double range_max,
                                 const Rotation &rotation,
                                 const std::array<double, 3> &translation,
                                 int limit = 180, double max_range = 30.0);

// Distinct scans must agree throughout the full rolling time window.
class StableWindow {
public:
  StableWindow(double duration = 3.0, int min_samples = 5,
               double max_translation = 0.15, double max_yaw = 0.1);
  void reset();
  bool update(double stamp, bool good, const Pose2d &correction);
  bool ready() const { return ready_; }

private:
  double duration_, max_translation_, max_yaw_;
  int min_samples_;
  bool ready_ = false;
  std::deque<std::array<double, 4>> samples_;
};

bool statusIsReady(const nlohmann::json &status, double now,
                   const std::string &backend = "", double timeout = 1.0);

} // namespace wbmm_localization
