#pragma once

#include "wbmm_core/robot_model.hpp"

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

namespace wbmm::search
{

enum class CollisionMode {kRequireChecker, kDisabled};
enum class SearchStatus
{
  kSuccess,
  kInvalidInput,
  kMissingCollisionChecker,
  kInvalidStart,
  kInvalidGoal,
  kCollisionCheckerError,
  kNoPath,
  kTimeout,
  kNodeLimit,
};

// SI units: v in m/s, omega in rad/s, duration in s.
struct MotionPrimitive
{
  double v{0.0};
  double omega{0.0};
  double duration{0.4};
};

struct KinoAstarConfig
{
  double min_x{-10.0};
  double max_x{10.0};
  double min_y{-10.0};
  double max_y{10.0};
  double position_resolution{0.1};
  std::size_t yaw_bins{72};
  double primitive_duration{0.4};
  std::vector<double> speed_samples{-1.0, -0.5, 0.0, 0.5, 1.0};
  std::vector<double> yaw_rate_samples{-1.0, -0.5, 0.0, 0.5, 1.0};
  double position_tolerance{0.15};
  double yaw_tolerance{0.1};
  double reverse_weight{0.5};
  double rotation_weight{0.1};
  // Soft costs between adjacent primitives, not acceleration limits.
  // The first primitive has no previous control; input start velocities remain
  // metadata. Zero-speed rotation preserves the last forward/reverse direction.
  double gear_switch_weight{1.0};
  double speed_change_weight{0.1};
  double yaw_rate_change_weight{0.1};
  double max_search_time{2.0};
  std::size_t max_nodes{100000};
  CollisionMode collision_mode{CollisionMode::kRequireChecker};
  // Sample every primitive, even when collision checking is disabled, to check
  // the search bounds. A footprint-aware checker remains the caller's concern.
  double max_sample_time{0.05};
  double max_sample_distance{0.05};
  double max_sample_angle{0.05};
};

// true means the base pose is valid. The header identifies the planning frame;
// its stamp is request metadata, not a time-varying obstacle query timestamp.
using BaseCollisionChecker = std::function<bool(
    const wbmm::core::Header &, const wbmm::core::BaseState &)>;

struct BaseSearchResult
{
  wbmm::core::Header header;
  std::vector<wbmm::core::BaseState> path;
  std::vector<MotionPrimitive> primitives;
  double path_length{0.0};
  double total_cost{0.0};
  double solve_time{0.0};
  std::size_t generated_nodes{0};
  std::size_t expanded_nodes{0};
  bool collision_checked{false};
  bool success{false};
  SearchStatus status{SearchStatus::kInvalidInput};
  std::string message;
};

// Constant-control exact differential-drive integration. time_from_start is
// relative to this primitive, in [0, duration]. Invalid arguments throw
// std::invalid_argument. The returned velocities describe this primitive.
[[nodiscard]] wbmm::core::BaseState propagate(
  const wbmm::core::BaseState & start,
  const MotionPrimitive & primitive,
  double time_from_start);

[[nodiscard]] const char * statusName(SearchStatus status) noexcept;

class KinoAstar
{
public:
  explicit KinoAstar(KinoAstarConfig config = {});

  // Search only uses x/y/yaw as physical state. Start/goal v and omega are not
  // boundary conditions; abs(lateral_velocity) <= 1e-8 is accepted as noise.
  [[nodiscard]] BaseSearchResult search(
    const wbmm::core::Header & header,
    const wbmm::core::BaseState & start,
    const wbmm::core::BaseState & goal,
    const wbmm::core::RobotLimits & limits,
    const BaseCollisionChecker & collision_checker = {}) const;

private:
  KinoAstarConfig config_;
};

}  // namespace wbmm::search
