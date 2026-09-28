#include "wbmm_search/arm_seed_search.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <utility>

namespace wbmm::search
{
namespace
{

using Clock = std::chrono::steady_clock;

// Halton sequence: deterministic, well distributed, and reproducible across
// runs and platforms. A clock-seeded engine would make a planning failure
// impossible to reproduce, which is the main practical drawback of the RRT*
// stage this replaces.
double halton(std::size_t index, std::size_t base)
{
  double result = 0.0;
  double fraction = 1.0;
  while (index > 0U) {
    fraction /= static_cast<double>(base);
    result += fraction * static_cast<double>(index % base);
    index /= base;
  }
  return result;
}

constexpr std::size_t kPrimes[] = {2U, 3U, 5U, 7U, 11U, 13U, 17U, 19U, 23U, 29U};

double component(std::size_t index, std::size_t joint)
{
  return halton(
    index + 1U, kPrimes[joint % (sizeof(kPrimes) / sizeof(kPrimes[0]))]);
}

bool finite(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(),
    [](double value) {return std::isfinite(value);});
}

// Resamples a polyline at a fixed arc-length spacing, keeping the last point.
std::vector<wbmm::core::BaseState> resample(
  const std::vector<wbmm::core::BaseState> & path, double spacing)
{
  std::vector<wbmm::core::BaseState> out;
  if (path.empty()) {
    return out;
  }
  out.push_back(path.front());
  if (path.size() == 1U) {
    return out;
  }

  double carried = 0.0;
  for (std::size_t i = 1U; i < path.size(); ++i) {
    const auto & from = path[i - 1U];
    const auto & to = path[i];
    const double dx = to.x - from.x;
    const double dy = to.y - from.y;
    const double segment = std::hypot(dx, dy);
    if (segment <= 0.0) {
      continue;
    }
    double travelled = spacing - carried;
    while (travelled <= segment) {
      const double ratio = travelled / segment;
      wbmm::core::BaseState state = to;
      state.x = from.x + ratio * dx;
      state.y = from.y + ratio * dy;
      out.push_back(state);
      travelled += spacing;
    }
    carried = segment - (travelled - spacing);
  }

  // Always keep the true endpoint so the seed covers the whole path.
  const auto & last = path.back();
  if (std::hypot(out.back().x - last.x, out.back().y - last.y) > 1.0e-9) {
    out.push_back(last);
  }
  return out;
}

wbmm::core::JointState blend(
  const wbmm::core::JointState & from, const wbmm::core::JointState & to,
  double ratio)
{
  wbmm::core::JointState out = to;
  for (std::size_t j = 0U; j < out.positions.size(); ++j) {
    out.positions[j] = from.positions[j] + ratio * (to.positions[j] - from.positions[j]);
    out.velocities[j] =
      from.velocities[j] + ratio * (to.velocities[j] - from.velocities[j]);
  }
  return out;
}

// The base moves while the arm reconfigures, so the swept check interpolates
// both. Checking the whole transition at the destination base pose alone would
// demand that the arm already be reconfigured before it arrives, which no
// greedy search can satisfy.
wbmm::core::BaseState blendBase(
  const wbmm::core::BaseState & from, const wbmm::core::BaseState & to,
  double ratio)
{
  wbmm::core::BaseState out = to;
  out.x = from.x + ratio * (to.x - from.x);
  out.y = from.y + ratio * (to.y - from.y);
  double delta = to.yaw - from.yaw;
  while (delta > M_PI) {
    delta -= 2.0 * M_PI;
  }
  while (delta < -M_PI) {
    delta += 2.0 * M_PI;
  }
  out.yaw = from.yaw + ratio * delta;
  return out;
}

bool feasibleAt(
  const WholeBodyCollisionChecker & checker, const wbmm::core::Header & header,
  const wbmm::core::BaseState & base, const wbmm::core::JointState & joints)
{
  wbmm::core::WholeBodyState state;
  state.header = header;
  state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
  state.base = base;
  state.joints = joints;
  return checker(header, state);
}

}  // namespace

wbmm::core::SearchResult ArmSeedResult::toSearchResult(
  wbmm::core::SearchResult base) const
{
  if (success) {
    base.base_path = base_path;
    base.arm_seed = arm_seed;
    base.phases.assign(base_path.size(), wbmm::core::ExecutionPhase::kNavigate);
    base.path_length = 0.0;
    for (std::size_t i = 1U; i < base_path.size(); ++i) {
      base.path_length += std::hypot(
        base_path[i].x - base_path[i - 1U].x,
        base_path[i].y - base_path[i - 1U].y);
    }
    base.solve_time = solve_time;
    base.success = true;
  }
  return base;
}

ArmSeedResult searchArmSeed(
  const wbmm::core::Header & header,
  const std::vector<wbmm::core::BaseState> & base_path,
  const wbmm::core::JointState & initial_joints,
  const wbmm::core::RobotLimits & limits,
  const WholeBodyCollisionChecker & collision_checker,
  const ArmSeedConfig & config)
{
  ArmSeedResult result;
  const auto started = Clock::now();

  const auto finish = [&result, started](ArmSeedStatus status, std::string message)
  {
    result.status = status;
    result.message = std::move(message);
    result.success = status == ArmSeedStatus::kSuccess;
    result.solve_time =
      std::chrono::duration<double>(Clock::now() - started).count();
    return result;
  };

  const std::size_t joint_count = initial_joints.positions.size();
  if (base_path.size() < 2U) {
    return finish(ArmSeedStatus::kInvalidInput, "base_path needs at least two points.");
  }
  if (joint_count == 0U) {
    return finish(ArmSeedStatus::kInvalidInput, "initial_joints is empty.");
  }
  if (initial_joints.names.size() != joint_count ||
    initial_joints.velocities.size() != joint_count) {
    return finish(
      ArmSeedStatus::kInvalidInput,
      "initial_joints names/positions/velocities have inconsistent sizes.");
  }
  if (limits.joint_min.size() != joint_count ||
    limits.joint_max.size() != joint_count) {
    return finish(
      ArmSeedStatus::kInvalidInput,
      "limits.joint_min/joint_max must match the joint count.");
  }
  for (std::size_t j = 0U; j < joint_count; ++j) {
    if (!(limits.joint_min[j] <= initial_joints.positions[j]) ||
      !(initial_joints.positions[j] <= limits.joint_max[j]))
    {
      return finish(
        ArmSeedStatus::kInvalidInput,
        "initial_joints violate the joint limits.");
    }
  }
  if (!finite(initial_joints.positions) || !finite(limits.joint_min) ||
    !finite(limits.joint_max))
  {
    return finish(ArmSeedStatus::kInvalidInput, "Non-finite joint data.");
  }
  if (!collision_checker) {
    return finish(
      ArmSeedStatus::kMissingCollisionChecker,
      "A whole-body collision checker is required.");
  }
  if (!std::isfinite(config.waypoint_spacing) || config.waypoint_spacing <= 0.0 ||
    !std::isfinite(config.max_joint_step) || config.max_joint_step <= 0.0)
  {
    return finish(ArmSeedStatus::kInvalidInput, "waypoint_spacing and max_joint_step must be positive.");
  }
  if (config.candidates_per_waypoint == 0U) {
    return finish(ArmSeedStatus::kInvalidInput, "candidates_per_waypoint must be positive.");
  }

  result.base_path = resample(base_path, config.waypoint_spacing);
  result.waypoints = result.base_path.size();

  wbmm::core::JointState current = initial_joints;
  result.arm_seed.reserve(result.base_path.size());

  for (std::size_t w = 0U; w < result.base_path.size(); ++w) {
    if (std::chrono::duration<double>(Clock::now() - started).count() >
      config.max_search_time)
    {
      return finish(ArmSeedStatus::kTimeout, "Arm seed search timed out.");
    }

    bool accepted = false;

    for (std::size_t candidate = 0U; candidate < config.candidates_per_waypoint;
      ++candidate)
    {
      wbmm::core::JointState trial;
      if (candidate == 0U) {
        // Carry-over first: the cheapest and most continuous option.
        trial = current;
      } else {
        trial = current;
        const double scale =
          config.max_joint_step * (1.0 + static_cast<double>(candidate) /
          static_cast<double>(config.candidates_per_waypoint));
        for (std::size_t j = 0U; j < joint_count; ++j) {
          const double span = limits.joint_max[j] - limits.joint_min[j];
          const double draw = component(config.sequence_offset + candidate, j);
          const double offset = scale * (2.0 * draw - 1.0);
          trial.positions[j] = std::clamp(
            current.positions[j] + offset, limits.joint_min[j],
            limits.joint_max[j]);
          if (span > 0.0) {
            // Keep the value inside the range even after clamping.
            trial.positions[j] = std::clamp(
              trial.positions[j], limits.joint_min[j], limits.joint_max[j]);
          }
          trial.velocities[j] = 0.0;
        }
      }

      // Check the whole transition, not just the endpoint: a candidate can be
      // free while the swept motion through it is not. The base pose is
      // interpolated too, because the base keeps moving while the arm
      // reconfigures.
      const std::size_t steps = std::max<std::size_t>(1U, config.interpolation_steps);
      const auto & previous_base =
        w == 0U ? result.base_path[0] : result.base_path[w - 1U];
      bool transition_free = true;
      for (std::size_t s = 1U; s <= steps; ++s) {
        const double ratio = static_cast<double>(s) / static_cast<double>(steps);
        if (!feasibleAt(
            collision_checker, header, blendBase(previous_base, result.base_path[w], ratio),
            blend(current, trial, ratio)))
        {
          transition_free = false;
          break;
        }
      }
      ++result.checked_candidates;
      if (!transition_free) {
        continue;
      }

      // One-step look-ahead. Without it the search is greedy and only reacts
      // once the base has already entered a region the current configuration
      // cannot serve, at which point no swept transition can escape. Requiring
      // the candidate to also survive the next waypoint makes the arm start
      // reconfiguring one step early, which is what the physical robot does.
      if (w + 1U < result.base_path.size() &&
        !feasibleAt(
          collision_checker, header, result.base_path[w + 1U], trial))
      {
        continue;
      }

      current = trial;
      accepted = true;
      break;
    }

    if (!accepted) {
      result.waypoints = w;
      return finish(
        ArmSeedStatus::kNoFeasibleSeed,
        "No collision-free arm configuration was found at waypoint " +
        std::to_string(w) + " of " + std::to_string(result.base_path.size()) +
        " after " + std::to_string(config.candidates_per_waypoint) +
        " candidates.");
    }
    result.arm_seed.push_back(current);
  }

  return finish(ArmSeedStatus::kSuccess, "ok");
}

}  // namespace wbmm::search
