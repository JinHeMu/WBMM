#include "wbmm_search/kino_astar.hpp"
#include "wbmm_core/validation.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace wbmm::search
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kNoParent = std::numeric_limits<std::size_t>::max();
constexpr double kLateralTolerance = 1.0e-8;
using Clock = std::chrono::steady_clock;

double wrap(double angle) {return std::atan2(std::sin(angle), std::cos(angle));}

bool finiteState(const wbmm::core::BaseState & state)
{
  return std::isfinite(state.x) && std::isfinite(state.y) && std::isfinite(state.yaw) &&
         std::isfinite(state.linear_velocity) && std::isfinite(state.lateral_velocity) &&
         std::isfinite(state.yaw_rate);
}

bool inBounds(const wbmm::core::BaseState & state, const KinoAstarConfig & config)
{
  return state.x >= config.min_x && state.x <= config.max_x &&
         state.y >= config.min_y && state.y <= config.max_y;
}

struct Key
{
  std::int64_t x;
  std::int64_t y;
  std::size_t yaw;
  // Cost depends on control history even without acceleration constraints.
  // These fields collapse when their associated weights are zero.
  std::size_t previous_action{kNoParent};
  int direction{0};  // Last nonzero v: +1 forward, -1 reverse, 0 not yet moving.
  bool operator==(const Key & other) const
  {
    return x == other.x && y == other.y && yaw == other.yaw &&
           previous_action == other.previous_action && direction == other.direction;
  }
};

struct KeyHash
{
  std::size_t operator()(const Key & key) const
  {
    std::size_t hash = std::hash<std::int64_t>{}(key.x);
    const auto combine = [&hash](std::size_t value) {
        hash ^= value + 0x9e3779b9U + (hash << 6U) + (hash >> 2U);
      };
    combine(std::hash<std::int64_t>{}(key.y));
    combine(std::hash<std::size_t>{}(key.yaw));
    combine(std::hash<std::size_t>{}(key.previous_action));
    combine(std::hash<int>{}(key.direction));
    return hash;
  }
};

Key discretize(
  const wbmm::core::BaseState & state, const KinoAstarConfig & config,
  std::size_t previous_action = kNoParent, int direction = 0)
{
  const auto stable_floor = [](double value) {
      // Exact bin boundaries can drift by a few ulps during repeated rollout.
      // Canonicalize those boundaries so straight motion is not pruned as if
      // it had remained in the previous cell. This never snaps the pose.
      const double nearest = std::round(value);
      const double tolerance = 8.0 * std::numeric_limits<double>::epsilon() *
        std::max(1.0, std::abs(value));
      return std::floor(std::abs(value - nearest) <= tolerance ? nearest : value);
    };
  const double angle = std::fmod(wrap(state.yaw) + 2.0 * kPi, 2.0 * kPi);
  const auto yaw = static_cast<std::size_t>(stable_floor(
      angle / (2.0 * kPi) * static_cast<double>(config.yaw_bins)));
  return {
    static_cast<std::int64_t>(stable_floor((state.x - config.min_x) / config.position_resolution)),
    static_cast<std::int64_t>(stable_floor((state.y - config.min_y) / config.position_resolution)),
    yaw % config.yaw_bins,
    config.speed_change_weight > 0.0 || config.yaw_rate_change_weight > 0.0 ?
    previous_action : kNoParent,
    config.gear_switch_weight > 0.0 ? direction : 0};
}

struct Node
{
  wbmm::core::BaseState state;
  Key key;
  double g;
  std::size_t parent;
  MotionPrimitive input;
};

int motionDirection(double v) {return v > 0.0 ? 1 : (v < 0.0 ? -1 : 0);}

double motionCost(
  const MotionPrimitive & input, const Node * current, const KinoAstarConfig & config)
{
  double cost = input.duration + config.reverse_weight * std::max(0.0, -input.v) * input.duration +
    config.rotation_weight * std::abs(input.omega) * input.duration;
  if (current != nullptr && current->parent != kNoParent) {
    // Skip zero-weight arithmetic, including overflowing differences of extreme
    // controls. Disabled costs must not invalidate an otherwise finite edge.
    if (config.speed_change_weight > 0.0) {
      cost += config.speed_change_weight * std::abs(input.v - current->input.v);
    }
    if (config.yaw_rate_change_weight > 0.0) {
      cost += config.yaw_rate_change_weight * std::abs(input.omega - current->input.omega);
    }
  }
  const int direction = motionDirection(input.v);
  if (current != nullptr && direction != 0 && current->key.direction != 0 &&
    direction != current->key.direction)
  {
    cost += config.gear_switch_weight;
  }
  return cost;
}

double sampleCount(const MotionPrimitive & input, const KinoAstarConfig & config)
{
  return std::ceil(std::max({1.0, input.duration / config.max_sample_time,
      std::abs(input.v) * input.duration / config.max_sample_distance,
      std::abs(input.omega) * input.duration / config.max_sample_angle}));
}

std::vector<MotionPrimitive> makeActions(
  const KinoAstarConfig & config, const wbmm::core::RobotLimits & limits)
{
  std::vector<MotionPrimitive> actions;
  for (double speed : config.speed_samples) {
    for (double yaw_rate : config.yaw_rate_samples) {
      MotionPrimitive input{speed * limits.max_base_speed,
        yaw_rate * limits.max_base_yaw_rate, config.primitive_duration};
      if (input.v == 0.0 && input.omega == 0.0) {continue;}
      const double count = sampleCount(input, config);
      if (!std::isfinite(count) || count >= static_cast<double>(std::numeric_limits<std::size_t>::max()) / 2.0 ||
        !std::isfinite(motionCost(input, nullptr, config)))
      {
        throw std::invalid_argument("Control duration, sampling or cost arithmetic overflow");
      }
      // Each distinct control has one stable action id for the history key.
      const auto duplicate = std::find_if(actions.begin(), actions.end(), [&](const auto & action) {
          return action.v == input.v && action.omega == input.omega;
        });
      if (duplicate == actions.end()) {actions.push_back(input);}
    }
  }
  if (actions.empty()) {
    throw std::invalid_argument("Control samples must include a nonzero action");
  }
  return actions;
}

struct QueueEntry
{
  double f;
  std::size_t node;
  bool operator<(const QueueEntry & other) const
  {
    // Stable tie order makes offline examples reproducible.
    return f == other.f ? node > other.node : f > other.f;
  }
};

std::string validateConfig(const KinoAstarConfig & config, const wbmm::core::RobotLimits & limits)
{
  const double positive[] = {
    config.position_resolution, config.primitive_duration, config.max_search_time,
    config.max_sample_time, config.max_sample_distance, config.max_sample_angle,
    limits.max_base_speed, limits.max_base_yaw_rate};
  for (double value : positive) {
    if (!std::isfinite(value) || value <= 0.0) {
      return "Resolutions, durations, sampling intervals and base speed limits must be finite and positive";
    }
  }
  const double nonnegative[] = {
    config.position_tolerance, config.yaw_tolerance, config.reverse_weight, config.rotation_weight,
    config.gear_switch_weight, config.speed_change_weight, config.yaw_rate_change_weight};
  for (double value : nonnegative) {
    if (!std::isfinite(value) || value < 0.0) {
      return "Goal tolerances and cost weights must be finite and non-negative";
    }
  }
  if (config.yaw_tolerance > kPi || config.yaw_bins == 0 || config.max_nodes == 0 ||
    config.yaw_bins > static_cast<std::size_t>(std::numeric_limits<int>::max()))
  {
    return "Invalid yaw tolerance, yaw bin count or node budget";
  }
  if (config.collision_mode != CollisionMode::kRequireChecker &&
    config.collision_mode != CollisionMode::kDisabled)
  {
    return "Invalid collision mode";
  }
  if (!std::isfinite(config.min_x) || !std::isfinite(config.max_x) ||
    !std::isfinite(config.min_y) || !std::isfinite(config.max_y) ||
    config.min_x >= config.max_x || config.min_y >= config.max_y)
  {
    return "Search bounds must be finite and ordered";
  }
  const double cells = std::max(
    (config.max_x - config.min_x) / config.position_resolution,
    (config.max_y - config.min_y) / config.position_resolution);
  if (!std::isfinite(cells) || cells >= static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 2.0) {
    return "Search bounds/resolution exceed the supported grid index range";
  }
  const auto valid_samples = [](const std::vector<double> & samples) {
      return !samples.empty() && std::all_of(samples.begin(), samples.end(), [](double value) {
          return std::isfinite(value) && std::abs(value) <= 1.0;
        });
    };
  if (!valid_samples(config.speed_samples) || !valid_samples(config.yaw_rate_samples)) {
    return "Control samples must be nonempty finite fractions in [-1, 1]";
  }
  return {};
}
}  // namespace

wbmm::core::BaseState propagate(
  const wbmm::core::BaseState & start, const MotionPrimitive & primitive, double time_from_start)
{
  if (!finiteState(start) || !std::isfinite(primitive.v) || !std::isfinite(primitive.omega) ||
    !std::isfinite(primitive.duration) || primitive.duration <= 0.0 ||
    !std::isfinite(time_from_start) || time_from_start < 0.0 || time_from_start > primitive.duration)
  {
    throw std::invalid_argument("Invalid differential-drive rollout arguments");
  }
  const double angle = primitive.omega * time_from_start;
  const double distance = primitive.v * time_from_start;
  if (!std::isfinite(angle) || !std::isfinite(distance)) {
    throw std::invalid_argument("Differential-drive rollout overflow");
  }
  // Midpoint heading times sinc gives the exact circular-arc displacement,
  // without dividing by omega near straight motion. It also handles v == 0.
  const double half_angle = 0.5 * angle;
  const double sinc = std::abs(half_angle) < 1.0e-8 ?
    1.0 - half_angle * half_angle / 6.0 : std::sin(half_angle) / half_angle;
  auto state = start;
  state.x += distance * sinc * std::cos(wrap(start.yaw) + half_angle);
  state.y += distance * sinc * std::sin(wrap(start.yaw) + half_angle);
  state.yaw = wrap(wrap(start.yaw) + angle);
  state.linear_velocity = primitive.v;
  state.lateral_velocity = 0.0;
  state.yaw_rate = primitive.omega;
  if (!finiteState(state)) {
    throw std::invalid_argument("Differential-drive rollout produced non-finite state");
  }
  return state;
}

const char * statusName(SearchStatus status) noexcept
{
  switch (status) {
    case SearchStatus::kSuccess: return "success";
    case SearchStatus::kInvalidInput: return "invalid_input";
    case SearchStatus::kMissingCollisionChecker: return "missing_collision_checker";
    case SearchStatus::kInvalidStart: return "invalid_start";
    case SearchStatus::kInvalidGoal: return "invalid_goal";
    case SearchStatus::kCollisionCheckerError: return "collision_checker_error";
    case SearchStatus::kNoPath: return "no_path";
    case SearchStatus::kTimeout: return "timeout";
    case SearchStatus::kNodeLimit: return "node_limit";
  }
  return "unknown";
}

KinoAstar::KinoAstar(KinoAstarConfig config) : config_(std::move(config)) {}

BaseSearchResult KinoAstar::search(
  const wbmm::core::Header & header, const wbmm::core::BaseState & start,
  const wbmm::core::BaseState & goal, const wbmm::core::RobotLimits & limits,
  const BaseCollisionChecker & collision_checker) const
{
  const auto begin = Clock::now();
  BaseSearchResult result;
  result.header = header;
  const auto elapsed = [&]() {return std::chrono::duration<double>(Clock::now() - begin).count();};
  const auto finish = [&](SearchStatus status, std::string message) {
      result.status = status;
      result.success = status == SearchStatus::kSuccess;
      if (!result.success) {
        result.path.clear();
        result.primitives.clear();
        result.path_length = 0.0;
        result.total_cost = 0.0;
      }
      result.message = std::move(message);
      result.solve_time = elapsed();
      return std::move(result);
    };
  // Reuse public core header validation through its spatial-velocity contract;
  // no fabricated WholeBodyState or arm joints are needed.
  wbmm::core::Twist metadata;
  metadata.header = header;
  const auto header_check = wbmm::core::validate(metadata);
  if (!header_check.ok) {
    return finish(SearchStatus::kInvalidInput, header_check.message);
  }
  const auto config_error = validateConfig(config_, limits);
  if (!config_error.empty()) {
    return finish(SearchStatus::kInvalidInput, config_error);
  }
  if (!finiteState(start) || !finiteState(goal) || std::abs(start.yaw) > kPi || std::abs(goal.yaw) > kPi) {
    return finish(SearchStatus::kInvalidInput, "Base states must be finite with yaw in [-pi, pi]");
  }
  if (std::abs(start.lateral_velocity) > kLateralTolerance ||
    std::abs(goal.lateral_velocity) > kLateralTolerance)
  {
    return finish(SearchStatus::kInvalidInput, "Differential-drive lateral velocity must be within 1e-8 of zero");
  }
  if (!inBounds(start, config_)) {
    return finish(SearchStatus::kInvalidStart, "Start is outside search bounds");
  }
  if (!inBounds(goal, config_)) {
    return finish(SearchStatus::kInvalidGoal, "Goal is outside search bounds");
  }
  const bool check_collision = config_.collision_mode == CollisionMode::kRequireChecker;
  if (check_collision && !collision_checker) {
    return finish(SearchStatus::kMissingCollisionChecker, "A base collision checker is required");
  }
  // The callback is the only externally supplied operation. Catch its failures
  // specifically; never reinterpret an exception as a free pose.
  bool checker_error = false;
  std::string checker_message;
  const auto valid_pose = [&](const wbmm::core::BaseState & state) {
      if (!check_collision) {return true;}
      try {
        return collision_checker(header, state);
      } catch (const std::exception & error) {
        checker_error = true;
        checker_message = error.what();
      } catch (...) {
        checker_error = true;
        checker_message = "Non-standard collision checker exception";
      }
      return false;
    };
  if (!valid_pose(start)) {
    return finish(checker_error ? SearchStatus::kCollisionCheckerError : SearchStatus::kInvalidStart,
      checker_error ? checker_message : "Start was rejected by the base collision checker");
  }
  if (!valid_pose(goal)) {
    return finish(checker_error ? SearchStatus::kCollisionCheckerError : SearchStatus::kInvalidGoal,
      checker_error ? checker_message : "Goal was rejected by the base collision checker");
  }

  std::vector<MotionPrimitive> actions;
  try {actions = makeActions(config_, limits);}
  catch (const std::invalid_argument & error) {return finish(SearchStatus::kInvalidInput, error.what());}

  // Protection stays here; the expansion loop below reads as
  // action -> propagate -> discretize -> cost -> collisionCheck -> OPEN.
  std::string primitive_error;
  const auto collisionCheck = [&](const wbmm::core::BaseState & state, const MotionPrimitive & input) {
      const auto samples = static_cast<std::size_t>(sampleCount(input, config_));
      for (std::size_t sample = 1; sample <= samples; ++sample) {
        if (elapsed() >= config_.max_search_time) {return SearchStatus::kTimeout;}
        const double time = input.duration * (static_cast<double>(sample) / static_cast<double>(samples));
        wbmm::core::BaseState point;
        try {point = propagate(state, input, time);}
        catch (const std::invalid_argument & error) {
          primitive_error = error.what();
          return SearchStatus::kInvalidInput;
        }
        if (!inBounds(point, config_)) {return SearchStatus::kNoPath;}
        if (!valid_pose(point)) {
          return checker_error ? SearchStatus::kCollisionCheckerError : SearchStatus::kNoPath;
        }
      }
      return SearchStatus::kSuccess;
    };
  const auto distance_to_goal = [&](const wbmm::core::BaseState & state) {
      return std::hypot(state.x - goal.x, state.y - goal.y);
    };
  const auto angle_to_goal = [&](const wbmm::core::BaseState & state) {
      return std::abs(wrap(state.yaw - goal.yaw));
    };
  const auto heuristic = [&](const wbmm::core::BaseState & state) {
      return std::max(
        std::max(0.0, distance_to_goal(state) - config_.position_tolerance) / limits.max_base_speed,
        std::max(0.0, angle_to_goal(state) - config_.yaw_tolerance) / limits.max_base_yaw_rate);
    };
  const auto reached = [&](const wbmm::core::BaseState & state) {
      return distance_to_goal(state) <= config_.position_tolerance &&
             angle_to_goal(state) <= config_.yaw_tolerance;
    };
  std::vector<Node> nodes;
  std::unordered_map<Key, std::size_t, KeyHash> best;
  std::priority_queue<QueueEntry> open;
  auto initial = start;
  initial.linear_velocity = 0.0;
  initial.lateral_velocity = 0.0;
  initial.yaw_rate = 0.0;
  const auto start_key = discretize(initial, config_);
  if (!std::isfinite(heuristic(initial))) {
    return finish(SearchStatus::kInvalidInput, "Initial heuristic arithmetic overflow");
  }
  nodes.push_back({initial, start_key, 0.0, kNoParent, {}});
  best.emplace(start_key, 0);
  open.push({heuristic(initial), 0});
  result.generated_nodes = 1;

  while (!open.empty()) {
    if (elapsed() >= config_.max_search_time) {
      return finish(SearchStatus::kTimeout, "Search time budget exhausted");
    }
    const auto entry = open.top();
    open.pop();
    // Copy the record: appending successors may reallocate the vector.
    const Node current = nodes[entry.node];
    if (best.at(current.key) != entry.node) {continue;}
    ++result.expanded_nodes;
    if (reached(current.state)) {
      for (std::size_t id = entry.node; id != kNoParent; id = nodes[id].parent) {
        result.path.push_back(nodes[id].state);
        if (nodes[id].parent != kNoParent) {result.primitives.push_back(nodes[id].input);}
      }
      std::reverse(result.path.begin(), result.path.end());
      std::reverse(result.primitives.begin(), result.primitives.end());
      for (const auto & input : result.primitives) {
        result.path_length += std::abs(input.v) * input.duration;
      }
      if (!std::isfinite(result.path_length)) {
        return finish(SearchStatus::kInvalidInput, "Path length arithmetic overflow");
      }
      result.total_cost = current.g;
      result.collision_checked = check_collision;
      return finish(SearchStatus::kSuccess,
        check_collision ? "Goal region reached; sampled base collision checks passed" :
        "Goal region reached; collision checking DISABLED (未检查碰撞)");
    }

    for (std::size_t action_id = 0; action_id < actions.size(); ++action_id) {
      const auto & input = actions[action_id];
      if (elapsed() >= config_.max_search_time) {
        return finish(SearchStatus::kTimeout, "Search time budget exhausted");
      }
      wbmm::core::BaseState successor;
      try {successor = propagate(current.state, input, input.duration);}
      catch (const std::invalid_argument & error) {return finish(SearchStatus::kInvalidInput, error.what());}
      if (!inBounds(successor, config_)) {continue;}
      const int direction = input.v == 0.0 ? current.key.direction : motionDirection(input.v);
      const auto key = discretize(successor, config_, action_id, direction);
      if (key == current.key) {continue;}
      const double g = current.g + motionCost(input, &current, config_);
      const double f = g + heuristic(successor);
      if (!std::isfinite(g) || !std::isfinite(f)) {
        return finish(SearchStatus::kInvalidInput, "Accumulated search cost overflow");
      }
      const auto previous = best.find(key);
      if (previous != best.end() && g >= nodes[previous->second].g) {continue;}

      const auto primitive_status = collisionCheck(current.state, input);
      if (primitive_status == SearchStatus::kNoPath) {continue;}
      if (primitive_status == SearchStatus::kTimeout) {
        return finish(primitive_status, "Search time budget exhausted");
      }
      if (primitive_status == SearchStatus::kInvalidInput) {return finish(primitive_status, primitive_error);}
      if (primitive_status == SearchStatus::kCollisionCheckerError) {return finish(primitive_status, checker_message);}
      if (nodes.size() >= config_.max_nodes) {
        return finish(SearchStatus::kNodeLimit, "Node record budget exhausted");
      }
      const auto id = nodes.size();
      nodes.push_back({successor, key, g, entry.node, input});
      best[key] = id;
      open.push({f, id});
      result.generated_nodes = nodes.size();
    }
  }
  return finish(SearchStatus::kNoPath, "Open set exhausted; no path at the configured discretization");
}
}  // namespace wbmm::search
