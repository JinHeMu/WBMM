#include "wbmm_planner/whole_body_planner.hpp"

#include <wbmm_traj_opt/minco.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <string>
#include <set>
#include <exception>
#include <utility>
#include <vector>

namespace wbmm::planning
{
namespace
{

using Clock = std::chrono::steady_clock;

constexpr int kDim = 8;

// Time-scaling retry budget, mirroring REMANI's bounded rescaling loop but
// applied to the emitted reference rather than to a sampled estimate.
constexpr int kMaxTimeScaleAttempts = 8;
constexpr double kTimeScaleStep = 1.05;

double secondsSince(const Clock::time_point & start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}

// Gear of a path: the sign of its first non-zero forward velocity. Returns 0
// when the path never moves, and kMixedGear when it reverses mid-path.
constexpr int kMixedGear = -2;

int pathGear(const std::vector<wbmm::core::BaseState> & path)
{
  int gear = 0;
  for (const auto & state : path) {
    if (std::abs(state.linear_velocity) < 1e-6) {
      continue;
    }
    const int sign = state.linear_velocity > 0.0 ? 1 : -1;
    if (gear == 0) {
      gear = sign;
    } else if (gear != sign) {
      return kMixedGear;
    }
  }
  return gear == 0 ? 1 : gear;
}

}  // namespace

WholeBodyPlanner::WholeBodyPlanner(PlannerConfig config) : config_(std::move(config))
{
}

PlanResult WholeBodyPlanner::plan(
    const PlanRequest &request,
    const wbmm::search::BaseCollisionChecker &base_checker,
    const wbmm::search::WholeBodyCollisionChecker &whole_body_checker,
    const TrajectoryOptimizer &optimizer) const {
  PlanResult result;

  const auto fail = [&result, this](std::string message)
  {
    result.success = false;
    result.trajectory.points.clear();
    result.message = std::move(message);
    last_error_ = result.message;
    return result;
  };

  if (request.header.frame_id.empty()) {
    return fail("Plan request has no frame_id.");
  }
  if (request.start_joints.positions.empty()) {
    return fail("Plan request has no joint state.");
  }
  if (request.start_joints.positions.size() != request.limits.joint_min.size() ||
    request.limits.joint_min.size() != request.limits.joint_max.size())
  {
    return fail("Joint state and limits have inconsistent sizes.");
  }
  if (!base_checker) {
    return fail("A base collision checker is required.");
  }
  if (!whole_body_checker) {
    return fail("A whole-body collision checker is required.");
  }

  // MINCO below has exactly two base coordinates and six arm coordinates.
  if (request.start_joints.positions.size() != 6U ||
      request.start_joints.names.size() != 6U ||
      request.start_joints.velocities.size() != 6U ||
      request.limits.max_joint_speed.size() != 6U) {
    return fail("This planner requires exactly six named arm joints and velocity limits.");
  }
  std::set<std::string> names;
  for (std::size_t j = 0; j < 6U; ++j) {
    if (request.start_joints.names[j].empty() ||
        !names.insert(request.start_joints.names[j]).second ||
        !std::isfinite(request.start_joints.positions[j]) ||
        !std::isfinite(request.start_joints.velocities[j]) ||
        !std::isfinite(request.limits.joint_min[j]) ||
        !std::isfinite(request.limits.joint_max[j]) ||
        request.limits.joint_min[j] > request.limits.joint_max[j] ||
        request.start_joints.positions[j] < request.limits.joint_min[j] ||
        request.start_joints.positions[j] > request.limits.joint_max[j] ||
        !std::isfinite(request.limits.max_joint_speed[j]) ||
        request.limits.max_joint_speed[j] <= 0.0) {
      return fail("Invalid joint state, names or limits.");
    }
  }
  for (double value : {config_.cruise_speed, config_.min_segment_duration,
      config_.max_joint_speed, config_.max_trajectory_duration,
      config_.validation_translation_step, config_.validation_angle_step,
      request.limits.max_base_speed, request.limits.max_base_yaw_rate}) {
    if (!std::isfinite(value) || value <= 0.0) {
      return fail("Planner timing, validation steps and speed limits must be positive and finite.");
    }
  }

  if (std::abs(request.start.linear_velocity) > 0.02 ||
      std::abs(request.start.yaw_rate) > 0.02 ||
      std::any_of(request.start_joints.velocities.begin(), request.start_joints.velocities.end(),
        [](double value) { return std::abs(value) > 0.02; })) {
    return fail("Rest-to-rest planner requires a stationary start state.");
  }

  const auto validateTrajectory =
      [&](const wbmm::core::WholeBodyTrajectory &trajectory) -> std::string {
    // Validate the actual sampled reference AND the linear interpolation used
    // by the bridge. Seed checks cannot certify a polynomial that cuts corners.
    // These are bounded discrete checks, not a continuous collision
    // certificate.
    const auto &points = trajectory.points;
    if (points.empty())
      return std::string("Empty candidate trajectory.");
    std::size_t checks = 0U;
    try {
      for (std::size_t i = 0; i < points.size(); ++i) {
        const auto &a = points[i == 0U ? 0U : i - 1U].state;
        const auto &b = points[i].state;
        double subdivisions = std::max(
            1.0,
            std::ceil(std::hypot(b.base.x - a.base.x, b.base.y - a.base.y) /
                      config_.validation_translation_step));
        subdivisions =
            std::max(subdivisions, std::ceil(std::abs(b.base.yaw - a.base.yaw) /
                                             config_.validation_angle_step));
        for (std::size_t j = 0; j < 6U; ++j) {
          subdivisions =
              std::max(subdivisions, std::ceil(std::abs(b.joints.positions[j] -
                                                        a.joints.positions[j]) /
                                               config_.validation_angle_step));
        }
        if (!std::isfinite(subdivisions) || subdivisions > 100000.0 - checks) {
          return std::string("Final trajectory validation budget exceeded.");
        }
        const auto count = static_cast<std::size_t>(subdivisions);
        for (std::size_t k = 1; k <= count; ++k) {
          const double alpha = static_cast<double>(k) / count;
          auto state = b;
          state.base.x = a.base.x + alpha * (b.base.x - a.base.x);
          state.base.y = a.base.y + alpha * (b.base.y - a.base.y);
          state.base.yaw = a.base.yaw + alpha * (b.base.yaw - a.base.yaw);
          for (std::size_t j = 0; j < 6U; ++j) {
            state.joints.positions[j] =
                a.joints.positions[j] +
                alpha * (b.joints.positions[j] - a.joints.positions[j]);
            if (state.joints.positions[j] <
                    request.limits.joint_min[j] - 1e-9 ||
                state.joints.positions[j] >
                    request.limits.joint_max[j] + 1e-9) {
              return std::string(
                  "Final trajectory exceeds joint position limits.");
            }
          }
          ++checks;
          if (!whole_body_checker(state.header, state)) {
            return std::string(
                "Final trajectory collision validation failed at sample " +
                std::to_string(i));
          }
        }
      }
    } catch (const std::exception &error) {
      return std::string("Final trajectory collision checker failed: " +
                         std::string(error.what()));
    } catch (...) {
      return std::string("Final trajectory collision checker failed with an "
                         "unknown exception.");
    }
    // Shaping may change the terminal tangent, so search success alone does not
    // imply that the published pose meets the original goal.
    const auto &end = points.back().state.base;
    if (std::hypot(end.x - request.goal.x, end.y - request.goal.y) >
            config_.base_search.position_tolerance + 1e-9 ||
        std::abs(std::remainder(end.yaw - request.goal.yaw, 2.0 * M_PI)) >
            config_.base_search.yaw_tolerance + 1e-9) {
      return std::string("Shaped trajectory does not meet the requested "
                         "terminal pose tolerance.");
    }
    if (request.goal_joints) {
      for (std::size_t j = 0; j < 6; ++j)
        if (std::abs(points.back().state.joints.positions[j] -
                     request.goal_joints->positions[j]) > 1e-6)
          return std::string(
              "Final trajectory does not meet requested terminal joints.");
    }
    return {};
  };

  // ---- 1. Base search -----------------------------------------------------
  const auto base_started = Clock::now();
  wbmm::search::KinoAstar astar(config_.base_search);
  const auto base = astar.search(
    request.header, request.start, request.goal, request.limits, base_checker);
  result.base_search_time = secondsSince(base_started);

  wbmm::search::WholeBodyRrtResult seed;
  if (base.success) {
    const auto seed_started = Clock::now();
    seed = wbmm::search::sampleArmRrt(request.header, base, request.start_joints, request.limits,
                                      whole_body_checker, config_.sample_rrt, request.goal_joints);
    result.arm_seed_time = secondsSince(seed_started);
    result.search_backend = "astar_sample_rrt";
    if (seed.fatal)
      return fail("Arm seeding failed: " + seed.message);
  } else if (base.status != wbmm::search::SearchStatus::kNoPath &&
             base.status != wbmm::search::SearchStatus::kTimeout &&
             base.status != wbmm::search::SearchStatus::kNodeLimit) {
    return fail("Base search failed: " + base.message);
  }
  if (!seed.success) {
    result.fallback_reason = base.success ? "Arm seeding failed: " + seed.message
                                          : "Base search failed: " + base.message;
    if (!config_.enable_whole_body_rrt)
      return fail(result.fallback_reason);
    result.whole_body_rrt_attempted = true;
    seed = wbmm::search::searchWholeBodyRrt(request.header, request.start, request.goal,
                                            request.start_joints, request.limits, base_checker,
                                            whole_body_checker, config_.base_search,
                                            config_.whole_body_rrt, request.goal_joints);
    result.whole_body_rrt_time = seed.solve_time;
    result.search_backend = "whole_body_rrt";
    if (!seed.success)
      return fail(result.fallback_reason + "; whole-body RRT: " + seed.message);
  }
  result.search = seed.search;

  // Keep the existing smooth MINCO route. Exact time-scaled primitives are a
  // conservative fallback for rotations, gear changes or failed smoothing.
  const auto shape = [&](bool allow_optimization) -> PlanResult {
    if (result.search.base_path.size() < 2U)
      return fail("Stationary goal requires a hold.");
    for (const auto &primitive : seed.primitives) {
      if (std::abs(primitive.v) < 1e-9)
        return fail("Path contains a stationary primitive; using segmented timing.");
    }
    // ---- 3. Gear ------------------------------------------------------------
    const int gear = pathGear(result.search.base_path);
    if (gear == kMixedGear) {
      return fail("The planned base path mixes forward and reverse segments. A gear flip is "
                  "a heading discontinuity and needs its own trajectory section, which is "
                  "not implemented yet.");
    }
    result.gear = gear;

    // ---- 4. Shape a MINCO through the waypoints -----------------------------
    const auto build_started = Clock::now();
    const auto &full_path = result.search.base_path;
    const auto &full_seed = result.search.arm_seed;
    const auto joint_count = static_cast<Eigen::Index>(request.start_joints.positions.size());

    // Decimate to the shaping control points, always keeping the last waypoint.
    const std::size_t stride = std::max<std::size_t>(1U, config_.minco_waypoint_stride);
    std::vector<std::size_t> control;
    for (std::size_t i = 0U; i < full_path.size(); i += stride) {
      control.push_back(i);
    }
    if (control.back() != full_path.size() - 1U) {
      control.push_back(full_path.size() - 1U);
    }
    // Remove redundant collinear control points in the full [x,y,q] space.
    // A tiny final segment with a minimum duration can otherwise make MINCO
    // overshoot and reverse even on a straight, collision-free search path.
    for (std::size_t k = 1U; k + 1U < control.size();) {
      const auto ia = control[k - 1U], ib = control[k], ic = control[k + 1U];
      const Eigen::Vector2d a(full_path[ia].x, full_path[ia].y);
      const Eigen::Vector2d b(full_path[ib].x, full_path[ib].y);
      const Eigen::Vector2d c(full_path[ic].x, full_path[ic].y);
      const double length2 = (c - a).squaredNorm();
      const double fraction = length2 > 1e-16 ? (b - a).dot(c - a) / length2 : -1.0;
      bool redundant =
          fraction > 0.0 && fraction < 1.0 && (b - (a + fraction * (c - a))).norm() <= 1e-8;
      for (std::size_t j = 0; redundant && j < 6U; ++j) {
        redundant =
            std::abs(full_seed[ib].positions[j] -
                     (full_seed[ia].positions[j] +
                      fraction * (full_seed[ic].positions[j] - full_seed[ia].positions[j]))) <=
            1e-8;
      }
      if (redundant) {
        control.erase(control.begin() + k);
      } else {
        ++k;
      }
    }
    if (control.size() < 2U) {
      return fail("Too few waypoints to shape a trajectory.");
    }

    const auto at = [&control, &full_path](std::size_t k) -> const wbmm::core::BaseState & {
      return full_path[control[k]];
    };
    const auto jointsAt = [&control, &full_seed](std::size_t k) -> const wbmm::core::JointState & {
      return full_seed[control[k]];
    };

    const auto piece_count = static_cast<int>(control.size()) - 1;

    const auto fillState = [&](const wbmm::core::BaseState &base_state,
                               const wbmm::core::JointState &joints) {
      Eigen::VectorXd position(2 + joint_count);
      position(0) = base_state.x;
      position(1) = base_state.y;
      for (Eigen::Index j = 0; j < joint_count; ++j) {
        position(2 + j) = joints.positions[static_cast<std::size_t>(j)];
      }
      return position;
    };

    Eigen::MatrixXd head(kDim, 4);
    Eigen::MatrixXd tail(kDim, 4);
    head.setZero();
    tail.setZero();
    head.col(0) = fillState(at(0U), jointsAt(0U));
    tail.col(0) = fillState(at(control.size() - 1U), jointsAt(control.size() - 1U));

    Eigen::MatrixXd inner(kDim, std::max(0, piece_count - 1));
    for (int i = 0; i < piece_count - 1; ++i) {
      inner.col(i) =
          fillState(at(static_cast<std::size_t>(i + 1)), jointsAt(static_cast<std::size_t>(i + 1)));
    }

    Eigen::VectorXd durations(piece_count);
    for (int i = 0; i < piece_count; ++i) {
      const auto from = static_cast<std::size_t>(i);
      const auto to = static_cast<std::size_t>(i + 1);
      const double distance = std::hypot(at(to).x - at(from).x, at(to).y - at(from).y);

      double joint_travel = 0.0;
      for (Eigen::Index j = 0; j < joint_count; ++j) {
        joint_travel =
            std::max(joint_travel, std::abs(jointsAt(to).positions[static_cast<std::size_t>(j)] -
                                            jointsAt(from).positions[static_cast<std::size_t>(j)]));
      }

      const double by_base = config_.cruise_speed > 0.0 ? distance / config_.cruise_speed : 0.0;
      const double by_arm =
          config_.max_joint_speed > 0.0 ? joint_travel / config_.max_joint_speed : 0.0;
      durations(i) = std::max({by_base, by_arm, config_.min_segment_duration});
    }

    if (allow_optimization && config_.enable_optimization && optimizer) {
      result.optimization_attempted = true;
      wbmm::traj_opt::OptimizerInput input;
      input.header = request.header;
      input.head_state = head;
      input.tail_state = tail;
      input.inner_points = inner;
      input.durations = durations;
      input.gear = gear;
      input.joint_names = request.start_joints.names;
      input.environment_revision = request.environment_revision;
      input.collision_model_revision = request.collision_model_revision;
      auto options = config_.optimizer;
      options.initial_yaw = request.start.yaw;
      options.max_base_speed = std::min(config_.builder.max_linear_velocity,
                                        request.limits.max_base_speed);
      options.max_yaw_rate = std::min(config_.builder.max_yaw_rate,
                                      request.limits.max_base_yaw_rate);
      options.max_joint_speed =
          std::min({config_.max_joint_speed, config_.builder.max_joint_velocity,
                    *std::min_element(request.limits.max_joint_speed.begin(),
                                      request.limits.max_joint_speed.end())});
      options.max_total_duration = config_.max_trajectory_duration;
      options.joint_min = Eigen::Map<const Eigen::VectorXd>(
          request.limits.joint_min.data(), joint_count);
      options.joint_max = Eigen::Map<const Eigen::VectorXd>(
          request.limits.joint_max.data(), joint_count);
      const auto started = Clock::now();
      try {
        auto candidate = optimizer(input, options);
        result.optimization_message = candidate.message;
        result.optimization_evaluations = candidate.evaluations;
        result.initial_cost = candidate.initial_cost;
        result.final_cost = candidate.final_cost;
        if (candidate.success && candidate.inner_points.rows() == kDim &&
            candidate.inner_points.cols() == piece_count - 1 &&
            candidate.durations.size() == piece_count &&
            candidate.inner_points.allFinite() &&
            candidate.durations.allFinite() &&
            (candidate.durations.array() > 0).all() &&
            std::isfinite(candidate.final_cost) &&
            candidate.final_cost <=
                candidate.initial_cost +
                    1e-8 * std::max(1.0, std::abs(candidate.initial_cost))) {
          inner = candidate.inner_points;
          durations = candidate.durations;
          result.optimization_applied = true;
        }
      } catch (const std::exception &error) {
        result.optimization_message = error.what();
      }
      result.optimization_time = secondsSince(started);
    }

    wbmm::traj_opt::MinSnapOpt<kDim> minco;
    // ---- 5. Publishable trajectory -----------------------------------------
    auto builder_config = config_.builder;
    builder_config.max_linear_velocity =
        std::min(builder_config.max_linear_velocity, request.limits.max_base_speed);
    builder_config.max_yaw_rate =
        std::min(builder_config.max_yaw_rate, request.limits.max_base_yaw_rate);
    builder_config.max_joint_velocity = std::min(
        builder_config.max_joint_velocity, *std::min_element(request.limits.max_joint_speed.begin(),
                                                             request.limits.max_joint_speed.end()));
    builder_config.gear = gear;
    builder_config.initial_yaw = request.start.yaw;
    builder_config.trajectory_id = config_.trajectory_id;
    builder_config.environment_revision = request.environment_revision;
    builder_config.collision_model_revision = request.collision_model_revision;

    // A rest-to-rest septic peaks at roughly 2.19x its average speed, so
    // allocating the duration from an average speed alone always overshoots the
    // envelope. Stretch the durations until the emitted reference fits, which is
    // the quantity that actually matters. REMANI does the same thing, but scales
    // on a dense-sample estimate instead of on the published trajectory.
    wbmm::traj_opt::TrajectoryBuildResult built;
    for (int attempt = 0; attempt < kMaxTimeScaleAttempts; ++attempt) {
      result.time_scale_attempts = attempt + 1;
      if (!durations.allFinite() || durations.sum() > config_.max_trajectory_duration) {
        return fail("Time scaling exceeds the trajectory duration budget.");
      }
      minco.reset(head, tail, piece_count);
      minco.generate(inner, durations);

      built = wbmm::traj_opt::buildWholeBodyTrajectory(
          minco.getTraj(gear), request.start_joints.names, request.header.frame_id, builder_config);
      if (!built.success) {
        result.build_time = secondsSince(build_started);
        return fail("Trajectory shaping failed: " + built.message);
      }
      result.max_linear_velocity = built.max_linear_velocity;
      result.max_yaw_rate = built.max_yaw_rate;
      result.max_joint_velocity = built.max_joint_velocity;
      result.max_heading_step = built.max_heading_step;
      if (built.within_limits) {
        break;
      }

      const double scale =
          std::max({built.max_linear_velocity / std::max(1e-9, builder_config.max_linear_velocity),
                    built.max_yaw_rate / std::max(1e-9, builder_config.max_yaw_rate),
                    built.max_joint_velocity / std::max(1e-9, builder_config.max_joint_velocity)});
      if (!(scale > 1.0) || !std::isfinite(scale)) {
        // Stretching time cannot fix this: the remaining violation is a heading
        // discontinuity, i.e. a gear flip with no planned turn.
        break;
      }
      durations *= kTimeScaleStep * scale;
    }
    result.build_time = secondsSince(build_started);

    if (!built.success) {
      return fail("Trajectory shaping failed: " + built.message);
    }
    if (!built.within_limits) {
      // Never publish a reference the controller cannot track; the caller has to
      // re-plan with a slower profile instead.
      return fail("Trajectory shaping produced an untrackable reference after " +
                  std::to_string(kMaxTimeScaleAttempts) +
                  " time-scaling attempts: " + built.message);
    }

    const double total_duration =
        built.trajectory.points.empty() ? 0.0 : built.trajectory.points.back().time_from_start;
    if (config_.max_trajectory_duration > 0.0 && total_duration > config_.max_trajectory_duration) {
      return fail("A trackable reference would take " + std::to_string(total_duration) +
                  " s, which exceeds the " + std::to_string(config_.max_trajectory_duration) +
                  " s budget. The controller envelope and the goal are inconsistent.");
    }

    result.trajectory = std::move(built.trajectory);
    result.trajectory_backend =
        result.optimization_applied ? "minco_optimized" : "minco";
    result.success = true;
    return result;
  };
  auto shaped = shape(true);
  bool candidate_validated = false;
  if (result.optimization_applied) {
    if (shaped.success) {
      const auto rejection = validateTrajectory(shaped.trajectory);
      if (!rejection.empty()) {
        shaped.success = false;
        shaped.message = rejection;
      } else
        candidate_validated = true;
    }
    if (!shaped.success) {
      result.optimization_applied = false;
      result.optimization_message =
          "Candidate rejected; using unoptimized seed: " + shaped.message;
      shaped = shape(false);
    }
  }
  if (!shaped.success) {
    if (!config_.enable_primitive_fallback)
      return shaped;
    result.fallback_reason += (result.fallback_reason.empty() ? "" : "; ") + shaped.message;
    result.trajectory = {};
    result.trajectory.trajectory_id = config_.trajectory_id;
    result.trajectory.environment_revision = request.environment_revision;
    result.trajectory.collision_model_revision = request.collision_model_revision;
    result.trajectory_backend = "time_scaled_primitives";
    result.optimization_applied = false;
    result.gear = pathGear(result.search.base_path);
    result.max_linear_velocity = result.max_yaw_rate = result.max_joint_velocity =
        result.max_heading_step = 0;
    const auto started = Clock::now();
    const double vmax =
        std::min(config_.builder.max_linear_velocity, request.limits.max_base_speed);
    const double wmax = std::min(config_.builder.max_yaw_rate, request.limits.max_base_yaw_rate);
    const double qmax = std::min({config_.builder.max_joint_velocity, config_.max_joint_speed,
                                  *std::min_element(request.limits.max_joint_speed.begin(),
                                                    request.limits.max_joint_speed.end())});
    for (double value :
         {vmax, wmax, qmax, config_.builder.sample_dt, config_.builder.max_heading_step})
      if (!std::isfinite(value) || value <= 0)
        return fail("Invalid primitive trajectory envelope.");
    double elapsed = 0;
    double previous_yaw = request.start.yaw;
    auto append = [&](std::size_t edge, double fraction, double speed, double t) {
      wbmm::core::WholeBodyTrajectoryPoint point;
      point.time_from_start = t;
      point.phase = wbmm::core::ExecutionPhase::kNavigate;
      point.state.header = request.header;
      point.state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
      point.state.base =
          seed.primitives.empty()
              ? request.start
              : wbmm::search::propagate(seed.search.base_path[edge], seed.primitives[edge],
                                        fraction * seed.primitives[edge].duration);
      auto &base_state = point.state.base;
      base_state.yaw = previous_yaw + std::remainder(base_state.yaw - previous_yaw, 2 * M_PI);
      result.max_heading_step =
          std::max(result.max_heading_step, std::abs(base_state.yaw - previous_yaw));
      previous_yaw = base_state.yaw;
      base_state.linear_velocity = seed.primitives.empty() ? 0 : seed.primitives[edge].v * speed;
      base_state.yaw_rate = seed.primitives.empty() ? 0 : seed.primitives[edge].omega * speed;
      base_state.lateral_velocity = 0;
      point.state.joints = seed.search.arm_seed[edge];
      for (std::size_t j = 0; j < 6; ++j) {
        const double delta = seed.primitives.empty() ? 0
                                                     : seed.search.arm_seed[edge + 1].positions[j] -
                                                           seed.search.arm_seed[edge].positions[j];
        point.state.joints.positions[j] += fraction * delta;
        point.state.joints.velocities[j] =
            seed.primitives.empty() ? 0 : delta * speed / seed.primitives[edge].duration;
        result.max_joint_velocity =
            std::max(result.max_joint_velocity, std::abs(point.state.joints.velocities[j]));
      }
      wbmm::core::WholeBodyInput input;
      input.base_model = wbmm::core::BaseModel::kDifferentialDrive;
      input.joint_names = request.start_joints.names;
      input.base_command = {base_state.linear_velocity, base_state.yaw_rate};
      input.joint_velocities = point.state.joints.velocities;
      point.feedforward_input = input;
      result.max_linear_velocity =
          std::max(result.max_linear_velocity, std::abs(base_state.linear_velocity));
      result.max_yaw_rate = std::max(result.max_yaw_rate, std::abs(base_state.yaw_rate));
      result.trajectory.points.push_back(std::move(point));
    };
    append(0, 0, 0, 0);
    for (std::size_t i = 0; i < seed.primitives.size(); ++i) {
      const auto &primitive = seed.primitives[i];
      double duration = std::max({config_.min_segment_duration,
                                  1.875 * std::abs(primitive.v) * primitive.duration / vmax,
                                  1.875 * std::abs(primitive.omega) * primitive.duration / wmax});
      for (std::size_t j = 0; j < 6; ++j)
        duration = std::max(duration, 1.875 *
                                          std::abs(seed.search.arm_seed[i + 1].positions[j] -
                                                   seed.search.arm_seed[i].positions[j]) /
                                          qmax);
      if (!std::isfinite(duration) || elapsed + duration > config_.max_trajectory_duration)
        return fail("Primitive trajectory duration budget exceeded.");
      const double dt =
          std::min(config_.builder.sample_dt, config_.builder.max_heading_step / wmax);
      const double count = std::ceil(duration / dt);
      if (!std::isfinite(count) || count > 100000 - result.trajectory.points.size())
        return fail("Primitive trajectory sample budget exceeded.");
      for (std::size_t k = 1; k <= static_cast<std::size_t>(count); ++k) {
        const double u = k / count;
        const double fraction = u * u * u * (10 + u * (-15 + 6 * u));
        const double rate = 30 * u * u * (1 - u) * (1 - u) / duration;
        append(i, fraction, rate * primitive.duration, elapsed + u * duration);
      }
      elapsed += duration;
    }
    if (seed.primitives.empty())
      append(0, 0, 0, config_.builder.sample_dt);
    result.build_time = secondsSince(started);
  } else {
    result = std::move(shaped);
  }
  if (!candidate_validated) {
    const auto rejection = validateTrajectory(result.trajectory);
    if (!rejection.empty())
      return fail(rejection);
  }
  result.success = true;
  result.message = "ok";
  last_error_.clear();
  return result;
}

}  // namespace wbmm::planning
