#pragma once

#include <functional>
#include <wbmm_core/robot_model.hpp>
#include <wbmm_core/trajectory.hpp>
#include <wbmm_core/types.hpp>
#include <wbmm_planner/search/arm_seed_search.hpp>
#include <wbmm_planner/search/kino_astar.hpp>
#include <wbmm_planner/search/whole_body_rrt.hpp>
#include <wbmm_planner/optimization/whole_body_optimizer.hpp>
#include <wbmm_planner/optimization/whole_body_trajectory_builder.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace wbmm::planning
{

using TrajectoryOptimizer = std::function<wbmm::traj_opt::OptimizerResult(
    const wbmm::traj_opt::OptimizerInput &,
    const wbmm::traj_opt::OptimizerConfig &)>;

struct PlannerConfig
{
  bool enable_optimization{false};
  wbmm::traj_opt::OptimizerConfig optimizer{};
  wbmm::search::KinoAstarConfig base_search{};
  wbmm::search::ArmSeedConfig arm_seed{};  // legacy profile, retained for callers
  wbmm::search::WholeBodyRrtConfig sample_rrt{};
  wbmm::search::WholeBodyRrtConfig whole_body_rrt{};
  bool enable_whole_body_rrt{true};
  bool enable_primitive_fallback{true};
  wbmm::traj_opt::TrajectoryBuilderConfig builder{};

  // Time allocation for the MINCO pieces. The base cruise speed sets the
  // nominal pace; a piece is stretched further when the arm has to travel far
  // during it, so the arm never has to move faster than max_joint_speed.
  // The MINCO is built through every Nth resampled waypoint. Passing through
  // all of them makes it chase the search's high-frequency wiggle, and at each
  // waypoint the base x velocity dips while the direction is changing, which
  // swings the tangent hard enough to demand an untrackable yaw rate. Letting
  // MINCO smooth across a few waypoints is what it is for. The arm seed is
  // computed at full resolution; the final output is independently checked
  // after shaping, including interpolation between its samples.
  std::size_t minco_waypoint_stride{3};

  // Resolution of final checks on the bridge-interpolated reference.
  double validation_translation_step{0.02};
  double validation_angle_step{0.03};

  double cruise_speed{0.35};
  double min_segment_duration{0.4};
  double max_joint_speed{1.57};

  // Upper bound on the total trajectory duration. Time scaling can always make
  // a reference trackable by stretching it, so without this budget a
  // misconfigured envelope would silently produce a several-minute trajectory
  // instead of reporting the problem.
  double max_trajectory_duration{60.0};

  std::string trajectory_id{"wbmm_plan"};
};

struct PlanRequest
{
  wbmm::core::Header header;
  wbmm::core::BaseState start;
  wbmm::core::JointState start_joints;
  wbmm::core::BaseState goal;
  std::optional<wbmm::core::JointState> goal_joints;
  wbmm::core::RobotLimits limits;

  // Carried into the published trajectory so a consumer can detect that it was
  // planned against a different map.
  std::uint64_t environment_revision{0};
  std::uint64_t collision_model_revision{0};
};

struct PlanResult
{
  bool success{false};
  std::string message;

  // Base path plus the arm configuration sequence found for it.
  wbmm::core::SearchResult search;
  // The publishable output.
  wbmm::core::WholeBodyTrajectory trajectory;

  std::string search_backend;
  std::string trajectory_backend;
  std::string fallback_reason;
  bool whole_body_rrt_attempted{false};
  double whole_body_rrt_time{0.0};

  bool optimization_attempted{false};
  bool optimization_applied{false};
  std::string optimization_message;
  double optimization_time{0.0};
  double initial_cost{0.0}, final_cost{0.0};
  int optimization_evaluations{0};

  // Diagnostics, seconds.
  double base_search_time{0.0};
  double arm_seed_time{0.0};
  double build_time{0.0};

  // Extremes of the emitted reference, filled whenever shaping ran. They are
  // what the envelope check and the time-scaling loop act on, so exposing them
  // makes a rejection explainable.
  double max_linear_velocity{0.0};
  double max_yaw_rate{0.0};
  double max_joint_velocity{0.0};
  double max_heading_step{0.0};
  int time_scale_attempts{0};

  // Differential-drive gear; -2 denotes a segmented mixed-gear trajectory.
  int gear{1};
};

// Orchestrates base search, arm seeding and trajectory shaping into one
// publishable whole-body trajectory.
//
// ROS-free: both collision queries are injected, matching the seam style of
// wbmm::search::KinoAstar. The node that owns ROS lives in a separate package.
class WholeBodyPlanner
{
public:
  explicit WholeBodyPlanner(PlannerConfig config = {});

  [[nodiscard]] PlanResult
  plan(const PlanRequest &request,
       const wbmm::search::BaseCollisionChecker &base_checker,
       const wbmm::search::WholeBodyCollisionChecker &whole_body_checker,
       const TrajectoryOptimizer &optimizer = {}) const;

  [[nodiscard]] const PlannerConfig & config() const noexcept {return config_;}
  [[nodiscard]] const std::string & lastError() const noexcept {return last_error_;}

private:
  PlannerConfig config_;
  mutable std::string last_error_;
};

}  // namespace wbmm::planning
