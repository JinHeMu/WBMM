#pragma once

#include <wbmm_core/trajectory.hpp>
#include <wbmm_core/types.hpp>
#include <wbmm_planner/optimization/minco.hpp>

#include <string>
#include <vector>

namespace wbmm::traj_opt
{

struct TrajectoryBuilderConfig
{
  // Output sampling period. The published trajectory is always sampled, never
  // polynomial coefficients, so the consumer never re-derives yaw.
  double sample_dt{0.05};

  // Deprecated compatibility settings. Heading now uses the velocity tangent;
  // smoothing it independently from x/y breaks differential-drive kinematics.
  double tangent_chord_length{0.15};
  double yaw_hold_distance{0.005};

  // Differential-drive gear: +1 forward, -1 reverse. It flips the sign of the
  // forward velocity and rotates the heading by pi, exactly as in REMANI.
  int gear{1};

  // Body heading at the start of the trajectory, in rad. The tangent is
  // undefined at rest, so the robot's current heading must seed the unwrapping;
  // without it the first moving sample would introduce a step. For a reverse
  // plan this is normally already the reversed heading, because a gear flip
  // without a planned turn is not a feasible reference.
  double initial_yaw{0.0};

  // A heading step larger than this between consecutive samples is reported as
  // a violation rather than emitted silently. It is the signature of a planned
  // gear flip with no turn in the trajectory.
  double max_heading_step{0.35};

  // Controller envelope. The builder never silently clamps: it reports a
  // violation so the caller can reject or re-time the trajectory.
  double max_linear_velocity{0.5};
  double max_yaw_rate{1.0};
  double max_joint_velocity{2.0};

  wbmm::core::ExecutionPhase phase{wbmm::core::ExecutionPhase::kNavigate};
  std::string trajectory_id{"trajectory"};
  std::uint64_t environment_revision{0};
  std::uint64_t collision_model_revision{0};
};

struct TrajectoryBuildResult
{
  bool success{false};
  std::string message;

  wbmm::core::WholeBodyTrajectory trajectory;

  // Observed extremes over the emitted samples, in the units of the contract.
  double max_linear_velocity{0.0};
  double max_yaw_rate{0.0};
  double max_joint_velocity{0.0};
  double max_heading_step{0.0};

  // False when any observed extreme exceeds the configured envelope. The
  // trajectory is still returned so the caller can inspect it.
  bool within_limits{true};
};

// Converts an optimized MINCO trajectory into the canonical WBMM whole-body
// trajectory.
//
// Input layout follows the whole-body contract: 2 base DoF followed by the
// joint angles, i.e. [x, y, q1..qn]. Yaw is NOT a MINCO dimension: a
// differential-drive base is nonholonomic, so the heading is defined by the
// direction of travel while moving; the rest-point heading is a fallback.
//
// The heading is emitted first and the yaw rate is then taken as the time
// derivative of the heading that was actually emitted, which keeps state and
// feedforward input mutually consistent. REMANI instead derived the yaw rate
// analytically as (v x a)/|v|^2, which diverges as 1/t when the base starts
// from rest (measured: about 4.4 rad/s at t = 0.05 s against a 1.0 rad/s
// controller limit) and produced a reference the base could not track.
[[nodiscard]] TrajectoryBuildResult buildWholeBodyTrajectory(
  const Trajectory<7> & minco,
  const std::vector<std::string> & joint_names,
  const std::string & frame_id,
  const TrajectoryBuilderConfig & config = {});

}  // namespace wbmm::traj_opt
