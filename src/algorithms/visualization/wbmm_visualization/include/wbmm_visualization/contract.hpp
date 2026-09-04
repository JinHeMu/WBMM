#pragma once

#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <Eigen/Core>

#include <array>
#include <string>
#include <utility>
#include <vector>

namespace wbmm_viz
{

// Fixed 9-name whole-body state contract shared by the planners. Index 2 is
// the base yaw; entries 3..8 are the six arm joints.
inline constexpr std::array<const char *, 9> kJointNames{
  "base_x", "base_y", "base_yaw", "joint_1", "joint_2", "joint_3",
  "joint_4", "joint_5", "joint_6"};

// Display-only 9D waypoint parsed from the JointTrajectory contract.
struct Waypoint
{
  double time{0.0};
  std::array<double, 9> state{};  // base_x, base_y, base_yaw, q1..q6
  std::string phase;
  bool in_contact{false};
};

struct ParsedTrajectory
{
  std::string frame_id;
  std::vector<Waypoint> waypoints;  // ascending time
  double duration{0.0};
};

// Parse the 9D whole-body contract. Validates exactly the 9 fixed joint names
// in order, at least one point, 9 positions per point and non-decreasing
// time_from_start (clamped, not rejected). Throws std::invalid_argument on
// contract violations. When phase_schedule is non-null and decodable, each
// waypoint gets the phase of its time interval and in_contact is set for
// TASK_CONSTRAINED phases; a malformed schedule is ignored (phases stay empty).
ParsedTrajectory parseJointTrajectory(
  const trajectory_msgs::msg::JointTrajectory & msg,
  const std::string * phase_schedule);

// Interpolate the trajectory at time t (clamped to the ends). Yaw is
// interpolated across the +/-pi wrap. Ported from the TA-WBMP demo playback
// semantics.
Waypoint waypointAt(const ParsedTrajectory & trajectory, double t);

// Ascending samples covering [start, end]: interval endpoints plus every
// stored waypoint strictly inside, clamped to the trajectory duration.
std::vector<Waypoint> samplesInInterval(const ParsedTrajectory & trajectory,
                                        double start, double end);

Eigen::VectorXd toStateVector(const Waypoint & waypoint);

// Phase schedule wire format:
//   "<t0> <PHASE_0>;<t1> <PHASE_1>;..."  (times in trajectory seconds)
// The interval [t_i, t_{i+1}) carries PHASE_i; the last entry extends to the
// end. Published atomically after the trajectory on a transient-local topic.
std::string encodePhaseSchedule(
  const std::vector<std::pair<double, std::string>> & intervals);
bool decodePhaseSchedule(const std::string & text,
                         std::vector<std::pair<double, std::string>> & out);
// Phase of time t; "" for an empty schedule. The interval [t_i, t_{i+1})
// carries the phase of entry i; the last entry extends to the end, and times
// before the first boundary take the first phase.
std::string phaseAt(const std::vector<std::pair<double, std::string>> & schedule,
                    double t);

}  // namespace wbmm_viz
