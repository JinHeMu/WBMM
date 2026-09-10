#pragma once

#include "wbmm_core/types.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace wbmm::core
{

struct TaskTrajectoryPoint
{
  double time_from_start{0.0};
  Pose pose;

  // Nominal task geometry only.
  // Force control is an execution-layer correction on the planned nominal trajectory,
  // not a field of the planned task trajectory.
  Vector3 tangent;
  Vector3 surface_normal;

  bool contact{false};
};

struct TaskTrajectory
{
  std::string task_id;
  std::vector<TaskTrajectoryPoint> points;
};

struct PhaseSegment
{
  double start_time{0.0};
  double end_time{0.0};
  ExecutionPhase phase{ExecutionPhase::kIdle};
  std::string task_id;
  bool contact{false};
};

using PhaseSchedule = std::vector<PhaseSegment>;

// Search only provides topology and an initial guess.
// It is not a final executable trajectory.
struct SearchResult
{
  std::vector<BaseState> base_path;
  std::vector<JointState> arm_seed;
  std::vector<ExecutionPhase> phases;

  double path_length{0.0};
  double solve_time{0.0};
  bool success{false};
};

struct WholeBodyTrajectoryPoint
{
  double time_from_start{0.0};
  WholeBodyState state;
  std::optional<WholeBodyInput> feedforward_input;

  ExecutionPhase phase{ExecutionPhase::kIdle};

  // Optional link to the task point being tracked.
  // Do not copy the complete TaskTrajectory into every robot trajectory point.
  std::optional<TaskTrajectoryPoint> task_reference;
};

struct WholeBodyTrajectory
{
  std::string trajectory_id;
  std::uint64_t environment_revision{0};
  std::uint64_t collision_model_revision{0};
  std::vector<WholeBodyTrajectoryPoint> points;
};

inline bool empty(const WholeBodyTrajectory & trajectory)
{
  return trajectory.points.empty();
}

inline double duration(const WholeBodyTrajectory & trajectory)
{
  return trajectory.points.empty() ? 0.0 : trajectory.points.back().time_from_start;
}

}  // namespace wbmm::core
