#pragma once

#include "wbmm_core/status.hpp"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace wbmm::core
{

// All linear, angular and temporal values use SI units unless a field says otherwise.
enum class ClockType
{
  kUnspecified = 0,
  kSystem,
  kSteady,
  kSimulation,
};

struct Timestamp
{
  std::int64_t nanoseconds{0};
  ClockType clock{ClockType::kUnspecified};
};

struct Header
{
  std::string frame_id;
  Timestamp stamp;
};

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// Internal order is explicitly w, x, y, z. ROS adapters must convert from xyzw.
struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Pose
{
  Header header;
  Vector3 position;
  Quaternion orientation;
};

struct Wrench
{
  Header header;
  Vector3 force;
  Vector3 torque;
};

enum class BaseModel
{
  kUnspecified = 0,
  kFixed,
  kDifferentialDrive,
  kOmnidirectional,
};

struct BaseState
{
  double x_m{0.0};
  double y_m{0.0};
  double yaw_rad{0.0};
  double linear_velocity_mps{0.0};
  double lateral_velocity_mps{0.0};
  double yaw_rate_radps{0.0};
};

struct JointState
{
  std::vector<std::string> names;
  std::vector<double> positions_rad;
  std::vector<double> velocities_radps;
  std::vector<double> efforts_nm;
};

struct WholeBodyState
{
  Header header;
  BaseModel base_model{BaseModel::kUnspecified};
  BaseState base;
  JointState joints;
};

struct WholeBodyInput
{
  Timestamp stamp;
  BaseModel base_model{BaseModel::kUnspecified};
  // fixed: empty; differential drive: [linear_velocity, yaw_rate];
  // omnidirectional: [longitudinal_velocity, lateral_velocity, yaw_rate].
  std::vector<double> base_command;
  std::vector<std::string> joint_names;
  std::vector<double> joint_velocities_radps;
};

struct WholeBodyTrajectoryPoint
{
  double time_from_start_s{0.0};
  WholeBodyState state;
  std::optional<WholeBodyInput> feedforward_input;
};

struct TaskTrajectoryPoint
{
  double time_from_start_s{0.0};
  Pose pose;
  std::optional<Wrench> desired_wrench;
};

struct TaskTrajectory
{
  std::string task_id;
  std::vector<TaskTrajectoryPoint> points;
};

struct WholeBodyTrajectory
{
  std::string trajectory_id;
  std::uint64_t environment_revision{0};
  std::uint64_t collision_model_revision{0};
  std::vector<WholeBodyTrajectoryPoint> points;
};

enum class UnknownSpacePolicy
{
  kReject = 0,
  kTreatAsOccupied,
  kTreatAsFree,
};

struct EnvironmentSnapshot
{
  Header header;
  std::uint64_t revision{0};
  std::uint64_t collision_model_revision{0};
  std::string source;
  std::string distance_semantics;
  double validity_horizon_s{0.0};
  UnknownSpacePolicy unknown_space_policy{UnknownSpacePolicy::kReject};
};

struct CollisionCheckResult
{
  bool collision_free{false};
  bool environment_checked{false};
  bool continuous_segment_checked{false};
  double minimum_clearance_m{0.0};
  std::string limiting_pair;
  std::uint64_t environment_revision{0};
  std::uint64_t collision_model_revision{0};
};

struct PlanningRequest
{
  WholeBodyState start;
  TaskTrajectory task;
  EnvironmentSnapshot environment;
};

struct PlanningResult
{
  Status status{ErrorCode::kNotConfigured, "planning has not been run"};
  std::optional<WholeBodyTrajectory> trajectory;
  double minimum_clearance_m{0.0};
  bool continuous_collision_checked{false};
  std::uint64_t environment_revision{0};
  std::uint64_t collision_model_revision{0};
};

struct ContactCorrection
{
  Header header;
  Vector3 translation_m;
  Vector3 rotation_rad;
};

enum class ExecutionPhase
{
  kIdle = 0,
  kNavigating,
  kApproaching,
  kTaskExecution,
  kRetreating,
  kFault,
};

enum class ControlMode
{
  kInactive = 0,
  kPosition,
  kVelocity,
  kContact,
  kHold,
};

struct ControlStatus
{
  Timestamp stamp;
  ExecutionPhase phase{ExecutionPhase::kIdle};
  ControlMode mode{ControlMode::kInactive};
  bool active{false};
  std::string detail;
};

struct Fault
{
  Timestamp stamp;
  ErrorCode code{ErrorCode::kOk};
  std::string source;
  std::string message;
  bool recoverable{false};
};

struct Matrix
{
  std::size_t rows{0};
  std::size_t cols{0};
  std::vector<double> row_major_data;
};

struct DistanceQuery
{
  Header header;
  Vector3 point;
};

struct DistanceQueryResult
{
  double signed_distance_m{0.0};
  Vector3 gradient;
  std::uint64_t environment_revision{0};
};

struct ContactControlRequest
{
  Pose nominal_pose;
  Wrench measured_wrench;
  Wrench desired_wrench;
  double dt_s{0.0};
};

struct AllocationRequest
{
  ContactCorrection correction;
  WholeBodyState measured_state;
};

struct NavigationRequest
{
  WholeBodyState start;
  Pose goal;
  EnvironmentSnapshot environment;
};

struct TrackingRequest
{
  WholeBodyTrajectory trajectory;
  WholeBodyState measured_state;
  double time_from_start_s{0.0};
};

enum class ReferenceOwner
{
  kNone = 0,
  kNavigation,
  kTaskExecution,
  kSafety,
};

struct OwnershipLease
{
  ReferenceOwner owner{ReferenceOwner::kNone};
  std::uint64_t generation{0};
  Timestamp acquired_at;
};

struct OperationContext
{
  // Empty means the caller did not supply the corresponding constraint.
  std::optional<std::int64_t> timeout_nanoseconds;
  std::optional<std::int64_t> maximum_data_age_nanoseconds;
  const std::atomic_bool * cancellation_requested{nullptr};

  [[nodiscard]] bool isCancellationRequested() const noexcept
  {
    return cancellation_requested != nullptr && cancellation_requested->load();
  }
};

}  // namespace wbmm::core
