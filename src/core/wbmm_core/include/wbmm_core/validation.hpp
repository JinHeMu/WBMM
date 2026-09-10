#pragma once

#include "wbmm_core/trajectory.hpp"
#include "wbmm_core/types.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace wbmm::core
{

struct ValidationResult
{
  bool ok{true};
  std::string message;

  explicit operator bool() const noexcept {return ok;}

  static ValidationResult failure(std::string message)
  {
    return {false, std::move(message)};
  }
};

namespace detail
{

inline constexpr double kTolerance = 1.0e-6;
inline constexpr double kPi = 3.14159265358979323846;

inline ValidationResult fail(std::string message)
{
  return ValidationResult::failure(std::move(message));
}

inline ValidationResult finite(const double value, const std::string & name)
{
  return std::isfinite(value) ? ValidationResult{} : fail(name + " must be finite");
}

inline ValidationResult header(const Header & value)
{
  if (value.frame_id.empty()) {
    return fail("Header.frame_id must not be empty");
  }
  const auto result = finite(value.stamp, "Header.stamp");
  if (!result.ok) {
    return result;
  }
  if (value.stamp < 0.0) {
    return fail("Header.stamp must be non-negative");
  }
  return value.clock == ClockDomain::kUnspecified ?
         fail("Header.clock must be specified") : ValidationResult{};
}

inline ValidationResult vectors(
  const Vector3 & linear, const Vector3 & angular, const std::string & name)
{
  const double values[] = {
    linear.x, linear.y, linear.z, angular.x, angular.y, angular.z};
  for (const double value : values) {
    if (!std::isfinite(value)) {
      return fail(name + " must be finite");
    }
  }
  return ValidationResult{};
}

inline ValidationResult pose(const Pose & value)
{
  const auto header_result = header(value.header);
  if (!header_result.ok) {
    return header_result;
  }
  const auto position = vectors(value.position, Vector3{}, "Pose.position");
  if (!position.ok) {
    return position;
  }
  const double q[] = {value.orientation.w, value.orientation.x, value.orientation.y,
    value.orientation.z};
  for (const double component : q) {
    if (!std::isfinite(component)) {
      return fail("Quaternion components must be finite");
    }
  }
  return isUnitQuaternion(value.orientation, kTolerance) ?
         ValidationResult{} : fail("Quaternion must be normalized");
}

inline ValidationResult joints(const JointState & value)
{
  if (value.names.size() != kSupportedJointCount ||
    value.positions.size() != kSupportedJointCount)
  {
    return fail("JointState must contain exactly six joint names and positions");
  }
  if ((!value.velocities.empty() && value.velocities.size() != kSupportedJointCount) ||
    (!value.efforts.empty() && value.efforts.size() != kSupportedJointCount))
  {
    return fail("JointState velocity/effort arrays must be empty or contain six values");
  }
  for (std::size_t i = 0; i < value.names.size(); ++i) {
    if (value.names[i].empty()) {
      return fail("JointState.names must not contain empty names");
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (value.names[i] == value.names[j]) {
        return fail("JointState.names must not contain duplicate names");
      }
    }
  }
  const auto check = [](const std::vector<double> & values, const char * name) {
      for (const double value : values) {
        if (!std::isfinite(value)) {
          return fail(std::string(name) + " must be finite");
        }
      }
      return ValidationResult{};
    };
  auto result = check(value.positions, "JointState.positions");
  if (!result.ok) {
    return result;
  }
  result = check(value.velocities, "JointState.velocities");
  return result.ok ? check(value.efforts, "JointState.efforts") : result;
}

inline ValidationResult taskPoint(const TaskTrajectoryPoint & value)
{
  auto result = finite(value.time_from_start, "TaskTrajectoryPoint.time_from_start");
  if (!result.ok || value.time_from_start < 0.0) {
    return fail("TaskTrajectoryPoint.time_from_start must be finite and non-negative");
  }
  result = pose(value.pose);
  if (!result.ok) {
    return result;
  }
  result = vectors(value.tangent, value.surface_normal, "TaskTrajectoryPoint vectors");
  return result;
}

}  // namespace detail

inline ValidationResult validate(const Twist & value)
{
  auto result = detail::header(value.header);
  return result.ok ? detail::vectors(value.linear, value.angular, "Twist") : result;
}

inline ValidationResult validate(const Wrench & value)
{
  auto result = detail::header(value.header);
  return result.ok ? detail::vectors(value.force, value.torque, "Wrench") : result;
}

inline ValidationResult validate(const WholeBodyState & value)
{
  const auto header_result = detail::header(value.header);
  if (!header_result.ok) {
    return header_result;
  }
  if (value.base_model != BaseModel::kDifferentialDrive) {
    return detail::fail("WholeBodyState.base_model must be kDifferentialDrive");
  }
  auto result = detail::finite(value.base.x, "WholeBodyState.base.x");
  if (!result.ok) {
    return result;
  }
  result = detail::finite(value.base.y, "WholeBodyState.base.y");
  if (!result.ok) {
    return result;
  }
  result = detail::finite(value.base.yaw, "WholeBodyState.base.yaw");
  if (!result.ok || value.base.yaw < -detail::kPi || value.base.yaw > detail::kPi) {
    return detail::fail("WholeBodyState.base.yaw must be finite and wrapped to [-pi, pi]");
  }
  result = detail::finite(value.base.linear_velocity, "WholeBodyState.base.linear_velocity");
  if (!result.ok) {
    return result;
  }
  result = detail::finite(value.base.lateral_velocity, "WholeBodyState.base.lateral_velocity");
  if (!result.ok) {
    return result;
  }
  result = detail::finite(value.base.yaw_rate, "WholeBodyState.base.yaw_rate");
  return result.ok ? detail::joints(value.joints) : result;
}

inline ValidationResult validate(const WholeBodyInput & value)
{
  auto result = detail::finite(value.stamp, "WholeBodyInput.stamp");
  if (!result.ok || value.stamp < 0.0) {
    return detail::fail("WholeBodyInput.stamp must be finite and non-negative");
  }
  if (value.clock == ClockDomain::kUnspecified) {
    return detail::fail("WholeBodyInput.clock must be specified");
  }
  if (value.base_model != BaseModel::kDifferentialDrive) {
    return detail::fail("WholeBodyInput.base_model must be kDifferentialDrive");
  }
  std::size_t expected_base_command_size = 0;
  switch (value.base_model) {
    case BaseModel::kFixed:
      expected_base_command_size = 0;
      break;
    case BaseModel::kDifferentialDrive:
      expected_base_command_size = kDifferentialBaseInputDim;
      break;
    case BaseModel::kOmnidirectional:
      expected_base_command_size = 3;
      break;
    case BaseModel::kUnspecified:
      return detail::fail("WholeBodyInput.base_model must be specified");
  }
  if (value.base_command.size() != expected_base_command_size) {
    return detail::fail("WholeBodyInput.base_command size must match base_model");
  }
  if (value.joint_names.size() != kSupportedJointCount ||
    value.joint_velocities.size() != kSupportedJointCount)
  {
    return detail::fail("WholeBodyInput must contain exactly six joint names and velocities");
  }
  for (std::size_t i = 0; i < value.joint_names.size(); ++i) {
    if (value.joint_names[i].empty()) {
      return detail::fail("WholeBodyInput.joint_names must not contain empty names");
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (value.joint_names[i] == value.joint_names[j]) {
        return detail::fail("WholeBodyInput.joint_names must not contain duplicates");
      }
    }
  }
  for (const double command : value.base_command) {
    if (!std::isfinite(command)) {
      return detail::fail("WholeBodyInput.base_command must be finite");
    }
  }
  for (const double velocity : value.joint_velocities) {
    if (!std::isfinite(velocity)) {
      return detail::fail("WholeBodyInput.joint_velocities must be finite");
    }
  }
  return ValidationResult{};
}

inline ValidationResult validate(const TaskTrajectory & value)
{
  if (value.task_id.empty() || value.points.empty()) {
    return detail::fail("TaskTrajectory must have an id and at least one point");
  }
  auto result = detail::taskPoint(value.points.front());
  if (!result.ok) {
    return result;
  }
  const auto & first = value.points.front();
  for (std::size_t i = 1; i < value.points.size(); ++i) {
    result = detail::taskPoint(value.points[i]);
    if (!result.ok) {
      return result;
    }
    if (value.points[i].time_from_start <= value.points[i - 1].time_from_start) {
      return detail::fail("TaskTrajectory time must be strictly increasing");
    }
    if (value.points[i].pose.header.frame_id != first.pose.header.frame_id ||
      value.points[i].pose.header.clock != first.pose.header.clock)
    {
      return detail::fail("TaskTrajectory points must share frame and clock");
    }
  }
  return ValidationResult{};
}

inline ValidationResult validate(
  const PhaseSchedule & value,
  const double trajectory_duration = -1.0)
{
  if (value.empty()) {
    return detail::fail("PhaseSchedule must not be empty");
  }
  for (std::size_t i = 0; i < value.size(); ++i) {
    const auto result = detail::finite(value[i].start_time, "PhaseSegment.start_time");
    if (!result.ok) {
      return result;
    }
    const auto end = detail::finite(value[i].end_time, "PhaseSegment.end_time");
    if (!end.ok) {
      return end;
    }
    if (value[i].start_time < 0.0 || value[i].end_time <= value[i].start_time) {
      return detail::fail("PhaseSegment must satisfy 0 <= start_time < end_time");
    }
    if (i == 0 && std::abs(value[i].start_time) > detail::kTolerance) {
      return detail::fail("PhaseSchedule must start at time 0");
    }
    if (i > 0 && std::abs(value[i - 1].end_time - value[i].start_time) > detail::kTolerance) {
      return detail::fail("PhaseSchedule must be contiguous without gaps or overlaps");
    }
  }
  if (trajectory_duration >= 0.0 &&
    std::abs(value.back().end_time - trajectory_duration) > detail::kTolerance)
  {
    return detail::fail("PhaseSchedule.back().end_time must match trajectory duration");
  }
  return ValidationResult{};
}

inline ValidationResult validate(const SearchResult & value)
{
  if (!std::isfinite(value.path_length) || !std::isfinite(value.solve_time) ||
    value.path_length < 0.0 || value.solve_time < 0.0)
  {
    return detail::fail("SearchResult path_length and solve_time must be finite and non-negative");
  }
  if (value.base_path.size() != value.arm_seed.size() ||
    value.base_path.size() != value.phases.size())
  {
    return detail::fail("SearchResult base_path, arm_seed and phases sizes must match");
  }
  if (value.success && value.base_path.empty()) {
    return detail::fail("SearchResult must not be empty when success is true");
  }
  for (std::size_t i = 0; i < value.base_path.size(); ++i) {
    const auto & base = value.base_path[i];
    if (!std::isfinite(base.x) || !std::isfinite(base.y) || !std::isfinite(base.yaw) ||
      !std::isfinite(base.linear_velocity) || !std::isfinite(base.lateral_velocity) ||
      !std::isfinite(base.yaw_rate) || base.yaw < -detail::kPi || base.yaw > detail::kPi)
    {
      return detail::fail("SearchResult base state must be finite and yaw wrapped");
    }
    const auto joint_result = detail::joints(value.arm_seed[i]);
    if (!joint_result.ok) {
      return joint_result;
    }
    if (value.phases[i] == ExecutionPhase::kIdle || value.phases[i] == ExecutionPhase::kFault) {
      return detail::fail("SearchResult phases must be active");
    }
  }
  return ValidationResult{};
}

inline ValidationResult validate(const WholeBodyTrajectory & value)
{
  if (value.trajectory_id.empty() || value.points.empty() ||
    value.environment_revision == 0 || value.collision_model_revision == 0)
  {
    return detail::fail("WholeBodyTrajectory id, points and revisions must be valid");
  }
  const auto & first = value.points.front();
  auto result = validate(first.state);
  if (!result.ok) {
    return result;
  }
  const auto frame_id = first.state.header.frame_id;
  const auto base_model = first.state.base_model;
  const auto clock = first.state.header.clock;
  const auto joint_names = first.state.joints.names;

  for (std::size_t i = 0; i < value.points.size(); ++i) {
    const auto & point = value.points[i];
    const auto time = detail::finite(point.time_from_start, "WholeBodyTrajectory.time_from_start");
    if (!time.ok || point.time_from_start < 0.0) {
      return detail::fail("WholeBodyTrajectory time must be finite and non-negative");
    }
    result = validate(point.state);
    if (!result.ok) {
      return result;
    }
    if (i > 0 && point.time_from_start <= value.points[i - 1].time_from_start) {
      return detail::fail("WholeBodyTrajectory time must be strictly increasing");
    }
    if (point.state.header.frame_id != frame_id ||
      point.state.base_model != base_model ||
      point.state.header.clock != clock ||
      point.state.joints.names != joint_names)
    {
      return detail::fail("WholeBodyTrajectory points must share frame, model, clock and joints");
    }
    if (point.feedforward_input.has_value()) {
      const auto input = validate(*point.feedforward_input);
      if (!input.ok ||
        point.feedforward_input->clock != clock ||
        point.feedforward_input->base_model != base_model ||
        point.feedforward_input->joint_names != joint_names)
      {
        return detail::fail("WholeBodyTrajectory feedforward_input must match its state");
      }
    }
    if (point.task_reference.has_value()) {
      const auto task = detail::taskPoint(*point.task_reference);
      if (!task.ok) {
        return task;
      }
      if (point.task_reference->pose.header.clock != clock ||
        point.task_reference->pose.header.frame_id != point.state.header.frame_id)
      {
        return detail::fail("WholeBodyTrajectory task_reference must match state frame and clock");
      }
    }
  }
  return ValidationResult{};
}

}  // namespace wbmm::core
