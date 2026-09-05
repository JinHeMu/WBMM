#include "wbmm_core/validation.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_set>

namespace wbmm::core
{
namespace
{

bool finite(const double value)
{
  return std::isfinite(value);
}

bool finite(const Vector3 & vector)
{
  return finite(vector.x) && finite(vector.y) && finite(vector.z);
}

Status validateFiniteVector(const std::vector<double> & values, const char * field)
{
  for (std::size_t index = 0; index < values.size(); ++index) {
    if (!finite(values[index])) {
      std::ostringstream stream;
      stream << field << '[' << index << "] is not finite";
      return {ErrorCode::kNonFiniteValue, stream.str()};
    }
  }
  return Status::Ok();
}

Status validateJointNames(const std::vector<std::string> & names)
{
  if (names.empty()) {
    return {ErrorCode::kInvalidDimension, "joint names must not be empty"};
  }

  std::unordered_set<std::string> unique_names;
  for (std::size_t index = 0; index < names.size(); ++index) {
    if (names[index].empty()) {
      return {ErrorCode::kInvalidArgument, "joint name must not be empty"};
    }
    if (!unique_names.insert(names[index]).second) {
      return {ErrorCode::kDuplicateJointName, "duplicate joint name: " + names[index]};
    }
  }
  return Status::Ok();
}

Status validateOptionalJointVector(
  const std::vector<double> & values, const std::size_t expected_size, const char * field)
{
  if (!values.empty() && values.size() != expected_size) {
    std::ostringstream stream;
    stream << field << " has dimension " << values.size() << ", expected " << expected_size;
    return {ErrorCode::kInvalidDimension, stream.str()};
  }
  return validateFiniteVector(values, field);
}

Status validateBaseState(const BaseState & base)
{
  const std::array<double, 6> values{
    base.x_m, base.y_m, base.yaw_rad, base.linear_velocity_mps,
    base.lateral_velocity_mps, base.yaw_rate_radps};
  for (const double value : values) {
    if (!finite(value)) {
      return {ErrorCode::kNonFiniteValue, "base state contains a non-finite value"};
    }
  }
  return Status::Ok();
}

std::size_t expectedBaseCommandDimension(const BaseModel model)
{
  switch (model) {
    case BaseModel::kFixed: return 0;
    case BaseModel::kDifferentialDrive: return 2;
    case BaseModel::kOmnidirectional: return 3;
    case BaseModel::kUnspecified: break;
  }
  return std::numeric_limits<std::size_t>::max();
}

}  // namespace

Status validateTimestamp(const Timestamp & stamp)
{
  if (stamp.clock == ClockType::kUnspecified) {
    return {ErrorCode::kInvalidTimestamp, "timestamp clock must be specified"};
  }
  if (stamp.nanoseconds < 0) {
    return {ErrorCode::kInvalidTimestamp, "timestamp must not be negative"};
  }
  return Status::Ok();
}

Status validateHeader(const Header & header)
{
  if (header.frame_id.empty()) {
    return {ErrorCode::kEmptyFrame, "frame_id must not be empty"};
  }
  return validateTimestamp(header.stamp);
}

Status validatePose(const Pose & pose, const double quaternion_tolerance)
{
  if (const auto status = validateHeader(pose.header); !status.ok()) {
    return status;
  }
  if (!finite(pose.position)) {
    return {ErrorCode::kNonFiniteValue, "pose position contains a non-finite value"};
  }
  if (!finite(pose.orientation.w) || !finite(pose.orientation.x) ||
    !finite(pose.orientation.y) || !finite(pose.orientation.z))
  {
    return {ErrorCode::kNonFiniteValue, "pose quaternion contains a non-finite value"};
  }
  if (!finite(quaternion_tolerance) || quaternion_tolerance < 0.0) {
    return {ErrorCode::kInvalidArgument, "quaternion tolerance must be finite and non-negative"};
  }

  const double squared_norm =
    pose.orientation.w * pose.orientation.w + pose.orientation.x * pose.orientation.x +
    pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z;
  if (std::abs(squared_norm - 1.0) > quaternion_tolerance) {
    return {ErrorCode::kInvalidQuaternion, "pose quaternion must be normalized in wxyz order"};
  }
  return Status::Ok();
}

Status validateWrench(const Wrench & wrench)
{
  if (const auto status = validateHeader(wrench.header); !status.ok()) {
    return status;
  }
  if (!finite(wrench.force) || !finite(wrench.torque)) {
    return {ErrorCode::kNonFiniteValue, "wrench contains a non-finite value"};
  }
  return Status::Ok();
}

Status validateJointState(const JointState & joints)
{
  if (const auto status = validateJointNames(joints.names); !status.ok()) {
    return status;
  }
  if (joints.positions_rad.size() != joints.names.size()) {
    return {ErrorCode::kInvalidDimension, "joint position dimension must match joint names"};
  }
  if (const auto status = validateFiniteVector(
      joints.positions_rad,
      "joint positions"); !status.ok())
  {
    return status;
  }
  if (const auto status = validateOptionalJointVector(
      joints.velocities_radps, joints.names.size(), "joint velocities"); !status.ok())
  {
    return status;
  }
  return validateOptionalJointVector(joints.efforts_nm, joints.names.size(), "joint efforts");
}

Status validateJointOrder(
  const std::vector<std::string> & actual,
  const std::vector<std::string> & expected)
{
  if (const auto status = validateJointNames(actual); !status.ok()) {
    return status;
  }
  if (const auto status = validateJointNames(expected); !status.ok()) {
    return {status.code(), "invalid expected joint order: " + status.message()};
  }
  if (actual.size() != expected.size()) {
    return {ErrorCode::kInvalidDimension, "joint order dimensions do not match"};
  }
  for (std::size_t index = 0; index < actual.size(); ++index) {
    if (actual[index] != expected[index]) {
      std::ostringstream stream;
      stream << "joint order mismatch at index " << index << ": got " << actual[index]
             << ", expected " << expected[index];
      return {ErrorCode::kJointNameMismatch, stream.str()};
    }
  }
  return Status::Ok();
}

Status validateWholeBodyState(const WholeBodyState & state)
{
  if (const auto status = validateHeader(state.header); !status.ok()) {
    return status;
  }
  if (state.base_model == BaseModel::kUnspecified) {
    return {ErrorCode::kUnsupportedBaseModel, "base model must be specified"};
  }
  if (const auto status = validateBaseState(state.base); !status.ok()) {
    return status;
  }
  return validateJointState(state.joints);
}

Status validateWholeBodyInput(const WholeBodyInput & input)
{
  if (const auto status = validateTimestamp(input.stamp); !status.ok()) {
    return status;
  }
  const std::size_t expected_base_dimension = expectedBaseCommandDimension(input.base_model);
  if (expected_base_dimension == std::numeric_limits<std::size_t>::max()) {
    return {ErrorCode::kUnsupportedBaseModel, "input base model must be specified"};
  }
  if (input.base_command.size() != expected_base_dimension) {
    std::ostringstream stream;
    stream << "base command has dimension " << input.base_command.size()
           << ", expected " << expected_base_dimension;
    return {ErrorCode::kInvalidDimension, stream.str()};
  }
  if (const auto status = validateFiniteVector(input.base_command, "base command"); !status.ok()) {
    return status;
  }
  if (const auto status = validateJointNames(input.joint_names); !status.ok()) {
    return status;
  }
  if (input.joint_velocities_radps.size() != input.joint_names.size()) {
    return {ErrorCode::kInvalidDimension, "joint command dimension must match joint names"};
  }
  return validateFiniteVector(input.joint_velocities_radps, "joint command");
}

Status validateTaskTrajectory(const TaskTrajectory & trajectory)
{
  if (trajectory.task_id.empty()) {
    return {ErrorCode::kInvalidArgument, "task_id must not be empty"};
  }
  if (trajectory.points.empty()) {
    return {ErrorCode::kInvalidDimension, "task trajectory must contain at least one point"};
  }

  double previous_time = -1.0;
  std::string expected_frame;
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & point = trajectory.points[index];
    if (!finite(point.time_from_start_s) || point.time_from_start_s < 0.0 ||
      point.time_from_start_s <= previous_time)
    {
      return {ErrorCode::kInvalidTimestamp,
        "task trajectory times must be finite and strictly increasing"};
    }
    if (const auto status = validatePose(point.pose); !status.ok()) {
      return {status.code(), "invalid task trajectory pose: " + status.message()};
    }
    if (index == 0) {
      expected_frame = point.pose.header.frame_id;
    } else if (point.pose.header.frame_id != expected_frame) {
      return {ErrorCode::kInvalidArgument, "task trajectory points must use one explicit frame"};
    }
    if (point.desired_wrench.has_value()) {
      if (const auto status = validateWrench(*point.desired_wrench); !status.ok()) {
        return {status.code(), "invalid desired wrench: " + status.message()};
      }
      if (point.desired_wrench->header.frame_id != expected_frame) {
        return {ErrorCode::kInvalidArgument, "task pose and desired wrench frames must match"};
      }
    }
    previous_time = point.time_from_start_s;
  }
  return Status::Ok();
}

Status validateWholeBodyTrajectory(const WholeBodyTrajectory & trajectory)
{
  if (trajectory.trajectory_id.empty()) {
    return {ErrorCode::kInvalidArgument, "trajectory_id must not be empty"};
  }
  if (trajectory.environment_revision == 0 || trajectory.collision_model_revision == 0) {
    return {ErrorCode::kEnvironmentRevisionMismatch, "trajectory revisions must be non-zero"};
  }
  if (trajectory.points.empty()) {
    return {ErrorCode::kInvalidDimension, "whole-body trajectory must contain at least one point"};
  }

  double previous_time = -1.0;
  std::vector<std::string> expected_joint_order;
  BaseModel expected_base_model = BaseModel::kUnspecified;
  std::string expected_frame;
  for (std::size_t index = 0; index < trajectory.points.size(); ++index) {
    const auto & point = trajectory.points[index];
    if (!finite(point.time_from_start_s) || point.time_from_start_s < 0.0 ||
      point.time_from_start_s <= previous_time)
    {
      return {ErrorCode::kInvalidTimestamp,
        "whole-body trajectory times must be finite and strictly increasing"};
    }
    if (const auto status = validateWholeBodyState(point.state); !status.ok()) {
      return {status.code(), "invalid whole-body trajectory state: " + status.message()};
    }
    if (index == 0) {
      expected_joint_order = point.state.joints.names;
      expected_base_model = point.state.base_model;
      expected_frame = point.state.header.frame_id;
    } else {
      if (const auto status =
        validateJointOrder(point.state.joints.names, expected_joint_order); !status.ok())
      {
        return status;
      }
      if (point.state.base_model != expected_base_model) {
        return {ErrorCode::kUnsupportedBaseModel, "base model changes within trajectory"};
      }
      if (point.state.header.frame_id != expected_frame) {
        return {ErrorCode::kInvalidArgument, "state frame changes within trajectory"};
      }
    }
    if (point.feedforward_input.has_value()) {
      if (const auto status = validateWholeBodyInput(*point.feedforward_input); !status.ok()) {
        return {status.code(), "invalid feedforward input: " + status.message()};
      }
      if (point.feedforward_input->base_model != expected_base_model) {
        return {ErrorCode::kUnsupportedBaseModel, "state and input base models differ"};
      }
      if (const auto status = validateJointOrder(
          point.feedforward_input->joint_names, expected_joint_order); !status.ok())
      {
        return status;
      }
    }
    previous_time = point.time_from_start_s;
  }
  return Status::Ok();
}

Status validateEnvironmentSnapshot(const EnvironmentSnapshot & snapshot)
{
  if (const auto status = validateHeader(snapshot.header); !status.ok()) {
    return status;
  }
  if (snapshot.revision == 0 || snapshot.collision_model_revision == 0) {
    return {ErrorCode::kEnvironmentRevisionMismatch, "environment revisions must be non-zero"};
  }
  if (snapshot.source.empty() || snapshot.distance_semantics.empty()) {
    return {ErrorCode::kInvalidArgument, "environment source and distance semantics are required"};
  }
  if (!finite(snapshot.validity_horizon_s) || snapshot.validity_horizon_s <= 0.0) {
    return {ErrorCode::kInvalidArgument,
      "environment validity horizon must be positive and finite"};
  }
  return Status::Ok();
}

Status validateMatrix(const Matrix & matrix)
{
  if (matrix.rows == 0 || matrix.cols == 0) {
    return {ErrorCode::kInvalidDimension, "matrix dimensions must be non-zero"};
  }
  if (matrix.cols > std::numeric_limits<std::size_t>::max() / matrix.rows ||
    matrix.row_major_data.size() != matrix.rows * matrix.cols)
  {
    return {ErrorCode::kInvalidDimension, "matrix data dimension does not match rows and columns"};
  }
  return validateFiniteVector(matrix.row_major_data, "matrix data");
}

Status DifferentialDriveContract::validateState(
  const WholeBodyState & state,
  const std::vector<std::string> & expected_joint_order)
{
  if (const auto status = validateWholeBodyState(state); !status.ok()) {
    return status;
  }
  if (state.base_model != BaseModel::kDifferentialDrive) {
    return {ErrorCode::kUnsupportedBaseModel, "differential-drive state required"};
  }
  if (std::abs(state.base.lateral_velocity_mps) > 1.0e-9) {
    return {ErrorCode::kSafetyLimitExceeded, "differential-drive lateral velocity must be zero"};
  }
  return validateJointOrder(state.joints.names, expected_joint_order);
}

Status DifferentialDriveContract::validateInput(
  const WholeBodyInput & input,
  const std::vector<std::string> & expected_joint_order)
{
  if (const auto status = validateWholeBodyInput(input); !status.ok()) {
    return status;
  }
  if (input.base_model != BaseModel::kDifferentialDrive) {
    return {ErrorCode::kUnsupportedBaseModel, "differential-drive input required"};
  }
  return validateJointOrder(input.joint_names, expected_joint_order);
}

}  // namespace wbmm::core
