#include "wbmm_math/conversions.hpp"

#include "wbmm_core/validation.hpp"

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <sstream>
#include <unordered_map>
#include <utility>

namespace wbmm::math
{
namespace
{

using RowMajorMatrixXd =
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

core::Result<Eigen::VectorXd> reorderValues(
  const std::vector<std::string> & actual_names,
  const std::vector<double> & actual_values,
  const std::vector<std::string> & expected_names,
  const char * field_name)
{
  if (actual_names.size() != actual_values.size()) {
    return core::Result<Eigen::VectorXd>::Failure(
      {core::ErrorCode::kInvalidDimension,
        std::string(field_name) + " dimension must match actual joint names"});
  }
  if (actual_names.size() != expected_names.size()) {
    return core::Result<Eigen::VectorXd>::Failure(
      {core::ErrorCode::kInvalidDimension,
        std::string(field_name) + " and expected joint order dimensions do not match"});
  }
  if (const auto status = core::validateJointOrder(actual_names, actual_names); !status.ok()) {
    return core::Result<Eigen::VectorXd>::Failure(status);
  }
  if (const auto status = core::validateJointOrder(expected_names, expected_names); !status.ok()) {
    return core::Result<Eigen::VectorXd>::Failure(
      {status.code(), "invalid expected joint order: " + status.message()});
  }

  std::unordered_map<std::string, std::size_t> actual_index;
  actual_index.reserve(actual_names.size());
  for (std::size_t index = 0; index < actual_names.size(); ++index) {
    actual_index.emplace(actual_names[index], index);
  }

  Eigen::VectorXd result(static_cast<Eigen::Index>(expected_names.size()));
  for (std::size_t output_index = 0; output_index < expected_names.size(); ++output_index) {
    const auto input_index = actual_index.find(expected_names[output_index]);
    if (input_index == actual_index.end()) {
      return core::Result<Eigen::VectorXd>::Failure(
        {core::ErrorCode::kJointNameMismatch,
          "required joint is missing: " + expected_names[output_index]});
    }
    const double value = actual_values[input_index->second];
    if (!std::isfinite(value)) {
      return core::Result<Eigen::VectorXd>::Failure(
        {core::ErrorCode::kNonFiniteValue,
          std::string(field_name) + " contains a non-finite value"});
    }
    result[static_cast<Eigen::Index>(output_index)] = value;
  }
  return core::Result<Eigen::VectorXd>::Success(std::move(result));
}

core::Status validateDifferentialDriveModel(const core::BaseModel base_model)
{
  if (base_model != core::BaseModel::kDifferentialDrive) {
    return {core::ErrorCode::kUnsupportedBaseModel, "differential-drive model required"};
  }
  return core::Status::Ok();
}

}  // namespace

Eigen::Vector3d toEigen(const core::Vector3 & vector)
{
  return {vector.x, vector.y, vector.z};
}

core::Vector3 fromEigen(const Eigen::Vector3d & vector)
{
  return {vector.x(), vector.y(), vector.z()};
}

Eigen::Quaterniond toEigen(const core::Quaternion & quaternion)
{
  return {quaternion.w, quaternion.x, quaternion.y, quaternion.z};
}

core::Quaternion fromEigen(const Eigen::Quaterniond & quaternion)
{
  return {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
}

core::Result<Eigen::Isometry3d> toEigenTransform(const core::Pose & pose)
{
  if (const auto status = core::validatePose(pose); !status.ok()) {
    return core::Result<Eigen::Isometry3d>::Failure(status);
  }
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = toEigen(pose.orientation).toRotationMatrix();
  transform.translation() = toEigen(pose.position);
  return core::Result<Eigen::Isometry3d>::Success(std::move(transform));
}

core::Result<core::Pose> fromEigenTransform(
  const Eigen::Isometry3d & transform,
  const core::Header & header,
  const double tolerance)
{
  if (const auto status = core::validateHeader(header); !status.ok()) {
    return core::Result<core::Pose>::Failure(status);
  }
  if (!std::isfinite(tolerance) || tolerance <= 0.0) {
    return core::Result<core::Pose>::Failure(
      {core::ErrorCode::kInvalidArgument, "transform tolerance must be positive and finite"});
  }
  if (!transform.matrix().allFinite()) {
    return core::Result<core::Pose>::Failure(
      {core::ErrorCode::kNonFiniteValue, "transform contains a non-finite value"});
  }

  const Eigen::Matrix3d rotation = transform.rotation();
  const Eigen::Matrix3d orthogonality_error =
    rotation.transpose() * rotation - Eigen::Matrix3d::Identity();
  if (orthogonality_error.norm() > tolerance ||
    std::abs(rotation.determinant() - 1.0) > tolerance)
  {
    return core::Result<core::Pose>::Failure(
      {core::ErrorCode::kInvalidQuaternion, "transform rotation is not in SO(3)"});
  }

  Eigen::Quaterniond quaternion(rotation);
  quaternion.normalize();
  core::Pose pose;
  pose.header = header;
  pose.position = fromEigen(transform.translation());
  pose.orientation = fromEigen(quaternion);
  if (const auto status = core::validatePose(pose, tolerance * 10.0); !status.ok()) {
    return core::Result<core::Pose>::Failure(status);
  }
  return core::Result<core::Pose>::Success(std::move(pose));
}

core::Result<Vector6d> toEigenWrench(const core::Wrench & wrench)
{
  if (const auto status = core::validateWrench(wrench); !status.ok()) {
    return core::Result<Vector6d>::Failure(status);
  }
  Vector6d result;
  result << wrench.force.x, wrench.force.y, wrench.force.z,
    wrench.torque.x, wrench.torque.y, wrench.torque.z;
  return core::Result<Vector6d>::Success(std::move(result));
}

core::Result<core::Wrench> fromEigenWrench(
  const Vector6d & wrench,
  const core::Header & header)
{
  if (const auto status = core::validateHeader(header); !status.ok()) {
    return core::Result<core::Wrench>::Failure(status);
  }
  if (!wrench.allFinite()) {
    return core::Result<core::Wrench>::Failure(
      {core::ErrorCode::kNonFiniteValue, "Eigen wrench contains a non-finite value"});
  }
  core::Wrench result;
  result.header = header;
  result.force = {wrench[0], wrench[1], wrench[2]};
  result.torque = {wrench[3], wrench[4], wrench[5]};
  return core::Result<core::Wrench>::Success(std::move(result));
}

core::Result<Eigen::MatrixXd> toEigenMatrix(const core::Matrix & matrix)
{
  if (const auto status = core::validateMatrix(matrix); !status.ok()) {
    return core::Result<Eigen::MatrixXd>::Failure(status);
  }
  const auto maximum_index = static_cast<std::size_t>(
    std::numeric_limits<Eigen::Index>::max());
  if (matrix.rows > maximum_index || matrix.cols > maximum_index) {
    return core::Result<Eigen::MatrixXd>::Failure(
      {core::ErrorCode::kInvalidDimension, "matrix dimensions exceed Eigen index range"});
  }
  const Eigen::Map<const RowMajorMatrixXd> mapped(
    matrix.row_major_data.data(),
    static_cast<Eigen::Index>(matrix.rows),
    static_cast<Eigen::Index>(matrix.cols));
  Eigen::MatrixXd result = mapped;
  return core::Result<Eigen::MatrixXd>::Success(std::move(result));
}

core::Result<core::Matrix> fromEigenMatrix(const Eigen::MatrixXd & matrix)
{
  if (matrix.rows() <= 0 || matrix.cols() <= 0) {
    return core::Result<core::Matrix>::Failure(
      {core::ErrorCode::kInvalidDimension, "Eigen matrix dimensions must be positive"});
  }
  if (!matrix.allFinite()) {
    return core::Result<core::Matrix>::Failure(
      {core::ErrorCode::kNonFiniteValue, "Eigen matrix contains a non-finite value"});
  }

  core::Matrix result;
  result.rows = static_cast<std::size_t>(matrix.rows());
  result.cols = static_cast<std::size_t>(matrix.cols());
  if (result.cols > std::numeric_limits<std::size_t>::max() / result.rows) {
    return core::Result<core::Matrix>::Failure(
      {core::ErrorCode::kInvalidDimension, "Eigen matrix size overflows core storage"});
  }
  result.row_major_data.resize(result.rows * result.cols);
  Eigen::Map<RowMajorMatrixXd> mapped(
    result.row_major_data.data(), matrix.rows(), matrix.cols());
  mapped = matrix;
  return core::Result<core::Matrix>::Success(std::move(result));
}

core::Result<Eigen::VectorXd> jointPositions(
  const core::JointState & joints,
  const std::vector<std::string> & expected_joint_order)
{
  if (const auto status = core::validateJointState(joints); !status.ok()) {
    return core::Result<Eigen::VectorXd>::Failure(status);
  }
  return reorderValues(
    joints.names, joints.positions_rad, expected_joint_order, "joint positions");
}

core::Result<Eigen::VectorXd> jointVelocities(
  const core::JointState & joints,
  const std::vector<std::string> & expected_joint_order)
{
  if (const auto status = core::validateJointState(joints); !status.ok()) {
    return core::Result<Eigen::VectorXd>::Failure(status);
  }
  if (joints.velocities_radps.empty()) {
    return core::Result<Eigen::VectorXd>::Failure(
      {core::ErrorCode::kInvalidDimension, "joint velocities are required for Eigen conversion"});
  }
  return reorderValues(
    joints.names, joints.velocities_radps, expected_joint_order, "joint velocities");
}

core::Result<Eigen::VectorXd> differentialDriveStateVector(
  const core::WholeBodyState & state,
  const std::vector<std::string> & expected_joint_order)
{
  if (const auto status = core::validateWholeBodyState(state); !status.ok()) {
    return core::Result<Eigen::VectorXd>::Failure(status);
  }
  if (const auto status = validateDifferentialDriveModel(state.base_model); !status.ok()) {
    return core::Result<Eigen::VectorXd>::Failure(status);
  }
  if (std::abs(state.base.lateral_velocity_mps) > 1.0e-9) {
    return core::Result<Eigen::VectorXd>::Failure(
      {core::ErrorCode::kSafetyLimitExceeded,
        "differential-drive lateral velocity must be zero"});
  }
  auto joints = jointPositions(state.joints, expected_joint_order);
  if (!joints.ok()) {
    return joints;
  }

  Eigen::VectorXd result(
    static_cast<Eigen::Index>(core::DifferentialDriveContract::stateDimension(
      expected_joint_order.size())));
  result[0] = state.base.x_m;
  result[1] = state.base.y_m;
  result[2] = state.base.yaw_rad;
  result.tail(joints.value().size()) = joints.value();
  return core::Result<Eigen::VectorXd>::Success(std::move(result));
}

core::Result<Eigen::VectorXd> differentialDriveInputVector(
  const core::WholeBodyInput & input,
  const std::vector<std::string> & expected_joint_order)
{
  if (const auto status = core::validateWholeBodyInput(input); !status.ok()) {
    return core::Result<Eigen::VectorXd>::Failure(status);
  }
  if (const auto status = validateDifferentialDriveModel(input.base_model); !status.ok()) {
    return core::Result<Eigen::VectorXd>::Failure(status);
  }
  auto joints = reorderValues(
    input.joint_names, input.joint_velocities_radps,
    expected_joint_order, "joint command");
  if (!joints.ok()) {
    return joints;
  }

  Eigen::VectorXd result(
    static_cast<Eigen::Index>(core::DifferentialDriveContract::inputDimension(
      expected_joint_order.size())));
  result[0] = input.base_command[0];
  result[1] = input.base_command[1];
  result.tail(joints.value().size()) = joints.value();
  return core::Result<Eigen::VectorXd>::Success(std::move(result));
}

core::Result<core::WholeBodyInput> differentialDriveInputFromVector(
  const Eigen::VectorXd & input,
  const core::Timestamp & stamp,
  const std::vector<std::string> & joint_order)
{
  if (const auto status = core::validateTimestamp(stamp); !status.ok()) {
    return core::Result<core::WholeBodyInput>::Failure(status);
  }
  if (const auto status = core::validateJointOrder(joint_order, joint_order); !status.ok()) {
    return core::Result<core::WholeBodyInput>::Failure(status);
  }
  const auto expected_size = static_cast<Eigen::Index>(
    core::DifferentialDriveContract::inputDimension(joint_order.size()));
  if (input.size() != expected_size) {
    std::ostringstream stream;
    stream << "differential-drive input has dimension " << input.size()
           << ", expected " << expected_size;
    return core::Result<core::WholeBodyInput>::Failure(
      {core::ErrorCode::kInvalidDimension, stream.str()});
  }
  if (!input.allFinite()) {
    return core::Result<core::WholeBodyInput>::Failure(
      {core::ErrorCode::kNonFiniteValue,
        "differential-drive input contains a non-finite value"});
  }

  core::WholeBodyInput result;
  result.stamp = stamp;
  result.base_model = core::BaseModel::kDifferentialDrive;
  result.base_command = {input[0], input[1]};
  result.joint_names = joint_order;
  result.joint_velocities_radps.resize(joint_order.size());
  for (std::size_t index = 0; index < joint_order.size(); ++index) {
    result.joint_velocities_radps[index] = input[static_cast<Eigen::Index>(index + 2)];
  }
  return core::Result<core::WholeBodyInput>::Success(std::move(result));
}

}  // namespace wbmm::math
