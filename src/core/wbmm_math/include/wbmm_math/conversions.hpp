#pragma once

#include "wbmm_core/types.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>
#include <vector>

namespace wbmm::math
{

using Vector6d = Eigen::Matrix<double, 6, 1>;

Eigen::Vector3d toEigen(const core::Vector3 & vector);
core::Vector3 fromEigen(const Eigen::Vector3d & vector);

Eigen::Quaterniond toEigen(const core::Quaternion & quaternion);
core::Quaternion fromEigen(const Eigen::Quaterniond & quaternion);

core::Result<Eigen::Isometry3d> toEigenTransform(const core::Pose & pose);
core::Result<core::Pose> fromEigenTransform(
  const Eigen::Isometry3d & transform,
  const core::Header & header,
  double tolerance = 1.0e-9);

core::Result<Vector6d> toEigenWrench(const core::Wrench & wrench);
core::Result<core::Wrench> fromEigenWrench(
  const Vector6d & wrench,
  const core::Header & header);

core::Result<Eigen::MatrixXd> toEigenMatrix(const core::Matrix & matrix);
core::Result<core::Matrix> fromEigenMatrix(const Eigen::MatrixXd & matrix);

core::Result<Eigen::VectorXd> jointPositions(
  const core::JointState & joints,
  const std::vector<std::string> & expected_joint_order);

core::Result<Eigen::VectorXd> jointVelocities(
  const core::JointState & joints,
  const std::vector<std::string> & expected_joint_order);

// Adapter-facing conversion. It maps by name and never guesses array order.
core::Result<Eigen::VectorXd> differentialDriveStateVector(
  const core::WholeBodyState & state,
  const std::vector<std::string> & expected_joint_order);

core::Result<Eigen::VectorXd> differentialDriveInputVector(
  const core::WholeBodyInput & input,
  const std::vector<std::string> & expected_joint_order);

core::Result<core::WholeBodyInput> differentialDriveInputFromVector(
  const Eigen::VectorXd & input,
  const core::Timestamp & stamp,
  const std::vector<std::string> & joint_order);

}  // namespace wbmm::math
