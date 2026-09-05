#pragma once

#include "wbmm_core/types.hpp"

#include <cstddef>
#include <string>
#include <vector>

namespace wbmm::core
{

Status validateTimestamp(const Timestamp & stamp);
Status validateHeader(const Header & header);
Status validatePose(const Pose & pose, double quaternion_tolerance = 1.0e-6);
Status validateWrench(const Wrench & wrench);
Status validateJointState(const JointState & joints);
Status validateJointOrder(
  const std::vector<std::string> & actual,
  const std::vector<std::string> & expected);
Status validateWholeBodyState(const WholeBodyState & state);
Status validateWholeBodyInput(const WholeBodyInput & input);
Status validateTaskTrajectory(const TaskTrajectory & trajectory);
Status validateWholeBodyTrajectory(const WholeBodyTrajectory & trajectory);
Status validateEnvironmentSnapshot(const EnvironmentSnapshot & snapshot);
Status validateMatrix(const Matrix & matrix);

struct DifferentialDriveContract final
{
  static constexpr std::size_t kBaseStateDimension = 3;
  static constexpr std::size_t kBaseInputDimension = 2;

  static constexpr std::size_t stateDimension(std::size_t joint_count) noexcept
  {
    return kBaseStateDimension + joint_count;
  }

  static constexpr std::size_t inputDimension(std::size_t joint_count) noexcept
  {
    return kBaseInputDimension + joint_count;
  }

  static Status validateState(
    const WholeBodyState & state,
    const std::vector<std::string> & expected_joint_order);

  static Status validateInput(
    const WholeBodyInput & input,
    const std::vector<std::string> & expected_joint_order);
};

}  // namespace wbmm::core
