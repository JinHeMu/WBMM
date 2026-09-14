#include "wbmm_ros_interfaces/wbmm_conversions.hpp"

#include <stdexcept>

namespace wbmm::ros_interfaces
{

std::optional<wbmm::core::WholeBodyState> toCoreState(
  const Eigen::Ref<const Eigen::VectorXd> & state,
  const std::vector<std::string> & joint_names,
  const wbmm::core::Header & header)
{
  const auto expected_size = static_cast<Eigen::Index>(3 + joint_names.size());
  if (state.size() != expected_size) {
    return std::nullopt;
  }

  wbmm::core::WholeBodyState result;
  result.header = header;
  result.base_model = wbmm::core::BaseModel::kDifferentialDrive;
  result.base.x = state[0];
  result.base.y = state[1];
  result.base.yaw = state[2];
  result.joints.names = joint_names;
  result.joints.positions.resize(joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    result.joints.positions[i] = state[3 + static_cast<Eigen::Index>(i)];
  }
  return result;
}

Eigen::VectorXd toEigenState(const wbmm::core::WholeBodyState & state)
{
  if (state.joints.names.size() != state.joints.positions.size()) {
    throw std::invalid_argument(
            "WholeBodyState joint names and positions must have the same size");
  }
  Eigen::VectorXd result(
    static_cast<Eigen::Index>(3 + state.joints.positions.size()));
  result[0] = state.base.x;
  result[1] = state.base.y;
  result[2] = state.base.yaw;
  for (std::size_t i = 0; i < state.joints.positions.size(); ++i) {
    result[3 + static_cast<Eigen::Index>(i)] = state.joints.positions[i];
  }
  return result;
}

wbmm::core::Wrench toCoreWrench(
  const Eigen::Matrix<double, 6, 1> & wrench,
  const wbmm::core::Header & header)
{
  wbmm::core::Wrench result;
  result.header = header;
  result.force.x = wrench[0];
  result.force.y = wrench[1];
  result.force.z = wrench[2];
  result.torque.x = wrench[3];
  result.torque.y = wrench[4];
  result.torque.z = wrench[5];
  return result;
}

Eigen::Matrix<double, 6, 1> toEigenWrench(const wbmm::core::Wrench & wrench)
{
  Eigen::Matrix<double, 6, 1> result;
  result << wrench.force.x, wrench.force.y, wrench.force.z,
    wrench.torque.x, wrench.torque.y, wrench.torque.z;
  return result;
}

std::optional<wbmm::core::WholeBodyInput> toCoreInput(
  const Eigen::Ref<const Eigen::VectorXd> & input,
  const std::vector<std::string> & joint_names,
  double stamp,
  wbmm::core::ClockDomain clock)
{
  if (input.size() != static_cast<Eigen::Index>(2 + joint_names.size())) {
    return std::nullopt;
  }

  wbmm::core::WholeBodyInput result;
  result.stamp = stamp;
  result.clock = clock;
  result.base_model = wbmm::core::BaseModel::kDifferentialDrive;
  result.base_command = {input[0], input[1]};
  result.joint_names = joint_names;
  result.joint_velocities.resize(joint_names.size());
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    result.joint_velocities[i] = input[2 + static_cast<Eigen::Index>(i)];
  }
  return result;
}

Eigen::VectorXd toEigenInput(const wbmm::core::WholeBodyInput & input)
{
  if (input.joint_names.size() != input.joint_velocities.size()) {
    throw std::invalid_argument(
            "WholeBodyInput joint names and velocities must have the same size");
  }
  Eigen::VectorXd result(static_cast<Eigen::Index>(
      input.base_command.size() + input.joint_velocities.size()));
  for (std::size_t i = 0; i < input.base_command.size(); ++i) {
    result[static_cast<Eigen::Index>(i)] = input.base_command[i];
  }
  for (std::size_t i = 0; i < input.joint_velocities.size(); ++i) {
    result[static_cast<Eigen::Index>(input.base_command.size() + i)] =
      input.joint_velocities[i];
  }
  return result;
}

wbmm::core::WholeBodyInput makeZeroWholeBodyInput(
  const std::vector<std::string> & joint_names,
  double stamp,
  wbmm::core::ClockDomain clock)
{
  wbmm::core::WholeBodyInput result;
  result.stamp = stamp;
  result.clock = clock;
  result.base_model = wbmm::core::BaseModel::kDifferentialDrive;
  result.base_command = {0.0, 0.0};
  result.joint_names = joint_names;
  result.joint_velocities.assign(joint_names.size(), 0.0);
  return result;
}

}  // namespace wbmm::ros_interfaces
