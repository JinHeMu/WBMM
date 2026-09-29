#include "wbmm_pinocchio/pinocchio_robot_model.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace wbmm::pinocchio
{

PinocchioRobotModel::PinocchioRobotModel(
  KinematicModelPtr model, double max_base_speed, double max_base_yaw_rate)
: model_(std::move(model)), frames_(model_), kinematics_data_(model_)
{
  if (model_ == nullptr) {
    throw std::invalid_argument("PinocchioRobotModel requires a KinematicModel.");
  }
  if (!std::isfinite(max_base_speed) || !std::isfinite(max_base_yaw_rate)) {
    throw std::invalid_argument("base speed limits must be finite");
  }

  for (const auto & joint : model_->controlledJoints()) {
    limits_.joint_min.push_back(joint.has_position_limit
      ? joint.lower : -std::numeric_limits<double>::infinity());
    limits_.joint_max.push_back(joint.has_position_limit
      ? joint.upper : std::numeric_limits<double>::infinity());
    limits_.max_joint_speed.push_back(
      joint.has_velocity_limit ? joint.velocity : 0.0);
  }
  limits_.max_base_speed = std::max(0.0, max_base_speed);
  limits_.max_base_yaw_rate = std::max(0.0, max_base_yaw_rate);
}

std::size_t PinocchioRobotModel::stateDimension() const
{
  return model_->stateDimension();
}

std::size_t PinocchioRobotModel::inputDimension() const
{
  return model_->inputDimension();
}

wbmm::core::BaseModel PinocchioRobotModel::baseModel() const
{
  return wbmm::core::BaseModel::kDifferentialDrive;
}

const std::vector<std::string> & PinocchioRobotModel::jointNames() const
{
  return model_->controlledJointNames();
}

const wbmm::core::RobotLimits & PinocchioRobotModel::limits() const
{
  return limits_;
}

bool PinocchioRobotModel::hasFrame(const std::string & frame_name) const
{
  return model_->hasFrame(frame_name);
}

bool PinocchioRobotModel::forwardKinematics(
  const wbmm::core::WholeBodyState & state,
  const std::string & link_name,
  wbmm::core::Pose & pose) const
{
  if (state.base_model != wbmm::core::BaseModel::kDifferentialDrive) {
    return false;
  }
  if (!kinematics_data_.update(state)) {
    return false;
  }
  return frames_.framePose(kinematics_data_, link_name, pose);
}

bool PinocchioRobotModel::frameJacobian(
  const wbmm::core::WholeBodyState & state,
  const std::string & link_name,
  Eigen::Ref<Eigen::MatrixXd> jacobian) const
{
  if (state.base_model != wbmm::core::BaseModel::kDifferentialDrive) {
    return false;
  }
  if (!kinematics_data_.update(
      state, UpdateMode::kJacobians))
  {
    return false;
  }
  return frames_.frameJacobian(kinematics_data_, link_name, jacobian);
}

bool PinocchioRobotModel::validate(
  const wbmm::core::WholeBodyState & state, std::string * message) const
{
  return model_->validateState(state, message);
}

bool PinocchioRobotModel::validate(
  const wbmm::core::WholeBodyInput & input, std::string * message) const
{
  return model_->validateInput(input, message);
}

}  // namespace wbmm::pinocchio
