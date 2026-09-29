#include "wbmm_pinocchio/kinematics_data.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace wbmm::pinocchio
{
namespace
{

Eigen::Matrix3d rotationZ(double yaw)
{
  Eigen::Matrix3d rotation;
  rotation << std::cos(yaw), -std::sin(yaw), 0.0,
    std::sin(yaw), std::cos(yaw), 0.0,
    0.0, 0.0, 1.0;
  return rotation;
}

const ::pinocchio::Model & requireModel(const KinematicModelPtr & model)
{
  if (model == nullptr) {
    throw std::invalid_argument("KinematicsData requires a KinematicModel.");
  }
  return model->model();
}

}  // namespace

KinematicsData::KinematicsData(KinematicModelPtr model)
: model_(std::move(model)), data_(requireModel(model_))
{
  configuration_ = Eigen::VectorXd::Zero(model_->model().nq);
}

bool KinematicsData::update(
  const wbmm::core::WholeBodyState & state, UpdateMode mode)
{
  updated_ = false;
  jacobians_ready_ = false;
  last_message_.clear();

  // 显示等消费者可能还没有标注 base_model；把它当作平面差速基座处理。
  // core::RobotModel 的限位/合同校验仍会要求显式 kDifferentialDrive。
  if (state.base_model != wbmm::core::BaseModel::kDifferentialDrive &&
    state.base_model != wbmm::core::BaseModel::kUnspecified)
  {
    last_message_ = "Only planar (differential-drive) base states are supported.";
    return false;
  }
  if (!std::isfinite(state.base.x) || !std::isfinite(state.base.y) ||
    !std::isfinite(state.base.yaw))
  {
    last_message_ = "Base pose must be finite.";
    return false;
  }

  if (!model_->configurationFromState(state, configuration_, &last_message_)) {
    return false;
  }

  // state_base_frame 在 state 系的位姿是 (x, y, yaw)，root 在 state_base_frame
  // 里的位姿是 root_from_state_base，因此
  //   state <- root = T(x, y, yaw) * (root <- state_base)^{-1}
  Eigen::Isometry3d world_from_state_base = Eigen::Isometry3d::Identity();
  world_from_state_base.linear() = rotationZ(state.base.yaw);
  world_from_state_base.translation() = Eigen::Vector3d(state.base.x, state.base.y, 0.0);
  world_from_root_ =
    world_from_state_base * model_->rootFromStateBase().inverse();

  state_ = state;

  const auto & model = model_->model();

  try {
    ::pinocchio::forwardKinematics(model, data_, configuration_);
    ::pinocchio::updateFramePlacements(model, data_);
    if (mode == UpdateMode::kJacobians) {
      // 帧 Jacobian 的提取（getFrameJacobian）复用这一份 data_.J，避免每个
      // 球/末端各算一次整树。
      ::pinocchio::computeJointJacobians(model, data_, configuration_);
      ::pinocchio::updateFramePlacements(model, data_);
      jacobians_ready_ = true;
    }
  } catch (const std::exception & exception) {
    last_message_ = std::string("Pinocchio kinematics failed: ") + exception.what();
    return false;
  }

  updated_ = true;
  return true;
}

Eigen::Isometry3d KinematicsData::toStateFrame(
  const Eigen::Isometry3d & root_pose) const
{
  return world_from_root_ * root_pose;
}

}  // namespace wbmm::pinocchio
