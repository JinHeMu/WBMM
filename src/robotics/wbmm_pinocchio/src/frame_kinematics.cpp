#include "wbmm_pinocchio/frame_kinematics.hpp"

#include <pinocchio/algorithm/frames.hpp>

#include <cmath>
#include <stdexcept>
#include <string>

namespace wbmm::pinocchio
{
namespace
{

bool sameModel(
  const KinematicModelPtr & model, const KinematicsData & kinematics_data,
  std::string * message)
{
  if (kinematics_data.model() != model) {
    if (message != nullptr) {
      *message = "KinematicsData belongs to a different KinematicModel.";
    }
    return false;
  }
  if (!kinematics_data.updated()) {
    if (message != nullptr) {
      *message = "KinematicsData was not updated successfully.";
    }
    return false;
  }
  return true;
}

wbmm::core::Quaternion quaternionFromRotation(const Eigen::Matrix3d & rotation)
{
  Eigen::Quaterniond quaternion(rotation);
  if (quaternion.norm() < 1.0e-12) {
    return {1.0, 0.0, 0.0, 0.0};
  }
  quaternion.normalize();
  // 合同内部四元数为 wxyz，并保持 w >= 0 的规范表示。
  if (quaternion.w() < 0.0) {
    quaternion.coeffs() *= -1.0;
  }
  return {quaternion.w(), quaternion.x(), quaternion.y(), quaternion.z()};
}

}  // namespace

FrameKinematics::FrameKinematics(KinematicModelPtr model)
: model_(std::move(model))
{
  if (model_ == nullptr) {
    throw std::invalid_argument("FrameKinematics requires a KinematicModel.");
  }
  for (const auto & link : model_->description().links) {
    link_names_.push_back(link.name);
  }
}

bool FrameKinematics::hasFrame(const std::string & frame_name) const
{
  return model_->hasFrame(frame_name);
}

bool FrameKinematics::framePlacement(
  const KinematicsData & kinematics_data, const std::string & frame_name,
  Eigen::Isometry3d & placement, std::string * message) const
{
  if (!sameModel(model_, kinematics_data, message)) {
    return false;
  }
  const auto frame_id = model_->frameId(frame_name);
  if (frame_id >= static_cast<::pinocchio::FrameIndex>(model_->model().nframes)) {
    if (message != nullptr) {
      *message = "Unknown frame '" + frame_name + "'.";
    }
    return false;
  }
  const auto & data = kinematics_data.data();
  Eigen::Isometry3d root_pose = Eigen::Isometry3d::Identity();
  root_pose.translation() = data.oMf[frame_id].translation();
  root_pose.linear() = data.oMf[frame_id].rotation();
  placement = kinematics_data.toStateFrame(root_pose);
  return placement.matrix().allFinite();
}

std::map<std::string, Eigen::Isometry3d> FrameKinematics::linkPlacements(
  const KinematicsData & kinematics_data) const
{
  std::map<std::string, Eigen::Isometry3d> placements;
  if (!sameModel(model_, kinematics_data, nullptr)) {
    return placements;
  }
  const auto & data = kinematics_data.data();
  for (const auto & name : link_names_) {
    const auto frame_id = model_->frameId(name);
    if (frame_id >= static_cast<::pinocchio::FrameIndex>(model_->model().nframes)) {
      continue;
    }
    Eigen::Isometry3d root_pose = Eigen::Isometry3d::Identity();
    root_pose.translation() = data.oMf[frame_id].translation();
    root_pose.linear() = data.oMf[frame_id].rotation();
    placements.emplace(name, kinematics_data.toStateFrame(root_pose));
  }
  return placements;
}

bool FrameKinematics::frameJacobian(
  KinematicsData & kinematics_data, const std::string & frame_name,
  Eigen::Ref<Eigen::MatrixXd> jacobian, std::string * message) const
{
  if (!sameModel(model_, kinematics_data, message)) {
    return false;
  }
  if (!kinematics_data.jacobiansReady()) {
    if (message != nullptr) {
      *message = "KinematicsData was not updated with kJacobians.";
    }
    return false;
  }
  const auto & model = model_->model();
  if (jacobian.rows() != 6 ||
    jacobian.cols() != static_cast<Eigen::Index>(model_->inputDimension()))
  {
    if (message != nullptr) {
      *message = "Jacobian must be 6 x inputDimension.";
    }
    return false;
  }
  const auto frame_id = model_->frameId(frame_name);
  if (frame_id >= static_cast<::pinocchio::FrameIndex>(model.nframes)) {
    if (message != nullptr) {
      *message = "Unknown frame '" + frame_name + "'.";
    }
    return false;
  }

  Eigen::MatrixXd local = Eigen::MatrixXd::Zero(6, model.nv);
  ::pinocchio::getFrameJacobian(
    model, kinematics_data.data(), frame_id,
    ::pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, local);

  const Eigen::Matrix3d rotation = kinematics_data.stateFromRootRotation();
  jacobian.setZero();
  // 底盘线速度 v：沿底盘自身航向。
  jacobian.block<3, 1>(0, 0) = rotation * Eigen::Vector3d::UnitX();
  // 底盘角速度 omega：平移项为 z x (R * p_root)，角速度项为 state 系 z。
  const Eigen::Vector3d radius = rotation * kinematics_data.data().oMf[frame_id].translation();
  jacobian.block<3, 1>(0, 1) = Eigen::Vector3d::UnitZ().cross(radius);
  jacobian.block<3, 1>(3, 1) = Eigen::Vector3d::UnitZ();
  // 关节速度：root 系 Jacobian 旋转到 state 系。列顺序必须与调用方将要乘的
  // 输入向量 u 的关节顺序一致，即 state.joints.names 的顺序（core 合同），
  // 而不是模型内部的固定顺序。
  const auto & state = kinematics_data.state();
  if (state.joints.names.size() != model_->controlledJoints().size()) {
    if (message != nullptr) {
      *message = "State joint names do not match the controlled joints.";
    }
    return false;
  }
  for (std::size_t i = 0U; i < state.joints.names.size(); ++i) {
    const auto controlled = model_->controlledJointIndex(state.joints.names[i]);
    if (controlled == static_cast<std::size_t>(-1)) {
      if (message != nullptr) {
        *message = "Unknown joint name '" + state.joints.names[i] + "'.";
      }
      return false;
    }
    const auto & joint = model_->controlledJoints()[controlled];
    const Eigen::Index column = static_cast<Eigen::Index>(2U + i);
    jacobian.block<3, 1>(0, column) =
      rotation * local.block<3, 1>(0, joint.idx_v);
    jacobian.block<3, 1>(3, column) =
      rotation * local.block<3, 1>(3, joint.idx_v);
  }
  return jacobian.allFinite();
}

bool FrameKinematics::framePose(
  const KinematicsData & kinematics_data, const std::string & frame_name,
  wbmm::core::Pose & pose, std::string * message) const
{
  Eigen::Isometry3d placement;
  if (!framePlacement(kinematics_data, frame_name, placement, message)) {
    return false;
  }
  pose.header = kinematics_data.state().header;
  pose.position.x = placement.translation().x();
  pose.position.y = placement.translation().y();
  pose.position.z = placement.translation().z();
  pose.orientation = quaternionFromRotation(placement.linear());
  return true;
}

}  // namespace wbmm::pinocchio
