#include "whole_body_force_control/pinocchio_robot_model.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace whole_body_force_control
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

PinocchioRobotModel::PinocchioRobotModel(
  const std::string & urdf_file, double max_base_speed,
  double max_base_yaw_rate)
{
  try {
    pinocchio::urdf::buildModel(urdf_file, model_);
  } catch (const std::exception & exception) {
    throw std::runtime_error(
            "Failed to build Pinocchio model from '" + urdf_file + "': " +
            exception.what());
  }

  // 第一版合同只支持差速底盘 + 六关节；URDF 中其余固定关节不进入 q。
  const auto joint_count = static_cast<std::size_t>(model_.njoints);
  for (std::size_t joint_id = 1; joint_id < joint_count; ++joint_id) {
    const auto & joint = model_.joints[joint_id];
    if (joint.nq() == 1 && joint.nv() == 1) {
      joint_names_.push_back(model_.names[joint_id]);
      joint_position_indices_.push_back(
        static_cast<Eigen::Index>(joint.idx_q()));
      joint_velocity_indices_.push_back(
        static_cast<Eigen::Index>(joint.idx_v()));
      limits_.joint_min.push_back(model_.lowerPositionLimit[joint.idx_q()]);
      limits_.joint_max.push_back(model_.upperPositionLimit[joint.idx_q()]);
      limits_.max_joint_speed.push_back(model_.velocityLimit[joint.idx_v()]);
    }
  }
  if (joint_names_.size() != wbmm::core::kSupportedJointCount) {
    throw std::runtime_error(
            "PinocchioRobotModel expects exactly six 1-DoF joints, got " +
            std::to_string(joint_names_.size()));
  }
  if (!std::isfinite(max_base_speed) || !std::isfinite(max_base_yaw_rate)) {
    throw std::invalid_argument("base speed limits must be finite");
  }
  limits_.max_base_speed = std::max(0.0, max_base_speed);
  limits_.max_base_yaw_rate = std::max(0.0, max_base_yaw_rate);
}

std::size_t PinocchioRobotModel::stateDimension() const
{
  return 3 + static_cast<std::size_t>(model_.nq);
}

std::size_t PinocchioRobotModel::inputDimension() const
{
  return 2 + static_cast<std::size_t>(model_.nv);
}

wbmm::core::BaseModel PinocchioRobotModel::baseModel() const
{
  return wbmm::core::BaseModel::kDifferentialDrive;
}

const std::vector<std::string> & PinocchioRobotModel::jointNames() const
{
  return joint_names_;
}

const wbmm::core::RobotLimits & PinocchioRobotModel::limits() const
{
  return limits_;
}

bool PinocchioRobotModel::hasFrame(const std::string & frame_name) const
{
  return !frame_name.empty() && model_.existFrame(frame_name);
}

bool PinocchioRobotModel::stateToConfiguration(
  const wbmm::core::WholeBodyState & state,
  Eigen::VectorXd & configuration,
  std::string * message) const
{
  if (state.joints.names.size() != joint_names_.size() ||
    state.joints.positions.size() != joint_names_.size())
  {
    if (message != nullptr) {
      *message = "state must contain exactly six named joint positions";
    }
    return false;
  }

  configuration = Eigen::VectorXd::Zero(model_.nq);
  for (std::size_t joint = 0; joint < joint_names_.size(); ++joint) {
    const auto iterator = std::find(
      state.joints.names.begin(), state.joints.names.end(), joint_names_[joint]);
    if (iterator == state.joints.names.end()) {
      if (message != nullptr) {
        *message = "state is missing joint '" + joint_names_[joint] + "'";
      }
      return false;
    }
    const auto state_index = static_cast<std::size_t>(
      std::distance(state.joints.names.begin(), iterator));
    configuration[joint_position_indices_[joint]] =
      state.joints.positions[state_index];
  }
  if (!configuration.allFinite()) {
    if (message != nullptr) {
      *message = "state joint positions must be finite";
    }
    return false;
  }
  return true;
}

bool PinocchioRobotModel::forwardKinematics(
  const wbmm::core::WholeBodyState & state,
  const std::string & link_name,
  wbmm::core::Pose & pose) const
{
  if (state.base_model != wbmm::core::BaseModel::kDifferentialDrive) {
    return false;
  }
  const auto frame_id = model_.getFrameId(link_name);
  if (frame_id >= model_.frames.size()) {
    return false;
  }
  Eigen::VectorXd configuration;
  if (!stateToConfiguration(state, configuration, nullptr)) {
    return false;
  }

  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, configuration);
  pinocchio::updateFramePlacements(model_, data);
  const auto & placement = data.oMf[frame_id];

  const Eigen::Matrix3d base_rotation = rotationZ(state.base.yaw);
  const Eigen::Vector3d local_position = placement.translation();
  const Eigen::Vector3d world_position = base_rotation * local_position;

  pose.header = state.header;
  pose.position.x = world_position.x() + state.base.x;
  pose.position.y = world_position.y() + state.base.y;
  pose.position.z = world_position.z();
  pose.orientation = quaternionFromRotation(
    base_rotation * placement.rotation());
  return true;
}

bool PinocchioRobotModel::frameJacobian(
  const wbmm::core::WholeBodyState & state,
  const std::string & link_name,
  Eigen::Ref<Eigen::MatrixXd> jacobian) const
{
  if (jacobian.rows() != 6 ||
    jacobian.cols() != static_cast<Eigen::Index>(inputDimension()))
  {
    return false;
  }
  if (state.base_model != wbmm::core::BaseModel::kDifferentialDrive) {
    return false;
  }
  const auto frame_id = model_.getFrameId(link_name);
  if (frame_id >= model_.frames.size()) {
    return false;
  }
  Eigen::VectorXd configuration;
  if (!stateToConfiguration(state, configuration, nullptr)) {
    return false;
  }

  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, configuration);
  pinocchio::updateFramePlacements(model_, data);
  Eigen::MatrixXd local_jacobian = Eigen::MatrixXd::Zero(6, model_.nv);
  pinocchio::computeFrameJacobian(
    model_, data, configuration, frame_id,
    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, local_jacobian);

  // computeFrameJacobian 的关节列是 Pinocchio/URDF 顺序；合同要求列顺序
  // 与输入向量的关节顺序一致，因此按 state.joints.names 重排一次。
  Eigen::MatrixXd state_ordered_jacobian = Eigen::MatrixXd::Zero(6, model_.nv);
  for (std::size_t state_joint = 0;
    state_joint < state.joints.names.size(); ++state_joint)
  {
    const auto iterator = std::find(
      joint_names_.begin(), joint_names_.end(), state.joints.names[state_joint]);
    if (iterator == joint_names_.end()) {
      return false;
    }
    const auto model_joint = static_cast<Eigen::Index>(
      std::distance(joint_names_.begin(), iterator));
    state_ordered_jacobian.col(static_cast<Eigen::Index>(state_joint)) =
      local_jacobian.col(model_joint);
  }

  const Eigen::Matrix3d base_rotation = rotationZ(state.base.yaw);
  const Eigen::Vector3d ee_world = base_rotation * data.oMf[frame_id].translation();
  const Eigen::Vector3d z_axis = Eigen::Vector3d::UnitZ();
  const Eigen::Index base_columns = 2;

  jacobian.setZero();
  // 底盘线速度 v：沿底盘自身航向。
  jacobian.block<3, 1>(0, 0) = base_rotation * Eigen::Vector3d::UnitX();
  // 底盘角速度 omega：平移项为 z x (R * p_model)，角速度项为世界 z。
  jacobian.block<3, 1>(0, 1) = z_axis.cross(ee_world);
  jacobian.block<3, 1>(3, 1) = z_axis;
  // 关节速度：机械臂基座系 Jacobian 旋转到世界系。
  jacobian.block(0, base_columns, 3, model_.nv) =
    base_rotation * state_ordered_jacobian.topRows<3>();
  jacobian.block(3, base_columns, 3, model_.nv) =
    base_rotation * state_ordered_jacobian.bottomRows<3>();
  return jacobian.allFinite();
}

bool PinocchioRobotModel::validate(
  const wbmm::core::WholeBodyState & state, std::string * message) const
{
  const auto fail = [message](const std::string & text) {
      if (message != nullptr) {
        *message = text;
      }
      return false;
    };

  if (state.base_model != wbmm::core::BaseModel::kDifferentialDrive) {
    return fail("WholeBodyState.base_model must be kDifferentialDrive");
  }
  std::string joint_error;
  if (!sameJointSet(state.joints.names, &joint_error)) {
    return fail(joint_error);
  }
  if (state.joints.positions.size() != joint_names_.size()) {
    return fail("WholeBodyState must contain six joint positions");
  }

  constexpr double kTolerance = 1.0e-6;
  for (std::size_t joint = 0; joint < joint_names_.size(); ++joint) {
    const auto iterator = std::find(
      state.joints.names.begin(), state.joints.names.end(), joint_names_[joint]);
    const auto state_index = static_cast<std::size_t>(
      std::distance(state.joints.names.begin(), iterator));
    const double position = state.joints.positions[state_index];
    if (!std::isfinite(position)) {
      return fail("JointState.positions must be finite");
    }
    if (position < limits_.joint_min[joint] - kTolerance ||
      position > limits_.joint_max[joint] + kTolerance)
    {
      return fail(
              "Joint '" + joint_names_[joint] + "' position is outside its limits");
    }
  }

  if (!state.joints.velocities.empty()) {
    if (state.joints.velocities.size() != joint_names_.size()) {
      return fail("JointState.velocities must be empty or contain six values");
    }
    for (std::size_t joint = 0; joint < joint_names_.size(); ++joint) {
      const auto iterator = std::find(
        state.joints.names.begin(), state.joints.names.end(), joint_names_[joint]);
      const auto state_index = static_cast<std::size_t>(
        std::distance(state.joints.names.begin(), iterator));
      const double velocity = state.joints.velocities[state_index];
      if (!std::isfinite(velocity)) {
        return fail("JointState.velocities must be finite");
      }
      if (limits_.max_joint_speed[joint] > 0.0 &&
        std::abs(velocity) > limits_.max_joint_speed[joint] + kTolerance)
      {
        return fail(
                "Joint '" + joint_names_[joint] + "' velocity exceeds its limit");
      }
    }
  }
  return true;
}

bool PinocchioRobotModel::validate(
  const wbmm::core::WholeBodyInput & input, std::string * message) const
{
  const auto fail = [message](const std::string & text) {
      if (message != nullptr) {
        *message = text;
      }
      return false;
    };

  if (input.base_model != wbmm::core::BaseModel::kDifferentialDrive) {
    return fail("WholeBodyInput.base_model must be kDifferentialDrive");
  }
  if (input.base_command.size() != 2) {
    return fail("WholeBodyInput.base_command must be [v, omega]");
  }
  if (!std::isfinite(input.base_command[0]) ||
    !std::isfinite(input.base_command[1]))
  {
    return fail("WholeBodyInput.base_command must be finite");
  }
  if (limits_.max_base_speed > 0.0 &&
    std::abs(input.base_command[0]) > limits_.max_base_speed)
  {
    return fail("WholeBodyInput linear velocity exceeds max_base_speed");
  }
  if (limits_.max_base_yaw_rate > 0.0 &&
    std::abs(input.base_command[1]) > limits_.max_base_yaw_rate)
  {
    return fail("WholeBodyInput angular velocity exceeds max_base_yaw_rate");
  }

  std::string joint_error;
  if (!sameJointSet(input.joint_names, &joint_error)) {
    return fail(joint_error);
  }
  if (input.joint_velocities.size() != joint_names_.size()) {
    return fail("WholeBodyInput must contain six joint velocities");
  }
  for (std::size_t joint = 0; joint < joint_names_.size(); ++joint) {
    const auto iterator = std::find(
      input.joint_names.begin(), input.joint_names.end(), joint_names_[joint]);
    const auto input_index = static_cast<std::size_t>(
      std::distance(input.joint_names.begin(), iterator));
    const double velocity = input.joint_velocities[input_index];
    if (!std::isfinite(velocity)) {
      return fail("WholeBodyInput.joint_velocities must be finite");
    }
    if (limits_.max_joint_speed[joint] > 0.0 &&
      std::abs(velocity) > limits_.max_joint_speed[joint] + 1.0e-6)
    {
      return fail(
              "Joint '" + joint_names_[joint] + "' velocity exceeds its limit");
    }
  }
  return true;
}

bool PinocchioRobotModel::sameJointSet(
  const std::vector<std::string> & names, std::string * message) const
{
  if (names.size() != joint_names_.size()) {
    if (message != nullptr) {
      *message = "state/input must contain exactly six joint names";
    }
    return false;
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    if (names[i].empty()) {
      if (message != nullptr) {
        *message = "joint names must not be empty";
      }
      return false;
    }
    if (std::find(joint_names_.begin(), joint_names_.end(), names[i]) ==
      joint_names_.end())
    {
      if (message != nullptr) {
        *message = "unknown joint name '" + names[i] + "'";
      }
      return false;
    }
    for (std::size_t j = 0; j < i; ++j) {
      if (names[i] == names[j]) {
        if (message != nullptr) {
          *message = "joint names must not contain duplicates";
        }
        return false;
      }
    }
  }
  return true;
}

}  // namespace whole_body_force_control
