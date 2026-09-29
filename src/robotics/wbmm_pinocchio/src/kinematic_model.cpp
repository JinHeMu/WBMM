#include "wbmm_pinocchio/kinematic_model.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace wbmm::pinocchio
{
namespace
{

constexpr double kLimitTolerance = 1.0e-6;

// 标量关节的配置分量：revolute/prismatic 为 1，continuous 为 2(cos, sin)。
void writeJointConfiguration(
  const ::pinocchio::Model & model, ::pinocchio::JointIndex joint_id, double value,
  Eigen::VectorXd & configuration)
{
  const Eigen::Index idx_q = model.idx_qs[joint_id];
  if (model.joints[joint_id].nq() == 1) {
    configuration[idx_q] = value;
  } else {
    configuration[idx_q] = std::cos(value);
    configuration[idx_q + 1] = std::sin(value);
  }
}

}  // namespace

KinematicModel::KinematicModel(
  std::shared_ptr<const wbmm::robot_model::RobotDescription> description,
  wbmm::robot_model::RobotModelConfig config)
: description_(std::move(description))
{
  const auto & robot = *description_;

  root_frame_ = robot.root_link;
  state_base_frame_ = config.state_base_frame.empty()
    ? robot.root_link : config.state_base_frame;

  if (!wbmm::robot_model::fixedTransform(
      robot, root_frame_, state_base_frame_, root_from_state_base_))
  {
    throw std::invalid_argument(
      "No fixed-only chain from root link '" + root_frame_ +
      "' to state_base_frame '" + state_base_frame_ + "'.");
  }

  try {
    ::pinocchio::urdf::buildModelFromXML(robot.xml, model_);
  } catch (const std::exception & exception) {
    throw std::invalid_argument(
      "Failed to build Pinocchio model from '" + robot.source +
      "': " + exception.what());
  }

  // 非 fixed、非标量的关节无法用 9D/8D 合同表达；必须先明确报错。
  for (::pinocchio::JointIndex joint_id = 1;
    joint_id < static_cast<::pinocchio::JointIndex>(model_.njoints); ++joint_id)
  {
    const std::string & name = model_.names[joint_id];
    const auto * joint = robot.findJoint(name);
    if (joint == nullptr) {
      throw std::invalid_argument(
        "Pinocchio joint '" + name + "' has no URDF description.");
    }
    if (joint->isFixed()) {
      continue;
    }
    if (!joint->isScalar()) {
      throw std::invalid_argument(
        "Joint '" + name + "' is '" + wbmm::robot_model::toString(joint->type) +
        "'; only fixed/revolute/continuous/prismatic joints are supported.");
    }
  }

  for (const auto & name : config.controlled_joints) {
    const auto * joint = robot.findJoint(name);
    if (joint == nullptr) {
      throw std::invalid_argument("Controlled joint '" + name + "' is not in the URDF.");
    }
    const ::pinocchio::JointIndex joint_id = model_.getJointId(name);
    if (joint_id >= static_cast<::pinocchio::JointIndex>(model_.njoints) ||
      model_.names[joint_id] != name)
    {
      throw std::invalid_argument(
        "Controlled joint '" + name + "' is not in the Pinocchio model.");
    }
    const int nq = model_.joints[joint_id].nq();
    const int nv = model_.joints[joint_id].nv();
    if ((nq != 1 && nq != 2) || nv != 1) {
      throw std::invalid_argument(
        "Controlled joint '" + name + "' has unsupported nq/nv = " +
        std::to_string(nq) + "/" + std::to_string(nv) + ".");
    }

    ControlledJoint entry;
    entry.name = name;
    entry.joint_id = joint_id;
    entry.idx_q = model_.idx_qs[joint_id];
    entry.idx_v = model_.idx_vs[joint_id];
    entry.nq = nq;
    entry.nv = nv;
    entry.has_position_limit = joint->has_position_limit;
    entry.lower = joint->has_position_limit
      ? joint->lower : -std::numeric_limits<double>::infinity();
    entry.upper = joint->has_position_limit
      ? joint->upper : std::numeric_limits<double>::infinity();
    entry.has_velocity_limit = joint->has_velocity_limit;
    entry.velocity = joint->has_velocity_limit ? joint->velocity : 0.0;
    controlled_joint_names_.push_back(name);
    controlled_joints_.push_back(entry);
  }

  if (controlled_joints_.empty()) {
    throw std::invalid_argument("KinematicModel requires at least one controlled joint.");
  }

  locked_joints_ = config.locked_joints;
  for (const auto & entry : locked_joints_) {
    const auto * joint = robot.findJoint(entry.first);
    if (joint == nullptr || !joint->isScalar()) {
      throw std::invalid_argument(
        "Locked joint '" + entry.first + "' is not a scalar URDF joint.");
    }
  }

  // 状态只携带受控关节的位置（每个关节一个标量），输入携带底盘 2 维加
  // 每个受控关节一个速度。
  state_dimension_ = 3U + controlled_joints_.size();
  input_dimension_ = 2U + controlled_joints_.size();
}

std::shared_ptr<const KinematicModel> KinematicModel::create(
  wbmm::robot_model::RobotModelDescriptionPtr description)
{
  if (description == nullptr) {
    throw std::invalid_argument("KinematicModel requires a RobotModelDescription.");
  }
  return create(
    std::make_shared<const wbmm::robot_model::RobotDescription>(
      description->description),
    description->config);
}

std::shared_ptr<const KinematicModel> KinematicModel::create(
  std::shared_ptr<const wbmm::robot_model::RobotDescription> description,
  wbmm::robot_model::RobotModelConfig config)
{
  if (description == nullptr) {
    throw std::invalid_argument("KinematicModel requires a RobotDescription.");
  }
  const auto validation = wbmm::robot_model::validate(config, *description);
  if (!validation.ok) {
    throw std::invalid_argument(
      "Invalid robot config: " + validation.message);
  }
  return std::shared_ptr<const KinematicModel>(
    new KinematicModel(std::move(description), std::move(config)));
}

bool KinematicModel::hasFrame(const std::string & frame_name) const
{
  return !frame_name.empty() && model_.existFrame(frame_name);
}

::pinocchio::FrameIndex KinematicModel::frameId(
  const std::string & frame_name) const
{
  if (!hasFrame(frame_name)) {
    return static_cast<::pinocchio::FrameIndex>(model_.nframes);
  }
  return model_.getFrameId(frame_name);
}

::pinocchio::JointIndex KinematicModel::jointId(
  const std::string & joint_name) const
{
  const auto joint_id = model_.getJointId(joint_name);
  if (joint_id >= static_cast<::pinocchio::JointIndex>(model_.njoints) ||
    model_.names[joint_id] != joint_name)
  {
    return static_cast<::pinocchio::JointIndex>(model_.njoints);
  }
  return joint_id;
}

std::size_t KinematicModel::controlledJointIndex(
  const std::string & joint_name) const noexcept
{
  for (std::size_t i = 0U; i < controlled_joint_names_.size(); ++i) {
    if (controlled_joint_names_[i] == joint_name) {
      return i;
    }
  }
  return static_cast<std::size_t>(-1);
}

bool KinematicModel::configurationFromState(
  const wbmm::core::WholeBodyState & state, Eigen::VectorXd & configuration,
  std::string * message) const
{
  const auto fail = [message](const std::string & text) {
      if (message != nullptr) {
        *message = text;
      }
      return false;
    };

  const std::size_t joint_count = controlled_joint_names_.size();
  if (state.joints.names.size() != joint_count ||
    state.joints.positions.size() != joint_count)
  {
    return fail(
      "state must contain exactly " + std::to_string(joint_count) +
      " named joint positions");
  }

  std::map<std::string, double> values;
  for (std::size_t i = 0U; i < joint_count; ++i) {
    const std::string & name = state.joints.names[i];
    if (name.empty()) {
      return fail("joint names must not be empty");
    }
    if (controlledJointIndex(name) == static_cast<std::size_t>(-1)) {
      return fail("unknown joint name '" + name + "'");
    }
    if (!values.emplace(name, state.joints.positions[i]).second) {
      return fail("joint names must not contain duplicates");
    }
    if (!std::isfinite(state.joints.positions[i])) {
      return fail("state joint positions must be finite");
    }
  }
  for (const auto & entry : locked_joints_) {
    values.emplace(entry.first, entry.second);
  }

  configuration = Eigen::VectorXd::Zero(model_.nq);
  try {
    // 受控与锁定关节：直接写入。
    for (const auto & joint : description_->joints) {
      if (!joint.isScalar()) {
        continue;
      }
      const auto found = values.find(joint.name);
      if (found == values.end()) {
        continue;
      }
      const auto joint_id = jointId(joint.name);
      if (joint_id >= static_cast<::pinocchio::JointIndex>(model_.njoints)) {
        continue;
      }
      writeJointConfiguration(model_, joint_id, found->second, configuration);
    }
  } catch (const std::exception & exception) {
    return fail(exception.what());
  }

  if (!configuration.allFinite()) {
    return fail("state joint positions must be finite");
  }
  return true;
}

bool KinematicModel::validateState(
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

  const std::size_t joint_count = controlled_joint_names_.size();
  if (state.joints.names.size() != joint_count) {
    return fail(
      "WholeBodyState must contain exactly " + std::to_string(joint_count) +
      " joint names");
  }
  for (std::size_t i = 0U; i < joint_count; ++i) {
    if (state.joints.names[i].empty() ||
      controlledJointIndex(state.joints.names[i]) == static_cast<std::size_t>(-1))
    {
      return fail("unknown joint name '" + state.joints.names[i] + "'");
    }
    for (std::size_t j = 0U; j < i; ++j) {
      if (state.joints.names[i] == state.joints.names[j]) {
        return fail("joint names must not contain duplicates");
      }
    }
  }
  if (state.joints.positions.size() != joint_count) {
    return fail(
      "WholeBodyState must contain exactly " + std::to_string(joint_count) +
      " joint positions");
  }

  for (std::size_t i = 0U; i < joint_count; ++i) {
    const auto controlled = controlledJointIndex(state.joints.names[i]);
    const auto & joint = controlled_joints_[controlled];
    const double position = state.joints.positions[i];
    if (!std::isfinite(position)) {
      return fail("JointState.positions must be finite");
    }
    if (joint.has_position_limit &&
      (position < joint.lower - kLimitTolerance ||
      position > joint.upper + kLimitTolerance))
    {
      return fail("Joint '" + joint.name + "' position is outside its limits");
    }
  }

  if (!state.joints.velocities.empty()) {
    if (state.joints.velocities.size() != joint_count) {
      return fail(
        "JointState.velocities must be empty or contain " +
        std::to_string(joint_count) + " values");
    }
    for (std::size_t i = 0U; i < joint_count; ++i) {
      const auto controlled = controlledJointIndex(state.joints.names[i]);
      const auto & joint = controlled_joints_[controlled];
      const double velocity = state.joints.velocities[i];
      if (!std::isfinite(velocity)) {
        return fail("JointState.velocities must be finite");
      }
      if (joint.has_velocity_limit && joint.velocity > 0.0 &&
        std::abs(velocity) > joint.velocity + kLimitTolerance)
      {
        return fail("Joint '" + joint.name + "' velocity exceeds its limit");
      }
    }
  }
  return true;
}

bool KinematicModel::validateInput(
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

  const std::size_t joint_count = controlled_joint_names_.size();
  if (input.joint_names.size() != joint_count) {
    return fail(
      "WholeBodyInput must contain exactly " + std::to_string(joint_count) +
      " joint names");
  }
  if (input.joint_velocities.size() != joint_count) {
    return fail(
      "WholeBodyInput must contain exactly " + std::to_string(joint_count) +
      " joint velocities");
  }
  for (std::size_t i = 0U; i < joint_count; ++i) {
    const auto controlled = controlledJointIndex(input.joint_names[i]);
    if (controlled == static_cast<std::size_t>(-1)) {
      return fail("unknown joint name '" + input.joint_names[i] + "'");
    }
    for (std::size_t j = 0U; j < i; ++j) {
      if (input.joint_names[i] == input.joint_names[j]) {
        return fail("joint names must not contain duplicates");
      }
    }
    const auto & joint = controlled_joints_[controlled];
    const double velocity = input.joint_velocities[i];
    if (!std::isfinite(velocity)) {
      return fail("WholeBodyInput.joint_velocities must be finite");
    }
    if (joint.has_velocity_limit && joint.velocity > 0.0 &&
      std::abs(velocity) > joint.velocity + kLimitTolerance)
    {
      return fail("Joint '" + joint.name + "' velocity exceeds its limit");
    }
  }
  return true;
}

}  // namespace wbmm::pinocchio
