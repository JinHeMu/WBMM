#include "wbmm_robot_model/robot_model_config.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <sstream>

namespace wbmm::robot_model
{
namespace
{

std::string scalarToString(double value)
{
  std::ostringstream stream;
  stream.precision(17);
  stream << value;
  return stream.str();
}

}  // namespace

RobotModelConfig RobotModelConfig::defaultsFor(const RobotDescription & description)
{
  RobotModelConfig config;
  config.state_base_frame = description.root_link;
  config.controlled_joints = description.scalarJointNames();
  if (description.findLink("tool0") != nullptr) {
    config.end_effectors.emplace("tool", "tool0");
  }
  return config;
}

ModelConfigValidation validate(
  const RobotModelConfig & config, const RobotDescription & description)
{
  const auto fail = [](const std::string & text) {
      ModelConfigValidation result;
      result.ok = false;
      result.message = text;
      return result;
    };

  if (description.empty()) {
    return fail("RobotDescription is empty.");
  }
  if (config.state_base_frame.empty()) {
    return fail("RobotModelConfig.state_base_frame must not be empty.");
  }
  if (description.findLink(config.state_base_frame) == nullptr) {
    return fail(
      "RobotModelConfig.state_base_frame '" + config.state_base_frame +
      "' is not a URDF link.");
  }

  if (config.controlled_joints.empty()) {
    return fail("RobotModelConfig.controlled_joints must not be empty.");
  }
  std::set<std::string> controlled;
  for (const auto & name : config.controlled_joints) {
    if (name.empty()) {
      return fail("RobotModelConfig.controlled_joints contains an empty name.");
    }
    const auto * joint = description.findJoint(name);
    if (joint == nullptr) {
      return fail("Controlled joint '" + name + "' does not exist in the URDF.");
    }
    if (!joint->isScalar()) {
      return fail(
        "Controlled joint '" + name + "' is '" + toString(joint->type) +
        "', expected revolute/continuous/prismatic.");
    }
    if (!controlled.insert(name).second) {
      return fail("Controlled joint '" + name + "' is listed twice.");
    }
  }

  for (const auto & entry : config.locked_joints) {
    const auto * joint = description.findJoint(entry.first);
    if (joint == nullptr) {
      return fail("Locked joint '" + entry.first + "' does not exist in the URDF.");
    }
    if (joint->isFixed()) {
      return fail("Locked joint '" + entry.first + "' is already fixed.");
    }
    if (!std::isfinite(entry.second)) {
      return fail("Locked joint '" + entry.first + "' has a non-finite position.");
    }
    if (controlled.count(entry.first) != 0U) {
      return fail("Joint '" + entry.first + "' is both controlled and locked.");
    }
  }

  for (const auto & joint : description.joints) {
    // 非受控、非锁定的可动关节会让状态到 q 的映射不完整，必须显式处理。
    if (joint.isScalar() &&
      controlled.count(joint.name) == 0U &&
      config.locked_joints.count(joint.name) == 0U)
    {
      return fail(
        "Movable joint '" + joint.name +
        "' is neither controlled nor locked; add it to one of them explicitly.");
    }
  }

  for (const auto & entry : config.end_effectors) {
    if (entry.first.empty() || entry.second.empty()) {
      return fail("RobotModelConfig.end_effectors contains an empty name.");
    }
    if (description.findLink(entry.second) == nullptr) {
      return fail(
        "End-effector frame '" + entry.second + "' (name '" + entry.first +
        "') is not a URDF link.");
    }
  }

  if (!config.base_collision_link.empty() &&
    description.findLink(config.base_collision_link) == nullptr)
  {
    return fail(
      "RobotModelConfig.base_collision_link '" + config.base_collision_link +
      "' is not a URDF link.");
  }

  for (const auto & entry : config.collision_groups) {
    if (entry.first.empty()) {
      return fail("RobotModelConfig.collision_groups contains an empty group name.");
    }
    for (const auto & link : entry.second) {
      if (description.findLink(link) == nullptr) {
        return fail(
          "Collision group '" + entry.first + "' references unknown link '" +
          link + "'.");
      }
    }
  }

  ModelConfigValidation result;
  result.ok = true;
  result.message = "ok";
  return result;
}

std::uint64_t contentId(const RobotModelConfig & config) noexcept
{
  std::ostringstream stream;
  stream << "state_base_frame=" << config.state_base_frame << ';';
  stream << "controlled=";
  for (const auto & joint : config.controlled_joints) {
    stream << joint << ',';
  }
  stream << ";locked=";
  for (const auto & entry : config.locked_joints) {
    stream << entry.first << '=' << scalarToString(entry.second) << ',';
  }
  stream << ";ee=";
  for (const auto & entry : config.end_effectors) {
    stream << entry.first << '=' << entry.second << ',';
  }
  stream << ";base_collision_link=" << config.base_collision_link << ';';
  stream << "groups=";
  for (const auto & entry : config.collision_groups) {
    stream << entry.first << ':';
    for (const auto & link : entry.second) {
      stream << link << ',';
    }
    stream << ';';
  }
  return fnv1a64(stream.str());
}

}  // namespace wbmm::robot_model
