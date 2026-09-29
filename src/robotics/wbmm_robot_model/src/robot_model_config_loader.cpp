#include "wbmm_robot_model/robot_model_config_loader.hpp"

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace wbmm::robot_model
{
namespace
{

RobotModelConfigLoadResult fail(const std::string & message)
{
  RobotModelConfigLoadResult result;
  result.success = false;
  result.message = message;
  return result;
}

}  // namespace

RobotModelConfigLoadResult RobotModelConfigLoader::fromYaml(
  const std::string & yaml, const RobotDescription & description,
  const std::string & source)
{
  YAML::Node root;
  try {
    root = YAML::Load(yaml);
  } catch (const YAML::Exception & exception) {
    return fail(
      "Cannot parse robot config YAML '" + source + "': " + exception.what());
  }
  if (!root || !root.IsMap()) {
    return fail(
      "Robot config '" + source + "' must be a YAML mapping.");
  }

  RobotModelConfig config = RobotModelConfig::defaultsFor(description);

  try {
    // 拒绝未实现或拼错的配置，避免 YAML 看似生效而实际被忽略。
    static const std::set<std::string> known_fields{
      "state_base_frame", "controlled_joints", "locked_joints",
      "end_effectors", "base_collision_link", "collision_groups"};
    for (const auto & entry : root) {
      const std::string name = entry.first.as<std::string>();
      if (known_fields.count(name) == 0U) {
        return fail("Unknown robot config field '" + name + "' in '" + source + "'.");
      }
    }

    if (root["state_base_frame"]) {
      config.state_base_frame = root["state_base_frame"].as<std::string>();
    }
    if (root["controlled_joints"]) {
      config.controlled_joints =
        root["controlled_joints"].as<std::vector<std::string>>();
    }
    if (root["locked_joints"]) {
      const auto & node = root["locked_joints"];
      if (!node.IsMap()) {
        return fail("locked_joints must be a mapping of joint name to position.");
      }
      config.locked_joints.clear();
      for (const auto & entry : node) {
        config.locked_joints[entry.first.as<std::string>()] =
          entry.second.as<double>();
      }
    }
    if (root["end_effectors"]) {
      const auto & node = root["end_effectors"];
      if (!node.IsMap()) {
        return fail("end_effectors must be a mapping of name to frame.");
      }
      config.end_effectors.clear();
      for (const auto & entry : node) {
        config.end_effectors[entry.first.as<std::string>()] =
          entry.second.as<std::string>();
      }
    }
    if (root["base_collision_link"]) {
      config.base_collision_link =
        root["base_collision_link"].as<std::string>();
    }
    if (root["collision_groups"]) {
      const auto & node = root["collision_groups"];
      if (!node.IsMap()) {
        return fail("collision_groups must be a mapping of group name to links.");
      }
      config.collision_groups.clear();
      for (const auto & entry : node) {
        config.collision_groups[entry.first.as<std::string>()] =
          entry.second.as<std::vector<std::string>>();
      }
    }
  } catch (const YAML::Exception & exception) {
    return fail(
      "Invalid robot config '" + source + "': " + exception.what());
  }

  const auto validation = validate(config, description);
  if (!validation.ok) {
    return fail("Invalid robot config '" + source + "': " + validation.message);
  }

  RobotModelConfigLoadResult result;
  result.success = true;
  result.message = "ok";
  result.config = std::move(config);
  return result;
}

RobotModelConfigLoadResult RobotModelConfigLoader::fromFile(
  const std::string & yaml_path, const RobotDescription & description)
{
  std::ifstream stream(yaml_path);
  if (!stream) {
    return fail("Cannot read robot config file '" + yaml_path + "'.");
  }
  std::ostringstream content;
  content << stream.rdbuf();
  return fromYaml(content.str(), description, yaml_path);
}

}  // namespace wbmm::robot_model
