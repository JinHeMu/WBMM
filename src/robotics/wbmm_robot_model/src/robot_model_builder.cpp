#include "wbmm_robot_model/robot_model_description.hpp"

#include <sstream>

namespace wbmm::robot_model
{
namespace
{

std::uint64_t combine(std::uint64_t a, std::uint64_t b, const std::string & extra)
{
  std::ostringstream stream;
  stream << a << ':' << b << ':' << extra;
  return fnv1a64(stream.str());
}

}  // namespace

std::string RobotModelDescription::contentIdHex() const
{
  std::ostringstream stream;
  stream << std::hex << content_id;
  return stream.str();
}

RobotModelDescriptionResult buildRobotModelDescription(
  const RobotDescription & description, const RobotModelConfig & config)
{
  RobotModelDescriptionResult result;

  const auto validation = validate(config, description);
  if (!validation.ok) {
    result.message = "Invalid robot config: " + validation.message;
    return result;
  }

  result.model.description = description;
  result.model.config = config;
  result.model.collision_spheres = buildCollisionSphereModel(description, config);
  if (!result.model.collision_spheres.success) {
    result.message =
      "Cannot build collision sphere model: " + result.model.collision_spheres.message;
    return result;
  }

  const std::string extra = result.model.collision_spheres.strategy;
  result.model.content_id = combine(
    description.contentId(), contentId(config), extra);

  result.success = true;
  result.message = "ok";
  return result;
}

}  // namespace wbmm::robot_model
