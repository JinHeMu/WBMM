#include "wbmm_robot_model/collision_sphere_model.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace wbmm::robot_model
{
namespace
{

constexpr double kAxisTolerance = 1.0e-5;

std::string geometryName(const CollisionDescription & collision,
                         const std::string & link)
{
  if (!collision.name.empty()) {
    return link + "/" + collision.name;
  }
  return link + "/collision_" + std::to_string(collision.index);
}

}  // namespace

std::size_t CollisionSphereModel::armSphereCount() const noexcept
{
  std::size_t total = 0U;
  for (const auto & group : arm) {
    total += group.spheres.size();
  }
  return total;
}

CollisionSphereModel buildCollisionSphereModel(
  const RobotDescription & description, const RobotModelConfig & config)
{
  CollisionSphereModel result;

  if (description.empty()) {
    result.message = "RobotDescription is empty.";
    return result;
  }
  if (config.controlled_joints.empty()) {
    result.message = "RobotModelConfig.controlled_joints must not be empty.";
    return result;
  }

  result.root_link = description.root_link;
  result.base_collision_link = config.base_collision_link;
  result.arm_joint_names = config.controlled_joints;

  // 语义分组：link -> 额外标签。
  std::map<std::string, std::vector<std::string>> link_group_tags;
  for (const auto & entry : config.collision_groups) {
    for (const auto & link : entry.second) {
      link_group_tags[link].push_back(entry.first);
    }
  }

  // 解析受控关节，保留调用方给出的顺序。
  std::vector<const JointDescription *> arm_joints;
  arm_joints.reserve(config.controlled_joints.size());
  for (const auto & name : config.controlled_joints) {
    const auto * joint = description.findJoint(name);
    if (joint == nullptr) {
      result.message = "Joint '" + name + "' does not exist in the URDF.";
      return result;
    }
    if (!joint->isScalar()) {
      result.message = "Joint '" + name + "' is '" + toString(joint->type) +
        "', expected revolute or continuous.";
      return result;
    }
    arm_joints.push_back(joint);
  }

  result.base.name = "base";
  result.base.owner_link = config.base_collision_link;

  if (!config.base_collision_link.empty()) {
    const auto * base_link = description.findLink(config.base_collision_link);
    if (base_link == nullptr) {
      result.message = "Base collision link '" + config.base_collision_link +
        "' does not exist in the URDF.";
      return result;
    }

    std::size_t sphere_index = 0U;
    for (const auto & collision : base_link->collisions) {
      if (collision.geometry.type != GeometryType::kSphere) {
        result.skipped_geometries.push_back(
          geometryName(collision, base_link->name));
        continue;
      }
      CollisionSphere sphere;
      sphere.id = base_link->name + "_sphere_" + std::to_string(sphere_index);
      sphere.owner_link = base_link->name;
      sphere.center_in_link = collision.origin.translation();
      sphere.radius = collision.geometry.radius;
      sphere.source_geometry = geometryName(collision, base_link->name);
      sphere.group_tags.push_back("base");
      for (const auto & tag : link_group_tags[base_link->name]) {
        sphere.group_tags.push_back(tag);
      }
      result.base.spheres.push_back(std::move(sphere));
      ++sphere_index;
    }

    if (result.base.spheres.empty()) {
      result.message = "Base collision link '" + config.base_collision_link +
        "' carries no sphere collision geometry.";
      return result;
    }

    if (!fixedTransform(
        description, result.root_link, config.base_collision_link,
        result.root_from_base_collision))
    {
      result.message = "No fixed-only chain from root link '" + result.root_link +
        "' to base collision link '" + config.base_collision_link + "'.";
      return result;
    }
  } else {
    result.base.owner_link.clear();
  }

  // 第一段臂链：base_collision_link -> 第一个受控关节的 parent。之后每一跳
  // 都经过可动关节。
  if (!result.base_collision_link.empty()) {
    if (!fixedTransform(
        description, result.base_collision_link, arm_joints.front()->parent_link,
        result.base_from_arm_mount))
    {
      result.message = "No fixed-only chain from '" + result.base_collision_link +
        "' to the first joint's parent link '" + arm_joints.front()->parent_link +
        "'.";
      return result;
    }
  }

  for (std::size_t i = 0U; i < arm_joints.size(); ++i) {
    const JointDescription & joint = *arm_joints[i];

    std::vector<LinkCollision> collected;
    collectFixedSubtreeCollisions(description, joint.child_link, collected);

    CollisionSphereGroup group;
    group.name = joint.name;
    group.owner_link = joint.child_link;

    std::size_t sphere_index = 0U;
    const auto addSphere = [&](const Eigen::Vector3d & center, double radius,
                               const std::string & source_geometry) {
        CollisionSphere sphere;
        sphere.id = joint.name + "_sphere_" + std::to_string(sphere_index);
        sphere.owner_link = joint.child_link;
        sphere.center_in_link = center;
        sphere.radius = radius;
        sphere.source_geometry = source_geometry;
        sphere.group_tags.push_back("arm");
        sphere.group_tags.push_back(joint.name);
        for (const auto & tag : link_group_tags[joint.child_link]) {
          sphere.group_tags.push_back(tag);
        }
        result.max_arm_radius = std::max(result.max_arm_radius, radius);
        group.spheres.push_back(std::move(sphere));
        ++sphere_index;
      };

    for (const auto & entry : collected) {
      if (entry.geometry.type != GeometryType::kSphere) {
        result.skipped_geometries.push_back(entry.source_geometry);
        continue;
      }
      addSphere(
        entry.transform_in_link.translation(), entry.geometry.radius,
        entry.source_geometry);
    }

    // REMANI 折叠：第一个可动关节的 parent link 上、位于关节轴上的球在关节
    // 旋转下不变，因此可以并入 joint 0 的组，而不是被丢弃。
    if (i == 0U) {
      const auto * parent_link = description.findLink(joint.parent_link);
      if (parent_link != nullptr) {
        for (const auto & collision : parent_link->collisions) {
          if (collision.geometry.type != GeometryType::kSphere) {
            continue;
          }
          const Eigen::Vector3d center = collision.origin.translation();
          const Eigen::Vector3d offset = center - joint.origin.translation();
          const Eigen::Vector3d radial =
            offset - offset.dot(joint.axis) * joint.axis;
          if (radial.norm() < kAxisTolerance) {
            addSphere(offset, collision.geometry.radius,
                      geometryName(collision, parent_link->name));
          }
        }
      }
    }

    result.arm.push_back(std::move(group));
  }

  if (result.armSphereCount() == 0U) {
    result.message = "No arm collision spheres were found in the URDF.";
    return result;
  }

  result.success = true;
  result.message = "ok";
  return result;
}

}  // namespace wbmm::robot_model
