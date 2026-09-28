#include "wbmm_collision/urdf_collision_model.hpp"

#include <tinyxml2.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace wbmm::collision
{
namespace
{

constexpr double kAxisTolerance = 1.0e-5;

struct LinkSphere
{
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
};

struct JointRecord
{
  std::string name;
  std::string type;
  std::string parent_link;
  std::string child_link;
  Eigen::Vector3d origin_translation{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d origin_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
  double lower{0.0};
  double upper{0.0};
};

struct UrdfDocument
{
  // link name -> spheres expressed in that link's frame
  std::map<std::string, std::vector<LinkSphere>> link_spheres;
  std::vector<JointRecord> joints;
  std::map<std::string, std::size_t> joint_by_child;
};

bool parseVector3(const char * text, Eigen::Vector3d & out)
{
  if (text == nullptr) {
    return false;
  }
  std::istringstream stream(text);
  return static_cast<bool>(stream >> out.x() >> out.y() >> out.z());
}

Eigen::Matrix3d rpyToRotation(double roll, double pitch, double yaw)
{
  return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

bool parseOrigin(const tinyxml2::XMLElement * element, Eigen::Vector3d & translation,
                 Eigen::Matrix3d & rotation)
{
  translation.setZero();
  rotation.setIdentity();
  const auto * origin = element->FirstChildElement("origin");
  if (origin == nullptr) {
    return true;
  }
  Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
  parseVector3(origin->Attribute("xyz"), translation);
  if (parseVector3(origin->Attribute("rpy"), rpy)) {
    rotation = rpyToRotation(rpy.x(), rpy.y(), rpy.z());
  }
  return true;
}

bool parseDocument(const std::string & path, UrdfDocument & document, std::string & error)
{
  tinyxml2::XMLDocument xml;
  if (xml.LoadFile(path.c_str()) != tinyxml2::XML_SUCCESS) {
    error = "Cannot parse URDF file '" + path + "': " +
      (xml.ErrorStr() != nullptr ? xml.ErrorStr() : "unknown error");
    return false;
  }

  const auto * robot = xml.FirstChildElement("robot");
  if (robot == nullptr) {
    error = "URDF has no <robot> element.";
    return false;
  }

  for (const auto * link = robot->FirstChildElement("link"); link != nullptr;
    link = link->NextSiblingElement("link"))
  {
    const char * link_name = link->Attribute("name");
    if (link_name == nullptr) {
      error = "URDF contains a <link> without a name.";
      return false;
    }
    std::vector<LinkSphere> spheres;
    for (const auto * collision = link->FirstChildElement("collision");
      collision != nullptr; collision = collision->NextSiblingElement("collision"))
    {
      const auto * geometry = collision->FirstChildElement("geometry");
      if (geometry == nullptr) {
        continue;
      }
      // Only spheres participate. Boxes/cylinders/meshes are ignored, exactly
      // like the REMANI loader, which logs and skips them.
      const auto * sphere = geometry->FirstChildElement("sphere");
      if (sphere == nullptr) {
        continue;
      }
      double radius = 0.0;
      if (sphere->QueryDoubleAttribute("radius", &radius) != tinyxml2::XML_SUCCESS ||
        !(radius > 0.0))
      {
        error = std::string("Link '") + link_name +
          "' has a sphere collision with a missing or non-positive radius.";
        return false;
      }
      LinkSphere entry;
      entry.radius = radius;
      Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
      parseOrigin(collision, entry.center, rotation);
      spheres.push_back(entry);
    }
    document.link_spheres[link_name] = std::move(spheres);
  }

  for (const auto * joint = robot->FirstChildElement("joint"); joint != nullptr;
    joint = joint->NextSiblingElement("joint"))
  {
    JointRecord record;
    const char * name = joint->Attribute("name");
    const char * type = joint->Attribute("type");
    const auto * parent = joint->FirstChildElement("parent");
    const auto * child = joint->FirstChildElement("child");
    if (name == nullptr || type == nullptr || parent == nullptr || child == nullptr ||
      parent->Attribute("link") == nullptr || child->Attribute("link") == nullptr)
    {
      error = "URDF contains a malformed <joint> element.";
      return false;
    }
    record.name = name;
    record.type = type;
    record.parent_link = parent->Attribute("link");
    record.child_link = child->Attribute("link");

    parseOrigin(joint, record.origin_translation, record.origin_rotation);

    const auto * axis = joint->FirstChildElement("axis");
    if (axis != nullptr) {
      Eigen::Vector3d raw = Eigen::Vector3d::UnitZ();
      if (parseVector3(axis->Attribute("xyz"), raw) && raw.norm() > kAxisTolerance) {
        record.axis = raw.normalized();
      }
    }

    const auto * limit = joint->FirstChildElement("limit");
    if (limit != nullptr) {
      limit->QueryDoubleAttribute("lower", &record.lower);
      limit->QueryDoubleAttribute("upper", &record.upper);
    }

    document.joint_by_child[record.child_link] = document.joints.size();
    document.joints.push_back(std::move(record));
  }

  if (document.link_spheres.empty()) {
    error = "URDF declares no links.";
    return false;
  }
  return true;
}

// Collects every sphere reachable from `link` through FIXED joints only, with
// the centres re-expressed in `link`'s frame.
void collectFixedSubtree(
  const UrdfDocument & document, const std::string & link,
  const Eigen::Matrix4d & link_from_current,
  std::vector<std::pair<Eigen::Vector3d, double>> & out)
{
  const auto spheres = document.link_spheres.find(link);
  if (spheres != document.link_spheres.end()) {
    for (const auto & sphere : spheres->second) {
      const Eigen::Vector4d point(sphere.center.x(), sphere.center.y(),
                                  sphere.center.z(), 1.0);
      const Eigen::Vector4d transformed = link_from_current * point;
      out.emplace_back(transformed.head<3>(), sphere.radius);
    }
  }

  for (const auto & joint : document.joints) {
    if (joint.parent_link != link || joint.type != "fixed") {
      continue;
    }
    Eigen::Matrix4d placement = Eigen::Matrix4d::Identity();
    placement.block<3, 3>(0, 0) = joint.origin_rotation;
    placement.block<3, 1>(0, 3) = joint.origin_translation;
    collectFixedSubtree(
      document, joint.child_link, link_from_current * placement, out);
  }
}

}  // namespace

namespace
{

Eigen::Matrix4d placementOf(const JointRecord & joint)
{
  Eigen::Matrix4d placement = Eigen::Matrix4d::Identity();
  placement.block<3, 3>(0, 0) = joint.origin_rotation;
  placement.block<3, 1>(0, 3) = joint.origin_translation;
  return placement;
}

// Fixed-only chain from `from` to `to`, walking the joint graph in either
// direction. Returns false when the two links are not connected by fixed joints
// alone.
bool fixedPath(
  const UrdfDocument & document, const std::string & from,
  const std::string & to, Eigen::Matrix4d & out, std::set<std::string> & visited)
{
  if (from == to) {
    out = Eigen::Matrix4d::Identity();
    return true;
  }
  if (!visited.insert(from).second) {
    return false;
  }
  for (const auto & joint : document.joints) {
    if (joint.type != "fixed") {
      continue;
    }
    const Eigen::Matrix4d placement = placementOf(joint);
    if (joint.parent_link == from) {
      Eigen::Matrix4d tail = Eigen::Matrix4d::Identity();
      if (fixedPath(document, joint.child_link, to, tail, visited)) {
        out = placement * tail;
        return true;
      }
    } else if (joint.child_link == from) {
      Eigen::Matrix4d tail = Eigen::Matrix4d::Identity();
      if (fixedPath(document, joint.parent_link, to, tail, visited)) {
        out = placement.inverse() * tail;
        return true;
      }
    }
  }
  return false;
}

bool fixedPath(
  const UrdfDocument & document, const std::string & from,
  const std::string & to, Eigen::Matrix4d & out)
{
  std::set<std::string> visited;
  return fixedPath(document, from, to, out, visited);
}

}  // namespace

std::size_t UrdfCollisionModel::armSphereCount() const
{
  std::size_t total = 0U;
  for (const auto & group : arm) {
    total += group.spheres.size();
  }
  return total;
}

UrdfCollisionModel loadUrdfCollisionModel(
  const std::string & urdf_path, const std::vector<std::string> & joint_names,
  const std::string & base_collision_link)
{
  UrdfCollisionModel result;

  if (joint_names.empty()) {
    result.message = "joint_names must not be empty.";
    return result;
  }

  UrdfDocument document;
  if (!parseDocument(urdf_path, document, result.message)) {
    return result;
  }

  // Resolve the requested movable joints in the caller's order.
  for (const auto & name : joint_names) {
    UrdfJointKinematics kinematics;
    bool found = false;
    for (const auto & record : document.joints) {
      if (record.name != name) {
        continue;
      }
      if (record.type != "revolute" && record.type != "continuous") {
        result.message = "Joint '" + name + "' is '" + record.type +
          "', expected revolute or continuous.";
        return result;
      }
      kinematics.name = record.name;
      kinematics.parent_link = record.parent_link;
      kinematics.child_link = record.child_link;
      kinematics.origin_translation = record.origin_translation;
      kinematics.origin_rotation = record.origin_rotation;
      kinematics.axis = record.axis;
      kinematics.lower = record.lower;
      kinematics.upper = record.upper;
      found = true;
      break;
    }
    if (!found) {
      result.message = "Joint '" + name + "' does not exist in the URDF.";
      return result;
    }
    result.joints.push_back(std::move(kinematics));
  }

  // Root link: the only link that is never a joint child.
  {
    std::set<std::string> children;
    for (const auto & record : document.joints) {
      children.insert(record.child_link);
    }
    for (const auto & entry : document.link_spheres) {
      if (children.find(entry.first) == children.end()) {
        result.root_link = entry.first;
        break;
      }
    }
    if (result.root_link.empty()) {
      result.message = "URDF has no single root link.";
      return result;
    }
  }

  // Base footprint group: the spheres of the declared base collision link.
  if (!base_collision_link.empty()) {
    const auto spheres = document.link_spheres.find(base_collision_link);
    if (spheres == document.link_spheres.end()) {
      result.message = "Base collision link '" + base_collision_link +
        "' does not exist in the URDF.";
      return result;
    }
    if (spheres->second.empty()) {
      result.message = "Base collision link '" + base_collision_link +
        "' carries no sphere collision geometry.";
      return result;
    }
    for (std::size_t i = 0U; i < spheres->second.size(); ++i) {
      CollisionSphere sphere;
      sphere.name = base_collision_link + "_sphere_" + std::to_string(i);
      sphere.link_name = base_collision_link;
      sphere.group = CollisionGroup::kBase;
      sphere.center = spheres->second[i].center;
      sphere.radius = spheres->second[i].radius;
      result.base.spheres.push_back(std::move(sphere));
    }

    result.base_collision_link = base_collision_link;
    if (!fixedPath(
        document, result.root_link, base_collision_link,
        result.root_from_base_collision))
    {
      result.message = "No fixed-only chain from root link '" + result.root_link +
        "' to base collision link '" + base_collision_link + "'.";
      return result;
    }
  }

  // Arm groups: the fixed subtree of each movable joint's child link.
  const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();

  // Arm mount: the single fixed hop from the base collision link to the first
  // movable joint's parent link. Every later hop goes through a movable joint,
  // so there is no fixed-only chain beyond this one and the consumer composes
  // the rest from the joint transforms.
  if (!result.base_collision_link.empty() &&
    !fixedPath(
      document, result.base_collision_link, result.joints[0].parent_link,
      result.base_from_arm_mount))
  {
    result.message = "No fixed-only chain from '" + result.base_collision_link +
      "' to the first joint's parent link '" + result.joints[0].parent_link + "'.";
    return result;
  }

  for (std::size_t i = 0U; i < result.joints.size(); ++i) {
    const auto & joint = result.joints[i];

    std::vector<std::pair<Eigen::Vector3d, double>> collected;
    collectFixedSubtree(document, joint.child_link, identity, collected);

    // REMANI fold: for the first movable joint, a sphere on the parent link that
    // is centred on the joint axis is invariant under that joint's rotation, so
    // it can join group 0 instead of being dropped.
    if (i == 0U) {
      const auto parent = document.link_spheres.find(joint.parent_link);
      if (parent != document.link_spheres.end()) {
        for (const auto & sphere : parent->second) {
          const Eigen::Vector3d offset = sphere.center - joint.origin_translation;
          const Eigen::Vector3d radial =
            offset - offset.dot(joint.axis) * joint.axis;
          if (radial.norm() < kAxisTolerance) {
            collected.emplace_back(offset, sphere.radius);
          }
        }
      }
    }

    CollisionModel group;
    for (std::size_t k = 0U; k < collected.size(); ++k) {
      CollisionSphere sphere;
      sphere.name = joint.name + "_sphere_" + std::to_string(k);
      sphere.link_name = joint.child_link;
      sphere.group = CollisionGroup::kArm;
      sphere.center = collected[k].first;
      sphere.radius = collected[k].second;
      result.max_arm_radius = std::max(result.max_arm_radius, sphere.radius);
      group.spheres.push_back(std::move(sphere));
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

}  // namespace wbmm::collision
