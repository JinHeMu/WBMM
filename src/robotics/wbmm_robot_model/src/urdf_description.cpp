#include "wbmm_robot_model/robot_description_loader.hpp"

#include <urdf_model/joint.h>
#include <urdf_model/link.h>
#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <functional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace wbmm::robot_model
{
namespace
{

constexpr double kAxisEpsilon = 1.0e-12;

const char * kUnknownJoint = "unknown";
const char * kUnknownGeometry = "unknown";

bool toIsometry(
  const urdf::Pose & pose, Eigen::Isometry3d & transform, std::string & error)
{
  transform = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d translation(
    pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond rotation(
    pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  if (!translation.allFinite() || !rotation.coeffs().allFinite()) {
    error = "URDF origin contains non-finite values.";
    return false;
  }
  if (rotation.norm() < 1.0e-12) {
    error = "URDF origin has a zero quaternion.";
    return false;
  }
  rotation.normalize();
  transform.translation() = translation;
  transform.linear() = rotation.toRotationMatrix();
  return true;
}

JointType convertJointType(int type)
{
  switch (type) {
    case urdf::Joint::REVOLUTE:
      return JointType::kRevolute;
    case urdf::Joint::CONTINUOUS:
      return JointType::kContinuous;
    case urdf::Joint::PRISMATIC:
      return JointType::kPrismatic;
    case urdf::Joint::FIXED:
      return JointType::kFixed;
    case urdf::Joint::FLOATING:
      return JointType::kFloating;
    case urdf::Joint::PLANAR:
      return JointType::kPlanar;
    default:
      return JointType::kUnknown;
  }
}

GeometryType convertGeometryType(int type)
{
  switch (type) {
    case urdf::Geometry::SPHERE:
      return GeometryType::kSphere;
    case urdf::Geometry::BOX:
      return GeometryType::kBox;
    case urdf::Geometry::CYLINDER:
      return GeometryType::kCylinder;
    case urdf::Geometry::MESH:
      return GeometryType::kMesh;
    default:
      return GeometryType::kUnknown;
  }
}

bool convertGeometry(
  const urdf::Geometry & geometry, GeometryDescription & out, std::string & error)
{
  out = GeometryDescription{};
  out.type = convertGeometryType(geometry.type);
  switch (geometry.type) {
    case urdf::Geometry::SPHERE: {
      const auto & sphere = static_cast<const urdf::Sphere &>(geometry);
      out.radius = sphere.radius;
      if (!(out.radius > 0.0) || !std::isfinite(out.radius)) {
        error = "sphere radius must be positive and finite.";
        return false;
      }
      return true;
    }
    case urdf::Geometry::BOX: {
      const auto & box = static_cast<const urdf::Box &>(geometry);
      out.size = Eigen::Vector3d(box.dim.x, box.dim.y, box.dim.z);
      if (!out.size.allFinite() || out.size.minCoeff() <= 0.0) {
        error = "box dimensions must be positive and finite.";
        return false;
      }
      return true;
    }
    case urdf::Geometry::CYLINDER: {
      const auto & cylinder = static_cast<const urdf::Cylinder &>(geometry);
      out.radius = cylinder.radius;
      out.length = cylinder.length;
      if (!(out.radius > 0.0) || !(out.length > 0.0) ||
        !std::isfinite(out.radius) || !std::isfinite(out.length))
      {
        error = "cylinder radius and length must be positive and finite.";
        return false;
      }
      return true;
    }
    case urdf::Geometry::MESH: {
      const auto & mesh = static_cast<const urdf::Mesh &>(geometry);
      out.mesh_filename = mesh.filename;
      out.mesh_scale = Eigen::Vector3d(mesh.scale.x, mesh.scale.y, mesh.scale.z);
      if (out.mesh_filename.empty()) {
        error = "mesh filename is empty.";
        return false;
      }
      if (!out.mesh_scale.allFinite() || out.mesh_scale.minCoeff() <= 0.0) {
        error = "mesh scale must be positive and finite.";
        return false;
      }
      return true;
    }
    default:
      error = "unsupported geometry type.";
      return false;
  }
}

}  // namespace

const char * toString(JointType type) noexcept
{
  switch (type) {
    case JointType::kRevolute:
      return "revolute";
    case JointType::kContinuous:
      return "continuous";
    case JointType::kPrismatic:
      return "prismatic";
    case JointType::kFixed:
      return "fixed";
    case JointType::kFloating:
      return "floating";
    case JointType::kPlanar:
      return "planar";
    case JointType::kUnknown:
      break;
  }
  return kUnknownJoint;
}

const char * toString(GeometryType type) noexcept
{
  switch (type) {
    case GeometryType::kSphere:
      return "sphere";
    case GeometryType::kBox:
      return "box";
    case GeometryType::kCylinder:
      return "cylinder";
    case GeometryType::kMesh:
      return "mesh";
    case GeometryType::kUnknown:
      break;
  }
  return kUnknownGeometry;
}

bool JointDescription::isScalar() const noexcept
{
  return type == JointType::kRevolute || type == JointType::kContinuous ||
         type == JointType::kPrismatic;
}

bool JointDescription::isFixed() const noexcept
{
  return type == JointType::kFixed;
}

std::uint64_t fnv1a64(const std::string & text) noexcept
{
  std::uint64_t hash = 1469598103934665603ULL;
  for (const unsigned char byte : text) {
    hash ^= static_cast<std::uint64_t>(byte);
    hash *= 1099511628211ULL;
  }
  return hash;
}

const LinkDescription * RobotDescription::findLink(const std::string & name) const
{
  const auto found = link_index.find(name);
  if (found == link_index.end()) {
    return nullptr;
  }
  return &links[found->second];
}

const JointDescription * RobotDescription::findJoint(const std::string & name) const
{
  const auto found = joint_index.find(name);
  if (found == joint_index.end()) {
    return nullptr;
  }
  return &joints[found->second];
}

const JointDescription * RobotDescription::findJointByChildLink(
  const std::string & link_name) const
{
  const auto found = joint_by_child_link.find(link_name);
  if (found == joint_by_child_link.end()) {
    return nullptr;
  }
  return &joints[found->second];
}

std::vector<std::string> RobotDescription::scalarJointNames() const
{
  std::vector<std::string> names;
  for (const auto & joint : joints) {
    if (joint.isScalar()) {
      names.push_back(joint.name);
    }
  }
  return names;
}

bool fixedTransform(
  const RobotDescription & description, const std::string & from,
  const std::string & to, Eigen::Isometry3d & transform)
{
  transform = Eigen::Isometry3d::Identity();
  if (description.findLink(from) == nullptr ||
    description.findLink(to) == nullptr)
  {
    return false;
  }
  if (from == to) {
    return true;
  }

  // 返回 from -> to 的位姿（即 to 在 from 系中的表达）。只在 fixed 关节上
  // 行走，父子方向都要试。
  std::set<std::string> visited;
  std::function<bool(const std::string &, Eigen::Isometry3d &)> search =
    [&](const std::string & current, Eigen::Isometry3d & out) -> bool {
      if (current == to) {
        out = Eigen::Isometry3d::Identity();
        return true;
      }
      if (!visited.insert(current).second) {
        return false;
      }
      // 向父节点方向：origin 是 parent->child，取逆。
      const JointDescription * parent = description.findJointByChildLink(current);
      if (parent != nullptr && parent->isFixed()) {
        Eigen::Isometry3d tail = Eigen::Isometry3d::Identity();
        if (search(parent->parent_link, tail)) {
          out = parent->origin.inverse() * tail;
          return true;
        }
      }
      // 向子节点方向：origin 就是 parent->child。
      for (const auto & joint : description.joints) {
        if (!joint.isFixed() || joint.parent_link != current) {
          continue;
        }
        Eigen::Isometry3d tail = Eigen::Isometry3d::Identity();
        if (search(joint.child_link, tail)) {
          out = joint.origin * tail;
          return true;
        }
      }
      return false;
    };

  return search(from, transform);
}

void collectFixedSubtreeCollisions(
  const RobotDescription & description, const std::string & link,
  std::vector<LinkCollision> & out)
{
  const auto * description_link = description.findLink(link);
  if (description_link == nullptr) {
    return;
  }
  for (const auto & collision : description_link->collisions) {
    LinkCollision entry;
    entry.source_link = link;
    entry.source_geometry = collision.name.empty()
      ? link + "/collision_" + std::to_string(collision.index)
      : link + "/" + collision.name;
    entry.transform_in_link = collision.origin;
    entry.geometry = collision.geometry;
    out.push_back(std::move(entry));
  }

  for (const auto & joint : description.joints) {
    if (!joint.isFixed() || joint.parent_link != link) {
      continue;
    }
    const std::size_t before = out.size();
    collectFixedSubtreeCollisions(description, joint.child_link, out);
    for (std::size_t i = before; i < out.size(); ++i) {
      out[i].transform_in_link = joint.origin * out[i].transform_in_link;
    }
  }
}

namespace
{

RobotDescriptionLoadResult convertModel(
  const urdf::ModelInterface & model, const std::string & xml,
  const std::string & source)
{
  RobotDescriptionLoadResult result;
  if (model.getRoot() == nullptr) {
    result.message = "URDF has no root link ('" + source + "').";
    return result;
  }

  RobotDescription & description = result.description;
  description.name = model.name_;
  description.source = source;
  description.xml = xml;
  description.root_link = model.getRoot()->name;
  description.content_id = fnv1a64(xml);

  for (const auto & entry : model.links_) {
    const urdf::Link & link = *entry.second;
    if (link.name.empty()) {
      result.message = "URDF contains a <link> without a name.";
      return result;
    }
    LinkDescription description_link;
    description_link.name = link.name;

    if (link.inertial) {
      std::string error;
      description_link.has_inertial = true;
      if (!toIsometry(link.inertial->origin, description_link.inertial.origin, error)) {
        result.message = "Link '" + link.name + "' inertial: " + error;
        return result;
      }
      description_link.inertial.mass = link.inertial->mass;
      Eigen::Matrix3d inertia;
      inertia << link.inertial->ixx, link.inertial->ixy, link.inertial->ixz,
        link.inertial->ixy, link.inertial->iyy, link.inertial->iyz,
        link.inertial->ixz, link.inertial->iyz, link.inertial->izz;
      description_link.inertial.inertia = inertia;
    }

    for (const auto & visual : link.visual_array) {
      if (!visual || !visual->geometry) {
        continue;
      }
      VisualDescription entry_visual;
      entry_visual.name = visual->name;
      std::string error;
      if (!toIsometry(visual->origin, entry_visual.origin, error)) {
        result.message = "Link '" + link.name + "' visual: " + error;
        return result;
      }
      if (!convertGeometry(*visual->geometry, entry_visual.geometry, error)) {
        result.message = "Link '" + link.name + "' visual " + error;
        return result;
      }
      if (visual->material) {
        entry_visual.material_name = visual->material->name;
        entry_visual.has_color = true;
        entry_visual.color = Eigen::Vector4d(
          visual->material->color.r, visual->material->color.g,
          visual->material->color.b, visual->material->color.a);
      } else if (!visual->material_name.empty()) {
        entry_visual.material_name = visual->material_name;
      }
      description_link.visuals.push_back(std::move(entry_visual));
    }

    for (std::size_t i = 0; i < link.collision_array.size(); ++i) {
      const auto & collision = link.collision_array[i];
      if (!collision || !collision->geometry) {
        continue;
      }
      CollisionDescription entry_collision;
      entry_collision.name = collision->name;
      entry_collision.index = i;
      std::string error;
      if (!toIsometry(collision->origin, entry_collision.origin, error)) {
        result.message = "Link '" + link.name + "' collision: " + error;
        return result;
      }
      if (!convertGeometry(*collision->geometry, entry_collision.geometry, error)) {
        result.message = "Link '" + link.name + "' collision " + error;
        return result;
      }
      description_link.collisions.push_back(std::move(entry_collision));
    }

    description.link_index[description_link.name] = description.links.size();
    description.links.push_back(std::move(description_link));
  }

  for (const auto & entry : model.joints_) {
    const urdf::Joint & joint = *entry.second;
    JointDescription description_joint;
    description_joint.name = joint.name;
    description_joint.type = convertJointType(joint.type);
    description_joint.parent_link = joint.parent_link_name;
    description_joint.child_link = joint.child_link_name;

    std::string error;
    if (!toIsometry(joint.parent_to_joint_origin_transform, description_joint.origin, error)) {
      result.message = "Joint '" + joint.name + "': " + error;
      return result;
    }

    Eigen::Vector3d axis(joint.axis.x, joint.axis.y, joint.axis.z);
    if (!axis.allFinite() || axis.norm() < kAxisEpsilon) {
      axis = Eigen::Vector3d::UnitZ();
    } else {
      axis.normalize();
    }
    description_joint.axis = axis;

    if (joint.limits) {
      description_joint.has_effort_limit = true;
      description_joint.effort = joint.limits->effort;
      description_joint.has_velocity_limit = true;
      description_joint.velocity = joint.limits->velocity;
      if (description_joint.type == JointType::kRevolute ||
        description_joint.type == JointType::kPrismatic)
      {
        description_joint.has_position_limit = true;
        description_joint.lower = joint.limits->lower;
        description_joint.upper = joint.limits->upper;
      }
    }

    if (joint.mimic) {
      result.message = "Mimic joint '" + joint.name +
        "' is unsupported by the WBMM robot model.";
      return result;
    }

    description.joint_index[description_joint.name] = description.joints.size();
    description.joint_by_child_link[description_joint.child_link] =
      description.joints.size();
    description.joints.push_back(std::move(description_joint));
  }

  result.success = true;
  result.message = "ok";
  return result;
}

}  // namespace

RobotDescriptionLoadResult RobotDescriptionLoader::fromFile(
  const std::string & urdf_path)
{
  RobotDescriptionLoadResult result;
  if (urdf_path.empty()) {
    result.message = "URDF path must not be empty.";
    return result;
  }
  std::ifstream stream(urdf_path);
  if (!stream) {
    result.message = "Cannot read URDF file '" + urdf_path + "'.";
    return result;
  }
  std::ostringstream content;
  content << stream.rdbuf();
  return fromXml(content.str(), urdf_path);
}

RobotDescriptionLoadResult RobotDescriptionLoader::fromXml(
  const std::string & xml, const std::string & source)
{
  RobotDescriptionLoadResult result;
  if (xml.empty()) {
    result.message = "URDF XML must not be empty.";
    return result;
  }
  urdf::ModelInterfaceSharedPtr model = urdf::parseURDF(xml);
  if (!model) {
    result.message = "Cannot parse URDF XML from '" + source +
      "' (urdfdom returned no model).";
    return result;
  }
  return convertModel(*model, xml, source);
}

RobotDescription loadRobotDescription(const std::string & urdf_path)
{
  auto result = RobotDescriptionLoader::fromFile(urdf_path);
  if (!result.success) {
    throw std::invalid_argument(result.message);
  }
  return std::move(result.description);
}

RobotDescription parseRobotDescription(
  const std::string & xml, const std::string & source)
{
  auto result = RobotDescriptionLoader::fromXml(xml, source);
  if (!result.success) {
    throw std::invalid_argument(result.message);
  }
  return std::move(result.description);
}

}  // namespace wbmm::robot_model
