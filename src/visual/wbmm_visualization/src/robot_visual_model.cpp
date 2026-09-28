#include <algorithm>
#include <cmath>
#include <filesystem>
#include <functional>
#include <set>
#include <stdexcept>
#include <wbmm_visualization/robot_visual_model.hpp>

namespace wbmm::visualization {
namespace {
Eigen::Isometry3d transform(const urdf::Pose &pose) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() =
      Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  Eigen::Quaterniond q(pose.rotation.w, pose.rotation.x, pose.rotation.y,
                       pose.rotation.z);
  if (!t.translation().allFinite() || !q.coeffs().allFinite() ||
      q.norm() < 1e-12)
    throw std::invalid_argument("Invalid URDF origin.");
  t.linear() = q.normalized().toRotationMatrix();
  return t;
}
bool scalar(const urdf::Joint &j) {
  return j.type == urdf::Joint::REVOLUTE || j.type == urdf::Joint::CONTINUOUS ||
         j.type == urdf::Joint::PRISMATIC;
}
void positive(double value) {
  if (!std::isfinite(value) || value <= 0)
    throw std::invalid_argument(
        "URDF visual scale must be positive and finite.");
}
void colorValue(double value) {
  if (!std::isfinite(value) || value < 0 || value > 1)
    throw std::invalid_argument("RGBA must be in [0,1].");
}
} // namespace

RobotVisualModel::RobotVisualModel(const std::string &xml,
                                   const std::vector<std::string> &names,
                                   std::string base,
                                   std::map<std::string, double> defaults,
                                   std::string directory)
    : joint_names_(names), defaults_(std::move(defaults)) {
  if (!model_.initString(xml) || !model_.getRoot())
    throw std::invalid_argument("Cannot parse robot URDF.");
  base_frame_ = base.empty() ? model_.getRoot()->name : std::move(base);
  auto base_link = model_.getLink(base_frame_);
  if (!base_link)
    throw std::invalid_argument("base_frame is not a URDF link: " +
                                base_frame_);
  for (auto link = base_link; link->parent_joint; link = link->getParent())
    if (link->parent_joint->type != urdf::Joint::FIXED)
      throw std::invalid_argument(
          "base_frame must be connected to the URDF root by fixed joints.");
  std::set<std::string> controlled;
  for (const auto &name : names) {
    const auto joint = model_.getJoint(name);
    if (!controlled.insert(name).second || !joint || !scalar(*joint) ||
        joint->mimic)
      throw std::invalid_argument(
          "Duplicate, missing, fixed or mimic controlled joint: " + name);
  }
  for (const auto &entry : defaults_) {
    const auto joint = model_.getJoint(entry.first);
    if (!joint || !scalar(*joint) || joint->mimic ||
        controlled.count(entry.first) || !std::isfinite(entry.second))
      throw std::invalid_argument("Invalid auxiliary default joint: " +
                                  entry.first);
  }
  for (const auto &entry : model_.joints_) {
    const auto &joint = *entry.second;
    if (joint.type == urdf::Joint::FIXED)
      continue;
    if (!scalar(joint))
      throw std::invalid_argument("Only fixed/revolute/continuous/prismatic "
                                  "URDF joints are supported: " +
                                  joint.name);
    Eigen::Vector3d axis(joint.axis.x, joint.axis.y, joint.axis.z);
    if (!axis.allFinite() || axis.norm() < 1e-12)
      throw std::invalid_argument("Invalid URDF joint axis: " + joint.name);
    if (!joint.mimic && !controlled.count(joint.name) &&
        !defaults_.count(joint.name))
      throw std::invalid_argument(
          "Unconfigured movable joint; provide its display default: " +
          joint.name);
    if (joint.mimic) {
      std::set<std::string> seen{joint.name};
      auto source = joint.mimic;
      while (source) {
        auto parent = model_.getJoint(source->joint_name);
        if (!seen.insert(source->joint_name).second || !parent ||
            !scalar(*parent) || !std::isfinite(source->multiplier) ||
            !std::isfinite(source->offset))
          throw std::invalid_argument("Invalid or cyclic mimic joint: " +
                                      joint.name);
        source = parent->mimic;
      }
    }
  }
  // Stable link/name ordering makes marker ids deterministic across updates.
  for (const auto &entry : model_.links_)
    for (const auto &geometry : entry.second->visual_array) {
      if (!geometry || !geometry->geometry)
        continue;
      Visual visual;
      visual.link = entry.first;
      visual.origin = transform(geometry->origin);
      auto &m = visual.prototype;
      m.action = m.ADD;
      m.pose.orientation.w = 1;
      m.scale.x = m.scale.y = m.scale.z = 1;
      switch (geometry->geometry->type) {
      case urdf::Geometry::MESH: {
        auto mesh = std::dynamic_pointer_cast<urdf::Mesh>(geometry->geometry);
        m.type = m.MESH_RESOURCE;
        m.mesh_resource = mesh->filename;
        m.scale.x = mesh->scale.x;
        m.scale.y = mesh->scale.y;
        m.scale.z = mesh->scale.z;
        if (m.mesh_resource.empty())
          throw std::invalid_argument("Empty URDF mesh filename.");
        if (m.mesh_resource.find("://") == std::string::npos) {
          std::filesystem::path path(m.mesh_resource);
          if (path.is_relative()) {
            if (directory.empty())
              throw std::invalid_argument(
                  "Relative mesh needs urdf_file or mesh_directory.");
            path = std::filesystem::path(directory) / path;
          }
          m.mesh_resource =
              "file://" +
              std::filesystem::absolute(path).lexically_normal().string();
        }
        break;
      }
      case urdf::Geometry::BOX: {
        auto box = std::dynamic_pointer_cast<urdf::Box>(geometry->geometry);
        m.type = m.CUBE;
        m.scale.x = box->dim.x;
        m.scale.y = box->dim.y;
        m.scale.z = box->dim.z;
        break;
      }
      case urdf::Geometry::CYLINDER: {
        auto cylinder =
            std::dynamic_pointer_cast<urdf::Cylinder>(geometry->geometry);
        m.type = m.CYLINDER;
        m.scale.x = m.scale.y = 2 * cylinder->radius;
        m.scale.z = cylinder->length;
        break;
      }
      case urdf::Geometry::SPHERE: {
        auto sphere =
            std::dynamic_pointer_cast<urdf::Sphere>(geometry->geometry);
        m.type = m.SPHERE;
        m.scale.x = m.scale.y = m.scale.z = 2 * sphere->radius;
        break;
      }
      default:
        throw std::invalid_argument("Unsupported URDF visual geometry.");
      }
      positive(m.scale.x);
      positive(m.scale.y);
      positive(m.scale.z);
      if (geometry->material) {
        const auto &c = geometry->material->color;
        visual.has_material = true;
        m.color.r = c.r;
        m.color.g = c.g;
        m.color.b = c.b;
        m.color.a = c.a;
        for (double v : {c.r, c.g, c.b, c.a})
          colorValue(v);
      }
      visuals_.push_back(std::move(visual));
    }
  if (visuals_.empty())
    throw std::invalid_argument(
        "URDF has no visual geometry (collision geometry is not used).");
}

bool RobotVisualModel::hasLink(const std::string &name) const {
  return bool(model_.getLink(name));
}

void RobotVisualModel::validateState(
    const wbmm::core::WholeBodyState &s) const {
  if (s.header.frame_id.empty() || !std::isfinite(s.base.x) ||
      !std::isfinite(s.base.y) || !std::isfinite(s.base.yaw) ||
      s.joints.names.size() != joint_names_.size() ||
      s.joints.positions.size() != joint_names_.size())
    throw std::invalid_argument(
        "Invalid visual state frame/base/joint dimensions.");
  std::set<std::string> seen;
  for (std::size_t j = 0; j < s.joints.names.size(); ++j)
    if (!std::isfinite(s.joints.positions[j]) ||
        !seen.insert(s.joints.names[j]).second ||
        std::find(joint_names_.begin(), joint_names_.end(),
                  s.joints.names[j]) == joint_names_.end())
      throw std::invalid_argument(
          "Visual state has non-finite positions or wrong joint names.");
}

std::map<std::string, Eigen::Isometry3d>
RobotVisualModel::linkPoses(const wbmm::core::WholeBodyState &s) const {
  validateState(s);
  auto values = defaults_;
  for (std::size_t j = 0; j < s.joints.names.size(); ++j)
    values[s.joints.names[j]] = s.joints.positions[j];
  std::function<double(const std::string &)> value =
      [&](const std::string &name) -> double {
    auto found = values.find(name);
    if (found != values.end())
      return found->second;
    const auto mimic = model_.getJoint(name)->mimic;
    const double result =
        mimic->multiplier * value(mimic->joint_name) + mimic->offset;
    if (!std::isfinite(result))
      throw std::invalid_argument("Non-finite mimic joint position.");
    values[name] = result;
    return result;
  };
  std::map<std::string, Eigen::Isometry3d> poses;
  std::function<void(urdf::LinkConstSharedPtr, const Eigen::Isometry3d &)>
      visit;
  visit = [&](urdf::LinkConstSharedPtr link, const Eigen::Isometry3d &parent) {
    poses.emplace(link->name, parent);
    for (const auto &joint : link->child_joints) {
      auto t = transform(joint->parent_to_joint_origin_transform);
      if (joint->type != urdf::Joint::FIXED) {
        const Eigen::Vector3d axis =
            Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z)
                .normalized();
        auto motion = Eigen::Isometry3d::Identity();
        if (joint->type == urdf::Joint::PRISMATIC)
          motion.translation() = axis * value(joint->name);
        else
          motion.linear() =
              Eigen::AngleAxisd(value(joint->name), axis).toRotationMatrix();
        t = t * motion;
      }
      visit(model_.getLink(joint->child_link_name), parent * t);
    }
  };
  visit(model_.getRoot(), Eigen::Isometry3d::Identity());
  auto world_base = Eigen::Isometry3d::Identity();
  world_base.translation() = Eigen::Vector3d(s.base.x, s.base.y, 0);
  world_base.linear() = Eigen::AngleAxisd(s.base.yaw, Eigen::Vector3d::UnitZ())
                            .toRotationMatrix();
  const Eigen::Isometry3d world_root =
      world_base * poses.at(base_frame_).inverse();
  for (auto &entry : poses)
    entry.second = world_root * entry.second;
  return poses;
}

visualization_msgs::msg::MarkerArray
RobotVisualModel::markers(const wbmm::core::WholeBodyState &s,
                          const std::string &ns,
                          const VisualStyle &style) const {
  for (double c : {style.red, style.green, style.blue, style.alpha})
    colorValue(c);
  const auto poses = linkPoses(s);
  visualization_msgs::msg::MarkerArray array;
  array.markers.reserve(visuals_.size());
  for (std::size_t i = 0; i < visuals_.size(); ++i) {
    const auto &v = visuals_[i];
    auto m = v.prototype;
    m.header.frame_id =
        s.header.frame_id; // Zero stamp: RViz uses latest inter-frame TF.
    m.ns = ns;
    m.id = static_cast<int>(i);
    const Eigen::Isometry3d t = poses.at(v.link) * v.origin;
    const Eigen::Quaterniond q(t.linear());
    m.pose.position.x = t.translation().x();
    m.pose.position.y = t.translation().y();
    m.pose.position.z = t.translation().z();
    m.pose.orientation.w = q.w();
    m.pose.orientation.x = q.x();
    m.pose.orientation.y = q.y();
    m.pose.orientation.z = q.z();
    if (!(style.use_urdf_materials && v.has_material)) {
      m.color.r = style.red;
      m.color.g = style.green;
      m.color.b = style.blue;
      m.color.a = 1;
    }
    m.color.a *= style.alpha;
    m.mesh_use_embedded_materials = style.use_embedded_materials;
    array.markers.push_back(std::move(m));
  }
  return array;
}
} // namespace wbmm::visualization
