#include <wbmm_visualization/robot_visual_model.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <set>
#include <stdexcept>

namespace wbmm::visualization
{
namespace
{

Eigen::Isometry3d transform(const Eigen::Isometry3d & origin)
{
  if (!origin.matrix().allFinite()) {
    throw std::invalid_argument("Invalid URDF origin.");
  }
  return origin;
}

void positive(double value)
{
  if (!std::isfinite(value) || value <= 0) {
    throw std::invalid_argument(
        "URDF visual scale must be positive and finite.");
  }
}

void colorValue(double value)
{
  if (!std::isfinite(value) || value < 0 || value > 1) {
    throw std::invalid_argument("RGBA must be in [0,1].");
  }
}

std::string resolveMeshResource(
  const std::string & filename, const std::string & directory)
{
  if (filename.empty()) {
    throw std::invalid_argument("Empty URDF mesh filename.");
  }
  if (filename.find("://") != std::string::npos) {
    return filename;
  }
  std::filesystem::path path(filename);
  if (path.is_relative()) {
    if (directory.empty()) {
      throw std::invalid_argument(
          "Relative mesh needs urdf_file or mesh_directory.");
    }
    path = std::filesystem::path(directory) / path;
  }
  return "file://" + std::filesystem::absolute(path).lexically_normal().string();
}

}  // namespace

RobotVisualModel::RobotVisualModel(
  std::shared_ptr<const wbmm::robot_model::RobotDescription> description,
  wbmm::robot_model::RobotModelConfig config, std::string mesh_directory)
: kinematic_(wbmm::pinocchio::KinematicModel::create(
    std::move(description), std::move(config))),
  frames_(kinematic_),
  kinematics_data_(kinematic_)
{
  const auto & links = kinematic_->description().links;
  // Stable link/name ordering makes marker ids deterministic across updates.
  for (const auto & link : links) {
    for (const auto & visual : link.visuals) {
      Visual entry;
      entry.link = link.name;
      entry.origin = transform(visual.origin);
      auto & marker = entry.prototype;
      marker.action = marker.ADD;
      marker.pose.orientation.w = 1;
      marker.scale.x = marker.scale.y = marker.scale.z = 1;
      switch (visual.geometry.type) {
        case wbmm::robot_model::GeometryType::kMesh:
          marker.type = marker.MESH_RESOURCE;
          marker.mesh_resource = resolveMeshResource(
            visual.geometry.mesh_filename, mesh_directory);
          marker.scale.x = visual.geometry.mesh_scale.x();
          marker.scale.y = visual.geometry.mesh_scale.y();
          marker.scale.z = visual.geometry.mesh_scale.z();
          break;
        case wbmm::robot_model::GeometryType::kBox:
          marker.type = marker.CUBE;
          marker.scale.x = visual.geometry.size.x();
          marker.scale.y = visual.geometry.size.y();
          marker.scale.z = visual.geometry.size.z();
          break;
        case wbmm::robot_model::GeometryType::kCylinder:
          marker.type = marker.CYLINDER;
          marker.scale.x = marker.scale.y = 2 * visual.geometry.radius;
          marker.scale.z = visual.geometry.length;
          break;
        case wbmm::robot_model::GeometryType::kSphere:
          marker.type = marker.SPHERE;
          marker.scale.x = marker.scale.y = marker.scale.z =
            2 * visual.geometry.radius;
          break;
        default:
          throw std::invalid_argument("Unsupported URDF visual geometry.");
      }
      positive(marker.scale.x);
      positive(marker.scale.y);
      positive(marker.scale.z);
      if (visual.has_color) {
        entry.has_material = true;
        marker.color.r = visual.color.x();
        marker.color.g = visual.color.y();
        marker.color.b = visual.color.z();
        marker.color.a = visual.color.w();
        for (double value : {marker.color.r, marker.color.g, marker.color.b,
                             marker.color.a})
          colorValue(value);
      }
      visuals_.push_back(std::move(entry));
    }
  }
  if (visuals_.empty()) {
    throw std::invalid_argument(
        "URDF has no visual geometry (collision geometry is not used).");
  }
}

bool RobotVisualModel::hasLink(const std::string & name) const
{
  return kinematic_->description().findLink(name) != nullptr;
}

void RobotVisualModel::validateState(
  const wbmm::core::WholeBodyState & state) const
{
  if (state.header.frame_id.empty() || !std::isfinite(state.base.x) ||
    !std::isfinite(state.base.y) || !std::isfinite(state.base.yaw))
  {
    throw std::invalid_argument(
        "Invalid visual state frame/base/joint dimensions.");
  }
  const auto & controlled = kinematic_->controlledJointNames();
  if (state.joints.names.size() != controlled.size() ||
    state.joints.positions.size() != controlled.size())
  {
    throw std::invalid_argument(
        "Invalid visual state frame/base/joint dimensions.");
  }
  std::set<std::string> seen;
  for (std::size_t j = 0; j < state.joints.names.size(); ++j) {
    if (!std::isfinite(state.joints.positions[j]) ||
      !seen.insert(state.joints.names[j]).second ||
      std::find(controlled.begin(), controlled.end(), state.joints.names[j]) ==
      controlled.end())
    {
      throw std::invalid_argument(
          "Visual state has non-finite positions or wrong joint names.");
    }
  }
}

std::map<std::string, Eigen::Isometry3d> RobotVisualModel::linkPoses(
  const wbmm::core::WholeBodyState & state) const
{
  validateState(state);
  if (!kinematics_data_.update(state)) {
    throw std::invalid_argument(
        "Visual forward kinematics failed: " + kinematics_data_.lastMessage());
  }
  return frames_.linkPlacements(kinematics_data_);
}

visualization_msgs::msg::MarkerArray RobotVisualModel::markers(
  const wbmm::core::WholeBodyState & state, const std::string & ns,
  const VisualStyle & style) const
{
  for (double c : {style.red, style.green, style.blue, style.alpha}) {
    colorValue(c);
  }
  const auto poses = linkPoses(state);
  visualization_msgs::msg::MarkerArray array;
  array.markers.reserve(visuals_.size());
  for (std::size_t i = 0; i < visuals_.size(); ++i) {
    const auto & visual = visuals_[i];
    auto marker = visual.prototype;
    marker.header.frame_id = state.header.frame_id;
    marker.ns = ns;
    marker.id = static_cast<int>(i);
    const auto found = poses.find(visual.link);
    if (found == poses.end()) {
      continue;
    }
    const Eigen::Isometry3d pose = found->second * visual.origin;
    const Eigen::Quaterniond q(pose.linear());
    marker.pose.position.x = pose.translation().x();
    marker.pose.position.y = pose.translation().y();
    marker.pose.position.z = pose.translation().z();
    marker.pose.orientation.w = q.w();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    if (!(style.use_urdf_materials && visual.has_material)) {
      marker.color.r = style.red;
      marker.color.g = style.green;
      marker.color.b = style.blue;
      marker.color.a = 1;
    }
    marker.color.a *= style.alpha;
    marker.mesh_use_embedded_materials = style.use_embedded_materials;
    array.markers.push_back(std::move(marker));
  }
  return array;
}

}  // namespace wbmm::visualization
