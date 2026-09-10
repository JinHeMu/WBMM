#include "wbmm_visualization/whole_body_kinematics.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <utility>

namespace wbmm_viz
{

namespace
{
Eigen::Matrix3d rotationZ(double yaw)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  Eigen::Matrix3d result;
  result << c, -s, 0.0, s, c, 0.0, 0.0, 0.0, 1.0;
  return result;
}
}  // namespace

WholeBodyKinematics::WholeBodyKinematics(const std::string & urdf_file,
                                         const std::string & ee_frame)
: urdf_file_(urdf_file)
{
  pinocchio::urdf::buildModel(urdf_file_, model_);
  ee_frame_id_ = model_.getFrameId(ee_frame);
  if (ee_frame_id_ >= model_.frames.size()) {
    throw std::runtime_error("End-effector frame not found: " + ee_frame);
  }
  // Package dirs three levels above the URDF (urdf/.. = package root), same
  // convention as the planner packages.
  const std::vector<std::string> package_dirs{
    std::filesystem::path(urdf_file_).parent_path().parent_path()
    .parent_path().string()};
  pinocchio::urdf::buildGeom(
    model_, urdf_file_, pinocchio::GeometryType::VISUAL,
    visual_model_, package_dirs);
  pinocchio::urdf::buildGeom(
    model_, urdf_file_, pinocchio::GeometryType::COLLISION,
    collision_model_, package_dirs);
}

std::vector<VisualGeometry> WholeBodyKinematics::visualGeometry(
  const Eigen::VectorXd & state_9d) const
{
  pinocchio::Data data(model_);
  pinocchio::GeometryData geometry_data(visual_model_);
  pinocchio::forwardKinematics(model_, data, state_9d.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  pinocchio::updateGeometryPlacements(
    model_, data, visual_model_, geometry_data);
  const Eigen::Matrix3d base_rotation = rotationZ(state_9d[2]);
  const Eigen::Vector3d base_translation(state_9d[0], state_9d[1], 0.0);
  std::vector<VisualGeometry> result;
  result.reserve(visual_model_.geometryObjects.size());
  for (std::size_t index = 0;
    index < visual_model_.geometryObjects.size(); ++index)
  {
    const auto & geometry = visual_model_.geometryObjects[index];
    const auto & local = geometry_data.oMg[index];
    VisualGeometry visual;
    visual.name = geometry.name;
    visual.mesh_path = "file://" +
      std::filesystem::absolute(geometry.meshPath).string();
    visual.mesh_scale = geometry.meshScale;
    visual.color = geometry.meshColor;
    visual.position = base_translation +
      base_rotation * local.translation();
    visual.orientation = Eigen::Quaterniond(
      base_rotation * local.rotation());
    result.push_back(std::move(visual));
  }
  return result;
}

std::vector<CollisionSphereGeometry> WholeBodyKinematics::collisionSpheres(
  const Eigen::VectorXd & state_9d) const
{
  pinocchio::Data data(model_);
  pinocchio::GeometryData geometry_data(collision_model_);
  pinocchio::forwardKinematics(model_, data, state_9d.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  pinocchio::updateGeometryPlacements(
    model_, data, collision_model_, geometry_data);
  const Eigen::Matrix3d base_rotation = rotationZ(state_9d[2]);
  const Eigen::Vector3d base_translation(state_9d[0], state_9d[1], 0.0);
  std::vector<CollisionSphereGeometry> result;
  result.reserve(collision_model_.geometryObjects.size());
  for (std::size_t index = 0;
    index < collision_model_.geometryObjects.size(); ++index)
  {
    const auto & geometry = collision_model_.geometryObjects[index];
    const auto sphere = std::dynamic_pointer_cast<hpp::fcl::Sphere>(
      geometry.geometry);
    if (!sphere) {
      continue;
    }
    const auto & local = geometry_data.oMg[index];
    CollisionSphereGeometry value;
    value.name = geometry.name;
    value.center = base_translation +
      base_rotation * local.translation();
    value.radius = sphere->radius;
    value.parent_joint = geometry.parentJoint;
    result.push_back(std::move(value));
  }
  return result;
}

Eigen::Vector3d WholeBodyKinematics::framePosition(
  const Eigen::VectorXd & state_9d, const std::string & frame) const
{
  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, state_9d.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  const pinocchio::FrameIndex frame_id = model_.getFrameId(frame);
  const Eigen::Matrix3d base_rotation = rotationZ(state_9d[2]);
  return Eigen::Vector3d(state_9d[0], state_9d[1], 0.0) +
    base_rotation * data.oMf[frame_id].translation();
}

Eigen::Matrix3d WholeBodyKinematics::frameRotation(
  const Eigen::VectorXd & state_9d, const std::string & frame) const
{
  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, state_9d.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  const pinocchio::FrameIndex frame_id = model_.getFrameId(frame);
  return rotationZ(state_9d[2]) * data.oMf[frame_id].rotation();
}

Eigen::Vector3d WholeBodyKinematics::eePosition(
  const Eigen::VectorXd & state_9d) const
{
  return framePosition(state_9d, model_.frames[ee_frame_id_].name);
}

Eigen::Matrix3d WholeBodyKinematics::eeRotation(
  const Eigen::VectorXd & state_9d) const
{
  return frameRotation(state_9d, model_.frames[ee_frame_id_].name);
}

}  // namespace wbmm_viz
