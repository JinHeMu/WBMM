#include "wbmm_pinocchio/sphere_kinematics.hpp"

#include <stdexcept>
#include <string>
#include <utility>

namespace wbmm::pinocchio
{
namespace
{

Eigen::Matrix3d skew(const Eigen::Vector3d & vector)
{
  Eigen::Matrix3d matrix = Eigen::Matrix3d::Zero();
  matrix(0, 1) = -vector.z();
  matrix(0, 2) = vector.y();
  matrix(1, 0) = vector.z();
  matrix(1, 2) = -vector.x();
  matrix(2, 0) = -vector.y();
  matrix(2, 1) = vector.x();
  return matrix;
}

}  // namespace

SphereKinematics::SphereKinematics(
  KinematicModelPtr model, wbmm::robot_model::CollisionSphereModel spheres)
: model_(std::move(model)),
  spheres_(std::move(spheres)),
  frames_(model_)
{
  if (model_ == nullptr) {
    throw std::invalid_argument("SphereKinematics requires a KinematicModel.");
  }
  if (!spheres_.success) {
    throw std::invalid_argument(
      "SphereKinematics requires a valid CollisionSphereModel: " +
      spheres_.message);
  }

  const auto addGroup = [&](const wbmm::robot_model::CollisionSphereGroup & group) {
      for (const auto & sphere : group.spheres) {
        Entry entry;
        entry.id = sphere.id;
        entry.owner_link = sphere.owner_link;
        entry.radius = sphere.radius;
        entry.center_in_link = sphere.center_in_link;
        entry.group_tags = sphere.group_tags;
        if (!frames_.hasFrame(sphere.owner_link)) {
          throw std::invalid_argument(
            "Collision sphere '" + sphere.id + "' references link '" +
            sphere.owner_link + "' which is not in the Pinocchio model.");
        }
        entry.frame_id = model_->frameId(sphere.owner_link);
        entries_.push_back(std::move(entry));
      }
    };

  addGroup(spheres_.base);
  for (const auto & group : spheres_.arm) {
    addGroup(group);
  }
}

bool SphereKinematics::centers(
  const KinematicsData & kinematics_data, std::vector<SphereSample> & samples,
  std::string * message) const
{
  if (kinematics_data.model() != model_ || !kinematics_data.updated()) {
    if (message != nullptr) {
      *message = "KinematicsData is not updated for this KinematicModel.";
    }
    return false;
  }
  const auto & data = kinematics_data.data();
  samples.clear();
  samples.reserve(entries_.size());
  for (const auto & entry : entries_) {
    Eigen::Isometry3d root_pose = Eigen::Isometry3d::Identity();
    root_pose.translation() = data.oMf[entry.frame_id].translation();
    root_pose.linear() = data.oMf[entry.frame_id].rotation();
    const Eigen::Isometry3d world = kinematics_data.toStateFrame(root_pose);

    SphereSample sample;
    sample.id = entry.id;
    sample.owner_link = entry.owner_link;
    sample.radius = entry.radius;
    sample.group_tags = entry.group_tags;
    sample.center = world * entry.center_in_link;
    if (!sample.center.allFinite()) {
      if (message != nullptr) {
        *message = "Sphere '" + entry.id + "' produced a non-finite center.";
      }
      return false;
    }
    samples.push_back(std::move(sample));
  }
  return true;
}

bool SphereKinematics::jacobians(
  KinematicsData & kinematics_data, std::vector<Eigen::MatrixXd> & jacobians,
  std::string * message) const
{
  if (kinematics_data.model() != model_ || !kinematics_data.updated()) {
    if (message != nullptr) {
      *message = "KinematicsData is not updated for this KinematicModel.";
    }
    return false;
  }
  if (!kinematics_data.jacobiansReady()) {
    if (message != nullptr) {
      *message = "KinematicsData was not updated with kJacobians.";
    }
    return false;
  }

  const auto & data = kinematics_data.data();
  const Eigen::Index input_dimension =
    static_cast<Eigen::Index>(model_->inputDimension());
  jacobians.clear();
  jacobians.reserve(entries_.size());

  for (const auto & entry : entries_) {
    Eigen::MatrixXd frame_jacobian(6, input_dimension);
    std::string frame_message;
    if (!frames_.frameJacobian(
        kinematics_data, entry.owner_link, frame_jacobian, &frame_message))
    {
      if (message != nullptr) {
        *message = "Sphere '" + entry.id + "': " + frame_message;
      }
      return false;
    }

    Eigen::Isometry3d root_pose = Eigen::Isometry3d::Identity();
    root_pose.translation() = data.oMf[entry.frame_id].translation();
    root_pose.linear() = data.oMf[entry.frame_id].rotation();
    const Eigen::Isometry3d world = kinematics_data.toStateFrame(root_pose);
    const Eigen::Vector3d radius = world.linear() * entry.center_in_link;

    // v_p = v_o + omega x r  =>  J_p = J_v - skew(r) * J_omega.
    Eigen::MatrixXd sphere_jacobian =
      frame_jacobian.topRows<3>() - skew(radius) * frame_jacobian.bottomRows<3>();
    if (!sphere_jacobian.allFinite()) {
      if (message != nullptr) {
        *message = "Sphere '" + entry.id + "' produced a non-finite Jacobian.";
      }
      return false;
    }
    jacobians.push_back(std::move(sphere_jacobian));
  }
  return true;
}

}  // namespace wbmm::pinocchio
