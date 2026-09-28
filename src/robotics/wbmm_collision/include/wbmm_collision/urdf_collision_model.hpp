#pragma once

#include "wbmm_collision/collision_model.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace wbmm::collision
{

// One revolute joint resolved from the URDF.
//
// The transform convention matches the REMANI mm_config one that the whole-body
// optimizer relies on:
//
//   T_parent_child(theta) = origin * AngleAxis(theta, axis)
//
// i.e. origin is the joint placement in the PARENT link frame and axis is
// expressed in the joint frame. Sphere centres in arm[i] are expressed in
// child_link, so the world transform of group i is the ordered product
//   T_world_base * T_q0 * T_0 * T_1 * ... * T_i
// with T_q0 the fixed base-to-arm mount.
struct UrdfJointKinematics
{
  std::string name;
  std::string parent_link;
  std::string child_link;
  Eigen::Vector3d origin_translation{Eigen::Vector3d::Zero()};
  Eigen::Matrix3d origin_rotation{Eigen::Matrix3d::Identity()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
  double lower{0.0};
  double upper{0.0};
};

// Sphere approximation extracted from a URDF, ROS-free (tinyxml2, not urdf::Model).
//
// This is the piece wbmm_collision deliberately does not own: collision_model.hpp
// states that the caller supplies the robot-specific sphere approximation and
// that the package only evaluates it. This loader is that supplier.
struct UrdfCollisionModel
{
  bool success{false};
  std::string message;

  // URDF root link (the only link that is never a joint child) and the link the
  // base sphere group was taken from.
  std::string root_link;
  std::string base_collision_link;

  // Fixed-only chain from root_link to base_collision_link. The planner's base
  // pose (x, y, yaw) describes base_footprint, which for this robot sits below
  // base_link, so this offset must not be dropped.
  Eigen::Matrix4d root_from_base_collision{Eigen::Matrix4d::Identity()};

  // Fixed-only chain from base_collision_link to joints[0].parent_link: the arm
  // mount, REMANI's T_q_0_. Only this first hop is fixed; every later hop goes
  // through a movable joint, so the consumer composes
  //   T = root_from_base_collision
  //     * base_from_arm_mount
  //     * T_0(q0) * T_1(q1) * ... * T_i(qi)
  // with T_k(q) = origin_k * AngleAxis(q, axis_k), to reach arm[i]'s child link.
  Eigen::Matrix4d base_from_arm_mount{Eigen::Matrix4d::Identity()};

  // Spheres of base_collision_link, expressed in that link's frame.
  CollisionModel base;

  // arm[i] holds the spheres owned by joints[i], expressed in
  // joints[i].child_link. Same size as joints.
  std::vector<CollisionModel> arm;

  std::vector<UrdfJointKinematics> joints;

  // Largest sphere radius over all arm groups. REMANI uses this as
  // manipulator_thickness_, the fallback collision radius for a joint group
  // that carries no sphere.
  double max_arm_radius{0.0};

  [[nodiscard]] std::size_t armSphereCount() const;
  [[nodiscard]] std::size_t baseSphereCount() const {return base.spheres.size();}
};

// Parses urdf_path and builds the sphere model.
//
// joint_names selects and orders the movable joints; every name must exist and
// be revolute or continuous. base_collision_link selects the link whose spheres
// form the base footprint group.
//
// Grouping rule, replicated from REMANI mm_config so that downstream behaviour
// (base footprint size, arm sphere count) is unchanged:
//   arm[i] = every sphere rigidly attached to joints[i].child_link, including
//            its whole fixed subtree, PLUS for i == 0 the sphere of its parent
//            link when that centre lies on joint 0's axis. A sphere centred on
//            the rotation axis is invariant under that rotation, so folding it
//            in is geometrically exact; REMANI needs the fold because otherwise
//            the shoulder sphere would belong to no joint group at all.
[[nodiscard]] UrdfCollisionModel loadUrdfCollisionModel(
  const std::string & urdf_path,
  const std::vector<std::string> & joint_names,
  const std::string & base_collision_link);

}  // namespace wbmm::collision
