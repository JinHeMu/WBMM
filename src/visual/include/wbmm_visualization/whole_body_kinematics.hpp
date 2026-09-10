#pragma once

#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>

namespace wbmm_viz
{

// World-frame visual geometry of one URDF body. The state convention used
// everywhere in this workspace is 9D:
//   [base_x, base_y, base_yaw, joint_1 .. joint_6]
// and the world pose of every geometry is the URDF local placement composed
// with the mobile-base rotation R(yaw) and translation (x, y, 0). All poses
// returned here are already baked into the world frame.
struct VisualGeometry
{
  std::string name;
  std::string mesh_path;  // "file://" + absolute mesh path
  Eigen::Vector3d mesh_scale{Eigen::Vector3d::Ones()};
  Eigen::Vector4d color{0.9, 0.9, 0.9, 1.0};  // URDF material (pinocchio meshColor)
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct CollisionSphereGeometry
{
  std::string name;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
  pinocchio::JointIndex parent_joint{0};
};

// Canonical URDF-based whole-body kinematics used for visualization.
// Replaces the two near-identical planner implementations of the same FK
// convention (TA-WBMP planner.cpp visualGeometry/collisionSpheres, WipePlanner
// planner.cpp visualGeometry).
class WholeBodyKinematics
{
public:
  WholeBodyKinematics(const std::string & urdf_file,
                      const std::string & ee_frame = "tool0");

  // World-frame visual meshes for a 9D state; the arm part is state.tail(nq).
  std::vector<VisualGeometry> visualGeometry(const Eigen::VectorXd & state_9d) const;
  std::vector<CollisionSphereGeometry> collisionSpheres(
    const Eigen::VectorXd & state_9d) const;
  // FK of an arbitrary URDF frame, world frame, base transform baked in.
  Eigen::Vector3d framePosition(const Eigen::VectorXd & state_9d,
                                const std::string & frame) const;
  Eigen::Matrix3d frameRotation(const Eigen::VectorXd & state_9d,
                                const std::string & frame) const;
  // Convenience wrappers for the configured end-effector frame.
  Eigen::Vector3d eePosition(const Eigen::VectorXd & state_9d) const;
  Eigen::Matrix3d eeRotation(const Eigen::VectorXd & state_9d) const;

  int armDof() const {return model_.nq;}
  const std::string & urdfFile() const {return urdf_file_;}

private:
  std::string urdf_file_;
  pinocchio::Model model_;
  pinocchio::GeometryModel visual_model_;
  pinocchio::GeometryModel collision_model_;
  pinocchio::FrameIndex ee_frame_id_{0};
};

}  // namespace wbmm_viz
