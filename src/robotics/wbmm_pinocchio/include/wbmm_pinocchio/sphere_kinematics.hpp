#pragma once

// ============================================================================
// SphereKinematics —— 共享碰撞球模型在世界/状态系下的球心与球心 Jacobian。
//
//   球运动学本质上只是
//       p_i^W = T_{L_i}^W(x) * p_i^{L_i}
//   半径、所属 link、局部球心来自共享 CollisionSphereModel；运动学只负责
//   更新球心位置和导数。ESDF 查询、安全裕量、碰撞判定属于 wbmm_collision，
//   不在这里。
//
//   wbmm_collision 里基于 MINCO 轨迹变量的解析梯度适配器是另一条链：
//   它复用同一份球模型，但求导对象是 z = [x, y, vx, vy, q]，不是这里的
//   u = [v, omega, qdot]。
// ============================================================================

#include <wbmm_pinocchio/frame_kinematics.hpp>
#include <wbmm_pinocchio/kinematics_data.hpp>
#include <wbmm_pinocchio/kinematic_model.hpp>

#include <wbmm_robot_model/collision_sphere_model.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace wbmm::pinocchio
{

struct SphereSample
{
  std::string id;
  std::string owner_link;
  double radius{0.0};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  std::vector<std::string> group_tags;
};

class SphereKinematics
{
public:
  SphereKinematics(
    KinematicModelPtr model, wbmm::robot_model::CollisionSphereModel spheres);

  [[nodiscard]] const wbmm::robot_model::CollisionSphereModel & sphereModel()
  const noexcept {return spheres_;}

  [[nodiscard]] std::size_t sphereCount() const noexcept {return entries_.size();}

  // 全部球心，表达在 state.header.frame_id。
  [[nodiscard]] bool centers(
    const KinematicsData & kinematics_data, std::vector<SphereSample> & samples,
    std::string * message = nullptr) const;

  // 每个球心的解析 Jacobian：d(position)/du，3 x inputDimension。
  // 需要 kinematics_data 以 kJacobians 更新过。
  [[nodiscard]] bool jacobians(
    KinematicsData & kinematics_data, std::vector<Eigen::MatrixXd> & jacobians,
    std::string * message = nullptr) const;

private:
  struct Entry
  {
    std::string id;
    std::string owner_link;
    double radius{0.0};
    Eigen::Vector3d center_in_link{Eigen::Vector3d::Zero()};
    std::vector<std::string> group_tags;
    ::pinocchio::FrameIndex frame_id{0};
  };

  KinematicModelPtr model_;
  wbmm::robot_model::CollisionSphereModel spheres_;
  FrameKinematics frames_;
  std::vector<Entry> entries_;
};

}  // namespace wbmm::pinocchio
