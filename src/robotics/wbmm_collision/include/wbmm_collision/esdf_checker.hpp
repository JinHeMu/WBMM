#pragma once

#include "wbmm_collision/check_types.hpp"
#include "wbmm_environment/esdf_grid.hpp"
#include "wbmm_pinocchio/kinematic_model.hpp"
#include "wbmm_pinocchio/kinematics_data.hpp"
#include "wbmm_pinocchio/sphere_kinematics.hpp"
#include "wbmm_robot_model/collision_sphere_model.hpp"

#include <Eigen/Core>

#include <cstddef>
#include <memory>
#include <string>

namespace wbmm::collision
{

// 环境碰撞检查：几何来自共享 CollisionSphereModel，运动学来自共享
// KinematicModel。
//
// 一次 check/checkBase 只做一次 forwardKinematics：KinematicsData 更新一次，
// SphereKinematics 批量给出全部球心，然后逐球查询 ESDF。不再为每个球所属
// link 重算整树。
//
// 本类持有可变的 KinematicsData，和 PinocchioRobotModel 一样不是线程安全
// 的：一个实例只在一个线程里使用。
class EsdfChecker
{
public:
  EsdfChecker(
    wbmm::pinocchio::KinematicModelPtr kinematic_model,
    std::shared_ptr<const wbmm::environment::EsdfGrid> environment,
    const wbmm::robot_model::CollisionSphereModel & collision_model,
    CollisionCheckOptions options = {});

  // Matches the data available in wbmm::search::BaseCollisionChecker.
  // Base geometry only; it does not certify the arm's navigation configuration.
  [[nodiscard]] CollisionResult checkBase(
    const wbmm::core::Header & header, const wbmm::core::BaseState & base) const;

  // Environment only. No self-collision or continuous trajectory checks.
  [[nodiscard]] CollisionResult check(
    const wbmm::core::WholeBodyState & state,
    CheckScope scope = CheckScope::kWholeBody) const;

  [[nodiscard]] std::size_t sphereCount() const noexcept;

private:
  [[nodiscard]] CollisionResult checkWithScope(
    const wbmm::core::WholeBodyState & state, CheckScope scope,
    bool validate_state) const;

  wbmm::pinocchio::KinematicModelPtr model_;
  std::shared_ptr<const wbmm::environment::EsdfGrid> environment_;
  // null 表示球模型为空或非法；此时 check 返回 kInvalidInput。
  std::unique_ptr<wbmm::pinocchio::SphereKinematics> sphere_kinematics_;
  // 一次查询只更新一次；变化的数据不跨实例共享。
  mutable std::unique_ptr<wbmm::pinocchio::KinematicsData> kinematics_data_;
  CollisionCheckOptions options_;
  std::string model_message_;
};

}  // namespace wbmm::collision
