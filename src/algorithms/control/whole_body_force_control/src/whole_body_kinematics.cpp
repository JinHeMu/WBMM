#include "whole_body_force_control/whole_body_kinematics.hpp"

#include <Eigen/Cholesky>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace whole_body_force_control
{
namespace
{

// 绕 Z 轴的 2D 旋转矩阵(底盘 yaw 用):
// 世界系 ↔ 底盘/机械臂基座局部系 的水平旋转。
Eigen::Matrix3d rotationZ(double yaw)
{
  Eigen::Matrix3d rotation;
  rotation << std::cos(yaw), -std::sin(yaw), 0.0,
    std::sin(yaw), std::cos(yaw), 0.0,
    0.0, 0.0, 1.0;
  return rotation;
}
}  // namespace

WholeBodyKinematics::WholeBodyKinematics(
  const std::string & urdf_file, const std::string & ee_frame)
{
  // pinocchio 模型根 = URDF 根 link(机械臂基座)。注意:整个运动学处理中
  // 模型是"静止基座"的 —— 底盘的 x/y/yaw 不在模型里,而在 state 里,
  // 由本类在模型外手工叠加(见 correctedState / framePosition 注释)。
  pinocchio::urdf::buildModel(urdf_file, model_);
  ee_frame_id_ = model_.getFrameId(ee_frame);
  if (ee_frame_id_ >= model_.frames.size()) {
    throw std::runtime_error("End-effector frame not found: " + ee_frame);
  }
}

// ----------------------------------------------------------------------------
// 核心:把标量修正位移分配到底盘与机械臂,返回修正后的 9D 状态。
//
//   输入语义:
//     state[3 + nq]  : 名义(未修正)9D 状态
//     world_direction: 世界系单位向量,修正位移沿它的方向(法线方向)
//     displacement   : 位移量(米)；>0 = 沿该方向推进(推深)
//
//   步骤:
//     A. 底盘份额:只取位移在 heading 上的投影 × base_share
//        (差速底盘无侧滑 → 只走航向),再限幅 ±max_base_delta；
//        yaw 不变(修正不转车头)。更新 corrected 的 [x, y]。
//     B. 剩余位移(desired − 底盘份额)交给机械臂:
//        旋转到机械臂基座局部系(为在 FK 坐标系里做数值迭代)；
//     C. 在局部系里迭代求解关节增量:把 EE 位置推到 target_position,
//        用位置雅可比的阻尼伪逆、每步限幅、关节限位约束。
//        ("迭代 + 限幅"保证小位移下 1~3 次迭代即收敛,且修正平滑。)
// ----------------------------------------------------------------------------
Eigen::VectorXd WholeBodyKinematics::correctedState(
  const Eigen::VectorXd & state, const Eigen::Vector3d & world_direction,
  double displacement, double base_share, double max_base_delta,
  double max_joint_delta) const
{
  // 防御:状态维度/数值合法性检查;方向向量退化(零向量)时直接返回原状态
  // (没有方向就没有修正,类行为是"no-op"而不是崩溃)。
  if (state.size() != stateDimension()) {
    throw std::invalid_argument(
      "Whole-body correction expects base pose plus every URDF arm joint");
  }
  if (!state.allFinite() || !world_direction.allFinite() ||
      !std::isfinite(displacement))
  {
    throw std::invalid_argument("Whole-body correction input is non-finite");
  }
  Eigen::Vector3d direction = world_direction;
  if (direction.norm() < 1.0e-9) {
    return state;
  }
  direction.normalize();
  base_share = std::clamp(base_share, 0.0, 1.0);   // 0=纯臂修正,1=最多底盘
  max_base_delta = std::abs(max_base_delta);
  max_joint_delta = std::abs(max_joint_delta);

  // ---- A. 底盘份额 ---------------------------------------------------------
  const Eigen::Vector3d desired_world_displacement = direction * displacement;
  const Eigen::Vector2d heading(std::cos(state[2]), std::sin(state[2]));
  // 只取世界系位移在航向上的投影(侧向分量底盘吃不下 → 全留给臂)
  const double requested_base_distance = base_share *
    heading.dot(desired_world_displacement.head<2>());
  // 限幅:单周期底盘位移不超过 max_base_delta(防大起大落,保参考连续性)
  const double base_distance = std::clamp(
    requested_base_distance, -max_base_delta, max_base_delta);
  // 把航向距离转回世界系位移,叠加到 base_x/base_y(yaw 不动)
  const Eigen::Vector2d base_displacement = heading * base_distance;

  // ---- B. 剩余位移 → 机械臂局部系 ------------------------------------------
  Eigen::VectorXd corrected = state;
  corrected.head<2>() += base_displacement;
  Eigen::Vector3d remaining_world_displacement = desired_world_displacement;
  remaining_world_displacement.head<2>() -= base_displacement;
  // Rz(−yaw) 把世界系位移转到基座局部系:pinocchio 的 FK/Jacobian 都
  // 在局部系里算,位移必须同坐标系才可比。
  const Eigen::Vector3d local_displacement =
    rotationZ(-state[2]) * remaining_world_displacement;

  // ---- C. 关节空间迭代求解(damped pseudo-inverse,最多 10 次) --------------
  pinocchio::Data data(model_);
  const Eigen::VectorXd nominal_q = state.tail(model_.nq);
  Eigen::VectorXd q = nominal_q;
  pinocchio::forwardKinematics(model_, data, nominal_q);
  pinocchio::updateFramePlacements(model_, data);
  // 目标位置 = 名义 EE 位置 + 局部系剩余位移
  const Eigen::Vector3d target_position =
    data.oMf[ee_frame_id_].translation() + local_displacement;

  for (int iteration = 0; iteration < 10; ++iteration) {
    pinocchio::forwardKinematics(model_, data, q);
    pinocchio::updateFramePlacements(model_, data);
    // 当前 EE 位置与目标的偏差(局部系)
    const Eigen::Vector3d error =
      target_position - data.oMf[ee_frame_id_].translation();
    if (error.norm() < 1.0e-5) {
      break;  // 已收敛(1e-5 m ≈ 0.01 mm,远小于控制精度)
    }
    // 只取 3×nq 的位置雅可比(法线位移修正不要求控制 EE 姿态:
    // 任务姿态由规划器保证,tool0 +Z 始终指向板法线)
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data, q, ee_frame_id_,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
    const Eigen::MatrixXd position_jacobian = jacobian.topRows(3);
    // δq = Jᵀ (J Jᵀ + λI)⁻¹ e —— 阻尼最小二乘(λ=1e-5)
    //   · J Jᵀ 奇异/病态时(接近奇异位形)λI 正则化,修正发散;
    //   · 6 轴臂 + 3 个位置约束 = 冗余系统,Jᵀ(...)⁻¹ 自动选最小范数解
    const Eigen::VectorXd delta = position_jacobian.transpose() *
      (position_jacobian * position_jacobian.transpose() +
       1.0e-5 * Eigen::Matrix3d::Identity()).ldlt().solve(error);
    // 限幅:相对名义构型的单步增量不超过 max_joint_delta(参考平滑、
    // 防止奇异附近大步长),并夹到关节限位内
    const Eigen::VectorXd bounded_delta =
      (q + delta - nominal_q).cwiseMax(-max_joint_delta)
      .cwiseMin(max_joint_delta);
    q = (nominal_q + bounded_delta)
      .cwiseMax(model_.lowerPositionLimit).cwiseMin(model_.upperPositionLimit);
  }
  corrected.tail(model_.nq) = q;
  return corrected;
}

Eigen::VectorXd WholeBodyKinematics::correctedState6D(
  const Eigen::VectorXd & state,
  const Eigen::Matrix<double, 6, 1> & local_correction,
  double base_share, double max_base_delta,
  double max_joint_delta) const
{
  if (state.size() != stateDimension()) {
    throw std::invalid_argument(
      "6D whole-body correction expects base pose plus every URDF arm joint");
  }
  if (!state.allFinite() || !local_correction.allFinite()) {
    throw std::invalid_argument("6D whole-body correction input is non-finite");
  }
  base_share = std::clamp(base_share, 0.0, 1.0);
  max_base_delta = std::abs(max_base_delta);
  max_joint_delta = std::abs(max_joint_delta);

  pinocchio::Data data(model_);
  const Eigen::VectorXd nominal_q = state.tail(model_.nq);
  pinocchio::forwardKinematics(model_, data, nominal_q);
  pinocchio::updateFramePlacements(model_, data);
  const auto nominal_placement = data.oMf[ee_frame_id_];

  // correction 的前三维沿名义 EE 自身轴定义，先变换到机械臂基座系，
  // 再变换到世界系，供差速底盘提取可实现的航向分量。
  const Eigen::Vector3d desired_local_translation =
    nominal_placement.rotation() * local_correction.head<3>();
  const Eigen::Vector3d desired_world_translation =
    rotationZ(state[2]) * desired_local_translation;
  const Eigen::Vector2d heading(std::cos(state[2]), std::sin(state[2]));
  const double requested_base_distance = base_share *
    heading.dot(desired_world_translation.head<2>());
  const double base_distance = std::clamp(
    requested_base_distance, -max_base_delta, max_base_delta);
  const Eigen::Vector2d base_displacement = heading * base_distance;

  Eigen::VectorXd corrected = state;
  corrected.head<2>() += base_displacement;
  Eigen::Vector3d remaining_local_translation = desired_local_translation;
  remaining_local_translation.head<2>() -=
    (rotationZ(-state[2]) *
    Eigen::Vector3d(base_displacement.x(), base_displacement.y(), 0.0)).head<2>();
  const Eigen::Vector3d target_position =
    nominal_placement.translation() + remaining_local_translation;
  const Eigen::Matrix3d target_rotation = nominal_placement.rotation() *
    pinocchio::exp3(local_correction.tail<3>());

  Eigen::VectorXd q = nominal_q;
  for (int iteration = 0; iteration < 20; ++iteration) {
    pinocchio::forwardKinematics(model_, data, q);
    pinocchio::updateFramePlacements(model_, data);
    const auto & current = data.oMf[ee_frame_id_];
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = target_position - current.translation();
    // LOCAL_WORLD_ALIGNED Jacobian 的角速度在基座坐标表达。
    error.tail<3>() = pinocchio::log3(target_rotation * current.rotation().transpose());
    if (error.head<3>().norm() < 1.0e-5 && error.tail<3>().norm() < 1.0e-5) {
      break;
    }
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data, q, ee_frame_id_,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
    const Eigen::VectorXd delta = jacobian.transpose() *
      (jacobian * jacobian.transpose() +
      1.0e-5 * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(error);
    const Eigen::VectorXd bounded_delta =
      (q + delta - nominal_q).cwiseMax(-max_joint_delta)
      .cwiseMin(max_joint_delta);
    q = (nominal_q + bounded_delta)
      .cwiseMax(model_.lowerPositionLimit).cwiseMin(model_.upperPositionLimit);
  }
  corrected.tail(model_.nq) = q;
  return corrected;
}

// 世界系 EE 位置:模型局部 FK 结果 + 底盘 [x, y, Rz(yaw)] 变换。
// 供 node.cpp 计算"实际 EE 沿响应方向的位移",与参考修正量对拍。
Eigen::Vector3d WholeBodyKinematics::framePosition(
  const Eigen::VectorXd & state) const
{
  if (state.size() != stateDimension()) {
    throw std::invalid_argument(
      "Frame position expects base pose plus every URDF arm joint");
  }
  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, state.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  const Eigen::Vector3d local = data.oMf[ee_frame_id_].translation();  // 基座系
  Eigen::Vector3d world = rotationZ(state[2]) * local;                 // 转世界系
  world.x() += state[0];
  world.y() += state[1];
  return world;
}

Eigen::Matrix3d WholeBodyKinematics::frameRotation(
  const Eigen::VectorXd & state) const
{
  if (state.size() != stateDimension()) {
    throw std::invalid_argument(
      "Frame rotation expects base pose plus every URDF arm joint");
  }
  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, state.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  return rotationZ(state[2]) * data.oMf[ee_frame_id_].rotation();
}

}  // namespace whole_body_force_control
