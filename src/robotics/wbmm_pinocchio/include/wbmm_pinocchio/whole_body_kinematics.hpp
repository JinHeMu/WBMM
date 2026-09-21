#pragma once

#include <wbmm_core/robot_model.hpp>

#include <Eigen/Core>

#include <string>

namespace wbmm::pinocchio
{

// ============================================================================
// WholeBodyKinematics —— 修正量的全身运动学分配器
//
//   导纳外环算出的"1 维法线位移 offset(标量,米)"怎么变成 9D 参考状态
//   x = [base_x, base_y, base_yaw, q1..q6] 的修正?由这个类负责。
//
//   本类只依赖 wbmm::core::RobotModel 合同(FK + frame Jacobian + 限位)，
//   不再直接解析 URDF、也不再直接调用 Pinocchio。具体机器人由 RobotModel
//   实现提供(当前为 PinocchioRobotModel)。
//
//   分配规则(与旧实现保持一致)：
//
//     1. 底盘只承担"沿自身航向"的分量(base_share 占比):
//        Tracer 是差速底盘,无侧滑约束 —— 不能横向平移,
//        所以把世界系位移先投影到 heading,再截断到 max_base_delta。
//        注意:yaw 本身从不被修正,只有 x/y 平移。
//
//     2. 剩余的位移(desired − base 份额)反解到关节空间:
//        在状态 frame 下对 EE 位置/姿态做阻尼最小二乘迭代,
//        每步增量限幅 max_joint_delta,并夹到 RobotModel 关节限位内。
// ============================================================================
class WholeBodyKinematics
{
public:
  WholeBodyKinematics(
    wbmm::core::RobotModelPtr model, std::string ee_frame,
    std::string frame_id = "odom");

  // 兼容旧调用方的便捷构造函数：内部构造 PinocchioRobotModel。
  // 新代码应优先传入 RobotModel，以便替换运动学后端。
  WholeBodyKinematics(const std::string & urdf_file, const std::string & ee_frame);

  // 把 (state, world_direction, displacement) 修正后的 9D 状态;
  // world_direction 是世界系单位方向(如法线响应方向),displacement 是
  // 该方向上的位移量(米,由导纳/力跟随输出)。
  [[nodiscard]] Eigen::VectorXd correctedState(
    const Eigen::VectorXd & state, const Eigen::Vector3d & world_direction,
    double displacement, double base_share, double max_base_delta,
    double max_joint_delta) const;

  // 六轴修正量按名义末端局部系解释：
  // 前三维是平移[m]，后三维是旋转向量[rad]。
  // 底盘仍只分担平移在自身航向上的分量；转动修正全部由机械臂完成。
  [[nodiscard]] Eigen::VectorXd correctedState6D(
    const Eigen::VectorXd & state,
    const Eigen::Matrix<double, 6, 1> & local_correction,
    double base_share, double max_base_delta,
    double max_joint_delta) const;

  // 六轴修正量按状态 frame(world/odom)解释：
  // 前三维是状态系平移[m]，后三维是绕状态系轴的旋转向量[rad]。
  // 旋转修正在名义姿态左侧作用：R_target = exp(world_rotation) * R_nominal。
  // 不设置人为底盘/关节相对偏移上限；关节只受 RobotModel 硬限位和 IK 可达性约束。
  [[nodiscard]] Eigen::VectorXd correctedStateWorld6D(
    const Eigen::VectorXd & state,
    const Eigen::Matrix<double, 6, 1> & world_correction,
    double base_share) const;

  // 计算给定 9D 状态下 EE 在该状态 frame 中的位置/姿态,用于状态上报/调试
  // (node.cpp 用它算实际 EE 位移,评估修正是否到位)。
  [[nodiscard]] Eigen::Vector3d framePosition(const Eigen::VectorXd & state) const;
  [[nodiscard]] Eigen::Matrix3d frameRotation(const Eigen::VectorXd & state) const;

  [[nodiscard]] int stateDimension() const;
  [[nodiscard]] int armDimension() const;

private:
  [[nodiscard]] wbmm::core::WholeBodyState makeState(
    const Eigen::Ref<const Eigen::VectorXd> & state) const;
  [[nodiscard]] bool framePose(
    const wbmm::core::WholeBodyState & state,
    wbmm::core::Pose & pose) const;
  [[nodiscard]] Eigen::VectorXd boundedJointStep(
    const Eigen::VectorXd & nominal_q, const Eigen::VectorXd & current_q,
    const Eigen::VectorXd & delta, double max_joint_delta) const;
  void requireStateDimension(const Eigen::VectorXd & state) const;

  wbmm::core::RobotModelPtr model_;
  std::string ee_frame_;
  std::string frame_id_;
};

}  // namespace wbmm::pinocchio
