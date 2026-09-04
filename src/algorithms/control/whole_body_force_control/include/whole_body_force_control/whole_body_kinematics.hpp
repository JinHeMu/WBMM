#pragma once

#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>

#include <string>

namespace whole_body_force_control
{

// ============================================================================
// WholeBodyKinematics —— 修正量的全身运动学分配器
//
//   导纳外环算出的"1 维法线位移 offset(标量,米)"怎么变成 9D 参考状态
//   x = [base_x, base_y, base_yaw, q1..q6] 的修正?由这个类负责。
//
//   核心思想(注释与实现见 whole_body_kinematics.cpp):
//
//     1. 底盘只承担"沿自身航向"的分量(base_share 占比):
//        Tracer 是差速底盘,无侧滑约束 —— 不能横向平移,
//        所以把世界系位移先投影到 heading,再截断到 max_base_delta。
//        注意:yaw 本身从不被修正,只有 x/y 平移。
//
//     2. 剩余的位移(desired − base 份额)反解到关节空间:
//        旋转到机械臂基座局部系,对 EE 位置用 damped pseudo-inverse
//        (Jᵀ(JJᵀ + λI)⁻¹)迭代求解关节增量,每步增量限幅
//        max_joint_delta,并夹到关节限位内。
//
//     pinocchio 模型根 = 机械臂基座(URDF),state 里的 base_x/y/yaw
//     在模型之外用 [x, y, Rz(yaw)] 手工叠加(见 framePosition)。
//
//   为什么这样分:力控只修正法线方向一个自由度,但真实系统只有一个
//   6 轴臂,推深需求可能超过臂长/奇异位形,适度借底盘沿航向的前移
//   来分担(差速底盘绕不过去法线方向的分量由臂承担)。
// ============================================================================
class WholeBodyKinematics
{
public:
  WholeBodyKinematics(const std::string & urdf_file,
                      const std::string & ee_frame);
  // 把 (state, world_direction, displacement) 修正后的 9D 状态;
  // world_direction 是世界系单位方向(如法线响应方向),displacement 是
  // 该方向上的位移量(米,由导纳/力跟随输出)。
  Eigen::VectorXd correctedState(
    const Eigen::VectorXd & state, const Eigen::Vector3d & world_direction,
    double displacement, double base_share, double max_base_delta,
    double max_joint_delta) const;
  // 六轴修正量按名义末端局部系解释：前三维是平移[m]，后三维是旋转向量[rad]。
  // 底盘仍只分担平移在自身航向上的分量；转动修正全部由机械臂完成。
  Eigen::VectorXd correctedState6D(
    const Eigen::VectorXd & state,
    const Eigen::Matrix<double, 6, 1> & local_correction,
    double base_share, double max_base_delta,
    double max_joint_delta) const;
  // 计算给定 9D 状态下 EE(tool0)的世界系位置,用于状态上报/调试
  // (node.cpp 用它算实际 EE 位移,评估修正是否到位)。
  Eigen::Vector3d framePosition(const Eigen::VectorXd & state) const;
  Eigen::Matrix3d frameRotation(const Eigen::VectorXd & state) const;
  int stateDimension() const {return model_.nq + 3;}  // 3(底盘) + 臂关节
  int armDimension() const {return model_.nq;}

private:
  pinocchio::Model model_;      // URDF 建出的运动学模型(臂部分)
  pinocchio::FrameIndex ee_frame_id_;  // EE 帧的索引,通常 "tool0"
};

}  // namespace whole_body_force_control
