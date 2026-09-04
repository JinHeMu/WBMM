#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

namespace whole_body_force_control
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using AxisMask6d = std::array<bool, 6>;

// 将 source 原点处、source 坐标表达的 wrench 变换到 target 原点。
// target_to_source 是 source 原点在 target 坐标中的位置，包含力臂力矩。
Vector6d transformWrench(
  const Vector6d & source_wrench,
  const Eigen::Matrix3d & target_rotation_source,
  const Eigen::Vector3d & target_to_source);

// ============================================================================
// rateLimitedStep —— 参考速率限制(限速器)
//
//   作用:把 current 朝 target 挪动,但单步(一个控制周期 dt)最多走
//   |max_rate| * dt。这是"参考侧导纳"能安全接入 MPC 的关键环节:
//
//      MPC 跟踪的是修正后的参考轨迹,参考一旦跳变,MPC 会为了追上它产生
//      很大的输入(甚至超出关节速度限制)。限速保证导纳修正量以一条
//      连续、有界斜率的曲线进入优化器。
//
//   用途:ForceFollower::update 用它把"目标offset"限速逼近
//        (见 controllers.cpp)。
// ============================================================================
double rateLimitedStep(double current, double target, double max_rate, double dt);

// ============================================================================
// AdmittanceController —— 二阶导纳(质量-阻尼-弹簧 全动力模型)
//
//   被控对象是"MPC 参考在任务法线方向上的推深 offset x"(单位 m),力学模型:
//
//        M·ẍ + D·ẋ + K·x = F_measured − F_desired          (1)
//
//      F_measured : 单轴力/力矩读数，类内部还会低通滤波
//      F_desired  : 期望力
//      x (offset) : 修正位移,>0 表示朝推力方向"推深"进接触面
//      M          : 虚拟质量 [kg]    —— 惯量,平滑参考运动
//      D          : 虚拟阻尼 [N·s/m] —— 耗散能量,防止振荡;
//                 D=0 时就是无阻尼弹簧振子(会一直振荡,不可用)
//      K          : 虚拟刚度 [N/m]   —— 弹性回复,同时兼当旧实现的
//                 "leak(泄漏)"角色:F→0 时 −K·x 项把 offset 拉回 0
//
//   两种语义(由调用方传入 desired_force 决定):
//
//      * F_desired = 0 (admittance 模式,被动导纳)
//          机器人像被一根弹簧+阻尼器拖着,顺着接触力方向柔顺地让,
//          offset 与接触力成正比 —— 用于"柔顺接触/力感知",不主动顶力。
//
//      * F_desired > 0 (constant_force 模式,恒力调节)
//          恒力误差调节:力小了自动推深一点,力大了自动缩回一点,
//          稳态时 F → F_des。这是主链 YAML 里 desired_force: 12.0 的用途。
//
//   稳态平衡(ẍ=ẋ=0):  x* = (F − F_des) / K
//
//   仓库默认参数 M=3, D=45, K=150 → 阻尼比 ζ = D/(2·√(M·K)) ≈ 1.06,
//   在临界阻尼附近:响应快、基本不振荡、不会超调过大。
//
//   数值积分:Semi-implicit (symplectic) Euler —— 先用加速度更新速度,
//   再用新速度更新位置(见 controllers.cpp 对应 update)。
// ============================================================================
class AdmittanceController
{
public:
  AdmittanceController(double desired_force, double mass, double damping,
                       double stiffness, double max_offset,
                       double max_velocity, double filter_alpha,
                       bool clamp_nonnegative = true);
  // 每个控制周期调用一次:输入本轮实测力 → 输出本轮修正位移 offset(米)
  double update(double measured_force, double dt);
  // 复位:offset/速度清零,并用当前实测力初始化滤波器(避免复位后跳变)
  void reset(double measured_force = 0.0);

  double offset() const {return offset_;}
  double velocity() const {return velocity_;}
  double measuredForce() const {return filtered_force_;}
  double desiredForce() const {return desired_force_;}

private:
  double desired_force_;   // 期望力/力矩；旧标量模式钳位到 ≥ 0
  double mass_;            // 虚拟质量 [kg],钳位到 ≥ 1e-6(防止除零)
  double damping_;         // 虚拟阻尼 [N·s/m],钳位到 ≥ 0
  double stiffness_;       // 虚拟刚度 [N/m],钳位到 ≥ 0
  double max_offset_;      // offset 钳位幅值:安全边界,防止导纳把人"推穿"板面
  double max_velocity_;    // 修正速度钳位:参考连续性 —— MPC 只能跟上限速后的参考
  double alpha_;           // 力滤波系数(指数滑动平均):α=1 不过滤,α→0 越平滑
  double filtered_force_{0.0};  // 低通滤波后的力
  double offset_{0.0};          // 修正位移(导纳输出),单位 m
  double velocity_{0.0};        // 修正速度,单位 m/s
  bool initialized_{false};     // 首帧需用实测力初始化滤波器
  bool clamp_nonnegative_{true};  // 旧标量接触力用幅值;6D 模式保留符号
};

// ============================================================================
// ForceFollower —— 无阻尼准静态力跟随(比 AdmittanceController 更简单)
//
//   不做动力学积分,每个周期直接由静力平衡求目标位移:
//
//        x_target = (F_measured − F_desired) / K          (2)
//
//   然后用 rateLimitedStep 把当前 offset 朝目标限速移动。
//   没有质量/阻尼项,因此不存在振荡模态(公式 (2) 是公式 (1) 的稳态解,
//   直接把参考"瞬态"跳过去)。
//
//   语义参数与 AdmittanceController 相同,只是"D-free / 准静态假设":
//   适合缓慢的准静态接触修正;需要惯性平滑或更快的动力学行为时用
//   AdmittanceController。
// ============================================================================
class ForceFollower
{
public:
  ForceFollower(double desired_force, double stiffness, double max_offset,
                double max_velocity, double filter_alpha,
                bool clamp_nonnegative = true,
                bool velocity_mode = false,
                double force_deadband = 0.0);
  // 每个控制周期调用一次:输入本轮实测力 → 输出本轮修正位移 offset(米)
  double update(double measured_force, double dt);
  void reset(double measured_force = 0.0);
  double offset() const {return offset_;}
  double velocity() const {return velocity_;}
  double measuredForce() const {return filtered_force_;}

private:
  double desired_force_;   // 期望力 [N]
  double stiffness_;       // 虚拟刚度 [N/m],钳位到 ≥ 1e-6(防止除零)
  double max_offset_;      // offset 钳位幅值
  double max_velocity_;    // 充当"爬坡速率":决定修正响应的快慢
  double alpha_;           // 力滤波系数(同上方)
  double filtered_force_{0.0};
  double offset_{0.0};
  double velocity_{0.0};   // 后向差分估算,只用于观测/上报,不参与积分
  bool initialized_{false};
  bool clamp_nonnegative_{true};
  bool velocity_mode_{false};  // true=速度型无限力跟随,不再受 max_offset/刚度平衡点限制
  double force_deadband_{0.0}; // 速度型模式下的力死区
};

// 六轴笛卡尔顺应控制器。轴顺序固定为
// [Fx, Fy, Fz, Tx, Ty, Tz] -> [dx, dy, dz, rx, ry, rz]。
// 未选中的导纳轴始终输出 0；恒力掩码只改变被选导纳轴的期望 wrench。
class CartesianComplianceController
{
public:
  CartesianComplianceController(
    const AxisMask6d & admittance_axes,
    const AxisMask6d & constant_force_axes,
    const Vector6d & desired_wrench,
    const Vector6d & mass,
    const Vector6d & damping,
    const Vector6d & stiffness,
    const Vector6d & max_offset,
    const Vector6d & max_velocity,
    const Vector6d & filter_alpha,
    bool force_follow = false);

  Vector6d update(const Vector6d & measured_wrench, double dt);
  void reset(const Vector6d & measured_wrench = Vector6d::Zero());

  const Vector6d & offset() const {return offset_;}
  const Vector6d & velocity() const {return velocity_;}
  const Vector6d & measuredWrench() const {return filtered_wrench_;}
  const AxisMask6d & admittanceAxes() const {return admittance_axes_;}
  const AxisMask6d & constantForceAxes() const {return constant_force_axes_;}

private:
  AxisMask6d admittance_axes_{};
  AxisMask6d constant_force_axes_{};
  bool force_follow_{false};
  std::array<std::unique_ptr<AdmittanceController>, 6> admittance_;
  std::array<std::unique_ptr<ForceFollower>, 6> followers_;
  Vector6d offset_{Vector6d::Zero()};
  Vector6d velocity_{Vector6d::Zero()};
  Vector6d filtered_wrench_{Vector6d::Zero()};
};

}  // namespace whole_body_force_control
