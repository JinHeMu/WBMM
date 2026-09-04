#include "whole_body_force_control/controllers.hpp"

#include <stdexcept>

namespace whole_body_force_control
{

Vector6d transformWrench(
  const Vector6d & source_wrench,
  const Eigen::Matrix3d & target_rotation_source,
  const Eigen::Vector3d & target_to_source)
{
  if (!source_wrench.allFinite() || !target_rotation_source.allFinite() ||
      !target_to_source.allFinite())
  {
    throw std::invalid_argument("wrench transform input is non-finite");
  }
  Vector6d target;
  target.head<3>() = target_rotation_source * source_wrench.head<3>();
  target.tail<3>() = target_rotation_source * source_wrench.tail<3>() +
    target_to_source.cross(target.head<3>());
  return target;
}

// ----------------------------------------------------------------------------
// 限速器实现:把 current 往 target 挪,但每步最多移动 |max_rate|·dt。
//
// 防御性检查:任一输入非有限(NaN/Inf),直接返回 current —— 即使上游传入
// 坏数据,参考也绝不会跳变(MPC 只接受连续参考,参考跳变=MPC 大输入)。
// dt 上限 0.05 s:若主循环被调度打乱(headless MuJoCo、系统负载波动),
// 单步移动量也不会突然放大,参考保持平滑。
// ----------------------------------------------------------------------------
double rateLimitedStep(double current, double target, double max_rate, double dt)
{
  if (!std::isfinite(current) || !std::isfinite(target) ||
      !std::isfinite(max_rate) || !std::isfinite(dt))
  {
    return current;
  }
  const double max_step = std::abs(max_rate) * std::clamp(dt, 0.0, 0.05);
  return current + std::clamp(target - current, -max_step, max_step);
}

// 构造函数:所有参数先"消毒"(钳位到物理合法区间)。
// 这样 update() 无需反复判断参数合法性,只专注状态积分;
// 非法配置(如 mass=0)在这里被兜住,而不是运行期炸掉。
AdmittanceController::AdmittanceController(
  double desired_force, double mass, double damping, double stiffness,
  double max_offset, double max_velocity, double filter_alpha,
  bool clamp_nonnegative)
: desired_force_(clamp_nonnegative ? std::max(0.0, desired_force) : desired_force),
  mass_(std::max(1.0e-6, mass)),
  damping_(std::max(0.0, damping)), stiffness_(std::max(0.0, stiffness)),
  max_offset_(std::abs(max_offset)), max_velocity_(std::abs(max_velocity)),
  alpha_(std::clamp(filter_alpha, 0.0, 1.0)),
  clamp_nonnegative_(clamp_nonnegative)
{}

// ----------------------------------------------------------------------------
// 导纳模型的一步积分(Semi-implicit Euler,也称 symplectic Euler):
//
//   1. 力滤波(指数滑动平均,一阶低通)
//        F_fil ← α·F + (1−α)·F_fil
//      首帧用实测力直接初始化滤波器,避免从 0 起步产生的暂态冲击。
//      α 取自 YAML filter_alpha(默认 0.25):越小越平滑、响应越慢。
//
//   2. 动力学积分(公式: M·ẍ + D·ẋ + K·x = F_fil − F_des)
//        a = (F_fil − F_des − D·v − K·x) / M
//        v ← clamp(v + dt·a, ±max_velocity)
//        x ← clamp(x + dt·v, ±max_offset)
//      Semi-implicit 与显式 Euler 的区别:先用"新"速度更新位置。
//      对保守系统(无阻尼)，不会出现显式 Euler 的能量发散,接触/弹簧
//      仿真常用这种格式;配合 D 略大的参数(仓库默认阻尼比≈1.06,
//      临界阻尼附近)整体响应平滑不振荡。
//
//   3. 触限处理:offset 顶到 ±max_offset 时把速度清零 —— 相当于边界上
//      的"非弹性碰撞"。防止速度在边界附近积累,导致持续的来回抖动。
//
//   钳位优先级 安全 > 精度:即使力误差很大,单周期修正量也被
//   max_velocity 与 max_offset 严格限制 —— 这保证注入 MPC 的参考
//   永远不会让机器人急动(参考侧导纳安全性的核心)。
// ----------------------------------------------------------------------------
double AdmittanceController::update(double measured_force, double dt)
{
  // 旧标量调用只理解"压进表面"的非负幅值；6D 包装器关闭该钳位，
  // 保留每个力/力矩轴的符号。
  if (clamp_nonnegative_) {
    measured_force = std::max(0.0, measured_force);
  }
  if (!initialized_) {
    filtered_force_ = measured_force;  // 首帧:直接用测量初始化,无暂态
    initialized_ = true;
  } else {
    filtered_force_ = alpha_ * measured_force + (1.0 - alpha_) * filtered_force_;
  }
  dt = std::clamp(dt, 0.0, 0.05);  // 调度抖动防御(同 rateLimitedStep)
  const double acceleration =
    (filtered_force_ - desired_force_ - damping_ * velocity_ -
     stiffness_ * offset_) / mass_;
  velocity_ = std::clamp(
    velocity_ + dt * acceleration, -max_velocity_, max_velocity_);
  const double previous_offset = offset_;
  offset_ = std::clamp(offset_ + dt * velocity_, -max_offset_, max_offset_);
  // offset 撞到钳位边界:清掉朝边界的速度分量("非弹性碰撞"),
  // 避免速度积累后在边界来回抖
  if ((offset_ >= max_offset_ && velocity_ > 0.0) ||
      (offset_ <= -max_offset_ && velocity_ < 0.0))
  {
    velocity_ = 0.0;
  }
  // dt ≤ 0(时钟异常):按上面公式 offset_ 本来就不变;这里显式回写,
  // 是防御未来改动破坏"异常时钟下状态冻结"这条不变式。
  if (dt <= 0.0) {
    offset_ = previous_offset;
  }
  return offset_;
}

void AdmittanceController::reset(double measured_force)
{
  filtered_force_ = clamp_nonnegative_ ? std::max(0.0, measured_force) : measured_force;
  offset_ = 0.0;
  velocity_ = 0.0;
  initialized_ = true;
}

ForceFollower::ForceFollower(
  double desired_force, double stiffness, double max_offset,
  double max_velocity, double filter_alpha, bool clamp_nonnegative,
  bool velocity_mode, double force_deadband)
: desired_force_(clamp_nonnegative ? std::max(0.0, desired_force) : desired_force),
  stiffness_(std::max(1.0e-6, stiffness)),
  max_offset_(std::abs(max_offset)), max_velocity_(std::abs(max_velocity)),
  alpha_(std::clamp(filter_alpha, 0.0, 1.0)),
  clamp_nonnegative_(clamp_nonnegative),
  velocity_mode_(velocity_mode),
  force_deadband_(std::abs(force_deadband))
{}

// ----------------------------------------------------------------------------
// 准静态力跟随的一步:
//
//   1. 同样的力滤波(首帧初始化)。
//   2. 静力平衡:    x_target = (F_fil − F_des) / K
//      含义:要把刚度 K 的弹簧压出 F_fil − F_des 牛的力需要多大形变。
//      与 AdmittanceController 的稳态解相同,但每个周期直接朝目标逼近,
//      不带质量/阻尼,因此不存在振荡模态。
//   3. 限速逼近:    offset ← rateLimitedStep(offset, x_target, max_velocity)
//      max_velocity 在这里充当"爬坡速率",决定修正的响应快慢。
//   4. velocity_ 用后向差分估算,仅供状态发布/观测,不参与积分。
// ----------------------------------------------------------------------------
double ForceFollower::update(double measured_force, double dt)
{
  if (clamp_nonnegative_) {
    measured_force = std::max(0.0, measured_force);
  }
  if (!initialized_) {
    filtered_force_ = measured_force;  // 首帧直接采用,避免暂态
    initialized_ = true;
  } else {
    filtered_force_ = alpha_ * measured_force + (1.0 - alpha_) * filtered_force_;
  }
  dt = std::clamp(dt, 0.0, 0.05);

  if (velocity_mode_) {
    // 速度型“无限”力跟随：不追求 F/K 的有限平衡点，也不因 max_offset 停止。
    // 有符号力误差超过死区时，offset 以 max_velocity 向力的方向持续积分；
    // 撤力后速度回到 0，机器人停在当前位置（不会弹回名义点）。
    const double error = filtered_force_ - desired_force_;
    double command_velocity = 0.0;
    if (error > force_deadband_) {
      command_velocity = max_velocity_;
    } else if (error < -force_deadband_) {
      command_velocity = -max_velocity_;
    }
    offset_ += command_velocity * dt;
    velocity_ = command_velocity;
    return offset_;
  }

  const double target_offset = std::clamp(
    (filtered_force_ - desired_force_) / stiffness_,
    -max_offset_, max_offset_);
  const double previous_offset = offset_;
  offset_ = rateLimitedStep(offset_, target_offset, max_velocity_, dt);
  velocity_ = dt > 1.0e-9 ? (offset_ - previous_offset) / dt : 0.0;
  return offset_;
}

void ForceFollower::reset(double measured_force)
{
  filtered_force_ = clamp_nonnegative_ ? std::max(0.0, measured_force) : measured_force;
  offset_ = 0.0;
  velocity_ = 0.0;
  initialized_ = true;
}

CartesianComplianceController::CartesianComplianceController(
  const AxisMask6d & admittance_axes,
  const AxisMask6d & constant_force_axes,
  const Vector6d & desired_wrench,
  const Vector6d & mass,
  const Vector6d & damping,
  const Vector6d & stiffness,
  const Vector6d & max_offset,
  const Vector6d & max_velocity,
  const Vector6d & filter_alpha,
  bool force_follow)
: admittance_axes_(admittance_axes),
  constant_force_axes_(constant_force_axes),
  force_follow_(force_follow)
{
  for (std::size_t i = 0; i < 6; ++i) {
    if (constant_force_axes_[i] && !admittance_axes_[i]) {
      throw std::invalid_argument("constant-force axes must be enabled admittance axes");
    }
    const double desired = constant_force_axes_[i] ? desired_wrench[i] : 0.0;
    admittance_[i] = std::make_unique<AdmittanceController>(
      desired, mass[i], damping[i], stiffness[i], max_offset[i],
      max_velocity[i], filter_alpha[i], false);
    followers_[i] = std::make_unique<ForceFollower>(
      desired, stiffness[i], max_offset[i], max_velocity[i],
      filter_alpha[i], false);
  }
}

Vector6d CartesianComplianceController::update(
  const Vector6d & measured_wrench, double dt)
{
  for (std::size_t i = 0; i < 6; ++i) {
    if (!admittance_axes_[i]) {
      admittance_[i]->reset(measured_wrench[i]);
      followers_[i]->reset(measured_wrench[i]);
      offset_[i] = 0.0;
      velocity_[i] = 0.0;
      filtered_wrench_[i] = measured_wrench[i];
      continue;
    }
    if (force_follow_) {
      offset_[i] = followers_[i]->update(measured_wrench[i], dt);
      velocity_[i] = followers_[i]->velocity();
      filtered_wrench_[i] = followers_[i]->measuredForce();
    } else {
      offset_[i] = admittance_[i]->update(measured_wrench[i], dt);
      velocity_[i] = admittance_[i]->velocity();
      filtered_wrench_[i] = admittance_[i]->measuredForce();
    }
  }
  return offset_;
}

void CartesianComplianceController::reset(const Vector6d & measured_wrench)
{
  for (std::size_t i = 0; i < 6; ++i) {
    admittance_[i]->reset(measured_wrench[i]);
    followers_[i]->reset(measured_wrench[i]);
  }
  offset_.setZero();
  velocity_.setZero();
  filtered_wrench_ = measured_wrench;
}

}  // namespace whole_body_force_control
