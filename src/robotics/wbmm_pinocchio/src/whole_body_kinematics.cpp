#include "wbmm_pinocchio/whole_body_kinematics.hpp"

#include "wbmm_pinocchio/pinocchio_robot_model.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>

namespace wbmm::pinocchio
{
namespace
{

Eigen::Vector3d positionOf(const wbmm::core::Vector3 & position)
{
  return {position.x, position.y, position.z};
}

Eigen::Matrix3d rotationOf(const wbmm::core::Quaternion & quaternion)
{
  const Eigen::Quaterniond eigen_quaternion(
    quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  if (eigen_quaternion.norm() < 1.0e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return eigen_quaternion.normalized().toRotationMatrix();
}

Eigen::Matrix3d exp3(const Eigen::Vector3d & rotation_vector)
{
  const double angle = rotation_vector.norm();
  if (angle < 1.0e-12) {
    return Eigen::Matrix3d::Identity();
  }
  return Eigen::AngleAxisd(angle, rotation_vector / angle).toRotationMatrix();
}

Eigen::Vector3d log3(const Eigen::Matrix3d & rotation)
{
  const Eigen::AngleAxisd angle_axis(rotation);
  if (!std::isfinite(angle_axis.angle())) {
    return Eigen::Vector3d::Zero();
  }
  return angle_axis.angle() * angle_axis.axis();
}

}  // namespace

WholeBodyKinematics::WholeBodyKinematics(
  wbmm::core::RobotModelPtr model, std::string ee_frame, std::string frame_id,
  wbmm::core::ClockDomain clock)
: model_(std::move(model)),
  ee_frame_(std::move(ee_frame)),
  frame_id_(std::move(frame_id)),
  clock_(clock)
{
  if (!model_) {
    throw std::invalid_argument("WholeBodyKinematics requires a RobotModel");
  }
  if (ee_frame_.empty()) {
    throw std::invalid_argument("WholeBodyKinematics requires a non-empty ee_frame");
  }
  if (frame_id_.empty()) {
    throw std::invalid_argument("WholeBodyKinematics requires a non-empty frame_id");
  }
  if (stateDimension() < 4) {
    throw std::invalid_argument(
            "WholeBodyKinematics requires base pose plus arm joints");
  }

  // 用零状态检查 frame 是否存在；
  // 具体机器人校验仍由 RobotModel::validate 负责。
  wbmm::core::Pose pose;
  if (!framePose(makeState(Eigen::VectorXd::Zero(stateDimension())), pose)) {
    throw std::runtime_error("End-effector frame not found: " + ee_frame_);
  }
}

WholeBodyKinematics::WholeBodyKinematics(
  const std::string & urdf_file, const std::string & ee_frame)
: WholeBodyKinematics(
    std::make_shared<PinocchioRobotModel>(urdf_file), ee_frame)
{}

int WholeBodyKinematics::stateDimension() const
{
  return static_cast<int>(model_->stateDimension());
}

int WholeBodyKinematics::armDimension() const
{
  return stateDimension() - 3;
}

void WholeBodyKinematics::requireStateDimension(const Eigen::VectorXd & state) const
{
  if (state.size() != stateDimension()) {
    throw std::invalid_argument(
            "Whole-body correction expects base pose plus every robot joint");
  }
}

wbmm::core::WholeBodyState WholeBodyKinematics::makeState(
  const Eigen::Ref<const Eigen::VectorXd> & state) const
{
  wbmm::core::WholeBodyState result;
  result.header.frame_id = frame_id_;
  result.header.stamp = 0.0;
  result.header.clock = clock_;
  result.base_model = model_->baseModel();
  result.base.x = state[0];
  result.base.y = state[1];
  result.base.yaw = state[2];

  const int arm_dimension = armDimension();
  result.joints.names = model_->jointNames();
  result.joints.positions.resize(static_cast<std::size_t>(arm_dimension));
  for (int i = 0; i < arm_dimension; ++i) {
    result.joints.positions[static_cast<std::size_t>(i)] = state[3 + i];
  }
  return result;
}

bool WholeBodyKinematics::framePose(
  const wbmm::core::WholeBodyState & state, wbmm::core::Pose & pose) const
{
  return model_->forwardKinematics(state, ee_frame_, pose);
}

Eigen::VectorXd WholeBodyKinematics::boundedJointStep(
  const Eigen::VectorXd & nominal_q, const Eigen::VectorXd & current_q,
  const Eigen::VectorXd & delta, double max_joint_delta) const
{
  Eigen::VectorXd next = nominal_q +
    (current_q + delta - nominal_q)
    .cwiseMax(-max_joint_delta)
    .cwiseMin(max_joint_delta);

  const auto & limits = model_->limits();
  if (limits.joint_min.size() == static_cast<std::size_t>(next.size()) &&
    limits.joint_max.size() == static_cast<std::size_t>(next.size()))
  {
    const Eigen::Map<const Eigen::VectorXd> lower(
      limits.joint_min.data(), static_cast<Eigen::Index>(limits.joint_min.size()));
    const Eigen::Map<const Eigen::VectorXd> upper(
      limits.joint_max.data(), static_cast<Eigen::Index>(limits.joint_max.size()));
    next = next.cwiseMax(lower).cwiseMin(upper);
  }
  return next;
}

// ----------------------------------------------------------------------------
// 核心:把标量修正位移分配到底盘与机械臂,返回修正后的 9D 状态。
//
//   输入语义:
//     state[3 + nq]  : 名义(未修正)9D 状态
//     world_direction: 状态 frame 下的单位方向向量,修正位移沿它的方向
//     displacement   : 位移量(米)；>0 = 沿该方向推进(推深)
//
//   步骤:
//     A. 底盘份额:只取位移在 heading 上的投影 × base_share
//        (差速底盘无侧滑 → 只走航向),再限幅 ±max_base_delta；yaw 不变。
//     B. 目标位姿:末端最终目标 = 名义位置 + 完整期望位移;
//        底盘分担后,机械臂在移动后的基座下解算剩余关节修正。
//     C. 在状态 frame 下迭代求解关节增量:用 RobotModel::frameJacobian
//        的平移行做阻尼最小二乘,每步限幅并夹到关节限位。
// ----------------------------------------------------------------------------
Eigen::VectorXd WholeBodyKinematics::correctedState(
  const Eigen::VectorXd & state, const Eigen::Vector3d & world_direction,
  double displacement, double base_share, double max_base_delta,
  double max_joint_delta) const
{
  requireStateDimension(state);
  if (!state.allFinite() || !world_direction.allFinite() ||
    !std::isfinite(displacement) || !std::isfinite(base_share) ||
    !std::isfinite(max_base_delta) || !std::isfinite(max_joint_delta))
  {
    throw std::invalid_argument("Whole-body correction input is non-finite");
  }
  Eigen::Vector3d direction = world_direction;
  if (direction.norm() < 1.0e-9) {
    return state;
  }
  direction.normalize();
  base_share = std::clamp(base_share, 0.0, 1.0);
  max_base_delta = std::abs(max_base_delta);
  max_joint_delta = std::abs(max_joint_delta);

  const wbmm::core::WholeBodyState nominal_state = makeState(state);
  wbmm::core::Pose nominal_pose;
  if (!framePose(nominal_state, nominal_pose)) {
    throw std::runtime_error("Whole-body correction FK failed for " + ee_frame_);
  }
  const Eigen::Vector3d nominal_position = positionOf(nominal_pose.position);

  // ---- A. 底盘份额 ---------------------------------------------------------
  const Eigen::Vector3d desired_world_displacement = direction * displacement;
  const Eigen::Vector2d heading(std::cos(state[2]), std::sin(state[2]));
  const double requested_base_distance = base_share *
    heading.dot(desired_world_displacement.head<2>());
  const double base_distance = std::clamp(
    requested_base_distance, -max_base_delta, max_base_delta);
  const Eigen::Vector2d base_displacement = heading * base_distance;

  // ---- B. 目标位姿 ---------------------------------------------------------
  // 底盘已经承担 base_displacement，末端最终目标仍必须是
  // "名义位置 + 完整期望位移"；机械臂在底盘移动后的新基座下
  // 解算剩余的关节修正。
  Eigen::VectorXd corrected = state;
  corrected.head<2>() += base_displacement;
  const Eigen::Vector3d target_position =
    nominal_position + desired_world_displacement;

  // ---- C. 关节空间迭代求解(damped pseudo-inverse,最多 10 次) ---------------
  const int arm_dimension = armDimension();
  const Eigen::Index base_input_columns =
    static_cast<Eigen::Index>(model_->inputDimension()) - arm_dimension;
  const Eigen::VectorXd nominal_q = state.tail(arm_dimension);
  Eigen::VectorXd q = nominal_q;
  wbmm::core::WholeBodyState candidate = makeState(corrected);

  for (int iteration = 0; iteration < 10; ++iteration) {
    candidate.joints.positions.assign(q.data(), q.data() + q.size());
    wbmm::core::Pose pose;
    if (!framePose(candidate, pose)) {
      throw std::runtime_error("Whole-body correction FK failed for " + ee_frame_);
    }
    const Eigen::Vector3d error = target_position - positionOf(pose.position);
    if (error.norm() < 1.0e-5) {
      break;
    }

    Eigen::MatrixXd jacobian(
      6, static_cast<Eigen::Index>(model_->inputDimension()));
    if (!model_->frameJacobian(candidate, ee_frame_, jacobian)) {
      throw std::runtime_error(
              "Whole-body correction Jacobian failed for " + ee_frame_);
    }
    const Eigen::MatrixXd position_jacobian =
      jacobian.middleCols(base_input_columns, arm_dimension).topRows(3);
    const Eigen::VectorXd delta = position_jacobian.transpose() *
      (position_jacobian * position_jacobian.transpose() +
      1.0e-5 * Eigen::Matrix3d::Identity()).ldlt().solve(error);
    q = boundedJointStep(nominal_q, q, delta, max_joint_delta);
  }
  corrected.tail(arm_dimension) = q;
  return corrected;
}

Eigen::VectorXd WholeBodyKinematics::correctedState6D(
  const Eigen::VectorXd & state,
  const Eigen::Matrix<double, 6, 1> & local_correction,
  double base_share, double max_base_delta, double max_joint_delta) const
{
  requireStateDimension(state);
  if (!state.allFinite() || !local_correction.allFinite() ||
    !std::isfinite(base_share) || !std::isfinite(max_base_delta) ||
    !std::isfinite(max_joint_delta))
  {
    throw std::invalid_argument("6D whole-body correction input is non-finite");
  }
  base_share = std::clamp(base_share, 0.0, 1.0);
  max_base_delta = std::abs(max_base_delta);
  max_joint_delta = std::abs(max_joint_delta);

  const wbmm::core::WholeBodyState nominal_state = makeState(state);
  wbmm::core::Pose nominal_pose;
  if (!framePose(nominal_state, nominal_pose)) {
    throw std::runtime_error("6D whole-body correction FK failed for " + ee_frame_);
  }
  const Eigen::Vector3d nominal_position = positionOf(nominal_pose.position);
  const Eigen::Matrix3d nominal_rotation = rotationOf(nominal_pose.orientation);

  // correction 前三维沿名义 EE 自身轴定义，先变换到状态 frame。
  const Eigen::Vector3d desired_world_translation =
    nominal_rotation * local_correction.head<3>();
  const Eigen::Vector2d heading(std::cos(state[2]), std::sin(state[2]));
  const double requested_base_distance = base_share *
    heading.dot(desired_world_translation.head<2>());
  const double base_distance = std::clamp(
    requested_base_distance, -max_base_delta, max_base_delta);
  const Eigen::Vector2d base_displacement = heading * base_distance;

  Eigen::VectorXd corrected = state;
  corrected.head<2>() += base_displacement;
  const Eigen::Vector3d target_position =
    nominal_position + desired_world_translation;
  const Eigen::Matrix3d target_rotation =
    nominal_rotation * exp3(local_correction.tail<3>());

  const int arm_dimension = armDimension();
  const Eigen::Index base_input_columns =
    static_cast<Eigen::Index>(model_->inputDimension()) - arm_dimension;
  const Eigen::VectorXd nominal_q = state.tail(arm_dimension);
  Eigen::VectorXd q = nominal_q;
  wbmm::core::WholeBodyState candidate = makeState(corrected);

  for (int iteration = 0; iteration < 20; ++iteration) {
    candidate.joints.positions.assign(q.data(), q.data() + q.size());
    wbmm::core::Pose pose;
    if (!framePose(candidate, pose)) {
      throw std::runtime_error(
              "6D whole-body correction FK failed for " + ee_frame_);
    }
    const Eigen::Matrix3d current_rotation = rotationOf(pose.orientation);
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = target_position - positionOf(pose.position);
    error.tail<3>() = log3(target_rotation * current_rotation.transpose());
    if (error.head<3>().norm() < 1.0e-5 && error.tail<3>().norm() < 1.0e-5) {
      break;
    }

    Eigen::MatrixXd jacobian(
      6, static_cast<Eigen::Index>(model_->inputDimension()));
    if (!model_->frameJacobian(candidate, ee_frame_, jacobian)) {
      throw std::runtime_error(
              "6D whole-body correction Jacobian failed for " + ee_frame_);
    }
    const Eigen::MatrixXd pose_jacobian =
      jacobian.middleCols(base_input_columns, arm_dimension);
    const Eigen::Matrix<double, 6, 1> delta = pose_jacobian.transpose() *
      (pose_jacobian * pose_jacobian.transpose() +
      1.0e-5 * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(error);
    q = boundedJointStep(nominal_q, q, delta, max_joint_delta);
  }
  corrected.tail(arm_dimension) = q;
  return corrected;
}

Eigen::VectorXd WholeBodyKinematics::correctedStateWorld6D(
  const Eigen::VectorXd & state,
  const Eigen::Matrix<double, 6, 1> & world_correction,
  double base_share, double max_base_delta, double max_joint_delta) const
{
  requireStateDimension(state);
  if (!state.allFinite() || !world_correction.allFinite() ||
    !std::isfinite(base_share) || !std::isfinite(max_base_delta) ||
    !std::isfinite(max_joint_delta))
  {
    throw std::invalid_argument(
            "world-frame whole-body correction input is non-finite");
  }
  base_share = std::clamp(base_share, 0.0, 1.0);
  max_base_delta = std::abs(max_base_delta);
  max_joint_delta = std::abs(max_joint_delta);

  const wbmm::core::WholeBodyState nominal_state = makeState(state);
  wbmm::core::Pose nominal_pose;
  if (!framePose(nominal_state, nominal_pose)) {
    throw std::runtime_error(
            "world-frame whole-body correction FK failed for " + ee_frame_);
  }
  const Eigen::Vector3d nominal_position = positionOf(nominal_pose.position);
  const Eigen::Matrix3d nominal_rotation = rotationOf(nominal_pose.orientation);

  const Eigen::Vector3d desired_world_translation =
    world_correction.head<3>();
  const Eigen::Vector2d heading(std::cos(state[2]), std::sin(state[2]));
  const double requested_base_distance = base_share *
    heading.dot(desired_world_translation.head<2>());
  const double base_distance = std::clamp(
    requested_base_distance, -max_base_delta, max_base_delta);
  const Eigen::Vector2d base_displacement = heading * base_distance;

  Eigen::VectorXd corrected = state;
  corrected.head<2>() += base_displacement;
  const Eigen::Vector3d target_position =
    nominal_position + desired_world_translation;
  const Eigen::Matrix3d target_rotation =
    exp3(world_correction.tail<3>()) * nominal_rotation;

  const int arm_dimension = armDimension();
  const Eigen::Index base_input_columns =
    static_cast<Eigen::Index>(model_->inputDimension()) - arm_dimension;
  const Eigen::VectorXd nominal_q = state.tail(arm_dimension);
  Eigen::VectorXd q = nominal_q;
  wbmm::core::WholeBodyState candidate = makeState(corrected);

  for (int iteration = 0; iteration < 20; ++iteration) {
    candidate.joints.positions.assign(q.data(), q.data() + q.size());
    wbmm::core::Pose pose;
    if (!framePose(candidate, pose)) {
      throw std::runtime_error(
              "world-frame whole-body correction FK failed for " + ee_frame_);
    }
    const Eigen::Matrix3d current_rotation = rotationOf(pose.orientation);
    Eigen::Matrix<double, 6, 1> error;
    error.head<3>() = target_position - positionOf(pose.position);
    error.tail<3>() = log3(target_rotation * current_rotation.transpose());
    if (error.head<3>().norm() < 1.0e-5 && error.tail<3>().norm() < 1.0e-5) {
      break;
    }

    Eigen::MatrixXd jacobian(
      6, static_cast<Eigen::Index>(model_->inputDimension()));
    if (!model_->frameJacobian(candidate, ee_frame_, jacobian)) {
      throw std::runtime_error(
              "world-frame whole-body correction Jacobian failed for " + ee_frame_);
    }
    const Eigen::MatrixXd pose_jacobian =
      jacobian.middleCols(base_input_columns, arm_dimension);
    const Eigen::Matrix<double, 6, 1> delta = pose_jacobian.transpose() *
      (pose_jacobian * pose_jacobian.transpose() +
      1.0e-5 * Eigen::Matrix<double, 6, 6>::Identity()).ldlt().solve(error);
    q = boundedJointStep(nominal_q, q, delta, max_joint_delta);
  }
  corrected.tail(arm_dimension) = q;
  return corrected;
}

Eigen::Vector3d WholeBodyKinematics::framePosition(
  const Eigen::VectorXd & state) const
{
  requireStateDimension(state);
  wbmm::core::Pose pose;
  if (!framePose(makeState(state), pose)) {
    throw std::runtime_error("Frame position failed for " + ee_frame_);
  }
  return positionOf(pose.position);
}

Eigen::Matrix3d WholeBodyKinematics::frameRotation(
  const Eigen::VectorXd & state) const
{
  requireStateDimension(state);
  wbmm::core::Pose pose;
  if (!framePose(makeState(state), pose)) {
    throw std::runtime_error("Frame rotation failed for " + ee_frame_);
  }
  return rotationOf(pose.orientation);
}

}  // namespace wbmm::pinocchio
