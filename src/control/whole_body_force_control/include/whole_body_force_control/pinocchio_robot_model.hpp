#pragma once

#include <wbmm_core/wbmm_core.hpp>

#include <pinocchio/multibody/model.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace whole_body_force_control
{

// ============================================================================
// PinocchioRobotModel —— wbmm::core::RobotModel 的第一版具体实现。
//
//   职责边界：
//   - 只实现 core 合同要求的 FK、frame Jacobian、关节命名/限位和校验；
//   - 不包含力控、轨迹、ROS 或安全逻辑；
//   - 是当前仓库中唯一直接理解 URDF / Pinocchio 的运动学适配器。
//
//   坐标系约定(与旧 WholeBodyKinematics 手工叠加方式完全一致)：
//   - URDF 模型根为机械臂基座(固定 base)，Pinocchio FK/Jacobian 在基座系；
//   - state 的 [x, y, yaw] 是基座在世界系中的位姿；
//   - FK 返回结果按 state.header.frame_id 表达：
//         p_world = Rz(yaw) * p_model + [x, y, 0]
//         R_world = Rz(yaw) * R_model
//   - Jacobian 满足 V = J(x) * u，u = [v, omega, qdot...]：
//       列 0 (v)     : 差速底盘沿自身航向的线速度；
//       列 1 (omega) : 绕世界 Z 的底盘角速度；
//       列 2..       : 关节速度，经 Rz(yaw) 旋转到世界系。
// ============================================================================
class PinocchioRobotModel final : public wbmm::core::RobotModel
{
public:
  // max_base_speed / max_base_yaw_rate <= 0 表示"尚未配置"，
  // limits() 会原样返回，具体限速由上层 profile 决定。
  explicit PinocchioRobotModel(
    const std::string & urdf_file,
    double max_base_speed = 0.0,
    double max_base_yaw_rate = 0.0);

  [[nodiscard]] std::size_t stateDimension() const override;
  [[nodiscard]] std::size_t inputDimension() const override;
  [[nodiscard]] wbmm::core::BaseModel baseModel() const override;
  [[nodiscard]] const std::vector<std::string> & jointNames() const override;
  [[nodiscard]] const wbmm::core::RobotLimits & limits() const override;

  [[nodiscard]] bool hasFrame(const std::string & frame_name) const;

  bool forwardKinematics(
    const wbmm::core::WholeBodyState & state,
    const std::string & link_name,
    wbmm::core::Pose & pose) const override;

  bool frameJacobian(
    const wbmm::core::WholeBodyState & state,
    const std::string & link_name,
    Eigen::Ref<Eigen::MatrixXd> jacobian) const override;

  bool validate(
    const wbmm::core::WholeBodyState & state,
    std::string * message = nullptr) const override;

  bool validate(
    const wbmm::core::WholeBodyInput & input,
    std::string * message = nullptr) const override;

private:
  // 按关节名称(而不是数组顺序)把 state 映射到 Pinocchio 配置向量 q。
  bool stateToConfiguration(
    const wbmm::core::WholeBodyState & state,
    Eigen::VectorXd & configuration,
    std::string * message) const;

  bool sameJointSet(
    const std::vector<std::string> & names,
    std::string * message) const;

  pinocchio::Model model_;
  std::vector<std::string> joint_names_;
  std::vector<Eigen::Index> joint_position_indices_;
  std::vector<Eigen::Index> joint_velocity_indices_;
  wbmm::core::RobotLimits limits_;
};

}  // namespace whole_body_force_control
