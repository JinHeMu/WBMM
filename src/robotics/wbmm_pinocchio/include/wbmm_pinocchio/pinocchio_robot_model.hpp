#pragma once

#include <wbmm_pinocchio/frame_kinematics.hpp>
#include <wbmm_pinocchio/kinematic_model.hpp>
#include <wbmm_pinocchio/kinematics_data.hpp>

#include <wbmm_core/wbmm_core.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace wbmm::pinocchio
{

// ============================================================================
// PinocchioRobotModel —— wbmm::core::RobotModel 的具体实现。
//
//   第一版就落在这里的合同：底盘 + 受控关节的维度、关节命名/顺序、限位、
//   FK 和 frame Jacobian。它不是"另一个 URDF 解析器"：模型来自统一的
//   KinematicModel（共享 Model + 每实例 KinematicsData），关节顺序和底盘参考
//   frame 来自 RobotModelConfig。
//
//   坐标系约定：
//   - state 的 [x, y, yaw] 描述 state_base_frame（语义给出，通常
//     base_footprint），模型 root 到它的固定变换由 KinematicModel 处理；
//   - FK 返回结果按 state.header.frame_id 表达；
//   - Jacobian 满足 V = J(x) * u，u = [v, omega, qdot...]，行序 [v; omega]，
//     参考点为 frame 原点。
// ============================================================================
class PinocchioRobotModel final : public wbmm::core::RobotModel
{
public:
  // max_base_speed / max_base_yaw_rate <= 0 表示"尚未配置"，limits() 会原样
  // 返回，具体限速由上层 profile 决定。
  explicit PinocchioRobotModel(
    KinematicModelPtr model, double max_base_speed = 0.0,
    double max_base_yaw_rate = 0.0);

  [[nodiscard]] std::size_t stateDimension() const override;
  [[nodiscard]] std::size_t inputDimension() const override;
  [[nodiscard]] wbmm::core::BaseModel baseModel() const override;
  [[nodiscard]] const std::vector<std::string> & jointNames() const override;
  [[nodiscard]] const wbmm::core::RobotLimits & limits() const override;

  [[nodiscard]] bool hasFrame(const std::string & frame_name) const;

  [[nodiscard]] KinematicModelPtr kinematicModel() const noexcept {return model_;}

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
  KinematicModelPtr model_;
  FrameKinematics frames_;
  // core 合同把 FK/Jacobian 声明为 const；运行时可变的 Data 属于这个实例，
  // 不与其他实例共享。
  mutable KinematicsData kinematics_data_;
  wbmm::core::RobotLimits limits_;
};

}  // namespace wbmm::pinocchio
