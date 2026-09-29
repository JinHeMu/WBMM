#pragma once

// ============================================================================
// KinematicModel 保存可共享的只读 Pinocchio 模型及关节索引。
//
// 使用 RobotDescription 中保留的 XML 建模；受控关节顺序、锁定位置和
// 底盘参考 link 来自 RobotModelConfig。运行时计算数据见 KinematicsData。
// ============================================================================

#include <wbmm_core/types.hpp>
#include <wbmm_robot_model/wbmm_robot_model.hpp>

#include <pinocchio/multibody/model.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace wbmm::pinocchio
{

// 一个受控关节在 pinocchio 模型里的索引与限位。
struct ControlledJoint
{
  std::string name;
  ::pinocchio::JointIndex joint_id{0};
  Eigen::Index idx_q{0};
  Eigen::Index idx_v{0};
  // 连续关节的 nq 为 2（cos/sin），此时 idx_q..idx_q+1 都要填。
  int nq{1};
  int nv{1};
  bool has_position_limit{false};
  double lower{0.0};
  double upper{0.0};
  bool has_velocity_limit{false};
  double velocity{0.0};
};

class KinematicModel
{
public:
  // 从聚合描述构建（复制只读描述，一次性开销）。
  [[nodiscard]] static std::shared_ptr<const KinematicModel> create(
    wbmm::robot_model::RobotModelDescriptionPtr description);

  // 只有描述和配置也能建模型：不需要碰撞几何的消费者（例如力控）不必
  // 被迫提供 base_collision_link 或 URDF 球。
  [[nodiscard]] static std::shared_ptr<const KinematicModel> create(
    std::shared_ptr<const wbmm::robot_model::RobotDescription> description,
    wbmm::robot_model::RobotModelConfig config);

  [[nodiscard]] const ::pinocchio::Model & model() const noexcept {return model_;}

  [[nodiscard]] const wbmm::robot_model::RobotDescription & description()
  const noexcept {return *description_;}

  [[nodiscard]] const std::string & rootFrame() const noexcept {return root_frame_;}

  // 9D 状态里 [x, y, yaw] 描述的 link（来自配置）。
  [[nodiscard]] const std::string & stateBaseFrame() const noexcept
  {return state_base_frame_;}

  // root <- state_base 的固定变换；p_root = rootFromStateBase * p_state_base。
  [[nodiscard]] const Eigen::Isometry3d & rootFromStateBase() const noexcept
  {return root_from_state_base_;}

  [[nodiscard]] const std::vector<std::string> & controlledJointNames()
  const noexcept {return controlled_joint_names_;}

  [[nodiscard]] const std::vector<ControlledJoint> & controlledJoints()
  const noexcept {return controlled_joints_;}

  [[nodiscard]] const std::map<std::string, double> & lockedJoints() const noexcept
  {return locked_joints_;}

  [[nodiscard]] bool hasFrame(const std::string & frame_name) const;

  [[nodiscard]] ::pinocchio::FrameIndex frameId(const std::string & frame_name) const;

  [[nodiscard]] ::pinocchio::JointIndex jointId(const std::string & joint_name) const;

  // 受控关节在受控列表中的下标；未找到返回 npos。
  [[nodiscard]] std::size_t controlledJointIndex(
    const std::string & joint_name) const noexcept;

  [[nodiscard]] std::size_t stateDimension() const noexcept {return state_dimension_;}

  [[nodiscard]] std::size_t inputDimension() const noexcept {return input_dimension_;}

  // 把 core 状态映射成 pinocchio 配置向量 q：
  //   受控关节按名字从 state 取值；锁定关节取配置位置。任何缺失/重复/未知关节都返回失败并给出消息。
  [[nodiscard]] bool configurationFromState(
    const wbmm::core::WholeBodyState & state, Eigen::VectorXd & configuration,
    std::string * message = nullptr) const;

  // 校验 core 状态：base model、关节集合、限位。
  [[nodiscard]] bool validateState(
    const wbmm::core::WholeBodyState & state, std::string * message = nullptr) const;

  [[nodiscard]] bool validateInput(
    const wbmm::core::WholeBodyInput & input, std::string * message = nullptr) const;

private:
  KinematicModel(
    std::shared_ptr<const wbmm::robot_model::RobotDescription> description,
    wbmm::robot_model::RobotModelConfig config);

  std::shared_ptr<const wbmm::robot_model::RobotDescription> description_;
  ::pinocchio::Model model_;
  std::string root_frame_;
  std::string state_base_frame_;
  Eigen::Isometry3d root_from_state_base_{Eigen::Isometry3d::Identity()};
  std::vector<std::string> controlled_joint_names_;
  std::vector<ControlledJoint> controlled_joints_;
  std::map<std::string, double> locked_joints_;
  std::size_t state_dimension_{0};
  std::size_t input_dimension_{0};
};

using KinematicModelPtr = std::shared_ptr<const KinematicModel>;

}  // namespace wbmm::pinocchio
