#pragma once

// ============================================================================
// KinematicsData 持有当前状态、Pinocchio Data 和 FK 结果。
//
// KinematicModel 可以共享；KinematicsData 是可变的，每个消费者或线程
// 单独持有一份。update(state) 总会计算 frame 位姿；Jacobian 按需计算。
// ============================================================================

#include <wbmm_pinocchio/kinematic_model.hpp>

#include <wbmm_core/types.hpp>

#include <pinocchio/multibody/data.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

namespace wbmm::pinocchio
{

enum class UpdateMode {kPoses, kJacobians};

class KinematicsData
{
public:
  explicit KinematicsData(KinematicModelPtr model);

  // 用给定状态刷新位姿；按需额外计算 Jacobian。
  // 返回 false 表示状态映射失败，lastMessage() 给出原因。
  [[nodiscard]] bool update(
    const wbmm::core::WholeBodyState & state,
    UpdateMode mode = UpdateMode::kPoses);

  [[nodiscard]] KinematicModelPtr model() const noexcept {return model_;}

  [[nodiscard]] const ::pinocchio::Data & data() const noexcept {return data_;}

  // 可变的 Data：帧 Jacobian 的提取（getFrameJacobian）会写入 data.oMf。
  // KinematicsData 本身就是每线程可变缓存，只有需要线性化时才用这个重载。
  [[nodiscard]] ::pinocchio::Data & data() noexcept {return data_;}

  [[nodiscard]] bool updated() const noexcept {return updated_;}

  [[nodiscard]] bool jacobiansReady() const noexcept
  {return jacobians_ready_;}

  [[nodiscard]] const wbmm::core::WholeBodyState & state() const noexcept
  {return state_;}

  // state 系 <- model root 的旋转，用于把 root 系的量表达在 state 系。
  [[nodiscard]] Eigen::Matrix3d stateFromRootRotation() const
  {return world_from_root_.linear();}

  // 把 pinocchio 的 root 系位姿表达成 state 系（state.header.frame_id）。
  [[nodiscard]] Eigen::Isometry3d toStateFrame(
    const Eigen::Isometry3d & root_pose) const;

  [[nodiscard]] const std::string & lastMessage() const noexcept
  {return last_message_;}

private:
  KinematicModelPtr model_;
  ::pinocchio::Data data_;
  wbmm::core::WholeBodyState state_;
  Eigen::VectorXd configuration_;
  Eigen::Isometry3d world_from_root_{Eigen::Isometry3d::Identity()};
  bool jacobians_ready_{false};
  bool updated_{false};
  std::string last_message_;
};

}  // namespace wbmm::pinocchio
