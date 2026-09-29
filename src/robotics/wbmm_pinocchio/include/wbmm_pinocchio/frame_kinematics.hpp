#pragma once

// ============================================================================
// FrameKinematics —— 任意 link/frame 的位姿与 Jacobian。
//
//   这是三层运动学的最底层：球运动学和末端运动学都建立在它之上。
//   所有查询消费一个已经 update() 过的 KinematicsData，不会自己做 FK。
// ============================================================================

#include <wbmm_pinocchio/kinematics_data.hpp>
#include <wbmm_pinocchio/kinematic_model.hpp>

#include <wbmm_core/types.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <map>
#include <string>
#include <vector>

namespace wbmm::pinocchio
{

class FrameKinematics
{
public:
  explicit FrameKinematics(KinematicModelPtr model);

  [[nodiscard]] const KinematicModelPtr & model() const noexcept {return model_;}

  [[nodiscard]] bool hasFrame(const std::string & frame_name) const;

  // 全部 URDF link（description 顺序），供显示遍历。
  [[nodiscard]] const std::vector<std::string> & linkNames() const noexcept
  {return link_names_;}

  // frame 在 state.header.frame_id 中的位姿。
  [[nodiscard]] bool framePlacement(
    const KinematicsData & kinematics_data, const std::string & frame_name,
    Eigen::Isometry3d & placement, std::string * message = nullptr) const;

  // 所有 link 的位姿，一次性返回。显示只需要这一份结果。
  [[nodiscard]] std::map<std::string, Eigen::Isometry3d> linkPlacements(
    const KinematicsData & kinematics_data) const;

  // V = J(x) * u，6 x inputDimension，行序 [v; omega]，参考点为 frame 原点，
  // 表达在 state.header.frame_id。与 core::RobotModel::frameJacobian 合同一致。
  // 取可变 kinematics_data：提取帧 Jacobian 会写 pinocchio Data 的帧缓存。
  [[nodiscard]] bool frameJacobian(
    KinematicsData & kinematics_data, const std::string & frame_name,
    Eigen::Ref<Eigen::MatrixXd> jacobian, std::string * message = nullptr) const;

  // core 合同位姿（wxyz 四元数，header = 状态 header）。
  [[nodiscard]] bool framePose(
    const KinematicsData & kinematics_data, const std::string & frame_name,
    wbmm::core::Pose & pose, std::string * message = nullptr) const;

private:
  KinematicModelPtr model_;
  std::vector<std::string> link_names_;
};

}  // namespace wbmm::pinocchio
