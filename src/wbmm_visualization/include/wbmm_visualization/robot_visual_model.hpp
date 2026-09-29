#pragma once

// ============================================================================
// RobotVisualModel —— 只负责显示的机器人可视化模型。
//
//   它不再自己解析 URDF、也不再自己实现关节树 FK：
//     - link/joint/visual 描述来自共享 RobotDescription；
//     - link 位姿来自共享 wbmm_pinocchio::FrameKinematics；
//     - 受控关节顺序、底盘参考 frame、锁定关节来自 RobotModelConfig。
//   显示只消费这些共享结果，因此不会与规划/控制看到不同的模型。
//
//   每个实例持有自己的 KinematicsData，不是线程安全的；一个显示节点在
//   一个线程里使用即可。
// ============================================================================

#include <wbmm_pinocchio/frame_kinematics.hpp>
#include <wbmm_pinocchio/kinematic_model.hpp>
#include <wbmm_pinocchio/kinematics_data.hpp>
#include <wbmm_robot_model/wbmm_robot_model.hpp>

#include <wbmm_core/trajectory.hpp>

#include <Eigen/Geometry>

#include <map>
#include <string>
#include <vector>

#include <visualization_msgs/msg/marker_array.hpp>

namespace wbmm::visualization
{

struct VisualStyle
{
  // Uniform tint distinguishes planner and MPC ghosts. URDF materials can be
  // restored when the original mesh appearance is preferred.
  double red{0.1}, green{0.8}, blue{0.65}, alpha{0.17};
  bool use_urdf_materials{false};
  bool use_embedded_materials{false};
};

// Display geometry and link placement only: no dynamics, limits enforcement,
// collision queries, commands or planning. Consumes the existing WBMM state.
class RobotVisualModel
{
public:
  RobotVisualModel(
    std::shared_ptr<const wbmm::robot_model::RobotDescription> description,
    wbmm::robot_model::RobotModelConfig config,
    std::string mesh_directory = {});

  [[nodiscard]] const std::vector<std::string> & jointNames() const
  {return kinematic_->controlledJointNames();}

  [[nodiscard]] std::size_t visualCount() const {return visuals_.size();}

  [[nodiscard]] const std::string & baseFrame() const
  {return kinematic_->stateBaseFrame();}

  [[nodiscard]] bool hasLink(const std::string & name) const;

  // 抛出 std::invalid_argument 的轻量校验：frame/base/joint 维度与有限性。
  void validateState(const wbmm::core::WholeBodyState & state) const;

  // 全部 link 在 state.header.frame_id 中的位姿。
  [[nodiscard]] std::map<std::string, Eigen::Isometry3d> linkPoses(
    const wbmm::core::WholeBodyState & state) const;

  [[nodiscard]] visualization_msgs::msg::MarkerArray markers(
    const wbmm::core::WholeBodyState & state, const std::string & ns,
    const VisualStyle & style) const;

  [[nodiscard]] const wbmm::pinocchio::KinematicModelPtr & kinematicModel() const
  {return kinematic_;}

private:
  struct Visual
  {
    std::string link;
    Eigen::Isometry3d origin;
    visualization_msgs::msg::Marker prototype;
    bool has_material{false};
  };

  wbmm::pinocchio::KinematicModelPtr kinematic_;
  wbmm::pinocchio::FrameKinematics frames_;
  mutable wbmm::pinocchio::KinematicsData kinematics_data_;
  std::vector<Visual> visuals_;
};

}  // namespace wbmm::visualization
