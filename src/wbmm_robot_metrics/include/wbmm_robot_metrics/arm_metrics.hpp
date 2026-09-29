#pragma once

#include "wbmm_core/robot_model.hpp"
#include "wbmm_robot_metrics/types.hpp"

namespace wbmm::metrics
{

class ArmMetrics
{
public:
  explicit ArmMetrics(wbmm::core::RobotModelPtr robot_model);

  [[nodiscard]] static bool validateOptions(
    const ArmMetricsOptions & options, std::string * message = nullptr);

  // Provider-neutral entry point. The caller owns kinematics and supplies a
  // 6 x inputDimension Jacobian with [linear; angular] rows. This lets OCS2
  // keep using ocs2::PinocchioInterface while other modules can use RobotModel.
  [[nodiscard]] static ArmKinematicMetrics evaluate(
    const Eigen::Ref<const Eigen::MatrixXd> & frame_jacobian,
    std::size_t arm_column_count,
    const wbmm::core::JointState & joints,
    const wbmm::core::RobotLimits & limits,
    const wbmm::core::Header & header,
    const std::string & link_name,
    ArmMetricsOptions options = {});

  // Uses the arm columns of RobotModel's [v, omega, qdot1..qdot6] Jacobian.
  // No planner cost or control decision is made by this module.
  [[nodiscard]] ArmKinematicMetrics evaluate(
    const wbmm::core::WholeBodyState & state,
    const std::string & link_name,
    ArmMetricsOptions options = {}) const;

private:
  wbmm::core::RobotModelPtr robot_model_;
};

}  // namespace wbmm::metrics
