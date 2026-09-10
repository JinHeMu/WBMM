#pragma once

#include "wbmm_core/types.hpp"

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>

namespace wbmm::core
{

struct RobotLimits
{
  std::vector<double> joint_min;
  std::vector<double> joint_max;
  std::vector<double> max_joint_speed;

  double max_base_speed{0.0};
  double max_base_yaw_rate{0.0};
};

// Minimal robot-model contract for the first version.
// It deliberately keeps only the operations needed by both planning and control:
// dimensions, joint order, limits, FK and the frame Jacobian.
class RobotModel
{
public:
  virtual ~RobotModel() = default;

  [[nodiscard]] virtual std::size_t stateDimension() const = 0;
  [[nodiscard]] virtual std::size_t inputDimension() const = 0;
  [[nodiscard]] virtual BaseModel baseModel() const = 0;
  [[nodiscard]] virtual const std::vector<std::string> & jointNames() const = 0;
  [[nodiscard]] virtual const RobotLimits & limits() const = 0;

  // The pose and spatial velocity are expressed in state.header.frame_id.
  virtual bool forwardKinematics(
    const WholeBodyState & state,
    const std::string & link_name,
    Pose & pose) const = 0;

  // Jacobian contract:
  //   V = J(x) * u
  //   V = [v; omega] in R^6.
  //
  // Row order:
  //   rows 0..2: linear velocity v
  //   rows 3..5: angular velocity omega
  //
  // Reference point:
  //   origin of the link_name frame, not the center of mass and not the
  //   world origin.
  //
  // Expressing frame:
  //   state.header.frame_id. Both v and omega are expressed in the same
  //   frame as the pose returned by forwardKinematics().
  //
  // Matrix size:
  //   6 x inputDimension().
  virtual bool frameJacobian(
    const WholeBodyState & state,
    const std::string & link_name,
    Eigen::Ref<Eigen::MatrixXd> jacobian) const = 0;

  // Model-specific semantic validation: concrete joint names/order, joint limits,
  // link/frame existence, and other robot-specific constraints.
  // Generic structural validation is provided by wbmm_core/validation.hpp and must be
  // applied by the caller before using the model-specific validation result.
  virtual bool validate(
    const WholeBodyState & state,
    std::string * message = nullptr) const = 0;

  virtual bool validate(
    const WholeBodyInput & input,
    std::string * message = nullptr) const = 0;
};

using RobotModelPtr = std::shared_ptr<RobotModel>;

}  // namespace wbmm::core
