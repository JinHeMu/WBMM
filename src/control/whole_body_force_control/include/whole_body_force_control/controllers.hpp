#pragma once

#include <wbmm_core/wbmm_core.hpp>

#include <Eigen/Core>

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>

namespace whole_body_force_control
{

using Vector6d = Eigen::Matrix<double, 6, 1>;
using AxisMask6d = std::array<bool, 6>;

// Convert a 6D local correction [dx, dy, dz, rx, ry, rz], expressed in the
// nominal TCP frame, into a task-space EndEffectorPose in frame_id.
//
//   p_target = p_nom + R_nom * dp_local
//   R_target = exp3(R_nom * dr_local) * R_nom
wbmm::core::EndEffectorPose makeEndEffectorPoseTarget(
  const wbmm::core::Pose & nominal_pose,
  const Vector6d & local_correction,
  const std::string & frame_id,
  double stamp);

// Transform a wrench expressed at the source origin and in source axes to the
// target origin and target axes.  target_rotation_source maps source vectors to
// target vectors; target_to_source is the source origin expressed in target.
Vector6d transformWrench(
  const Vector6d & source_wrench,
  const Eigen::Matrix3d & target_rotation_source,
  const Eigen::Vector3d & target_to_source);

// Single-axis second-order admittance:
//
//   M * x_ddot + D * x_dot + K * x = F_measured
//
// K = 0 is the force-following limit: with a constant measured force the
// velocity converges to F / D and the computed displacement keeps advancing.
class AdmittanceController
{
public:
  AdmittanceController(
    double mass,
    double damping,
    double stiffness,
    double max_velocity);

  double update(double measured_force, double dt);
  void reset(double measured_force = 0.0);

  // Anti-windup hook.  If the whole-body kinematics can only realize
  // |reachable_offset| < |offset_| in the same direction, shrink the
  // integrated offset to the reachable value and stop the integrator.  The
  // controller never flips sign just because a command saturated.  Returns
  // true when the internal state was reduced.
  bool limitOffset(double reachable_offset);

  double offset() const
  {
    return offset_;
  }

  double velocity() const
  {
    return velocity_;
  }

  double measuredForce() const
  {
    return measured_force_;
  }

private:
  double mass_;
  double damping_;
  double stiffness_;
  double max_velocity_;
  double measured_force_{0.0};
  double offset_{0.0};
  double velocity_{0.0};
};

// Independent six-axis admittance controller.  Axis order is fixed as
// [Fx, Fy, Fz, Tx, Ty, Tz] -> [dx, dy, dz, rx, ry, rz].
class CartesianComplianceController
{
public:
  CartesianComplianceController(
    const AxisMask6d & admittance_axes,
    const Vector6d & mass,
    const Vector6d & damping,
    const Vector6d & stiffness,
    const Vector6d & max_velocity);

  Vector6d update(const Vector6d & measured_wrench, double dt);
  void reset(const Vector6d & measured_wrench = Vector6d::Zero());

  // Clamp every enabled axis to the correction that the whole-body kinematics
  // actually realized.  Disabled axes are ignored.  Returns true if any axis
  // offset was reduced.
  bool clampOffset(const Vector6d & reachable_offset);

  const Vector6d & offset() const
  {
    return offset_;
  }

  const Vector6d & velocity() const
  {
    return velocity_;
  }

  const Vector6d & measuredWrench() const
  {
    return measured_wrench_;
  }

  const AxisMask6d & admittanceAxes() const
  {
    return admittance_axes_;
  }

private:
  AxisMask6d admittance_axes_{};
  std::array<std::unique_ptr<AdmittanceController>, 6> admittance_;
  Vector6d offset_{Vector6d::Zero()};
  Vector6d velocity_{Vector6d::Zero()};
  Vector6d measured_wrench_{Vector6d::Zero()};
};

}  // namespace whole_body_force_control
