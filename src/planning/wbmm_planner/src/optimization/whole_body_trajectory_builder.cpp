#include "wbmm_planner/optimization/whole_body_trajectory_builder.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>

namespace wbmm::traj_opt
{
namespace
{

constexpr double kTwoPi = 6.283185307179586;

double wrapToPi(double angle)
{
  angle = std::fmod(angle + M_PI, kTwoPi);
  if (angle < 0.0) {
    angle += kTwoPi;
  }
  return angle - M_PI;
}

// Smallest representative of `angle` close to `reference`.
double unwrapNear(double angle, double reference)
{
  return reference + wrapToPi(angle - reference);
}

bool isFinite(const Eigen::VectorXd & value)
{
  return value.array().isFinite().all();
}

}  // namespace

TrajectoryBuildResult buildWholeBodyTrajectory(
  const Trajectory<7> & minco, const std::vector<std::string> & joint_names,
  const std::string & frame_id, const TrajectoryBuilderConfig & config)
{
  TrajectoryBuildResult result;

  const auto base_dim = static_cast<Eigen::Index>(2);
  const auto joint_count = static_cast<Eigen::Index>(joint_names.size());

  if (joint_names.empty()) {
    result.message = "joint_names must not be empty.";
    return result;
  }
  if (minco.getPieceNum() <= 0) {
    result.message = "The MINCO trajectory has no pieces.";
    return result;
  }
  if (minco.getPieceDim() != base_dim + joint_count) {
    result.message = "MINCO dimension " + std::to_string(minco.getPieceDim()) +
      " does not match 2 base DoF + " + std::to_string(joint_count) + " joints.";
    return result;
  }
  if (frame_id.empty()) {
    result.message = "frame_id must not be empty.";
    return result;
  }
  if (!std::isfinite(config.sample_dt) || config.sample_dt <= 0.0) {
    result.message = "sample_dt must be positive.";
    return result;
  }
  if (config.gear != 1 && config.gear != -1) {
    result.message = "gear must be +1 or -1.";
    return result;
  }

  for (double value : {config.max_linear_velocity, config.max_yaw_rate,
      config.max_joint_velocity, config.max_heading_step, config.tangent_chord_length}) {
    if (!std::isfinite(value) || value <= 0.0) {
      result.message = "Controller limits and chord length must be positive and finite.";
      return result;
    }
  }
  if (!std::isfinite(config.initial_yaw) || !std::isfinite(config.yaw_hold_distance) ||
      config.yaw_hold_distance < 0.0) {
    result.message = "Invalid heading configuration.";
    return result;
  }
  const double duration = minco.getTotalDuration();
  if (!std::isfinite(duration) || duration <= 0.0) {
    result.message = "The MINCO trajectory has a non-positive duration.";
    return result;
  }

  if (duration / config.sample_dt > 100000.0) {
    result.message = "Trajectory sampling budget exceeded.";
    return result;
  }
  const auto sample_count =
    static_cast<std::size_t>(std::ceil(duration / config.sample_dt)) + 1U;
  const double dt = duration / static_cast<double>(sample_count - 1U);

  // Pass 1: sample positions and forward speed.
  std::vector<double> base_x(sample_count, 0.0);
  std::vector<double> base_y(sample_count, 0.0);
  std::vector<double> forward_velocity(sample_count, 0.0);

  for (std::size_t i = 0U; i < sample_count; ++i) {
    const double t = std::min(static_cast<double>(i) * dt, duration);
    const Eigen::VectorXd position = minco.getPos(t);
    const Eigen::VectorXd velocity = minco.getVel(t);
    if (!isFinite(position) || !isFinite(velocity)) {
      result.message = "MINCO produced a non-finite sample at t=" +
        std::to_string(t) + ".";
      return result;
    }
    base_x[i] = position(0);
    base_y[i] = position(1);
    forward_velocity[i] = static_cast<double>(config.gear) *
      std::hypot(velocity(0), velocity(1));
  }

  // A differential drive must face its velocity tangent while moving. At rest,
  // use the first defined tangent for leading stationary samples, then hold the
  // last defined tangent. A distant chord would introduce an artificial yaw
  // step between the rest sample and the first moving sample under time scaling.
  double first_heading = config.initial_yaw;
  for (std::size_t i = 0U; i < sample_count; ++i) {
    const auto v = minco.getVel(std::min(static_cast<double>(i) * dt, duration));
    if (std::hypot(v(0), v(1)) > 1e-8) {
      first_heading = unwrapNear(std::atan2(config.gear * v(1), config.gear * v(0)),
        config.initial_yaw);
      break;
    }
  }
  std::vector<double> heading(sample_count, first_heading);
  double previous_heading = first_heading;
  for (std::size_t i = 0U; i < sample_count; ++i) {
    const auto v = minco.getVel(std::min(static_cast<double>(i) * dt, duration));
    if (std::hypot(v(0), v(1)) > 1e-8) {
      heading[i] = unwrapNear(std::atan2(config.gear * v(1), config.gear * v(0)),
        previous_heading);
    } else {
      heading[i] = previous_heading;
    }
    result.max_heading_step = std::max(result.max_heading_step,
      std::abs(wrapToPi(heading[i] - (i == 0U ? config.initial_yaw : heading[i - 1U]))));
    previous_heading = heading[i];
  }

  // Pass 2: yaw rate as the derivative of the heading that was emitted, so the
  // published state and feedforward input are consistent by construction.
  std::vector<double> yaw_rate(sample_count, 0.0);
  for (std::size_t i = 0U; i + 1U < sample_count; ++i) {
    yaw_rate[i] = wrapToPi(heading[i + 1U] - heading[i]) / dt;
  }
  if (sample_count > 1U) {
    yaw_rate[sample_count - 1U] = yaw_rate[sample_count - 2U];
  }

  result.trajectory.trajectory_id = config.trajectory_id;
  result.trajectory.environment_revision = config.environment_revision;
  result.trajectory.collision_model_revision = config.collision_model_revision;
  result.trajectory.points.reserve(sample_count);

  for (std::size_t i = 0U; i < sample_count; ++i) {
    const double t = std::min(static_cast<double>(i) * dt, duration);
    const Eigen::VectorXd position = minco.getPos(t);
    const Eigen::VectorXd velocity = minco.getVel(t);

    wbmm::core::WholeBodyTrajectoryPoint point;
    point.time_from_start = t;
    point.phase = config.phase;

    point.state.header.frame_id = frame_id;
    point.state.header.stamp = t;
    point.state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
    point.state.base.x = base_x[i];
    point.state.base.y = base_y[i];
    point.state.base.yaw = heading[i];
    point.state.base.linear_velocity = forward_velocity[i];
    point.state.base.lateral_velocity = 0.0;
    point.state.base.yaw_rate = yaw_rate[i];

    point.state.joints.names = joint_names;
    point.state.joints.positions.resize(static_cast<std::size_t>(joint_count));
    point.state.joints.velocities.resize(static_cast<std::size_t>(joint_count));
    point.state.joints.efforts.assign(static_cast<std::size_t>(joint_count), 0.0);
    for (Eigen::Index j = 0; j < joint_count; ++j) {
      point.state.joints.positions[static_cast<std::size_t>(j)] =
        position(base_dim + j);
      point.state.joints.velocities[static_cast<std::size_t>(j)] =
        velocity(base_dim + j);
    }

    wbmm::core::WholeBodyInput input;
    input.stamp = t;
    input.base_model = wbmm::core::BaseModel::kDifferentialDrive;
    input.base_command = {forward_velocity[i], yaw_rate[i]};
    input.joint_names = joint_names;
    input.joint_velocities = point.state.joints.velocities;
    point.feedforward_input = std::move(input);

    result.trajectory.points.push_back(std::move(point));

    result.max_linear_velocity =
      std::max(result.max_linear_velocity, std::abs(forward_velocity[i]));
    result.max_yaw_rate =
      std::max(result.max_yaw_rate, std::abs(yaw_rate[i]));
    for (const double joint_velocity : result.trajectory.points.back()
      .state.joints.velocities)
    {
      result.max_joint_velocity =
        std::max(result.max_joint_velocity, std::abs(joint_velocity));
    }
  }

  result.within_limits =
    result.max_linear_velocity <= config.max_linear_velocity + 1e-9 &&
    result.max_yaw_rate <= config.max_yaw_rate + 1e-9 &&
    result.max_joint_velocity <= config.max_joint_velocity + 1e-9 &&
    result.max_heading_step <= config.max_heading_step + 1e-9;

  result.success = true;
  if (result.within_limits) {
    result.message = "ok";
  } else if (result.max_heading_step > config.max_heading_step + 1e-9) {
    result.message =
      "Trajectory contains a heading step of " +
      std::to_string(result.max_heading_step) +
      " rad, which means the planned gear flips without a planned turn.";
  } else {
    result.message = "Trajectory exceeds the configured controller envelope.";
  }
  return result;
}

}  // namespace wbmm::traj_opt
