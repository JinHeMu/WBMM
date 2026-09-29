#pragma once

#include <wbmm_planner_ros/msg/whole_body_trajectory.hpp>

#include <cstdint>
#include <string>
#include <vector>

namespace wbmm::reference_bridge
{

// One interpolated instant of a nominal whole-body trajectory.
// Units follow the message contract: m, rad, m/s, rad/s, s.
struct WholeBodySample
{
  double time_from_start{0.0};

  double base_x{0.0};
  double base_y{0.0};
  double base_yaw{0.0};
  double base_linear_velocity{0.0};
  double base_yaw_rate{0.0};

  std::vector<double> joint_positions;
  std::vector<double> joint_velocities;

  std::uint8_t phase{0};
};

// Validates and samples a wbmm_planner_ros/WholeBodyTrajectory.
//
// This class owns no ROS node and no clock: the caller decides which instant to
// query. It replaces the polynomial decoding and the tangent-based yaw
// reconstruction of the REMANI bridge, where the reconstructed yaw rate grew as
// 1/t near the trajectory start and could not be tracked by the base.
class TrajectorySampler
{
public:
  TrajectorySampler() = default;

  // Copies the payload. Call validate() first if the message came from a
  // network boundary; this constructor assumes the contract holds.
  explicit TrajectorySampler(
    const wbmm_planner_ros::msg::WholeBodyTrajectory & trajectory);

  // Structural contract check. On failure returns false and writes a reason.
  // Checks: non-empty header frame, at least one sample, finite and strictly
  // increasing sample times, per-sample array lengths, flattened joint array
  // length and a consistent joint order.
  [[nodiscard]] static bool validate(
    const wbmm_planner_ros::msg::WholeBodyTrajectory & trajectory,
    std::string * message);

  [[nodiscard]] bool empty() const noexcept {return time_.empty();}

  [[nodiscard]] double duration() const noexcept
  {
    return time_.empty() ? 0.0 : time_.back();
  }

  [[nodiscard]] const std::string & frameId() const noexcept {return frame_id_;}

  [[nodiscard]] const std::string & trajectoryId() const noexcept
  {
    return trajectory_id_;
  }

  [[nodiscard]] std::uint64_t environmentRevision() const noexcept
  {
    return environment_revision_;
  }

  [[nodiscard]] std::uint64_t collisionModelRevision() const noexcept
  {
    return collision_model_revision_;
  }

  [[nodiscard]] const std::vector<std::string> & jointNames() const noexcept
  {
    return joint_names_;
  }

  [[nodiscard]] std::size_t jointCount() const noexcept
  {
    return joint_names_.size();
  }

  [[nodiscard]] std::size_t sampleCount() const noexcept {return time_.size();}

  // Linear interpolation between samples. The query time is clamped into
  // [0, duration()], so a caller that runs past the end keeps receiving the
  // terminal pose instead of an extrapolated one.
  [[nodiscard]] WholeBodySample sample(double time_from_start) const;

private:
  std::string frame_id_;
  std::string trajectory_id_;
  std::uint64_t environment_revision_{0};
  std::uint64_t collision_model_revision_{0};

  std::vector<std::string> joint_names_;
  std::vector<double> time_;

  std::vector<double> base_x_;
  std::vector<double> base_y_;
  std::vector<double> base_yaw_;
  std::vector<double> base_linear_velocity_;
  std::vector<double> base_yaw_rate_;

  // Row-major, stride = joint_names_.size().
  std::vector<double> joint_positions_;
  std::vector<double> joint_velocities_;

  std::vector<std::uint8_t> phase_;
};

}  // namespace wbmm::reference_bridge
