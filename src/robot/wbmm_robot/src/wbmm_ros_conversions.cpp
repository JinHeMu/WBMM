#include "wbmm_robot/wbmm_ros_conversions.hpp"

#include "wbmm_robot/wbmm_conversions.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace wbmm::robot
{
namespace
{

double toSeconds(const builtin_interfaces::msg::Time & stamp)
{
  const double whole_seconds = static_cast<double>(stamp.sec);
  const double nanoseconds = static_cast<double>(stamp.nanosec);
  return whole_seconds + 1.0e-9 * nanoseconds;
}

}  // namespace

wbmm::core::Quaternion quaternionFromRos(
  const geometry_msgs::msg::Quaternion & quaternion)
{
  // ROS 顺序 xyzw -> core 顺序 wxyz。
  return {quaternion.w, quaternion.x, quaternion.y, quaternion.z};
}

geometry_msgs::msg::Quaternion quaternionToRos(
  const wbmm::core::Quaternion & quaternion)
{
  geometry_msgs::msg::Quaternion result;
  result.x = quaternion.x;
  result.y = quaternion.y;
  result.z = quaternion.z;
  result.w = quaternion.w;
  return result;
}

wbmm::core::Header headerFromRos(
  const std_msgs::msg::Header & header, wbmm::core::ClockDomain clock)
{
  wbmm::core::Header result;
  result.frame_id = header.frame_id;
  result.stamp = toSeconds(header.stamp);
  result.clock = clock;
  return result;
}

std::optional<wbmm::core::Wrench> wrenchFromRos(
  const geometry_msgs::msg::WrenchStamped & message,
  wbmm::core::ClockDomain clock,
  const std::string & fallback_frame)
{
  wbmm::core::Wrench result;
  result.header = headerFromRos(message.header, clock);
  if (result.header.frame_id.empty()) {
    result.header.frame_id = fallback_frame;
  }
  if (result.header.frame_id.empty() || !std::isfinite(result.header.stamp) ||
    result.header.stamp < 0.0)
  {
    return std::nullopt;
  }
  result.force.x = message.wrench.force.x;
  result.force.y = message.wrench.force.y;
  result.force.z = message.wrench.force.z;
  result.torque.x = message.wrench.torque.x;
  result.torque.y = message.wrench.torque.y;
  result.torque.z = message.wrench.torque.z;
  if (!wbmm::core::isFinite(result.force) ||
    !wbmm::core::isFinite(result.torque))
  {
    return std::nullopt;
  }
  return result;
}

std::optional<wbmm::core::WholeBodyState> wholeBodyStateFromMpcObservation(
  const ocs2_msgs::msg::MpcObservation & message,
  const std::vector<std::string> & joint_names,
  const std::string & frame_id,
  wbmm::core::ClockDomain clock)
{
  if (frame_id.empty() || joint_names.empty() ||
    message.state.value.size() != 3 + joint_names.size())
  {
    return std::nullopt;
  }
  if (!std::isfinite(message.time) || message.time < 0.0) {
    return std::nullopt;
  }

  Eigen::VectorXd state(
    static_cast<Eigen::Index>(message.state.value.size()));
  for (Eigen::Index i = 0; i < state.size(); ++i) {
    state[i] = static_cast<double>(
      message.state.value[static_cast<std::size_t>(i)]);
  }
  wbmm::core::Header header;
  header.frame_id = frame_id;
  header.stamp = message.time;
  header.clock = clock;
  return toCoreState(state, joint_names, header);
}

ocs2_msgs::msg::MpcTargetTrajectories toMpcTargetTrajectories(
  const wbmm::core::WholeBodyTrajectory & trajectory,
  double start_time,
  std::size_t input_dimension)
{
  ocs2_msgs::msg::MpcTargetTrajectories message;
  message.time_trajectory.reserve(trajectory.points.size());
  message.state_trajectory.reserve(trajectory.points.size());
  message.input_trajectory.reserve(trajectory.points.size());

  for (const auto & point : trajectory.points) {
    message.time_trajectory.push_back(start_time + point.time_from_start);

    ocs2_msgs::msg::MpcState state_message;
    const Eigen::VectorXd state = toEigenState(point.state);
    state_message.value.resize(static_cast<std::size_t>(state.size()));
    for (Eigen::Index i = 0; i < state.size(); ++i) {
      state_message.value[static_cast<std::size_t>(i)] =
        static_cast<float>(state[i]);
    }
    message.state_trajectory.push_back(std::move(state_message));

    std::vector<float> input_values(input_dimension, 0.0F);
    if (point.feedforward_input.has_value()) {
      const Eigen::VectorXd input = toEigenInput(*point.feedforward_input);
      const auto count = static_cast<std::size_t>(std::min<Eigen::Index>(
          input.size(), static_cast<Eigen::Index>(input_dimension)));
      for (std::size_t i = 0; i < count; ++i) {
        input_values[i] = static_cast<float>(
          input[static_cast<Eigen::Index>(i)]);
      }
    }
    ocs2_msgs::msg::MpcInput input_message;
    input_message.value = std::move(input_values);
    message.input_trajectory.push_back(std::move(input_message));
  }
  return message;
}

}  // namespace wbmm::robot
