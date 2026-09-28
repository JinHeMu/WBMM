#include "wbmm_reference_bridge/trajectory_sampler.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <set>

namespace wbmm::reference_bridge
{
namespace
{

bool isFinite(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(),
    [](double value) {return std::isfinite(value);});
}

std::string describe(const char * field, std::size_t expected, std::size_t actual)
{
  return std::string(field) + " has length " + std::to_string(actual) +
         ", expected " + std::to_string(expected) + ".";
}

}  // namespace

bool TrajectorySampler::validate(
  const wbmm_planning_msgs::msg::WholeBodyTrajectory & trajectory,
  std::string * message)
{
  const auto fail = [message](const std::string & reason)
  {
    if (message != nullptr)
    {
      *message = reason;
    }
    return false;
  };

  if (trajectory.header.frame_id.empty())
  {
    return fail("header.frame_id is empty; the reference frame must be explicit.");
  }

  const std::size_t samples = trajectory.time_from_start.size();
  if (samples == 0U)
  {
    return fail("time_from_start is empty; a trajectory needs at least one sample.");
  }

  if (trajectory.joint_names.empty())
  {
    return fail("joint_names is empty; the joint order must be explicit.");
  }

  std::set<std::string> names;
  for (const auto & name : trajectory.joint_names) {
    if (name.empty() || !names.insert(name).second) {
      return fail("joint_names must be nonempty and unique.");
    }
  }
  if (trajectory.time_from_start.front() != 0.0) {
    return fail("time_from_start must start at zero.");
  }
  const std::size_t joints = trajectory.joint_names.size();
  const std::size_t flat = samples * joints;

  if (!isFinite(trajectory.time_from_start))
  {
    return fail("time_from_start contains NaN or Inf.");
  }
  for (std::size_t i = 1U; i < samples; ++i)
  {
    if (!(trajectory.time_from_start[i] > trajectory.time_from_start[i - 1U]))
    {
      return fail(
        "time_from_start must be strictly increasing (violated at index " +
        std::to_string(i) + ").");
    }
  }

  if (trajectory.base_x.size() != samples)
  {
    return fail(describe("base_x", samples, trajectory.base_x.size()));
  }
  if (trajectory.base_y.size() != samples)
  {
    return fail(describe("base_y", samples, trajectory.base_y.size()));
  }
  if (trajectory.base_yaw.size() != samples)
  {
    return fail(describe("base_yaw", samples, trajectory.base_yaw.size()));
  }
  if (trajectory.base_linear_velocity.size() != samples)
  {
    return fail(describe(
      "base_linear_velocity", samples, trajectory.base_linear_velocity.size()));
  }
  if (trajectory.base_yaw_rate.size() != samples)
  {
    return fail(describe("base_yaw_rate", samples, trajectory.base_yaw_rate.size()));
  }
  if (trajectory.phase.size() != samples)
  {
    return fail(describe("phase", samples, trajectory.phase.size()));
  }

  if (trajectory.joint_positions.size() != flat)
  {
    return fail(describe("joint_positions", flat, trajectory.joint_positions.size()));
  }
  if (trajectory.joint_velocities.size() != flat)
  {
    return fail(describe("joint_velocities", flat, trajectory.joint_velocities.size()));
  }

  if (!isFinite(trajectory.base_x) || !isFinite(trajectory.base_y) ||
    !isFinite(trajectory.base_yaw) ||
    !isFinite(trajectory.base_linear_velocity) ||
    !isFinite(trajectory.base_yaw_rate) ||
    !isFinite(trajectory.joint_positions) ||
    !isFinite(trajectory.joint_velocities))
  {
    return fail("the trajectory contains NaN or Inf samples.");
  }

  if (message != nullptr)
  {
    message->clear();
  }
  return true;
}

TrajectorySampler::TrajectorySampler(
  const wbmm_planning_msgs::msg::WholeBodyTrajectory & trajectory)
: frame_id_(trajectory.header.frame_id),
  trajectory_id_(trajectory.trajectory_id),
  environment_revision_(trajectory.environment_revision),
  collision_model_revision_(trajectory.collision_model_revision),
  joint_names_(trajectory.joint_names),
  time_(trajectory.time_from_start),
  base_x_(trajectory.base_x),
  base_y_(trajectory.base_y),
  base_yaw_(trajectory.base_yaw),
  base_linear_velocity_(trajectory.base_linear_velocity),
  base_yaw_rate_(trajectory.base_yaw_rate),
  joint_positions_(trajectory.joint_positions),
  joint_velocities_(trajectory.joint_velocities),
  phase_(trajectory.phase)
{
}

WholeBodySample TrajectorySampler::sample(double time_from_start) const
{
  WholeBodySample result;
  result.joint_positions.assign(joint_names_.size(), 0.0);
  result.joint_velocities.assign(joint_names_.size(), 0.0);

  if (time_.empty())
  {
    return result;
  }

  const double clamped = std::clamp(time_from_start, time_.front(), time_.back());
  result.time_from_start = clamped;

  // Segment [lower, upper] with time_[lower] <= clamped <= time_[upper].
  std::size_t lower = 0U;
  std::size_t upper = 0U;
  if (clamped <= time_.front())
  {
    lower = upper = 0U;
  }
  else if (clamped >= time_.back())
  {
    lower = upper = time_.size() - 1U;
  }
  else
  {
    const auto upper_it = std::upper_bound(time_.begin(), time_.end(), clamped);
    upper = static_cast<std::size_t>(std::distance(time_.begin(), upper_it));
    lower = upper - 1U;
  }

  double alpha = 0.0;
  if (upper != lower)
  {
    const double span = time_[upper] - time_[lower];
    alpha = span > 0.0 ? (clamped - time_[lower]) / span : 0.0;
  }

  const auto blend = [alpha](double a, double b) {return a + alpha * (b - a);};

  result.base_x = blend(base_x_[lower], base_x_[upper]);
  result.base_y = blend(base_y_[lower], base_y_[upper]);
  result.base_yaw = blend(base_yaw_[lower], base_yaw_[upper]);
  result.base_linear_velocity =
    blend(base_linear_velocity_[lower], base_linear_velocity_[upper]);
  result.base_yaw_rate = blend(base_yaw_rate_[lower], base_yaw_rate_[upper]);

  const std::size_t joints = joint_names_.size();
  const std::size_t lower_offset = lower * joints;
  const std::size_t upper_offset = upper * joints;
  for (std::size_t joint = 0U; joint < joints; ++joint)
  {
    result.joint_positions[joint] =
      blend(joint_positions_[lower_offset + joint], joint_positions_[upper_offset + joint]);
    result.joint_velocities[joint] =
      blend(joint_velocities_[lower_offset + joint], joint_velocities_[upper_offset + joint]);
  }

  // Phase is a discrete label: zero-order hold, never interpolated.
  result.phase = phase_[lower];
  // A clamped terminal pose is a hold, never a continuing velocity command.
  if (time_from_start > time_.back() || time_from_start < time_.front()) {
    result.base_linear_velocity = 0.0;
    result.base_yaw_rate = 0.0;
    std::fill(result.joint_velocities.begin(), result.joint_velocities.end(), 0.0);
  }
  return result;
}

}  // namespace wbmm::reference_bridge
