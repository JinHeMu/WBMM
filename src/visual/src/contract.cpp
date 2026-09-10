#include "wbmm_visualization/contract.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace wbmm_viz
{

namespace
{
double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}
}  // namespace

ParsedTrajectory parseJointTrajectory(
  const trajectory_msgs::msg::JointTrajectory & msg,
  const std::string * phase_schedule)
{
  if (msg.joint_names.size() != kJointNames.size()) {
    throw std::invalid_argument(
      "whole-body trajectory needs exactly " +
      std::to_string(kJointNames.size()) + " joint names, got " +
      std::to_string(msg.joint_names.size()));
  }
  for (std::size_t index = 0; index < kJointNames.size(); ++index) {
    if (msg.joint_names[index] != kJointNames[index]) {
      throw std::invalid_argument(
        "whole-body trajectory joint name mismatch at index " +
        std::to_string(index) + ": expected \"" +
        std::string(kJointNames[index]) + "\", got \"" +
        msg.joint_names[index] + "\"");
    }
  }
  if (msg.points.empty()) {
    throw std::invalid_argument("whole-body trajectory has no points");
  }

  std::vector<std::pair<double, std::string>> schedule;
  if (phase_schedule != nullptr && !phase_schedule->empty()) {
    if (!decodePhaseSchedule(*phase_schedule, schedule)) {
      schedule.clear();  // malformed schedule: phases stay empty
    }
  }

  ParsedTrajectory result;
  result.frame_id = msg.header.frame_id;
  result.waypoints.reserve(msg.points.size());
  double previous_time = 0.0;
  for (const auto & point : msg.points) {
    if (point.positions.size() < kJointNames.size()) {
      throw std::invalid_argument(
        "whole-body trajectory point has " +
        std::to_string(point.positions.size()) +
        " positions, expected at least " +
        std::to_string(kJointNames.size()));
    }
    const double raw_time = point.time_from_start.sec +
      1.0e-9 * static_cast<double>(point.time_from_start.nanosec);
    const double time = std::max(previous_time, raw_time);
    Waypoint waypoint;
    waypoint.time = time;
    for (std::size_t index = 0; index < kJointNames.size(); ++index) {
      waypoint.state[index] = point.positions[index];
    }
    waypoint.phase = phaseAt(schedule, time);
    waypoint.in_contact = waypoint.phase == "TASK_CONSTRAINED";
    result.waypoints.push_back(std::move(waypoint));
    previous_time = time;
  }
  result.duration = result.waypoints.back().time;
  return result;
}

Waypoint waypointAt(const ParsedTrajectory & trajectory, double t)
{
  if (trajectory.waypoints.empty()) {
    throw std::invalid_argument("waypointAt on an empty trajectory");
  }
  const auto & waypoints = trajectory.waypoints;
  if (t <= waypoints.front().time) {
    return waypoints.front();
  }
  if (t >= waypoints.back().time) {
    return waypoints.back();
  }
  const auto second = std::lower_bound(
    waypoints.begin(), waypoints.end(), t,
    [](const Waypoint & waypoint, double time) {
      return waypoint.time < time;
    });
  const auto first = std::prev(second);
  const double duration = std::max(1.0e-9, second->time - first->time);
  const double ratio = std::clamp(
    (t - first->time) / duration, 0.0, 1.0);
  Waypoint result = *first;
  result.time = t;
  for (std::size_t index = 0; index < result.state.size(); ++index) {
    if (index == 2) {
      result.state[index] = wrapAngle(
        first->state[2] + ratio * wrapAngle(
          second->state[2] - first->state[2]));
    } else {
      result.state[index] = first->state[index] +
        ratio * (second->state[index] - first->state[index]);
    }
  }
  if (ratio >= 0.5) {
    result.phase = second->phase;
    result.in_contact = second->in_contact;
  }
  return result;
}

std::vector<Waypoint> samplesInInterval(const ParsedTrajectory & trajectory,
                                        double start, double end)
{
  start = std::clamp(start, 0.0, trajectory.duration);
  end = std::clamp(end, start, trajectory.duration);
  std::vector<Waypoint> result{waypointAt(trajectory, start)};
  for (const auto & waypoint : trajectory.waypoints) {
    if (waypoint.time > start + 1.0e-9 &&
      waypoint.time < end - 1.0e-9)
    {
      result.push_back(waypoint);
    }
  }
  if (end > start + 1.0e-9) {
    result.push_back(waypointAt(trajectory, end));
  }
  return result;
}

Eigen::VectorXd toStateVector(const Waypoint & waypoint)
{
  Eigen::VectorXd result(static_cast<Eigen::Index>(waypoint.state.size()));
  for (std::size_t index = 0; index < waypoint.state.size(); ++index) {
    result[static_cast<Eigen::Index>(index)] = waypoint.state[index];
  }
  return result;
}

std::string encodePhaseSchedule(
  const std::vector<std::pair<double, std::string>> & intervals)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6);
  for (std::size_t index = 0; index < intervals.size(); ++index) {
    if (index > 0) {
      stream << ';';
    }
    stream << intervals[index].first << ' ' << intervals[index].second;
  }
  return stream.str();
}

bool decodePhaseSchedule(const std::string & text,
                         std::vector<std::pair<double, std::string>> & out)
{
  out.clear();
  if (text.empty()) {
    return true;  // empty schedule is valid
  }
  if (text.back() == ';') {
    return false;  // trailing separator (std::getline would silently drop it)
  }
  std::istringstream stream(text);
  std::string item;
  double previous_time = -std::numeric_limits<double>::infinity();
  while (std::getline(stream, item, ';')) {
    if (item.empty()) {
      return false;
    }
    std::istringstream part(item);
    double time = 0.0;
    std::string phase;
    if (!(part >> time)) {
      return false;
    }
    if (!(part >> phase)) {
      return false;
    }
    std::string trailing;
    if (part >> trailing) {
      return false;  // garbage after the phase name
    }
    if (time < previous_time) {
      return false;  // times must be non-decreasing
    }
    out.emplace_back(time, std::move(phase));
    previous_time = time;
  }
  return true;
}

std::string phaseAt(const std::vector<std::pair<double, std::string>> & schedule,
                    double t)
{
  if (schedule.empty()) {
    return "";
  }
  std::string result = schedule.front().second;
  for (const auto & entry : schedule) {
    if (t >= entry.first) {
      result = entry.second;
    } else {
      break;
    }
  }
  return result;
}

}  // namespace wbmm_viz
