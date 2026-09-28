#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>
#include <wbmm_visualization/trajectory_display.hpp>

namespace wbmm::visualization {
namespace {
using wbmm::core::WholeBodyTrajectory;
void namesValid(const std::vector<std::string> &names) {
  std::set<std::string> seen;
  for (const auto &name : names)
    if (name.empty() || !seen.insert(name).second)
      throw std::invalid_argument("Empty/duplicate trajectory joint name.");
}
void timeValid(double t, double previous, bool first, bool strict) {
  if (!std::isfinite(t) || (!first && (strict ? t <= previous : t < previous)))
    throw std::invalid_argument("Trajectory times must be finite and ordered.");
}
void stateValid(const wbmm::core::WholeBodyState &state) {
  for (double v : {state.base.x, state.base.y, state.base.yaw})
    if (!std::isfinite(v))
      throw std::invalid_argument("Non-finite base pose.");
  for (double v : state.joints.positions)
    if (!std::isfinite(v))
      throw std::invalid_argument("Non-finite joint position.");
}
} // namespace

WholeBodyTrajectory
fromPlanner(const wbmm_planning_msgs::msg::WholeBodyTrajectory &msg,
            std::size_t limit) {
  WholeBodyTrajectory out;
  out.trajectory_id = msg.trajectory_id;
  out.environment_revision = msg.environment_revision;
  out.collision_model_revision = msg.collision_model_revision;
  const auto n = msg.time_from_start.size(), joints = msg.joint_names.size();
  if (n == 0)
    return out;
  if (n > limit || msg.header.frame_id.empty() || msg.base_x.size() != n ||
      msg.base_y.size() != n || msg.base_yaw.size() != n ||
      msg.joint_positions.size() / n != joints ||
      msg.joint_positions.size() % n != 0)
    throw std::invalid_argument(
        "Malformed planner pose arrays or point budget exceeded.");
  namesValid(msg.joint_names);
  out.points.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    timeValid(msg.time_from_start[i], i ? msg.time_from_start[i - 1] : 0,
              i == 0, true);
    if (msg.time_from_start[i] < 0)
      throw std::invalid_argument("Negative planner relative time.");
    wbmm::core::WholeBodyTrajectoryPoint p;
    p.time_from_start = msg.time_from_start[i];
    p.state.header.frame_id = msg.header.frame_id;
    p.state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
    p.state.base.x = msg.base_x[i];
    p.state.base.y = msg.base_y[i];
    p.state.base.yaw = msg.base_yaw[i];
    p.state.joints.names = msg.joint_names;
    p.state.joints.positions.assign(msg.joint_positions.begin() + i * joints,
                                    msg.joint_positions.begin() +
                                        (i + 1) * joints);
    stateValid(p.state);
    out.points.push_back(std::move(p));
  }
  return out;
}

WholeBodyTrajectory fromMpc(const ocs2_msgs::msg::MpcFlattenedController &msg,
                            const std::string &frame,
                            const std::vector<std::string> &names,
                            std::size_t limit) {
  WholeBodyTrajectory out;
  out.trajectory_id = "mpc_prediction";
  const auto n = msg.time_trajectory.size();
  if (n == 0 && msg.state_trajectory.empty())
    return out;
  if (n == 0 || n > limit || msg.state_trajectory.size() != n || frame.empty())
    throw std::invalid_argument(
        "Malformed MPC state/time arrays or missing MPC frame.");
  namesValid(names);
  out.points.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
    // Hybrid MPC policies may contain pre/post-event states at the same time.
    timeValid(msg.time_trajectory[i], i ? msg.time_trajectory[i - 1] : 0,
              i == 0, false);
    const auto &values = msg.state_trajectory[i].value;
    if (values.size() != 3 + names.size())
      throw std::invalid_argument(
          "MPC state layout must be [x,y,yaw,q...] in configured joint order.");
    wbmm::core::WholeBodyTrajectoryPoint p;
    p.time_from_start = msg.time_trajectory[i] - msg.time_trajectory.front();
    if (!std::isfinite(p.time_from_start))
      throw std::invalid_argument("MPC time span overflow.");
    p.state.header.frame_id = frame;
    p.state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
    p.state.base.x = values[0];
    p.state.base.y = values[1];
    p.state.base.yaw = values[2];
    p.state.joints.names = names;
    p.state.joints.positions.assign(values.begin() + 3, values.end());
    stateValid(p.state);
    out.points.push_back(std::move(p));
  }
  return out;
}

std::vector<std::size_t> selectSamples(const WholeBodyTrajectory &trajectory,
                                       double interval, std::size_t max_poses) {
  if (!std::isfinite(interval) || interval <= 0 || max_poses < 2)
    throw std::invalid_argument(
        "Display interval must be positive; max_robot_poses >= 2.");
  const auto &p = trajectory.points;
  for (std::size_t i = 0; i < p.size(); ++i)
    timeValid(p[i].time_from_start, i ? p[i - 1].time_from_start : 0, i == 0,
              false);
  if (p.empty())
    return {};
  if (p.size() == 1)
    return {0};
  const double duration = p.back().time_from_start - p.front().time_from_start;
  if (!std::isfinite(duration))
    throw std::invalid_argument("Trajectory time span overflow.");
  const auto count = static_cast<std::size_t>(
      std::min(static_cast<double>(std::min(max_poses, p.size())),
               std::max(2.0, std::ceil(duration / interval) + 1.0)));
  std::set<std::size_t> selected{0, p.size() - 1};
  for (std::size_t k = 1; k + 1 < count; ++k) {
    const double t = p.front().time_from_start +
                     duration * static_cast<double>(k) / (count - 1);
    auto it = std::lower_bound(p.begin(), p.end(), t,
                               [](const auto &sample, double value) {
                                 return sample.time_from_start < value;
                               });
    if (it != p.end())
      selected.insert(static_cast<std::size_t>(std::distance(p.begin(), it)));
  }
  return {selected.begin(), selected.end()};
}

visualization_msgs::msg::MarkerArray
renderTrajectory(const RobotVisualModel &model,
                 const WholeBodyTrajectory &trajectory,
                 const std::string &layer, const DisplayOptions &options) {
  const auto selected = selectSamples(trajectory, options.sample_interval,
                                      options.max_robot_poses);
  if (selected.empty())
    return visualization_msgs::msg::MarkerArray{};
  const std::size_t paths = static_cast<std::size_t>(options.show_base_path) +
                            static_cast<std::size_t>(options.show_ee_path);
  if (options.max_markers < paths ||
      selected.size() > (options.max_markers - paths) / model.visualCount())
    throw std::invalid_argument(
        "Robot ghosts exceed max_markers; reduce max_robot_poses.");
  if (options.show_ee_path && !model.hasLink(options.ee_frame))
    throw std::invalid_argument("Unknown EE link for path overlay.");
  if (!std::isfinite(options.line_width) || options.line_width <= 0)
    throw std::invalid_argument("Invalid path line width.");
  // Validate every pose, even if it will be omitted from the sparse display.
  for (const auto &p : trajectory.points) {
    model.validateState(p.state);
    if (p.state.header.frame_id !=
        trajectory.points.front().state.header.frame_id)
      throw std::invalid_argument("Mixed-frame visual trajectory.");
  }
  visualization_msgs::msg::MarkerArray out;
  for (std::size_t ghost = 0; ghost < selected.size(); ++ghost) {
    auto markers =
        model.markers(trajectory.points[selected[ghost]].state,
                      layer + "/robot_" + std::to_string(ghost), options.style);
    out.markers.insert(out.markers.end(),
                       std::make_move_iterator(markers.markers.begin()),
                       std::make_move_iterator(markers.markers.end()));
  }
  if (options.show_base_path || options.show_ee_path) {
    visualization_msgs::msg::Marker base;
    base.ns = layer + "/base_path";
    base.id = 0;
    base.header.frame_id = trajectory.points.front().state.header.frame_id;
    base.type = base.LINE_STRIP;
    base.action = base.ADD;
    base.pose.orientation.w = 1;
    base.scale.x = options.line_width;
    base.color.r = options.style.red;
    base.color.g = options.style.green;
    base.color.b = options.style.blue;
    base.color.a = 0.9;
    auto ee = base;
    ee.ns = layer + "/ee_path";
    for (const auto index : selected) {
      const auto &state = trajectory.points[index].state;
      geometry_msgs::msg::Point p;
      p.x = state.base.x;
      p.y = state.base.y;
      base.points.push_back(p);
      if (options.show_ee_path) {
        const auto poses = model.linkPoses(state);
        const auto &t = poses.at(options.ee_frame).translation();
        p.x = t.x();
        p.y = t.y();
        p.z = t.z();
        ee.points.push_back(p);
      }
    }
    if (options.show_base_path)
      out.markers.push_back(std::move(base));
    if (options.show_ee_path)
      out.markers.push_back(std::move(ee));
  }
  return out;
}

visualization_msgs::msg::MarkerArray
MarkerLayer::replace(visualization_msgs::msg::MarkerArray current) {
  std::map<std::pair<std::string, int>, std_msgs::msg::Header> next;
  for (const auto &m : current.markers)
    next[{m.ns, m.id}] = m.header;
  for (const auto &old : previous_)
    if (!next.count(old.first)) {
      visualization_msgs::msg::Marker m;
      m.ns = old.first.first;
      m.id = old.first.second;
      m.action = m.DELETE;
      m.header = old.second;
      current.markers.push_back(std::move(m));
    }
  previous_ = std::move(next);
  return current;
}
} // namespace wbmm::visualization
