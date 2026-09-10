#include "wbmm_visualization/markers.hpp"

#include <geometry_msgs/msg/point.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <utility>

namespace wbmm_viz
{

namespace
{
geometry_msgs::msg::Point point(const Eigen::Vector3d & value)
{
  geometry_msgs::msg::Point result;
  result.x = value.x();
  result.y = value.y();
  result.z = value.z();
  return result;
}

std_msgs::msg::Header headerFor(const MarkerContext & ctx)
{
  std_msgs::msg::Header result;
  result.frame_id = ctx.frame_id;
  result.stamp = ctx.stamp;
  return result;
}

void fillColor(visualization_msgs::msg::Marker & marker,
               const Eigen::Vector4d & rgba)
{
  marker.color.r = static_cast<float>(rgba.x());
  marker.color.g = static_cast<float>(rgba.y());
  marker.color.b = static_cast<float>(rgba.z());
  marker.color.a = static_cast<float>(rgba.w());
}
}  // namespace

void beginMarkerArray(visualization_msgs::msg::MarkerArray & out,
                      MarkerContext & ctx, const std::string & frame_id,
                      const rclcpp::Time & stamp)
{
  out.markers.clear();
  ctx.frame_id = frame_id;
  ctx.stamp = stamp;
  ctx.next_id = 0;
  visualization_msgs::msg::Marker clear;
  clear.header = headerFor(ctx);
  clear.action = visualization_msgs::msg::Marker::DELETEALL;
  out.markers.push_back(std::move(clear));
}

void appendRobotSnapshot(visualization_msgs::msg::MarkerArray & out,
                         MarkerContext & ctx,
                         const std::vector<VisualGeometry> & geometry,
                         const std::string & ns, const Eigen::Vector4d & rgba,
                         bool embedded_materials, bool include_tool_axis,
                         const Eigen::Vector3d * tool_pos,
                         const Eigen::Matrix3d * tool_rot)
{
  const auto header = headerFor(ctx);
  for (const auto & item : geometry) {
    visualization_msgs::msg::Marker mesh;
    mesh.header = header;
    mesh.ns = ns;
    mesh.id = ctx.next_id++;
    mesh.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mesh.action = visualization_msgs::msg::Marker::ADD;
    mesh.mesh_resource = item.mesh_path;
    mesh.mesh_use_embedded_materials = embedded_materials;
    mesh.pose.position = point(item.position);
    mesh.pose.orientation.x = item.orientation.x();
    mesh.pose.orientation.y = item.orientation.y();
    mesh.pose.orientation.z = item.orientation.z();
    mesh.pose.orientation.w = item.orientation.w();
    mesh.scale.x = item.mesh_scale.x();
    mesh.scale.y = item.mesh_scale.y();
    mesh.scale.z = item.mesh_scale.z();
    fillColor(mesh, rgba);
    out.markers.push_back(std::move(mesh));
  }
  if (include_tool_axis && tool_pos != nullptr && tool_rot != nullptr) {
    visualization_msgs::msg::Marker tool_axis;
    tool_axis.header = header;
    tool_axis.ns = ns + "_tool_axis";
    tool_axis.id = ctx.next_id++;
    tool_axis.type = visualization_msgs::msg::Marker::ARROW;
    tool_axis.action = visualization_msgs::msg::Marker::ADD;
    tool_axis.points.push_back(point(*tool_pos));
    tool_axis.points.push_back(point(*tool_pos + 0.16 * tool_rot->col(2)));
    tool_axis.scale.x = 0.012;
    tool_axis.scale.y = 0.028;
    tool_axis.scale.z = 0.040;
    Eigen::Vector4d arrow_color = rgba;
    arrow_color.w() = std::min(1.0, rgba.w() + 0.35);
    fillColor(tool_axis, arrow_color);
    out.markers.push_back(std::move(tool_axis));
  }
}

void appendLineStrip(visualization_msgs::msg::MarkerArray & out,
                     MarkerContext & ctx, const std::string & ns,
                     const std::vector<Eigen::Vector3d> & points,
                     const Eigen::Vector4d & rgba, double width)
{
  visualization_msgs::msg::Marker line;
  line.header = headerFor(ctx);
  line.ns = ns;
  line.id = ctx.next_id++;
  line.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line.action = visualization_msgs::msg::Marker::ADD;
  line.pose.orientation.w = 1.0;
  line.scale.x = width;
  fillColor(line, rgba);
  for (const auto & value : points) {
    line.points.push_back(point(value));
  }
  out.markers.push_back(std::move(line));
}

void appendTextLabel(visualization_msgs::msg::MarkerArray & out,
                     MarkerContext & ctx, const std::string & ns,
                     const Eigen::Vector3d & position, double height,
                     const Eigen::Vector4d & rgba, const std::string & text)
{
  visualization_msgs::msg::Marker label;
  label.header = headerFor(ctx);
  label.ns = ns;
  label.id = ctx.next_id++;
  label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  label.action = visualization_msgs::msg::Marker::ADD;
  label.pose.position = point(position);
  label.pose.orientation.w = 1.0;
  label.scale.z = height;
  fillColor(label, rgba);
  label.text = text;
  out.markers.push_back(std::move(label));
}

const Eigen::Vector4d & segmentColor(std::size_t segment)
{
  static const std::vector<Eigen::Vector4d> palette{
    {0.20, 0.65, 1.00, 1.0}, {1.00, 0.55, 0.10, 1.0},
    {0.20, 0.90, 0.35, 1.0}, {0.85, 0.30, 0.95, 1.0},
    {1.00, 0.85, 0.15, 1.0}, {0.10, 0.85, 0.85, 1.0},
    {0.95, 0.25, 0.40, 1.0}, {0.55, 0.80, 0.20, 1.0},
    {0.45, 0.45, 1.00, 1.0}, {1.00, 0.45, 0.70, 1.0}};
  return palette[segment % palette.size()];
}

const Eigen::Vector4d & phaseColor(const std::string & phase)
{
  static const Eigen::Vector4d navigate{0.55, 0.62, 0.72, 1.0};
  static const Eigen::Vector4d align{1.0, 0.75, 0.05, 1.0};
  static const Eigen::Vector4d approach{1.0, 0.45, 0.05, 1.0};
  static const Eigen::Vector4d task{0.05, 1.0, 0.25, 1.0};
  static const Eigen::Vector4d completed{1.0, 1.0, 1.0, 1.0};
  static const Eigen::Vector4d failed{1.0, 0.25, 0.25, 1.0};
  static const Eigen::Vector4d unknown{0.7, 0.7, 0.7, 1.0};
  if (phase == "NAVIGATE" || phase == "remani_navigation" ||
    phase == "waiting_navigation")
  {
    return navigate;
  }
  if (phase == "PRECONTACT_ALIGN" || phase == "wipe_planning") {
    return align;
  }
  if (phase == "PRECONTACT_APPROACH") {
    return approach;
  }
  if (phase == "TASK_CONSTRAINED" || phase == "continuous_contact_wiping") {
    return task;
  }
  if (phase == "completed") {
    return completed;
  }
  if (phase == "failed") {
    return failed;
  }
  return unknown;
}

void buildTimeSegments(const ParsedTrajectory & trajectory,
                       const WholeBodyKinematics & kinematics,
                       const TimeSegmentSpec & spec,
                       visualization_msgs::msg::MarkerArray & out,
                       MarkerContext & ctx)
{
  const double segment_duration = std::max(0.5, spec.duration);
  const int segment_count = std::max(
    1, static_cast<int>(std::ceil(
      trajectory.duration / segment_duration)));
  const int snapshots_per_segment = std::max(
    0, static_cast<int>(spec.snapshots_per_segment));
  for (int segment = 0; segment < segment_count; ++segment) {
    const double start = segment * segment_duration;
    const double end = std::min(
      trajectory.duration, (segment + 1) * segment_duration);
    const auto samples = samplesInInterval(trajectory, start, end);
    const Eigen::Vector4d color = segmentColor(
      static_cast<std::size_t>(segment));
    const std::string prefix = "time_segment_" + std::to_string(segment);

    std::vector<Eigen::Vector3d> base_points;
    std::vector<Eigen::Vector3d> ee_points;
    base_points.reserve(samples.size());
    ee_points.reserve(samples.size());
    for (const auto & sample : samples) {
      base_points.push_back(Eigen::Vector3d(
        sample.state[0], sample.state[1], 0.055));
      ee_points.push_back(kinematics.eePosition(toStateVector(sample)));
    }
    Eigen::Vector4d line_color = color;
    line_color.w() = 0.95;
    appendLineStrip(
      out, ctx, prefix + "_base", base_points, line_color, 0.045);
    appendLineStrip(
      out, ctx, prefix + "_ee", ee_points, line_color, 0.025);

    const Eigen::Vector3d label_position(
      samples.front().state[0], samples.front().state[1], 0.82);
    std::ostringstream label_text;
    label_text << "S" << segment << " [" << std::fixed
               << std::setprecision(1) << start << ", " << end << "] s";
    appendTextLabel(out, ctx, "time_segment_labels", label_position, 0.085,
                    color, label_text.str());

    const int snapshot_count = std::min<int>(
      snapshots_per_segment, static_cast<int>(samples.size()));
    for (int snapshot = 0; snapshot < snapshot_count; ++snapshot) {
      const std::size_t index = static_cast<std::size_t>(std::llround(
        static_cast<double>(snapshot + 1) * (samples.size() - 1) /
        static_cast<double>(snapshot_count + 1)));
      Eigen::Vector4d snapshot_color = color;
      snapshot_color.w() = 0.20;
      const Eigen::VectorXd state = toStateVector(samples[index]);
      const Eigen::Vector3d tool = kinematics.eePosition(state);
      const Eigen::Matrix3d tool_rotation = kinematics.eeRotation(state);
      appendRobotSnapshot(
        out, ctx, kinematics.visualGeometry(state), prefix + "_robot",
        snapshot_color, false, true, &tool, &tool_rotation);
    }
  }
}

void buildPlaybackFrame(const ParsedTrajectory & trajectory,
                        double playback_time, const PlaybackSpec & spec,
                        const WholeBodyKinematics & kinematics,
                        visualization_msgs::msg::MarkerArray & out,
                        MarkerContext & ctx)
{
  if (trajectory.waypoints.empty()) {
    return;
  }
  const double segment_duration = std::max(0.5, spec.segment_duration);
  const std::size_t segment_count = static_cast<std::size_t>(std::max(
    1.0, std::ceil(trajectory.duration / segment_duration)));
  const std::size_t segment = std::min(
    segment_count - 1,
    static_cast<std::size_t>(std::floor(
      playback_time / segment_duration)));
  const double segment_start = segment * segment_duration;
  const double segment_end = std::min(
    trajectory.duration, (segment + 1) * segment_duration);
  const Eigen::Vector4d color = segmentColor(segment);
  const auto complete_window = samplesInInterval(
    trajectory, segment_start, segment_end);
  const auto completed = samplesInInterval(trajectory, 0.0, playback_time);
  const Waypoint current = waypointAt(trajectory, playback_time);

  std::vector<Eigen::Vector3d> window_base;
  std::vector<Eigen::Vector3d> window_ee;
  window_base.reserve(complete_window.size());
  window_ee.reserve(complete_window.size());
  for (const auto & sample : complete_window) {
    window_base.push_back(Eigen::Vector3d(
      sample.state[0], sample.state[1], 0.055));
    window_ee.push_back(kinematics.eePosition(toStateVector(sample)));
  }
  std::vector<Eigen::Vector3d> played_base;
  std::vector<Eigen::Vector3d> played_ee;
  played_base.reserve(completed.size());
  played_ee.reserve(completed.size());
  for (const auto & sample : completed) {
    played_base.push_back(Eigen::Vector3d(
      sample.state[0], sample.state[1], 0.055));
    played_ee.push_back(kinematics.eePosition(toStateVector(sample)));
  }

  Eigen::Vector4d window_color = color;
  window_color.w() = 0.22;
  appendLineStrip(out, ctx, "active_window_base", window_base,
                  window_color, 0.055);
  appendLineStrip(out, ctx, "active_window_ee", window_ee,
                  window_color, 0.035);
  appendLineStrip(out, ctx, "played_base", played_base, color, 0.060);
  appendLineStrip(out, ctx, "played_ee", played_ee, color, 0.040);

  const Eigen::Vector3d tool = kinematics.eePosition(toStateVector(current));
  std::ostringstream text;
  text << "t=" << std::fixed << std::setprecision(1) << playback_time
       << " s | S" << segment << " | " << current.phase;
  appendTextLabel(out, ctx, "playback_time", tool + Eigen::Vector3d(0.0, 0.0, 0.24),
                  0.11, color, text.str());
}

void buildBasePath(const ParsedTrajectory & trajectory,
                   const std::string & phase_filter, nav_msgs::msg::Path & out)
{
  out.poses.clear();
  out.header.frame_id = trajectory.frame_id;
  for (const auto & waypoint : trajectory.waypoints) {
    if (!phase_filter.empty() && waypoint.phase != phase_filter) {
      continue;
    }
    geometry_msgs::msg::PoseStamped pose;
    pose.header = out.header;
    pose.pose.position.x = waypoint.state[0];
    pose.pose.position.y = waypoint.state[1];
    pose.pose.orientation.z = std::sin(0.5 * waypoint.state[2]);
    pose.pose.orientation.w = std::cos(0.5 * waypoint.state[2]);
    out.poses.push_back(std::move(pose));
  }
}

void buildEePath(const ParsedTrajectory & trajectory,
                 const WholeBodyKinematics & kinematics,
                 const std::string & phase_filter, nav_msgs::msg::Path & out)
{
  out.poses.clear();
  out.header.frame_id = trajectory.frame_id;
  for (const auto & waypoint : trajectory.waypoints) {
    if (!phase_filter.empty() && waypoint.phase != phase_filter) {
      continue;
    }
    const Eigen::VectorXd state = toStateVector(waypoint);
    const Eigen::Quaterniond orientation(kinematics.eeRotation(state));
    geometry_msgs::msg::PoseStamped pose;
    pose.header = out.header;
    pose.pose.position = point(kinematics.eePosition(state));
    pose.pose.orientation.x = orientation.x();
    pose.pose.orientation.y = orientation.y();
    pose.pose.orientation.z = orientation.z();
    pose.pose.orientation.w = orientation.w();
    out.poses.push_back(std::move(pose));
  }
}

}  // namespace wbmm_viz
