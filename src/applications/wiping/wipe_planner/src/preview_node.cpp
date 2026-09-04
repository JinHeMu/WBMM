#include "wipe_planner/planner.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <functional>
#include <iomanip>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace wipe_planner
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

std::vector<std::size_t> evenlySpaced(
  const std::vector<std::size_t> & candidates, int requested)
{
  if (candidates.empty() || requested <= 0) {
    return {};
  }
  const int count = std::min<int>(requested, candidates.size());
  std::vector<std::size_t> selected;
  selected.reserve(static_cast<std::size_t>(count));
  for (int index = 0; index < count; ++index) {
    const auto candidate = static_cast<std::size_t>(std::llround(
      static_cast<double>(index) * (candidates.size() - 1) /
      std::max(1, count - 1)));
    selected.push_back(candidates[candidate]);
  }
  return selected;
}

struct FutureTaskEntry
{
  int id{0};
  bool success{false};
  Eigen::VectorXd state{Eigen::VectorXd::Zero(9)};
  double execution_time{0.0};
  double approach_time{0.0};
  double base_travel{0.0};
  double normalized_joint_margin{0.0};
  double manipulability{0.0};
  double self_collision_clearance{0.0};
  std::string error;

  bool qualityAlert() const
  {
    return !success || normalized_joint_margin < 0.01 ||
           manipulability < 0.02 || self_collision_clearance < 0.001;
  }
};

std::vector<std::string> splitCsv(const std::string & line)
{
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

std::vector<FutureTaskEntry> loadFutureTaskEntries(const std::string & csv_file)
{
  std::ifstream stream(csv_file);
  if (!stream) {
    throw std::runtime_error("Cannot open future-task CSV: " + csv_file);
  }
  std::string line_value;
  std::getline(stream, line_value);
  const auto header = splitCsv(line_value);
  std::map<std::string, std::size_t> column;
  for (std::size_t index = 0; index < header.size(); ++index) {
    column[header[index]] = index;
  }
  const auto required = [&column](const std::string & name) {
      const auto match = column.find(name);
      if (match == column.end()) {
        throw std::runtime_error("Future-task CSV is missing column: " + name);
      }
      return match->second;
    };
  const std::vector<std::string> state_names{
    "base_x", "base_y", "base_yaw", "q1", "q2", "q3", "q4", "q5", "q6"};
  std::vector<FutureTaskEntry> entries;
  while (std::getline(stream, line_value)) {
    const auto fields = splitCsv(line_value);
    if (fields.size() < header.size()) {
      continue;
    }
    FutureTaskEntry entry;
    entry.id = std::stoi(fields[required("run_id")]);
    entry.success = std::stoi(fields[required("success")]) != 0;
    for (int state_index = 0; state_index < 9; ++state_index) {
      entry.state[state_index] = std::stod(fields[required(
        state_names[static_cast<std::size_t>(state_index)])]);
    }
    entry.execution_time = std::stod(fields[required("nominal_execution_time_s")]);
    entry.approach_time = std::stod(fields[required("approach_time_s")]);
    entry.base_travel = std::stod(fields[required("base_travel_m")]);
    entry.normalized_joint_margin = std::stod(
      fields[required("min_joint_margin_normalized")]);
    entry.manipulability = std::stod(fields[required("min_manipulability")]);
    entry.self_collision_clearance = std::stod(
      fields[required("min_self_collision_clearance_m")]);
    entry.error = fields[required("error")];
    entries.push_back(std::move(entry));
  }
  if (entries.empty()) {
    throw std::runtime_error("Future-task CSV contains no result rows: " + csv_file);
  }
  return entries;
}
}  // namespace

class WipePlanPreviewNode final : public rclcpp::Node
{
public:
  WipePlanPreviewNode()
  : Node("wipe_plan_preview")
  {
    declare_parameter("world_frame", "odom");
    declare_parameter("ee_frame", "tool0");
    declare_parameter("urdf_file", "");
    declare_parameter("task_file", "");
    declare_parameter("publish_delay", 0.5);
    declare_parameter("coverage_snapshots", 14);
    declare_parameter("coverage_alpha", 0.15);
    declare_parameter("show_plan_summary", true);
    declare_parameter("future_task_results_csv", "");
    declare_parameter<std::vector<double>>(
      "initial_state",
      {0.75, 2.06, 0.0, 0.0, 1.5707, 0.0, 1.5707, 3.14159, 0.785398});

    const auto transient = rclcpp::QoS(1).reliable().transient_local();
    base_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/wipe_planner/preview/base_path", transient);
    ee_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/wipe_planner/preview/ee_coverage_path", transient);
    scene_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/wipe_planner/preview/scene", transient);
    coverage_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/wipe_planner/back_end_mm_mesh_vis", transient);
    future_task_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/wipe_planner/future_task_results", transient);
    future_task_failure_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>(
      "/wipe_planner/future_task_failures", transient);
    status_publisher_ = create_publisher<std_msgs::msg::String>(
      "/wipe_planner/preview/status", transient);

    const std::string urdf_file = get_parameter("urdf_file").as_string();
    const std::string task_file = get_parameter("task_file").as_string();
    if (urdf_file.empty() || task_file.empty()) {
      throw std::runtime_error("urdf_file and task_file are required");
    }
    planner_ = std::make_unique<Planner>(
      urdf_file, get_parameter("ee_frame").as_string(), task_file);

    const double delay = std::max(0.05, get_parameter("publish_delay").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration<double>(delay),
      std::bind(&WipePlanPreviewNode::planAndPublish, this));
    RCLCPP_INFO(
      get_logger(),
      "Preview-only mode ready: this node publishes paths and MarkerArrays only; "
      "no robot, MPC, REMANI, or control-command interface is started.");
  }

private:
  std_msgs::msg::Header header() const
  {
    std_msgs::msg::Header value;
    value.frame_id = get_parameter("world_frame").as_string();
    value.stamp = now();
    return value;
  }

  visualization_msgs::msg::Marker line(
    const std_msgs::msg::Header & marker_header, const std::string & name,
    int id, double width, float red, float green, float blue) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header = marker_header;
    marker.ns = name;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = width;
    marker.color.r = red;
    marker.color.g = green;
    marker.color.b = blue;
    marker.color.a = 1.0F;
    return marker;
  }

  visualization_msgs::msg::MarkerArray robotSnapshots(
    const std::vector<Waypoint> & trajectory,
    const std::vector<std::size_t> & indices,
    const std::string & marker_namespace,
    const Eigen::Vector3d & color, double alpha) const
  {
    visualization_msgs::msg::MarkerArray result;
    visualization_msgs::msg::Marker clear;
    clear.header = header();
    clear.ns = marker_namespace;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    result.markers.push_back(clear);
    int marker_id = 100;
    for (const std::size_t index : indices) {
      appendRobotSnapshot(
        result, trajectory[index].state, clear.header, marker_namespace,
        marker_id, color, alpha, Eigen::Vector3d(0.05, 0.25, 1.0));
    }
    return result;
  }

  void appendRobotSnapshot(
    visualization_msgs::msg::MarkerArray & result,
    const Eigen::VectorXd & state,
    const std_msgs::msg::Header & marker_header,
    const std::string & marker_namespace,
    int & marker_id,
    const Eigen::Vector3d & color,
    double alpha,
    const Eigen::Vector3d & axis_color) const
  {
    for (const auto & geometry : planner_->visualGeometry(state)) {
      visualization_msgs::msg::Marker marker;
      marker.header = marker_header;
      marker.ns = marker_namespace;
      marker.id = marker_id++;
      marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.mesh_resource = geometry.mesh_path;
      marker.mesh_use_embedded_materials = false;
      marker.pose.position = point(geometry.position);
      marker.pose.orientation.x = geometry.orientation.x();
      marker.pose.orientation.y = geometry.orientation.y();
      marker.pose.orientation.z = geometry.orientation.z();
      marker.pose.orientation.w = geometry.orientation.w();
      marker.scale.x = geometry.mesh_scale.x();
      marker.scale.y = geometry.mesh_scale.y();
      marker.scale.z = geometry.mesh_scale.z();
      marker.color.r = static_cast<float>(color.x());
      marker.color.g = static_cast<float>(color.y());
      marker.color.b = static_cast<float>(color.z());
      marker.color.a = static_cast<float>(alpha);
      result.markers.push_back(std::move(marker));
    }

    const Eigen::Vector3d tool_position = planner_->framePosition(
      state, get_parameter("ee_frame").as_string());
    const Eigen::Vector3d tool_axis = planner_->frameRotation(
      state, get_parameter("ee_frame").as_string()).col(2);
    visualization_msgs::msg::Marker axis;
    axis.header = marker_header;
    axis.ns = marker_namespace + "_tool0_z";
    axis.id = marker_id++;
    axis.type = visualization_msgs::msg::Marker::ARROW;
    axis.action = visualization_msgs::msg::Marker::ADD;
    axis.points.push_back(point(tool_position));
    axis.points.push_back(point(tool_position + 0.18 * tool_axis));
    axis.scale.x = 0.014;
    axis.scale.y = 0.032;
    axis.scale.z = 0.045;
    axis.color.r = static_cast<float>(axis_color.x());
    axis.color.g = static_cast<float>(axis_color.y());
    axis.color.b = static_cast<float>(axis_color.z());
    axis.color.a = 1.0F;
    result.markers.push_back(std::move(axis));
  }

  void publishScene(
    const std::vector<Waypoint> & trajectory,
    const PlanReport & report,
    const std_msgs::msg::Header & marker_header)
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = marker_header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    const Eigen::Vector3d center = planner_->surfaceCenter();
    const Eigen::Vector2d x_limits = planner_->surfaceXLimits();
    const Eigen::Vector2d y_limits = planner_->surfaceYLimits();
    const Eigen::Vector2d z_limits = planner_->surfaceZLimits();
    visualization_msgs::msg::Marker board;
    board.header = marker_header;
    board.ns = "known_board";
    board.id = 0;
    board.type = visualization_msgs::msg::Marker::CUBE;
    board.action = visualization_msgs::msg::Marker::ADD;
    board.pose.position.x = 0.5 * (x_limits.x() + x_limits.y());
    board.pose.orientation.w = 1.0;
    board.scale.x = x_limits.y() - x_limits.x();
    if (planner_->surfaceType() == "horizontal") {
      board.ns = "known_table";
      board.pose.position.y = 0.5 * (y_limits.x() + y_limits.y());
      board.pose.position.z = center.z() - 0.025;
      board.scale.y = y_limits.y() - y_limits.x();
      board.scale.z = 0.05;
    } else {
      // Wall-parallel board: position and yaw follow the wall direction so
      // the cube stays flush on the surface plane for walls along any odom
      // axis (e.g. plane X=const). The cube's local Y is -normal_into_room,
      // so the near face lands exactly on the contact plane.
      const Eigen::Vector3d wall_dir = planner_->wallDirection();
      const double wall_offset = 0.5 * (x_limits.x() + x_limits.y()) -
                                 center.dot(wall_dir);
      board.pose.position.x = center.x() + wall_dir.x() * wall_offset;
      board.pose.position.y = center.y() + wall_dir.y() * wall_offset;
      board.pose.position.z = 0.5 * (z_limits.x() + z_limits.y());
      const double yaw = std::atan2(wall_dir.y(), wall_dir.x());
      board.pose.orientation.z = std::sin(0.5 * yaw);
      board.pose.orientation.w = std::cos(0.5 * yaw);
      board.scale.x = x_limits.y() - x_limits.x();
      board.scale.y = 0.05;
      board.scale.z = z_limits.y() - z_limits.x();
    }
    board.color.r = 0.15F;
    board.color.g = 0.45F;
    board.color.b = 0.95F;
    board.color.a = 0.92F;
    markers.markers.push_back(std::move(board));

    auto base_line = line(marker_header, "differential_base_path", 0,
      0.035, 0.40F, 0.45F, 0.52F);
    auto coverage_line = line(marker_header, "constrained_ee_coverage", 0,
      0.028, 0.05F, 1.0F, 0.30F);
    auto target_line = line(marker_header, "surface_contact_targets", 0,
      0.010, 1.0F, 0.15F, 0.10F);

    std::vector<std::size_t> contact_indices;
    nav_msgs::msg::Path base_path;
    nav_msgs::msg::Path ee_path;
    base_path.header = marker_header;
    ee_path.header = marker_header;
    for (std::size_t index = 0; index < trajectory.size(); ++index) {
      const auto & waypoint = trajectory[index];
      geometry_msgs::msg::PoseStamped base_pose;
      base_pose.header = marker_header;
      base_pose.pose.position.x = waypoint.state[0];
      base_pose.pose.position.y = waypoint.state[1];
      base_pose.pose.orientation.z = std::sin(0.5 * waypoint.state[2]);
      base_pose.pose.orientation.w = std::cos(0.5 * waypoint.state[2]);
      base_path.poses.push_back(std::move(base_pose));
      base_line.points.push_back(point(Eigen::Vector3d(
        waypoint.state[0], waypoint.state[1], 0.045)));

      const Eigen::Vector3d ee = planner_->framePosition(
        waypoint.state, get_parameter("ee_frame").as_string());
      if (waypoint.in_contact) {
        contact_indices.push_back(index);
        coverage_line.points.push_back(point(ee));
        target_line.points.push_back(point(waypoint.contact_target));
        geometry_msgs::msg::PoseStamped ee_pose;
        ee_pose.header = marker_header;
        ee_pose.pose.position = point(ee);
        const Eigen::Quaterniond orientation(planner_->frameRotation(
          waypoint.state, get_parameter("ee_frame").as_string()));
        ee_pose.pose.orientation.x = orientation.x();
        ee_pose.pose.orientation.y = orientation.y();
        ee_pose.pose.orientation.z = orientation.z();
        ee_pose.pose.orientation.w = orientation.w();
        ee_path.poses.push_back(std::move(ee_pose));
      }
    }
    markers.markers.push_back(std::move(base_line));
    markers.markers.push_back(std::move(coverage_line));
    markers.markers.push_back(std::move(target_line));

    if (!contact_indices.empty()) {
      for (int endpoint = 0; endpoint < 2; ++endpoint) {
        const std::size_t selected = endpoint == 0 ?
          contact_indices.front() : contact_indices.back();
        visualization_msgs::msg::Marker sphere;
        sphere.header = marker_header;
        sphere.ns = endpoint == 0 ? "wipe_start" : "wipe_end";
        sphere.id = 0;
        sphere.type = visualization_msgs::msg::Marker::SPHERE;
        sphere.action = visualization_msgs::msg::Marker::ADD;
        sphere.pose.position = point(trajectory[selected].contact_target);
        sphere.pose.orientation.w = 1.0;
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.085;
        sphere.color.r = endpoint == 0 ? 0.05F : 1.0F;
        sphere.color.g = endpoint == 0 ? 1.0F : 0.05F;
        sphere.color.b = 0.05F;
        sphere.color.a = 1.0F;
        markers.markers.push_back(std::move(sphere));
      }
    }

    std::ostringstream summary;
    summary << "WipePlanner: " << report.points << " points, "
            << std::fixed << std::setprecision(1) << report.duration << " s";
    if (get_parameter("show_plan_summary").as_bool()) {
      visualization_msgs::msg::Marker label;
      label.header = marker_header;
      label.ns = "plan_summary";
      label.id = 0;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      if (planner_->surfaceType() == "horizontal") {
        label.pose.position.x = x_limits.x();
        label.pose.position.y = y_limits.x() - 0.08;
        label.pose.position.z = center.z();
      } else {
        // Vertical: anchor the label on the wall at the x_limits lower end.
        const Eigen::Vector3d wall_dir = planner_->wallDirection();
        const Eigen::Vector3d wall_point = center +
          wall_dir * (x_limits.x() - center.dot(wall_dir));
        label.pose.position.x = wall_point.x();
        label.pose.position.y = wall_point.y();
        label.pose.position.z = z_limits.y() + 0.18;
      }
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.12;
      label.color.r = label.color.g = label.color.b = label.color.a = 1.0F;
      label.text = summary.str();
      markers.markers.push_back(std::move(label));
    }

    base_path_publisher_->publish(base_path);
    ee_path_publisher_->publish(ee_path);
    scene_publisher_->publish(markers);

    const auto selected_coverage = evenlySpaced(
      contact_indices, get_parameter("coverage_snapshots").as_int());
    auto robot_markers = robotSnapshots(
      trajectory, selected_coverage, "wipe_back_end_robot",
      Eigen::Vector3d(0.10, 0.55, 1.0),
      get_parameter("coverage_alpha").as_double());
    int precontact_marker_id = 100;
    appendRobotSnapshot(
      robot_markers, trajectory.front().state, marker_header,
      "wipe_precontact_robot", precontact_marker_id,
      Eigen::Vector3d(1.0, 0.82, 0.05), 0.82,
      Eigen::Vector3d(1.0, 0.82, 0.05));
    coverage_publisher_->publish(robot_markers);

    std_msgs::msg::String status;
    status.data = summary.str();
    status_publisher_->publish(status);
    RCLCPP_INFO(
      get_logger(),
      "Plan published: %zu waypoints, %.1f s, %zu EE poses, "
      "%zu constrained whole-body ghosts plus one yellow pre-contact pose; "
      "Hybrid A* expanded=%d, IK rejected=%d, collision rejected=%d",
      trajectory.size(), report.duration, ee_path.poses.size(),
      selected_coverage.size(),
      report.hybrid_expanded_nodes, report.reachability_rejections,
      report.collision_rejections);
  }

  void publishFutureTaskResults(
    const std::vector<FutureTaskEntry> & entries,
    const std::vector<Waypoint> & canonical_trajectory,
    const std_msgs::msg::Header & marker_header)
  {
    (void)canonical_trajectory;
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = marker_header;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);

    std::vector<const FutureTaskEntry *> successful;
    for (const auto & entry : entries) {
      if (entry.success) {
        successful.push_back(&entry);
      }
    }
    if (successful.empty()) {
      throw std::runtime_error("Future-task CSV contains no successful rollout");
    }
    const auto longest = *std::max_element(
      successful.begin(), successful.end(), [](const auto * first, const auto * second) {
        return first->execution_time < second->execution_time;
      });
    const auto shortest = *std::min_element(
      successful.begin(), successful.end(), [](const auto * first, const auto * second) {
        return first->execution_time < second->execution_time;
      });
    const auto minimum_clearance = *std::min_element(
      successful.begin(), successful.end(), [](const auto * first, const auto * second) {
        return first->self_collision_clearance < second->self_collision_clearance;
      });

    visualization_msgs::msg::Marker entry_points;
    entry_points.header = marker_header;
    entry_points.ns = "precontact_base_entries";
    entry_points.id = 0;
    entry_points.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    entry_points.action = visualization_msgs::msg::Marker::ADD;
    entry_points.pose.orientation.w = 1.0;
    entry_points.scale.x = entry_points.scale.y = entry_points.scale.z = 0.065;

    visualization_msgs::msg::Marker task_rays;
    task_rays.header = marker_header;
    task_rays.ns = "base_to_common_precontact_tool";
    task_rays.id = 0;
    task_rays.type = visualization_msgs::msg::Marker::LINE_LIST;
    task_rays.action = visualization_msgs::msg::Marker::ADD;
    task_rays.pose.orientation.w = 1.0;
    task_rays.scale.x = 0.004;

    const Eigen::Vector3d common_precontact = planner_->precontactTarget();
    for (const auto & entry : entries) {
      std_msgs::msg::ColorRGBA color;
      color.r = entry.success ? 0.05F : 1.0F;
      color.g = entry.success ? 0.95F : 0.08F;
      color.b = entry.success ? 0.20F : 0.05F;
      color.a = 0.92F;
      const Eigen::Vector3d base(entry.state[0], entry.state[1], 0.06);
      entry_points.points.push_back(point(base));
      entry_points.colors.push_back(color);
      task_rays.points.push_back(point(base));
      task_rays.points.push_back(point(common_precontact));
      color.a = 0.08F;
      task_rays.colors.push_back(color);
      task_rays.colors.push_back(color);
    }
    markers.markers.push_back(std::move(task_rays));
    markers.markers.push_back(std::move(entry_points));

    struct Highlight
    {
      const FutureTaskEntry * entry;
      std::string name;
      Eigen::Vector3d color;
      std::string metric;
    };
    const std::vector<Highlight> highlights{
      {shortest, "shortest_success", Eigen::Vector3d(0.05, 0.75, 1.0),
        "shortest success #" + std::to_string(shortest->id) + " = " +
        [&]() {std::ostringstream value; value << std::fixed << std::setprecision(2)
          << shortest->execution_time << " s"; return value.str();}()},
      {longest, "longest_success", Eigen::Vector3d(1.0, 0.55, 0.04),
        "longest success #" + std::to_string(longest->id) + " = " +
        [&]() {std::ostringstream value; value << std::fixed << std::setprecision(2)
          << longest->execution_time << " s"; return value.str();}()},
      {minimum_clearance, "minimum_clearance", Eigen::Vector3d(0.85, 0.05, 0.85),
        "min clearance #" + std::to_string(minimum_clearance->id) + " = " +
        [&]() {std::ostringstream value; value << std::fixed << std::setprecision(3)
          << 1000.0 * minimum_clearance->self_collision_clearance << " mm";
          return value.str();}()}
    };

    int label_id = 0;
    for (std::size_t highlight_index = 0;
         highlight_index < highlights.size(); ++highlight_index)
    {
      const auto & highlight = highlights[highlight_index];
      int marker_id = 100;
      appendRobotSnapshot(
        markers, highlight.entry->state, marker_header, highlight.name,
        marker_id, highlight.color, 0.62, highlight.color);

      visualization_msgs::msg::Marker label;
      label.header = marker_header;
      label.ns = "future_task_highlight_labels";
      label.id = label_id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position = point(planner_->framePosition(
        highlight.entry->state, get_parameter("ee_frame").as_string()) +
        Eigen::Vector3d(0.0, -0.05, 0.10 + 0.09 * highlight_index));
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.075;
      label.color.r = static_cast<float>(highlight.color.x());
      label.color.g = static_cast<float>(highlight.color.y());
      label.color.b = static_cast<float>(highlight.color.z());
      label.color.a = 1.0F;
      label.text = highlight.metric;
      markers.markers.push_back(std::move(label));

      PlanReport report;
      const auto rollout = planner_->planFromPrecontact(
        highlight.entry->state, report);
      auto rollout_line = line(
        marker_header, highlight.name + "_future_wipe", 0, 0.018,
        static_cast<float>(highlight.color.x()),
        static_cast<float>(highlight.color.y()),
        static_cast<float>(highlight.color.z()));
      for (const auto & waypoint : rollout) {
        if (waypoint.in_contact) {
          rollout_line.points.push_back(point(planner_->framePosition(
            waypoint.state, get_parameter("ee_frame").as_string())));
        }
      }
      markers.markers.push_back(std::move(rollout_line));
    }

    struct FailureHighlight
    {
      std::string pattern;
      std::string name;
      std::string label;
      Eigen::Vector3d color;
    };
    const std::vector<FailureHighlight> failure_types{
      {"keypoint 1/360", "failure_entry", "ENTRY_FAIL@1/360",
        Eigen::Vector3d(1.0, 0.08, 0.05)},
      {"keypoint 270/360", "failure_late", "LATE_FAIL@270/360",
        Eigen::Vector3d(1.0, 0.38, 0.02)},
      {"no-lateral-slip", "failure_no_slip", "NO_SLIP_FAIL",
        Eigen::Vector3d(0.95, 0.05, 0.75)}
    };
    visualization_msgs::msg::MarkerArray failure_markers;
    visualization_msgs::msg::Marker clear_failures;
    clear_failures.header = marker_header;
    clear_failures.action = visualization_msgs::msg::Marker::DELETEALL;
    failure_markers.markers.push_back(std::move(clear_failures));
    std::vector<const FutureTaskEntry *> shown_failures;
    int failure_label_id = 0;
    for (const auto & failure_type : failure_types) {
      const auto failure = std::find_if(
        entries.begin(), entries.end(), [&failure_type](const FutureTaskEntry & entry) {
          return !entry.success &&
                 entry.error.find(failure_type.pattern) != std::string::npos;
        });
      if (failure == entries.end()) {
        continue;
      }
      shown_failures.push_back(&*failure);
      const std::string marker_namespace =
        failure_type.name + "_" + std::to_string(failure->id);
      int robot_marker_id = 100;
      appendRobotSnapshot(
        failure_markers, failure->state, marker_header, marker_namespace,
        robot_marker_id, failure_type.color, 0.82, failure_type.color);

      const Eigen::Vector3d base(failure->state[0], failure->state[1], 0.10);
      auto cross = line(
        marker_header, marker_namespace + "_cross", 0, 0.025,
        static_cast<float>(failure_type.color.x()),
        static_cast<float>(failure_type.color.y()),
        static_cast<float>(failure_type.color.z()));
      cross.points.push_back(point(base + Eigen::Vector3d(-0.07, -0.07, 0.0)));
      cross.points.push_back(point(base + Eigen::Vector3d(0.07, 0.07, 0.0)));
      failure_markers.markers.push_back(std::move(cross));
      auto cross_second = line(
        marker_header, marker_namespace + "_cross", 1, 0.025,
        static_cast<float>(failure_type.color.x()),
        static_cast<float>(failure_type.color.y()),
        static_cast<float>(failure_type.color.z()));
      cross_second.points.push_back(point(base + Eigen::Vector3d(-0.07, 0.07, 0.0)));
      cross_second.points.push_back(point(base + Eigen::Vector3d(0.07, -0.07, 0.0)));
      failure_markers.markers.push_back(std::move(cross_second));

      visualization_msgs::msg::Marker label;
      label.header = marker_header;
      label.ns = "future_task_failure_labels";
      label.id = failure_label_id++;
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      label.pose.position = point(base + Eigen::Vector3d(
        0.0, -0.12, 0.48 + 0.12 * failure_label_id));
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.085;
      label.color.r = static_cast<float>(failure_type.color.x());
      label.color.g = static_cast<float>(failure_type.color.y());
      label.color.b = static_cast<float>(failure_type.color.z());
      label.color.a = 1.0F;
      label.text = "FAIL #" + std::to_string(failure->id) + ": " +
        failure_type.label;
      failure_markers.markers.push_back(std::move(label));
    }

    future_task_publisher_->publish(markers);
    future_task_failure_publisher_->publish(failure_markers);
    RCLCPP_INFO(
      get_logger(),
      "Pre-contact-conditioned result: %zu/%zu full wipe successes, "
      "%zu representative failures shown; shortest=#%d %.2f s, "
      "longest=#%d %.2f s, min clearance=#%d %.3f mm",
      successful.size(), entries.size(), shown_failures.size(),
      shortest->id, shortest->execution_time,
      longest->id, longest->execution_time, minimum_clearance->id,
      1000.0 * minimum_clearance->self_collision_clearance);
  }

  void planAndPublish()
  {
    timer_->cancel();
    try {
      const auto values = get_parameter("initial_state").as_double_array();
      if (values.size() != 9) {
        throw std::runtime_error("initial_state must contain 9 values");
      }
      Eigen::VectorXd seed(9);
      for (std::size_t index = 0; index < values.size(); ++index) {
        seed[static_cast<Eigen::Index>(index)] = values[index];
      }
      PlanReport report;
      const auto trajectory = planner_->plan(seed, report);
      if (trajectory.empty()) {
        throw std::runtime_error("planner returned an empty trajectory");
      }
      const auto marker_header = header();
      publishScene(trajectory, report, marker_header);
      const std::string results_csv =
        get_parameter("future_task_results_csv").as_string();
      if (!results_csv.empty()) {
        publishFutureTaskResults(
          loadFutureTaskEntries(results_csv), trajectory, marker_header);
      }
    } catch (const std::exception & error) {
      RCLCPP_FATAL(get_logger(), "Preview planning failed: %s", error.what());
      rclcpp::shutdown();
    }
  }

  std::unique_ptr<Planner> planner_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr base_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ee_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr scene_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr coverage_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr future_task_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    future_task_failure_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_publisher_;
};
}  // namespace wipe_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<wipe_planner::WipePlanPreviewNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("wipe_plan_preview"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
