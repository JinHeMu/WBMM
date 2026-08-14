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
#include <functional>
#include <iomanip>
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
      for (const auto & geometry : planner_->visualGeometry(trajectory[index].state)) {
        visualization_msgs::msg::Marker marker;
        marker.header = clear.header;
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
        trajectory[index].state, get_parameter("ee_frame").as_string());
      const Eigen::Vector3d tool_axis = planner_->frameRotation(
        trajectory[index].state, get_parameter("ee_frame").as_string()).col(2);
      visualization_msgs::msg::Marker axis;
      axis.header = clear.header;
      axis.ns = marker_namespace + "_tool0_z";
      axis.id = marker_id++;
      axis.type = visualization_msgs::msg::Marker::ARROW;
      axis.action = visualization_msgs::msg::Marker::ADD;
      axis.points.push_back(point(tool_position));
      axis.points.push_back(point(tool_position + 0.18 * tool_axis));
      axis.scale.x = 0.014;
      axis.scale.y = 0.032;
      axis.scale.z = 0.045;
      axis.color.r = 0.05F;
      axis.color.g = 0.25F;
      axis.color.b = 1.0F;
      axis.color.a = 1.0F;
      result.markers.push_back(std::move(axis));
    }
    return result;
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
    const Eigen::Vector2d z_limits = planner_->surfaceZLimits();
    visualization_msgs::msg::Marker board;
    board.header = marker_header;
    board.ns = "known_board";
    board.id = 0;
    board.type = visualization_msgs::msg::Marker::CUBE;
    board.action = visualization_msgs::msg::Marker::ADD;
    board.pose.position.x = 0.5 * (x_limits.x() + x_limits.y());
    board.pose.position.y = center.y() + 0.025;
    board.pose.position.z = 0.5 * (z_limits.x() + z_limits.y());
    board.pose.orientation.w = 1.0;
    board.scale.x = x_limits.y() - x_limits.x();
    board.scale.y = 0.05;
    board.scale.z = z_limits.y() - z_limits.x();
    board.color.r = 0.08F;
    board.color.g = 0.08F;
    board.color.b = 0.10F;
    board.color.a = 0.92F;
    markers.markers.push_back(std::move(board));

    auto base_line = line(marker_header, "differential_base_path", 0,
      0.035, 0.40F, 0.45F, 0.52F);
    auto coverage_line = line(marker_header, "constrained_ee_coverage", 0,
      0.028, 0.05F, 1.0F, 0.30F);
    auto target_line = line(marker_header, "board_contact_targets", 0,
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

    visualization_msgs::msg::Marker label;
    label.header = marker_header;
    label.ns = "plan_summary";
    label.id = 0;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position.x = x_limits.x();
    label.pose.position.y = center.y() - 0.08;
    label.pose.position.z = z_limits.y() + 0.18;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.12;
    label.color.r = label.color.g = label.color.b = label.color.a = 1.0F;
    std::ostringstream summary;
    summary << "WipePlanner: " << report.points << " points, "
            << std::fixed << std::setprecision(1) << report.duration << " s";
    label.text = summary.str();
    markers.markers.push_back(std::move(label));

    base_path_publisher_->publish(base_path);
    ee_path_publisher_->publish(ee_path);
    scene_publisher_->publish(markers);

    const auto selected_coverage = evenlySpaced(
      contact_indices, get_parameter("coverage_snapshots").as_int());
    coverage_publisher_->publish(robotSnapshots(
      trajectory, selected_coverage, "wipe_back_end_robot",
      Eigen::Vector3d(0.10, 0.55, 1.0),
      get_parameter("coverage_alpha").as_double()));

    std_msgs::msg::String status;
    status.data = summary.str();
    status_publisher_->publish(status);
    RCLCPP_INFO(
      get_logger(),
      "Plan published: %zu waypoints, %.1f s, %zu EE poses, "
      "%zu constrained whole-body ghosts; "
      "Hybrid A* expanded=%d, IK rejected=%d, collision rejected=%d",
      trajectory.size(), report.duration, ee_path.poses.size(),
      selected_coverage.size(),
      report.hybrid_expanded_nodes, report.reachability_rejections,
      report.collision_rejections);
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
      publishScene(trajectory, report, header());
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
