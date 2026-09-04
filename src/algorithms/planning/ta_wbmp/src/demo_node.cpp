#include "ta_wbmp/planner.hpp"
#include "wbmm_visualization/contract.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <ocs2_msgs/msg/mpc_input.hpp>
#include <ocs2_msgs/msg/mpc_state.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <traj_utils/msg/whole_body_goal.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace ta_wbmp
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

std::string boolJson(bool value)
{
  return value ? "true" : "false";
}
}  // namespace

class TaWbmpDemoNode final : public rclcpp::Node
{
public:
  TaWbmpDemoNode()
  : Node("ta_wbmp_demo")
  {
    declare_parameter("task_file", "");
    declare_parameter("urdf_file", "");
    declare_parameter("ee_frame", "tool0");
    declare_parameter("publish_delay", 0.5);
    declare_parameter("show_table_fixture", false);
    declare_parameter("publish_execution_interfaces", true);

    const std::string task_file = get_parameter("task_file").as_string();
    const std::string urdf_file = get_parameter("urdf_file").as_string();
    if (task_file.empty() || urdf_file.empty()) {
      throw std::runtime_error("task_file and urdf_file parameters are required");
    }
    planner_ = std::make_unique<TaskAwarePlanner>(
      urdf_file, get_parameter("ee_frame").as_string(), task_file);

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    trajectory_publisher_ = create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "/ta_wbmp/whole_body_trajectory", qos);
    // Phase schedule accompanies the trajectory; wbmm_visualization decodes it
    // for playback/segment coloring.
    phase_schedule_publisher_ = create_publisher<std_msgs::msg::String>(
      "/ta_wbmp/phase_schedule", qos);
    marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/ta_wbmp/markers", qos);
    report_publisher_ = create_publisher<std_msgs::msg::String>(
      "/ta_wbmp/report", qos);
    phase_publisher_ = create_publisher<std_msgs::msg::String>(
      "/ta_wbmp/phases", qos);
    remani_goal_publisher_ = create_publisher<traj_utils::msg::WholeBodyGoal>(
      "/ta_wbmp/execution/remani_goal", qos);
    ocs2_reference_publisher_ =
      create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(
      "/ta_wbmp/execution/ocs2_task_reference", qos);

    const double delay = std::max(
      0.05, get_parameter("publish_delay").as_double());
    planning_timer_ = create_wall_timer(
      std::chrono::duration<double>(delay),
      std::bind(&TaWbmpDemoNode::planAndPublish, this));
    RCLCPP_INFO(
      get_logger(),
      "TA-WBMP C++ planning-only demo ready; no cmd_vel, controller, MPC, "
      "REMANI execution, or hardware command publisher is created.");
  }

private:
  std_msgs::msg::Header header() const
  {
    std_msgs::msg::Header result;
    result.frame_id = plan_.frame_id;
    result.stamp = now();
    return result;
  }

  void planAndPublish()
  {
    planning_timer_->cancel();
    try {
      plan_ = planner_->plan();
      publishTrajectory();
      publishPhaseSchedule();
      publishMarkers();
      publishExecutionInterfaces();
      std_msgs::msg::String report_message;
      report_message.data = reportJson(plan_.report);
      report_publisher_->publish(report_message);
      std_msgs::msg::String phases;
      phases.data =
        "NAVIGATE -> PRECONTACT_ALIGN -> PRECONTACT_APPROACH -> "
        "TASK_CONSTRAINED";
      phase_publisher_->publish(phases);

      const auto & report = plan_.report;
      RCLCPP_INFO(
        get_logger(),
        "TA-WBMP C++ plan PASSED: %zu waypoints, %.2f s, task candidates "
        "%d/%d feasible, selected standoff=%.2f m offset=%.2f m; "
        "EE error=%.4f m axis=%.4f rad, joint margin=%.3f, "
        "manipulability=%.6g, nav clearance=%.3f m, lateral=%.3g m/s, "
        "speeds=(%.3f m/s, %.3f rad/s, %.3f rad/s)",
        report.waypoint_count, report.duration,
        report.feasible_candidate_count, report.candidate_count,
        report.selected_standoff, report.selected_longitudinal_offset,
        report.max_contact_position_error, report.max_tool_axis_error,
        report.minimum_joint_margin, report.minimum_manipulability,
        report.navigation_clearance, report.max_lateral_velocity,
        report.max_base_speed, report.max_angular_speed,
        report.max_joint_speed);
    } catch (const std::exception & error) {
      RCLCPP_FATAL(get_logger(), "TA-WBMP C++ planning failed: %s", error.what());
      rclcpp::shutdown();
    }
  }

  void publishTrajectory()
  {
    trajectory_msgs::msg::JointTrajectory message;
    message.header = header();
    message.joint_names = {
      "base_x", "base_y", "base_yaw", "joint_1", "joint_2", "joint_3",
      "joint_4", "joint_5", "joint_6"};
    for (std::size_t index = 0; index < plan_.waypoints.size(); ++index) {
      const auto & waypoint = plan_.waypoints[index];
      trajectory_msgs::msg::JointTrajectoryPoint trajectory_point;
      trajectory_point.positions.assign(
        waypoint.state.data(), waypoint.state.data() + waypoint.state.size());
      Eigen::VectorXd velocity = Eigen::VectorXd::Zero(waypoint.state.size());
      if (index + 1 < plan_.waypoints.size()) {
        const auto & next = plan_.waypoints[index + 1];
        const double dt = std::max(1.0e-9, next.time - waypoint.time);
        velocity = (next.state - waypoint.state) / dt;
        velocity[2] = wrapAngle(next.state[2] - waypoint.state[2]) / dt;
      }
      trajectory_point.velocities.assign(
        velocity.data(), velocity.data() + velocity.size());
      trajectory_point.time_from_start =
        rclcpp::Duration::from_seconds(waypoint.time);
      message.points.push_back(std::move(trajectory_point));
    }
    trajectory_publisher_->publish(message);
  }

  void publishPhaseSchedule()
  {
    std::vector<std::pair<double, std::string>> intervals;
    for (const auto & waypoint : plan_.waypoints) {
      if (intervals.empty() || intervals.back().second != waypoint.phase) {
        intervals.emplace_back(waypoint.time, waypoint.phase);
      }
    }
    std_msgs::msg::String message;
    message.data = wbmm_viz::encodePhaseSchedule(intervals);
    phase_schedule_publisher_->publish(message);
  }

  void publishExecutionInterfaces()
  {
    if (!get_parameter("publish_execution_interfaces").as_bool() ||
      plan_.remani_navigation_goal.size() != 9 ||
      plan_.task_entry_state.size() != 9 ||
      plan_.execution_start_index >= plan_.waypoints.size())
    {
      return;
    }

    traj_utils::msg::WholeBodyGoal goal;
    goal.header = header();
    goal.base_pose.position.x = plan_.remani_navigation_goal[0];
    goal.base_pose.position.y = plan_.remani_navigation_goal[1];
    goal.base_pose.orientation.z = std::sin(
      0.5 * plan_.remani_navigation_goal[2]);
    goal.base_pose.orientation.w = std::cos(
      0.5 * plan_.remani_navigation_goal[2]);
    goal.joint_names = {
      "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    goal.joint_positions.assign(
      plan_.remani_navigation_goal.data() + 3,
      plan_.remani_navigation_goal.data() + 9);
    remani_goal_publisher_->publish(goal);

    ocs2_msgs::msg::MpcTargetTrajectories reference;
    const double task_start_time =
      plan_.waypoints[plan_.execution_start_index].time;
    for (std::size_t index = plan_.execution_start_index;
      index < plan_.waypoints.size(); ++index)
    {
      const Waypoint & waypoint = plan_.waypoints[index];
      reference.time_trajectory.push_back(waypoint.time - task_start_time);
      ocs2_msgs::msg::MpcState state;
      state.value.reserve(9);
      for (Eigen::Index value = 0; value < waypoint.state.size(); ++value) {
        state.value.push_back(static_cast<float>(waypoint.state[value]));
      }
      reference.state_trajectory.push_back(std::move(state));

      Eigen::VectorXd input = Eigen::VectorXd::Zero(8);
      if (index + 1 < plan_.waypoints.size()) {
        const Waypoint & next = plan_.waypoints[index + 1];
        const double dt = std::max(1.0e-9, next.time - waypoint.time);
        const Eigen::Vector2d delta =
          next.state.head<2>() - waypoint.state.head<2>();
        input[0] = (
          std::cos(waypoint.state[2]) * delta.x() +
          std::sin(waypoint.state[2]) * delta.y()) / dt;
        input[1] = wrapAngle(next.state[2] - waypoint.state[2]) / dt;
        input.tail(6) =
          (next.state.tail(6) - waypoint.state.tail(6)) / dt;
      }
      ocs2_msgs::msg::MpcInput command;
      command.value.reserve(8);
      for (Eigen::Index value = 0; value < input.size(); ++value) {
        command.value.push_back(static_cast<float>(input[value]));
      }
      reference.input_trajectory.push_back(std::move(command));
    }
    ocs2_reference_publisher_->publish(reference);
    RCLCPP_INFO(
      get_logger(),
      "Published execution contracts: q_pre for REMANI and %zu-point "
      "transition+task reference for OCS2 on namespaced, non-command topics",
      reference.state_trajectory.size());
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear;
    clear.header = header();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear);
    int marker_id = 0;

    visualization_msgs::msg::Marker surface;
    surface.header = header();
    surface.ns = "task_surface";
    surface.id = marker_id++;
    surface.action = visualization_msgs::msg::Marker::ADD;
    surface.pose.orientation.w = 1.0;
    surface.color.r = 0.10F;
    surface.color.g = 0.16F;
    surface.color.b = 0.23F;
    surface.color.a = 0.72F;
    if (plan_.surface_type == "cylindrical") {
      surface.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
      const int patches = 48;
      const auto surface_point = [this](double angle, double z) {
          return Eigen::Vector3d(
            plan_.surface_center.x() +
            plan_.surface_radius * std::sin(angle),
            plan_.surface_center.y() -
            plan_.surface_radius * std::cos(angle), z);
        };
      for (int index = 0; index < patches; ++index) {
        const double u0 = static_cast<double>(index) / patches;
        const double u1 = static_cast<double>(index + 1) / patches;
        const double a0 = plan_.surface_parameter_limits[0] + u0 *
          (plan_.surface_parameter_limits[1] -
          plan_.surface_parameter_limits[0]);
        const double a1 = plan_.surface_parameter_limits[0] + u1 *
          (plan_.surface_parameter_limits[1] -
          plan_.surface_parameter_limits[0]);
        const Eigen::Vector3d p00 = surface_point(a0, plan_.z_limits[0]);
        const Eigen::Vector3d p10 = surface_point(a1, plan_.z_limits[0]);
        const Eigen::Vector3d p11 = surface_point(a1, plan_.z_limits[1]);
        const Eigen::Vector3d p01 = surface_point(a0, plan_.z_limits[1]);
        for (const auto & vertex : {p00, p10, p11, p00, p11, p01}) {
          surface.points.push_back(point(vertex));
        }
      }
    } else {
      surface.type = visualization_msgs::msg::Marker::CUBE;
      const double middle_u = 0.5 * (
        plan_.surface_u_limits[0] + plan_.surface_u_limits[1]);
      const double middle_v = 0.5 * (
        plan_.surface_v_limits[0] + plan_.surface_v_limits[1]);
      const Eigen::Vector3d center = plan_.surface_local_coordinates ?
        plan_.surface_center + middle_u * plan_.surface_axis_u +
        middle_v * plan_.surface_axis_v : Eigen::Vector3d(
        middle_u, plan_.surface_center.y(), middle_v);
      surface.pose.position = point(center);
      Eigen::Matrix3d orientation;
      orientation.col(0) = plan_.surface_axis_u.normalized();
      orientation.col(1) = plan_.surface_axis_v.normalized();
      orientation.col(2) = plan_.surface_normal.normalized();
      const Eigen::Quaterniond quaternion(orientation);
      surface.pose.orientation.x = quaternion.x();
      surface.pose.orientation.y = quaternion.y();
      surface.pose.orientation.z = quaternion.z();
      surface.pose.orientation.w = quaternion.w();
      surface.scale.x = std::abs(
        plan_.surface_u_limits[1] - plan_.surface_u_limits[0]);
      surface.scale.y = std::abs(
        plan_.surface_v_limits[1] - plan_.surface_v_limits[0]);
      surface.scale.z = 0.05;
    }
    markers.markers.push_back(std::move(surface));

    // The planned contact patch can be intentionally much smaller than the
    // physical table.  Draw a separate fixture around horizontal surfaces so
    // RViz communicates "wipe a table" without changing the task geometry or
    // the planner's collision model.
    if (get_parameter("show_table_fixture").as_bool() &&
      plan_.surface_type == "planar" &&
      std::abs(plan_.surface_normal.normalized().z()) > 0.90)
    {
      const Eigen::Vector3d axis_u = plan_.surface_axis_u.normalized();
      const Eigen::Vector3d axis_v = plan_.surface_axis_v.normalized();
      const Eigen::Vector3d normal = plan_.surface_normal.normalized();
      Eigen::Matrix3d orientation;
      orientation.col(0) = axis_u;
      orientation.col(1) = axis_v;
      orientation.col(2) = normal;
      const Eigen::Quaterniond quaternion(orientation);
      const double patch_u = std::abs(
        plan_.surface_u_limits[1] - plan_.surface_u_limits[0]);
      const double patch_v = std::abs(
        plan_.surface_v_limits[1] - plan_.surface_v_limits[0]);
      const double table_u = std::max(0.50, patch_u + 0.34);
      const double table_v = std::max(0.80, patch_v + 0.60);
      constexpr double kTopThickness = 0.06;
      constexpr double kLegHeight = 0.49;

      const auto table_cube = [&](
        const std::string & name, const Eigen::Vector3d & center,
        const Eigen::Vector3d & scale, int id)
        {
          visualization_msgs::msg::Marker marker;
          marker.header = header();
          marker.ns = name;
          marker.id = id;
          marker.type = visualization_msgs::msg::Marker::CUBE;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.pose.position = point(center);
          marker.pose.orientation.x = quaternion.x();
          marker.pose.orientation.y = quaternion.y();
          marker.pose.orientation.z = quaternion.z();
          marker.pose.orientation.w = quaternion.w();
          marker.scale.x = scale.x();
          marker.scale.y = scale.y();
          marker.scale.z = scale.z();
          marker.color.r = 0.52F;
          marker.color.g = 0.30F;
          marker.color.b = 0.12F;
          marker.color.a = name == "table_top" ? 0.88F : 0.92F;
          return marker;
        };

      const Eigen::Vector3d top_center =
        plan_.surface_center - 0.5 * kTopThickness * normal;
      markers.markers.push_back(table_cube(
        "table_top", top_center,
        Eigen::Vector3d(table_u, table_v, kTopThickness), marker_id++));
      const double leg_u = 0.5 * table_u - 0.06;
      const double leg_v = 0.5 * table_v - 0.06;
      for (const double sign_u : {-1.0, 1.0}) {
        for (const double sign_v : {-1.0, 1.0}) {
          const Eigen::Vector3d leg_center = plan_.surface_center +
            sign_u * leg_u * axis_u + sign_v * leg_v * axis_v -
            (kTopThickness + 0.5 * kLegHeight) * normal;
          markers.markers.push_back(table_cube(
            "table_legs", leg_center,
            Eigen::Vector3d(0.055, 0.055, kLegHeight), marker_id++));
        }
      }
    }

    int obstacle_id = 0;
    for (const auto & value : plan_.obstacles) {
      visualization_msgs::msg::Marker obstacle;
      obstacle.header = header();
      obstacle.ns = "navigation_obstacles";
      obstacle.id = obstacle_id++;
      obstacle.type = visualization_msgs::msg::Marker::CYLINDER;
      obstacle.action = visualization_msgs::msg::Marker::ADD;
      obstacle.pose.position.x = value.x();
      obstacle.pose.position.y = value.y();
      obstacle.pose.position.z = 0.35;
      obstacle.pose.orientation.w = 1.0;
      obstacle.scale.x = obstacle.scale.y = 2.0 * value.z();
      obstacle.scale.z = 0.70;
      obstacle.color.r = 0.85F;
      obstacle.color.g = 0.12F;
      obstacle.color.b = 0.10F;
      obstacle.color.a = 0.78F;
      markers.markers.push_back(std::move(obstacle));
    }

    const std::vector<std::tuple<std::string, Eigen::Vector3d>> phase_colors{
      {kPhaseNavigate, Eigen::Vector3d(0.55, 0.62, 0.72)},
      {kPhasePrecontactAlign, Eigen::Vector3d(1.0, 0.75, 0.05)},
      {kPhasePrecontactApproach, Eigen::Vector3d(1.0, 0.45, 0.05)},
      {kPhaseTask, Eigen::Vector3d(0.05, 1.0, 0.25)}};
    for (const auto & phase_color : phase_colors) {
      const auto & phase = std::get<0>(phase_color);
      const auto & color = std::get<1>(phase_color);
      visualization_msgs::msg::Marker line;
      line.header = header();
      line.ns = "phase_" + phase;
      line.id = marker_id++;
      line.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line.action = visualization_msgs::msg::Marker::ADD;
      line.pose.orientation.w = 1.0;
      line.scale.x = phase == kPhaseTask ? 0.040 : 0.025;
      line.color.r = static_cast<float>(color.x());
      line.color.g = static_cast<float>(color.y());
      line.color.b = static_cast<float>(color.z());
      line.color.a = 1.0F;
      for (const auto & waypoint : plan_.waypoints) {
        if (waypoint.phase == phase) {
          line.points.push_back(point(planner_->toolPosition(waypoint.state)));
        }
      }
      markers.markers.push_back(std::move(line));
    }

    const int normal_count = std::min<int>(12, plan_.task_targets.size());
    for (int sample = 0; sample < normal_count; ++sample) {
      const std::size_t index = static_cast<std::size_t>(std::llround(
        static_cast<double>(sample) * (plan_.task_targets.size() - 1) /
        std::max(1, normal_count - 1)));
      const Eigen::Vector3d target = plan_.task_targets[index];
      visualization_msgs::msg::Marker normal;
      normal.header = header();
      normal.ns = "task_normal_constraint";
      normal.id = marker_id++;
      normal.type = visualization_msgs::msg::Marker::ARROW;
      normal.action = visualization_msgs::msg::Marker::ADD;
      normal.points.push_back(point(target));
      normal.points.push_back(point(
        target - 0.14 * plan_.task_normals[index]));
      normal.scale.x = 0.012;
      normal.scale.y = 0.030;
      normal.scale.z = 0.045;
      normal.color.r = 0.2F;
      normal.color.g = 0.55F;
      normal.color.b = 1.0F;
      normal.color.a = 0.9F;
      markers.markers.push_back(std::move(normal));
    }

    visualization_msgs::msg::Marker label;
    label.header = header();
    label.ns = "summary";
    label.id = marker_id;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose.position.x = plan_.x_limits[0];
    label.pose.position.y = plan_.surface_center.y() - 0.12;
    label.pose.position.z = plan_.z_limits[1] + 0.22;
    label.pose.orientation.w = 1.0;
    label.scale.z = 0.11;
    label.color.r = label.color.g = label.color.b = label.color.a = 1.0F;
    label.text = "TA-WBMP C++ PASS | " +
      std::to_string(plan_.report.waypoint_count) + " points | " +
      std::to_string(plan_.report.feasible_candidate_count) + "/" +
      std::to_string(plan_.report.candidate_count) +
      " task-feasible candidates";
    markers.markers.push_back(std::move(label));
    marker_publisher_->publish(markers);
  }
  std::string reportJson(const PlanReport & report) const
  {
    std::ostringstream output;
    output << std::setprecision(12)
           << "{\"success\":" << boolJson(report.success)
           << ",\"task_name\":\"" << report.task_name << "\""
           << ",\"task_type\":\"" << report.task_type << "\""
           << ",\"state_dimension\":" << report.state_dimension
           << ",\"waypoint_count\":" << report.waypoint_count
           << ",\"duration\":" << report.duration
           << ",\"candidate_count\":" << report.candidate_count
           << ",\"feasible_candidate_count\":"
           << report.feasible_candidate_count
           << ",\"selected_candidate_id\":"
           << report.selected_candidate_id
           << ",\"selected_standoff\":" << report.selected_standoff
           << ",\"selected_longitudinal_offset\":"
           << report.selected_longitudinal_offset
           << ",\"selected_future_task_score\":"
           << report.selected_future_task_score
           << ",\"max_contact_position_error\":"
           << report.max_contact_position_error
           << ",\"max_tool_axis_error\":" << report.max_tool_axis_error
           << ",\"minimum_joint_margin\":" << report.minimum_joint_margin
           << ",\"minimum_manipulability\":"
           << report.minimum_manipulability
           << ",\"minimum_sigma\":" << report.minimum_sigma
           << ",\"task_base_path_length\":"
           << report.task_base_path_length
           << ",\"max_lateral_velocity\":" << report.max_lateral_velocity
           << ",\"max_base_speed\":" << report.max_base_speed
           << ",\"max_angular_speed\":" << report.max_angular_speed
           << ",\"max_joint_speed\":" << report.max_joint_speed
           << ",\"navigation_clearance\":" << report.navigation_clearance
           << ",\"constraints\":{";
    bool first = true;
    for (const auto & constraint : report.constraints) {
      output << (first ? "" : ",") << "\"" << constraint.first << "\":"
             << boolJson(constraint.second);
      first = false;
    }
    output << "}}";
    return output.str();
  }

  std::unique_ptr<TaskAwarePlanner> planner_;
  Plan plan_;
  rclcpp::TimerBase::SharedPtr planning_timer_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    trajectory_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
    phase_schedule_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    marker_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr report_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_publisher_;
  rclcpp::Publisher<traj_utils::msg::WholeBodyGoal>::SharedPtr
    remani_goal_publisher_;
  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr
    ocs2_reference_publisher_;
};
}  // namespace ta_wbmp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ta_wbmp::TaWbmpDemoNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("ta_wbmp_demo"), "%s", error.what());
  }
  rclcpp::shutdown();
  return 0;
}
