#include "wbmm_visualization/contract.hpp"
#include "wbmm_visualization/markers.hpp"
#include "wbmm_visualization/whole_body_kinematics.hpp"

#include <gtest/gtest.h>

#include <rclcpp/duration.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace wbmm_viz
{
namespace
{

std::string urdfFile()
{
  return std::string(WBMM_VIZ_WORKSPACE_DIR) +
    "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf";
}

trajectory_msgs::msg::JointTrajectory makeTrajectory(
  const std::vector<double> & times)
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.frame_id = "odom";
  msg.joint_names = {kJointNames.begin(), kJointNames.end()};
  for (const double time : times) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(9);
    for (int index = 0; index < 9; ++index) {
      point.positions[index] = 0.1 * (index + 1) + time;
    }
    point.time_from_start = rclcpp::Duration::from_seconds(time);
    msg.points.push_back(std::move(point));
  }
  return msg;
}

bool colorEquals(const visualization_msgs::msg::Marker & marker,
                 const Eigen::Vector4d & rgba, double tolerance = 1.0e-6)
{
  return std::abs(marker.color.r - rgba.x()) < tolerance &&
    std::abs(marker.color.g - rgba.y()) < tolerance &&
    std::abs(marker.color.b - rgba.z()) < tolerance &&
    std::abs(marker.color.a - rgba.w()) < tolerance;
}

bool colorEqualsRgb(const visualization_msgs::msg::Marker & marker,
                    const Eigen::Vector4d & rgba, double tolerance = 1.0e-3)
{
  return std::abs(marker.color.r - rgba.x()) < tolerance &&
    std::abs(marker.color.g - rgba.y()) < tolerance &&
    std::abs(marker.color.b - rgba.z()) < tolerance;
}

TEST(ContractParsing, ValidTrajectory)
{
  const auto msg = makeTrajectory({0.0, 1.0, 2.0});
  const ParsedTrajectory trajectory = parseJointTrajectory(msg, nullptr);
  EXPECT_EQ(trajectory.frame_id, "odom");
  EXPECT_EQ(trajectory.waypoints.size(), 3U);
  EXPECT_DOUBLE_EQ(trajectory.duration, 2.0);
  EXPECT_EQ(trajectory.waypoints[1].state[0], 0.1 * 1.0 + 1.0);
  EXPECT_EQ(trajectory.waypoints[2].state[8], 0.1 * 9.0 + 2.0);
}

TEST(ContractParsing, RejectsWrongJointNames)
{
  auto msg = makeTrajectory({0.0, 1.0});
  msg.joint_names.resize(8);
  EXPECT_THROW(parseJointTrajectory(msg, nullptr), std::invalid_argument);

  msg = makeTrajectory({0.0, 1.0});
  std::swap(msg.joint_names[0], msg.joint_names[1]);
  EXPECT_THROW(parseJointTrajectory(msg, nullptr), std::invalid_argument);
}

TEST(ContractParsing, RejectsEmptyAndShortPoints)
{
  auto msg = makeTrajectory({});
  EXPECT_THROW(parseJointTrajectory(msg, nullptr), std::invalid_argument);

  msg = makeTrajectory({0.0, 1.0});
  msg.points[0].positions.resize(8);
  EXPECT_THROW(parseJointTrajectory(msg, nullptr), std::invalid_argument);
}

TEST(ContractParsing, AcceptsSinglePointAndClampsTimes)
{
  const auto single = parseJointTrajectory(makeTrajectory({0.0}), nullptr);
  EXPECT_EQ(single.waypoints.size(), 1U);
  EXPECT_DOUBLE_EQ(single.duration, 0.0);

  // Non-decreasing requirement is clamped, not rejected.
  const auto clamped = parseJointTrajectory(makeTrajectory({2.0, 1.0}), nullptr);
  EXPECT_DOUBLE_EQ(clamped.waypoints[0].time, 2.0);
  EXPECT_DOUBLE_EQ(clamped.waypoints[1].time, 2.0);
  EXPECT_DOUBLE_EQ(clamped.duration, 2.0);
}

TEST(ContractParsing, AssignsPhaseSchedulePerInterval)
{
  const std::string schedule = "0.0 NAVIGATE;1.0 TASK_CONSTRAINED";
  const auto trajectory = parseJointTrajectory(
    makeTrajectory({0.0, 1.0, 2.0}), &schedule);
  EXPECT_EQ(trajectory.waypoints[0].phase, "NAVIGATE");
  EXPECT_FALSE(trajectory.waypoints[0].in_contact);
  // t == t_i already takes the new phase.
  EXPECT_EQ(trajectory.waypoints[1].phase, "TASK_CONSTRAINED");
  EXPECT_TRUE(trajectory.waypoints[1].in_contact);
  EXPECT_EQ(trajectory.waypoints[2].phase, "TASK_CONSTRAINED");
  EXPECT_TRUE(trajectory.waypoints[2].in_contact);
}

TEST(ContractParsing, IgnoresMalformedSchedule)
{
  const std::string schedule = "not a schedule";
  const auto trajectory = parseJointTrajectory(
    makeTrajectory({0.0, 1.0}), &schedule);
  EXPECT_TRUE(trajectory.waypoints[0].phase.empty());
  EXPECT_FALSE(trajectory.waypoints[0].in_contact);
}

TEST(YawInterpolation, WrapsAcrossPi)
{
  auto msg = makeTrajectory({0.0, 1.0});
  msg.points[0].positions[2] = 179.0 * M_PI / 180.0;
  msg.points[1].positions[2] = -179.0 * M_PI / 180.0;
  const ParsedTrajectory trajectory = parseJointTrajectory(msg, nullptr);
  const Waypoint mid = waypointAt(trajectory, 0.5);
  // The short way across the wrap: |yaw| stays near pi, not near 0.
  EXPECT_GT(std::abs(mid.state[2]), 3.0);
  EXPECT_LT(std::abs(mid.state[2]), M_PI + 1.0e-9);
  // Non-yaw components interpolate linearly.
  EXPECT_DOUBLE_EQ(mid.state[0], 0.5 * (msg.points[0].positions[0] +
                                        msg.points[1].positions[0]));
}

TEST(YawInterpolation, ClampsToEnds)
{
  auto msg = makeTrajectory({1.0, 2.0, 3.0});
  msg.points[0].positions[2] = 0.5;
  msg.points[2].positions[2] = -0.5;
  const ParsedTrajectory trajectory = parseJointTrajectory(msg, nullptr);
  const Waypoint before = waypointAt(trajectory, -1.0);
  const Waypoint after = waypointAt(trajectory, 99.0);
  EXPECT_DOUBLE_EQ(before.state[2], 0.5);
  EXPECT_DOUBLE_EQ(after.state[2], -0.5);
  EXPECT_DOUBLE_EQ(before.time, 1.0);
  EXPECT_DOUBLE_EQ(after.time, 3.0);
}

TEST(TimeSegments, ExactMarkerCountAndIds)
{
  const WholeBodyKinematics kinematics(urdfFile());
  const auto msg = makeTrajectory([]() {
    std::vector<double> times;
    for (int index = 0; index <= 30; ++index) {
      times.push_back(static_cast<double>(index));
    }
    return times;
  }());
  const ParsedTrajectory trajectory = parseJointTrajectory(msg, nullptr);
  const std::size_t meshes =
    kinematics.visualGeometry(Eigen::VectorXd::Zero(9)).size();

  MarkerContext ctx;
  visualization_msgs::msg::MarkerArray markers;
  beginMarkerArray(markers, ctx, "odom", rclcpp::Time());
  buildTimeSegments(
    trajectory, kinematics, {10.0, 2}, markers, ctx);

  // DELETEALL + 3 segments * (base line + ee line + label + 2 snapshots *
  // (meshes + tool axis)).
  constexpr int kSegments = 3;
  constexpr int kSnapshots = 2;
  const int expected = 1 + kSegments *
    (2 + 1 + kSnapshots * (static_cast<int>(meshes) + 1));
  EXPECT_EQ(markers.markers.size(), static_cast<std::size_t>(expected));
  EXPECT_EQ(markers.markers.front().action,
            visualization_msgs::msg::Marker::DELETEALL);
  for (std::size_t index = 1; index < markers.markers.size(); ++index) {
    EXPECT_EQ(markers.markers[index].id, static_cast<int>(index) - 1);
  }
}

TEST(TimeSegments, SegmentColorsAndNamespaces)
{
  const WholeBodyKinematics kinematics(urdfFile());
  const ParsedTrajectory trajectory =
    parseJointTrajectory(makeTrajectory({0.0, 15.0, 30.0}), nullptr);

  MarkerContext ctx;
  visualization_msgs::msg::MarkerArray markers;
  beginMarkerArray(markers, ctx, "odom", rclcpp::Time());
  buildTimeSegments(trajectory, kinematics, {10.0, 1}, markers, ctx);

  int labels = 0;
  for (const auto & marker : markers.markers) {
    if (marker.ns == "time_segment_1_base") {
      EXPECT_TRUE(colorEqualsRgb(marker, segmentColor(1)));
      EXPECT_NEAR(marker.color.a, 0.95F, 1.0e-6);
    }
    if (marker.ns == "time_segment_2_ee") {
      EXPECT_TRUE(colorEqualsRgb(marker, segmentColor(2)));
      EXPECT_NEAR(marker.color.a, 0.95F, 1.0e-6);
    }
    if (marker.ns == "time_segment_labels") {
      ++labels;
      EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      EXPECT_EQ(marker.text.substr(0, 1), "S");
      EXPECT_NE(marker.text.find("] s"), std::string::npos);
    }
  }
  EXPECT_EQ(labels, 3);
}

TEST(PlaybackFrame, WindowPlayedLabelPerSegment)
{
  const WholeBodyKinematics kinematics(urdfFile());
  const std::string schedule = "0.0 NAVIGATE;10.0 TASK_CONSTRAINED";
  const ParsedTrajectory trajectory = parseJointTrajectory(
    makeTrajectory({0.0, 5.0, 10.0, 15.0, 20.0, 25.0, 30.0}), &schedule);

  const auto frame = [&](double t) {
    MarkerContext ctx;
    visualization_msgs::msg::MarkerArray markers;
    beginMarkerArray(markers, ctx, "odom", rclcpp::Time());
    buildPlaybackFrame(trajectory, t, {10.0}, kinematics, markers, ctx);
    return markers;
  };

  for (const double t : {0.0, 15.0, 29.9}) {
    const auto markers = frame(t);
    // DELETEALL + 2 window lines + 2 played lines + 1 label.
    EXPECT_EQ(markers.markers.size(), 6U);
    EXPECT_EQ(markers.markers.front().action,
              visualization_msgs::msg::Marker::DELETEALL);
    for (const auto & marker : markers.markers) {
      EXPECT_NE(marker.type, visualization_msgs::msg::Marker::MESH_RESOURCE);
      EXPECT_EQ(marker.ns.find("robot"), std::string::npos);
    }
  }

  const auto mid = frame(15.0);
  visualization_msgs::msg::Marker label;
  for (const auto & marker : mid.markers) {
    if (marker.ns == "playback_time") {
      label = marker;
    }
  }
  EXPECT_EQ(label.text, "t=15.0 s | S1 | TASK_CONSTRAINED");
  EXPECT_TRUE(colorEquals(label, segmentColor(1), 1.0e-3));
  const auto head = frame(0.0);
  // active_window_base: segment color at window alpha.
  EXPECT_TRUE(colorEqualsRgb(head.markers[1], segmentColor(0)));
  EXPECT_NEAR(head.markers[1].color.a, 0.22F, 1.0e-6);
}

TEST(RobotSnapshot, MeshesAndToolAxis)
{
  std::vector<VisualGeometry> geometry(2);
  geometry[0].name = "body_a";
  geometry[0].mesh_path = "file:///tmp/body_a.stl";
  geometry[0].mesh_scale = Eigen::Vector3d(2.0, 1.0, 0.5);
  geometry[0].position = Eigen::Vector3d(1.0, 2.0, 3.0);
  geometry[1].name = "body_b";
  geometry[1].mesh_path = "file:///tmp/body_b.dae";

  MarkerContext ctx;
  visualization_msgs::msg::MarkerArray markers;
  beginMarkerArray(markers, ctx, "odom", rclcpp::Time());
  const Eigen::Vector4d rgba(0.2, 0.3, 0.4, 0.5);
  const Eigen::Vector3d tool(0.0, 0.0, 1.0);
  const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  appendRobotSnapshot(
    markers, ctx, geometry, "test_robot", rgba, true, true, &tool, &rotation);

  ASSERT_EQ(markers.markers.size(), 4U);  // DELETEALL + 2 meshes + arrow
  EXPECT_EQ(markers.markers[1].ns, "test_robot");
  EXPECT_EQ(markers.markers[1].id, 0);
  EXPECT_EQ(markers.markers[1].mesh_resource, "file:///tmp/body_a.stl");
  EXPECT_TRUE(markers.markers[1].mesh_use_embedded_materials);
  EXPECT_DOUBLE_EQ(markers.markers[1].scale.x, 2.0);
  EXPECT_DOUBLE_EQ(markers.markers[1].pose.position.x, 1.0);
  EXPECT_TRUE(colorEquals(markers.markers[1], rgba));
  EXPECT_EQ(markers.markers[2].id, 1);
  EXPECT_EQ(markers.markers[3].ns, "test_robot_tool_axis");
  EXPECT_EQ(markers.markers[3].type, visualization_msgs::msg::Marker::ARROW);
  ASSERT_EQ(markers.markers[3].points.size(), 2U);
  EXPECT_DOUBLE_EQ(markers.markers[3].points[1].z, 1.16);
  // Arrow alpha is base alpha + 0.35.
  EXPECT_NEAR(markers.markers[3].color.a, 0.85F, 1.0e-6);

  MarkerContext no_axis_ctx;
  visualization_msgs::msg::MarkerArray no_axis;
  beginMarkerArray(no_axis, no_axis_ctx, "odom", rclcpp::Time());
  appendRobotSnapshot(
    no_axis, no_axis_ctx, geometry, "plain", rgba, false, false, nullptr, nullptr);
  EXPECT_EQ(no_axis.markers.size(), 3U);  // DELETEALL + 2 meshes
}

TEST(PhaseScheduleCodec, RoundTrip)
{
  const std::vector<std::pair<double, std::string>> input{
    {0.0, "NAVIGATE"}, {12.5, "PRECONTACT_APPROACH"},
    {30.0, "TASK_CONSTRAINED"}};
  const std::string encoded = encodePhaseSchedule(input);
  std::vector<std::pair<double, std::string>> decoded;
  ASSERT_TRUE(decodePhaseSchedule(encoded, decoded));
  ASSERT_EQ(decoded.size(), input.size());
  for (std::size_t index = 0; index < input.size(); ++index) {
    EXPECT_DOUBLE_EQ(decoded[index].first, input[index].first);
    EXPECT_EQ(decoded[index].second, input[index].second);
  }
}

TEST(PhaseScheduleCodec, RejectsMalformed)
{
  std::vector<std::pair<double, std::string>> out;
  EXPECT_FALSE(decodePhaseSchedule("abc NAVIGATE", out));
  EXPECT_FALSE(decodePhaseSchedule("1.0 NAVIGATE extra", out));
  EXPECT_FALSE(decodePhaseSchedule("1.0", out));
  EXPECT_FALSE(decodePhaseSchedule("1.0 NAVIGATE;0.5 TASK", out));
  EXPECT_FALSE(decodePhaseSchedule("1.0 NAVIGATE;", out));
  EXPECT_TRUE(decodePhaseSchedule("", out));  // empty is valid
  EXPECT_TRUE(out.empty());
}

TEST(PhaseScheduleCodec, PhaseAtBoundaries)
{
  const std::vector<std::pair<double, std::string>> schedule{
    {1.0, "A"}, {2.0, "B"}};
  // Interval [t_i, t_{i+1}) carries PHASE_i; the last entry extends to the end.
  EXPECT_EQ(phaseAt(schedule, 0.5), "A");  // before first boundary: first phase
  EXPECT_EQ(phaseAt(schedule, 1.0), "A");  // [1.0, 2.0) carries A
  EXPECT_EQ(phaseAt(schedule, 1.5), "A");
  EXPECT_EQ(phaseAt(schedule, 2.0), "B");
  EXPECT_EQ(phaseAt(schedule, 99.0), "B");
  EXPECT_EQ(phaseAt({}, 1.0), "");
}

TEST(Paths, BaseAndEePhaseFiltered)
{
  const WholeBodyKinematics kinematics(urdfFile());
  const std::string schedule =
    "0.0 NAVIGATE;3.0 NAVIGATE;6.0 TASK_CONSTRAINED";
  const ParsedTrajectory trajectory = parseJointTrajectory(
    makeTrajectory({0.0, 3.0, 6.0, 9.0}), &schedule);

  nav_msgs::msg::Path base;
  buildBasePath(trajectory, "", base);
  EXPECT_EQ(base.poses.size(), trajectory.waypoints.size());
  EXPECT_EQ(base.header.frame_id, "odom");

  nav_msgs::msg::Path navigation;
  buildBasePath(trajectory, "NAVIGATE", navigation);
  EXPECT_EQ(navigation.poses.size(), 2U);
  EXPECT_NEAR(navigation.poses[0].pose.orientation.w,
              std::cos(0.5 * trajectory.waypoints[0].state[2]), 1.0e-9);

  nav_msgs::msg::Path ee;
  buildEePath(trajectory, kinematics, "TASK_CONSTRAINED", ee);
  EXPECT_EQ(ee.poses.size(), 2U);
  EXPECT_TRUE(std::isfinite(ee.poses[0].pose.position.x));
}

}  // namespace
}  // namespace wbmm_viz
