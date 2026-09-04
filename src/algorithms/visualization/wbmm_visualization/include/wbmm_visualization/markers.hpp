#pragma once

#include "wbmm_visualization/contract.hpp"
#include "wbmm_visualization/whole_body_kinematics.hpp"

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <string>
#include <vector>

namespace wbmm_viz
{

// Marker construction state: every publish starts with beginMarkerArray(),
// which prepends a DELETEALL marker and resets the id counter so each array is
// self-contained (ids sequential from 0, exactly like the TA-WBMP demo).
struct MarkerContext
{
  std::string frame_id;
  rclcpp::Time stamp;
  int next_id{0};
};

void beginMarkerArray(visualization_msgs::msg::MarkerArray & out,
                      MarkerContext & ctx, const std::string & frame_id,
                      const rclcpp::Time & stamp);

// One MESH_RESOURCE per VisualGeometry under ns, ids increment sequentially.
// Optionally appends an ARROW (ns + "_tool_axis", 0.16 m) along tool_rot.col(2)
// when include_tool_axis and both pointers are given.
void appendRobotSnapshot(visualization_msgs::msg::MarkerArray & out,
                         MarkerContext & ctx,
                         const std::vector<VisualGeometry> & geometry,
                         const std::string & ns, const Eigen::Vector4d & rgba,
                         bool embedded_materials, bool include_tool_axis,
                         const Eigen::Vector3d * tool_pos,
                         const Eigen::Matrix3d * tool_rot);

void appendLineStrip(visualization_msgs::msg::MarkerArray & out,
                     MarkerContext & ctx, const std::string & ns,
                     const std::vector<Eigen::Vector3d> & points,
                     const Eigen::Vector4d & rgba, double width);

void appendTextLabel(visualization_msgs::msg::MarkerArray & out,
                     MarkerContext & ctx, const std::string & ns,
                     const Eigen::Vector3d & position, double height,
                     const Eigen::Vector4d & rgba, const std::string & text);

struct TimeSegmentSpec
{
  double duration{15.0};
  int snapshots_per_segment{2};
};

// Per time segment: base/EE LINE_STRIPs, a "S<i> [a, b] s" TEXT label and
// snapshots_per_segment translucent whole-body mesh snapshots, all tinted with
// segmentColor(i). Ported from the TA-WBMP demo time-segment visualization.
void buildTimeSegments(const ParsedTrajectory & trajectory,
                       const WholeBodyKinematics & kinematics,
                       const TimeSegmentSpec & spec,
                       visualization_msgs::msg::MarkerArray & out,
                       MarkerContext & ctx);

struct PlaybackSpec
{
  double segment_duration{15.0};
};

// Playback frame at playback_time: active window (alpha 0.22) and already
// played part (alpha 1.0) of the base/EE lines, plus a
// "t=.. s | S<i> | <phase>" TEXT label, tinted with segmentColor(segment).
// The frame deliberately contains no robot mesh; the current robot is
// published separately on /wbmm/robot_mesh.
void buildPlaybackFrame(const ParsedTrajectory & trajectory,
                        double playback_time, const PlaybackSpec & spec,
                        const WholeBodyKinematics & kinematics,
                        visualization_msgs::msg::MarkerArray & out,
                        MarkerContext & ctx);

// 10-color time-window palette (TA-WBMP demo). Alpha 1.0.
const Eigen::Vector4d & segmentColor(std::size_t segment);
// Phase color basis: the four TA-WBMP phase colors plus WipePlanner phase-name
// aliases; completed white, failed red, unknown gray.
const Eigen::Vector4d & phaseColor(const std::string & phase);

// nav_msgs/Path encodings of the trajectory. phase_filter empty selects all
// waypoints, otherwise only those whose phase equals the filter (e.g.
// "NAVIGATE" for a navigation-only base path).
void buildBasePath(const ParsedTrajectory & trajectory,
                   const std::string & phase_filter, nav_msgs::msg::Path & out);
void buildEePath(const ParsedTrajectory & trajectory,
                 const WholeBodyKinematics & kinematics,
                 const std::string & phase_filter, nav_msgs::msg::Path & out);

}  // namespace wbmm_viz
