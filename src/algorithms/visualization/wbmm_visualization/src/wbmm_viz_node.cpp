#include "wbmm_visualization/contract.hpp"
#include "wbmm_visualization/markers.hpp"
#include "wbmm_visualization/whole_body_kinematics.hpp"

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace wbmm_viz
{

namespace
{
// Freshness window for the live JointState; older states fall back to the
// playback-interpolated trajectory state.
constexpr double kLiveStateFreshness = 0.5;
}  // namespace

// Reference whole-body visualization node. Consumes the plain data contract
// (9D JointTrajectory + phase schedule + live JointState/phase) and renders
// the unified /wbmm/* marker topics. All poses are baked into the marker
// frame; no tf lookup. Single-threaded executor assumed: subscriptions mutate
// shared state, timers read it.
class WbmmVizNode final : public rclcpp::Node
{
public:
  WbmmVizNode()
  : Node("wbmm_viz")
  {
    declare_parameter("urdf_file", "");
    declare_parameter("ee_frame", "tool0");
    declare_parameter("trajectory_topic", "/wbmm/whole_body_trajectory");
    declare_parameter("phase_schedule_topic", "/wbmm/phase_schedule");
    declare_parameter("live_state_topic", "/wbmm/live_state");
    declare_parameter("live_phase_topic", "/wbmm/live_phase");
    declare_parameter("time_segment_duration", 15.0);
    declare_parameter("segment_snapshots", 2);
    declare_parameter("playback_enabled", true);
    declare_parameter("playback_rate", 5.0);
    declare_parameter("playback_period", 0.10);
    declare_parameter("playback_loop", true);
    declare_parameter("robot_mesh_rate", 30.0);
    declare_parameter("publish_base_path", true);
    declare_parameter("publish_ee_path", true);
    declare_parameter("base_path_phase_filter", "");

    const std::string urdf_file = get_parameter("urdf_file").as_string();
    if (urdf_file.empty()) {
      throw std::runtime_error("urdf_file parameter is required");
    }
    kinematics_ = std::make_unique<WholeBodyKinematics>(
      urdf_file, get_parameter("ee_frame").as_string());

    const auto transient = rclcpp::QoS(1).reliable().transient_local();
    const auto reliable = rclcpp::QoS(1).reliable();

    trajectory_subscription_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      get_parameter("trajectory_topic").as_string(), transient,
      [this](const trajectory_msgs::msg::JointTrajectory & msg) {
        onTrajectory(msg);
      });
    phase_schedule_subscription_ = create_subscription<std_msgs::msg::String>(
      get_parameter("phase_schedule_topic").as_string(), transient,
      [this](const std_msgs::msg::String & msg) { onPhaseSchedule(msg); });
    live_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("live_state_topic").as_string(), reliable,
      [this](const sensor_msgs::msg::JointState & msg) { onLiveState(msg); });
    live_phase_subscription_ = create_subscription<std_msgs::msg::String>(
      get_parameter("live_phase_topic").as_string(), reliable,
      [this](const std_msgs::msg::String & msg) { live_phase_ = msg.data; });

    robot_mesh_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/wbmm/robot_mesh", transient);
    time_segments_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/wbmm/time_segments", transient);
    playback_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/wbmm/playback", reliable);
    base_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/wbmm/base_path", transient);
    ee_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/wbmm/ee_path", transient);

    if (get_parameter("playback_enabled").as_bool()) {
      playback_timer_ = create_wall_timer(
        std::chrono::duration<double>(
          get_parameter("playback_period").as_double()),
        std::bind(&WbmmVizNode::onPlaybackTick, this));
    }
    const double mesh_rate = std::max(
      1.0, get_parameter("robot_mesh_rate").as_double());
    mesh_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / mesh_rate),
      std::bind(&WbmmVizNode::onMeshTick, this));

    RCLCPP_INFO(
      get_logger(),
      "wbmm_viz ready: urdf=%s ee_frame=%s, publishing unified /wbmm/* topics "
      "(skip-generation active: no marker work without subscribers)",
      urdf_file.c_str(), get_parameter("ee_frame").as_string().c_str());
  }

private:
  void onTrajectory(const trajectory_msgs::msg::JointTrajectory & msg)
  {
    try {
      trajectory_ = parseJointTrajectory(
        msg, schedule_.empty() ? nullptr : &schedule_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(
        get_logger(), "Rejected whole-body trajectory: %s", error.what());
      return;
    }
    last_trajectory_msg_ = msg;
    playback_time_ = 0.0;
    frame_id_ = trajectory_.frame_id;
    if (time_segments_publisher_->get_subscription_count() > 0) {
      publishTimeSegments();
    }
    if ((publish_base_path() && base_path_publisher_->get_subscription_count() > 0) ||
      (publish_ee_path() && ee_path_publisher_->get_subscription_count() > 0))
    {
      publishPaths();
    }
    RCLCPP_INFO(
      get_logger(), "Parsed %zu-point whole-body trajectory, %.1f s",
      trajectory_.waypoints.size(), trajectory_.duration);
  }

  void onPhaseSchedule(const std_msgs::msg::String & msg)
  {
    std::vector<std::pair<double, std::string>> decoded;
    if (!decodePhaseSchedule(msg.data, decoded)) {
      RCLCPP_WARN(get_logger(), "Ignoring malformed phase schedule");
      return;
    }
    schedule_ = msg.data;
    if (trajectory_.waypoints.empty()) {
      return;  // trajectory will pick the schedule up on arrival
    }
    // The schedule arrived after the trajectory: re-parse to refresh the
    // per-waypoint phases and re-publish the phase-dependent outputs.
    try {
      trajectory_ = parseJointTrajectory(last_trajectory_msg_, &schedule_);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Re-parse failed: %s", error.what());
      return;
    }
    if (time_segments_publisher_->get_subscription_count() > 0) {
      publishTimeSegments();
    }
  }

  void onLiveState(const sensor_msgs::msg::JointState & msg)
  {
    if (msg.position.size() < 9) {
      RCLCPP_WARN_ONCE(
        get_logger(),
        "live_state needs 9 positions, got %zu; ignoring", msg.position.size());
      return;
    }
    Eigen::VectorXd state(9);
    for (int index = 0; index < 9; ++index) {
      state[index] = msg.position[index];
    }
    live_state_ = std::move(state);
    live_state_stamp_ = now();
    if (!msg.header.frame_id.empty()) {
      live_frame_id_ = msg.header.frame_id;
    }
  }

  void onPlaybackTick()
  {
    if (trajectory_.waypoints.empty()) {
      return;
    }
    const double period = std::max(
      0.02, get_parameter("playback_period").as_double());
    const double rate = std::max(
      0.0, get_parameter("playback_rate").as_double());
    playback_time_ += period * rate;
    if (playback_time_ > trajectory_.duration) {
      playback_time_ = get_parameter("playback_loop").as_bool() ?
        0.0 : trajectory_.duration;
    }
    if (playback_publisher_->get_subscription_count() > 0) {
      publishPlayback();
    }
  }

  void onMeshTick()
  {
    if (robot_mesh_publisher_->get_subscription_count() == 0) {
      return;
    }
    const auto state = currentDisplayState();
    if (!state) {
      return;
    }
    MarkerContext ctx;
    visualization_msgs::msg::MarkerArray markers;
    beginMarkerArray(markers, ctx, currentFrameId(), now());
    // Live phase tints the live robot; playback robots stay white (TA-WBMP
    // convention).
    const Eigen::Vector4d tint = live_phase_.empty() ?
      Eigen::Vector4d(1.0, 1.0, 1.0, 1.0) : phaseColor(live_phase_);
    appendRobotSnapshot(
      markers, ctx, kinematics_->visualGeometry(*state), "current_robot",
      tint, true, false, nullptr, nullptr);
    robot_mesh_publisher_->publish(markers);
  }

  void publishPlayback()
  {
    MarkerContext ctx;
    visualization_msgs::msg::MarkerArray markers;
    beginMarkerArray(markers, ctx, currentFrameId(), now());
    buildPlaybackFrame(
      trajectory_, playback_time_,
      {get_parameter("time_segment_duration").as_double()},
      *kinematics_, markers, ctx);
    playback_publisher_->publish(markers);
  }

  void publishTimeSegments()
  {
    MarkerContext ctx;
    visualization_msgs::msg::MarkerArray markers;
    beginMarkerArray(markers, ctx, currentFrameId(), now());
    buildTimeSegments(
      trajectory_, *kinematics_,
      {get_parameter("time_segment_duration").as_double(),
       static_cast<int>(get_parameter("segment_snapshots").as_int())},
      markers, ctx);
    time_segments_publisher_->publish(markers);
  }

  void publishPaths()
  {
    const std::string filter =
      get_parameter("base_path_phase_filter").as_string();
    if (publish_base_path() && base_path_publisher_->get_subscription_count() > 0) {
      nav_msgs::msg::Path path;
      buildBasePath(trajectory_, filter, path);
      base_path_publisher_->publish(path);
    }
    if (publish_ee_path() && ee_path_publisher_->get_subscription_count() > 0) {
      nav_msgs::msg::Path path;
      buildEePath(trajectory_, *kinematics_, filter, path);
      ee_path_publisher_->publish(path);
    }
  }

  std::optional<Eigen::VectorXd> currentDisplayState()
  {
    if (live_state_ && (now() - live_state_stamp_).seconds() < kLiveStateFreshness) {
      return live_state_;
    }
    if (!trajectory_.waypoints.empty()) {
      return toStateVector(waypointAt(trajectory_, playback_time_));
    }
    return std::nullopt;
  }

  bool publish_base_path() const
  {
    return get_parameter("publish_base_path").as_bool();
  }

  bool publish_ee_path() const
  {
    return get_parameter("publish_ee_path").as_bool();
  }

  std::string currentFrameId() const
  {
    if (!frame_id_.empty()) {
      return frame_id_;
    }
    if (!live_frame_id_.empty()) {
      return live_frame_id_;
    }
    return "odom";
  }

  std::unique_ptr<WholeBodyKinematics> kinematics_;
  ParsedTrajectory trajectory_;
  trajectory_msgs::msg::JointTrajectory last_trajectory_msg_;
  std::string schedule_;
  std::string frame_id_;
  std::string live_frame_id_;
  std::string live_phase_;
  std::optional<Eigen::VectorXd> live_state_;
  rclcpp::Time live_state_stamp_;
  double playback_time_{0.0};

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr
    trajectory_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    phase_schedule_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
    live_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr
    live_phase_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    robot_mesh_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    time_segments_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
    playback_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr base_path_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ee_path_publisher_;
  rclcpp::TimerBase::SharedPtr playback_timer_;
  rclcpp::TimerBase::SharedPtr mesh_timer_;
};

}  // namespace wbmm_viz

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wbmm_viz::WbmmVizNode>());
  rclcpp::shutdown();
  return 0;
}
