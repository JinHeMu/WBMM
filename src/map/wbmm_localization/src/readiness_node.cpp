#include "wbmm_localization/nodes.hpp"
#include "wbmm_localization/readiness.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace wbmm_localization {
namespace {
using Steady = std::chrono::steady_clock;
double elapsed(Steady::time_point from) {
  return std::chrono::duration<double>(Steady::now() - from).count();
}
double seconds(const builtin_interfaces::msg::Time &stamp) {
  return stamp.sec + stamp.nanosec * 1e-9;
}
Quaternion quaternion(const geometry_msgs::msg::Quaternion &q) {
  return {q.x, q.y, q.z, q.w};
}
} // namespace

struct LocalizationReadiness::Impl {
  rclcpp::Node &node;
  std::string backend, map_frame, odom_frame, base_frame;
  bool require_imu, ever_ready = false;
  double sensor_timeout, future_tolerance, match_distance, min_match_ratio,
      min_known_ratio;
  double max_scan_range, startup_timeout;
  int min_endpoints, max_endpoints, occupied_threshold;
  StableWindow window;
  std::unique_ptr<Grid> grid;
  std::optional<double> processed_stamp;
  std::optional<Agreement> agreement;
  std::string reason = "waiting_for_map";
  Steady::time_point started = Steady::now(), scan_received{}, odom_received{},
                     imu_received{};
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr map;
  sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
  nav_msgs::msg::Odometry::ConstSharedPtr odom;
  sensor_msgs::msg::Imu::ConstSharedPtr imu;
  std::unique_ptr<tf2_ros::Buffer> buffer;
  std::unique_ptr<tf2_ros::TransformListener> listener;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
  rclcpp::TimerBase::SharedPtr timer;

  explicit Impl(rclcpp::Node &owner) : node(owner) {
    backend = node.declare_parameter<std::string>("backend",
                                                  "cartographer_localization");
    const auto scan_topic =
        node.declare_parameter<std::string>("scan_topic", "/scan");
    const auto map_topic =
        node.declare_parameter<std::string>("map_topic", "/map");
    const auto odom_topic =
        node.declare_parameter<std::string>("odom_topic", "/odometry/filtered");
    const auto imu_topic =
        node.declare_parameter<std::string>("imu_topic", "/imu/data");
    require_imu = node.declare_parameter<bool>("require_imu", true);
    map_frame = node.declare_parameter<std::string>("map_frame", "map");
    odom_frame = node.declare_parameter<std::string>("odom_frame", "odom");
    base_frame =
        node.declare_parameter<std::string>("base_frame", "base_footprint");
    sensor_timeout = node.declare_parameter<double>("sensor_timeout", 1.0);
    future_tolerance = node.declare_parameter<double>("future_tolerance", 0.3);
    const auto duration =
        node.declare_parameter<double>("stable_duration", 3.0);
    const auto min_samples = node.declare_parameter<int>("min_samples", 5);
    const auto translation =
        node.declare_parameter<double>("max_correction_translation", 0.15);
    const auto angle =
        node.declare_parameter<double>("max_correction_yaw", 0.1);
    match_distance = node.declare_parameter<double>("match_distance", 0.15);
    min_match_ratio = node.declare_parameter<double>("min_match_ratio", 0.65);
    min_known_ratio = node.declare_parameter<double>("min_known_ratio", 0.7);
    min_endpoints = node.declare_parameter<int>("min_endpoints", 30);
    max_endpoints = node.declare_parameter<int>("max_endpoints", 180);
    max_scan_range = node.declare_parameter<double>("max_scan_range", 30.0);
    occupied_threshold = node.declare_parameter<int>("occupied_threshold", 65);
    startup_timeout = node.declare_parameter<double>("startup_timeout", 30.0);
    for (double value :
         {sensor_timeout, max_scan_range, startup_timeout, match_distance}) {
      if (!std::isfinite(value) || value <= 0)
        throw std::invalid_argument(
            "Readiness time/range must be finite and positive");
    }
    for (double value : {min_match_ratio, min_known_ratio}) {
      if (!std::isfinite(value) || value <= 0 || value > 1)
        throw std::invalid_argument("Readiness ratios must lie in (0, 1]");
    }
    if (min_endpoints < 1 || max_endpoints < min_endpoints ||
        occupied_threshold < 1 || occupied_threshold > 100 ||
        !std::isfinite(future_tolerance) || future_tolerance < 0) {
      throw std::invalid_argument(
          "Invalid readiness counts, occupancy or future tolerance");
    }
    window = StableWindow(duration, min_samples, translation, angle);
    buffer = std::make_unique<tf2_ros::Buffer>(node.get_clock());
    listener =
        std::make_unique<tf2_ros::TransformListener>(*buffer, &node, false);
    const auto latched = rclcpp::QoS(1).transient_local();
    ready_pub = node.create_publisher<std_msgs::msg::Bool>(
        "/localization/ready", latched);
    status_pub = node.create_publisher<std_msgs::msg::String>(
        "/localization/status", 10);
    map_sub = node.create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, latched,
        [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
          onMap(std::move(msg));
        });
    scan_sub = node.create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          scan = std::move(msg);
          scan_received = Steady::now();
        });
    odom_sub = node.create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          odom = std::move(msg);
          odom_received = Steady::now();
        });
    imu_sub = node.create_subscription<sensor_msgs::msg::Imu>(
        imu_topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::Imu::ConstSharedPtr msg) {
          imu = std::move(msg);
          imu_received = Steady::now();
        });
    timer = node.create_wall_timer(std::chrono::milliseconds(200),
                                   [this] { tick(); });
  }

  void onMap(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
    // A refreshed map_load_time alone must not restart the stability window.
    if (map && msg->info.width == map->info.width &&
        msg->info.height == map->info.height &&
        msg->info.resolution == map->info.resolution &&
        msg->info.origin == map->info.origin &&
        msg->header.frame_id == map->header.frame_id && msg->data == map->data)
      return;
    window.reset();
    processed_stamp.reset();
    grid.reset();
    map = std::move(msg);
    try {
      if (map->header.frame_id != map_frame)
        throw std::invalid_argument("Map frame mismatch");
      const auto q = quaternion(map->info.origin.orientation);
      if (std::abs(quaternionMatrix(q)[2][2] - 1.0) > 1e-5)
        throw std::invalid_argument("Map origin must be planar");
      if (map->info.width >
              static_cast<uint32_t>(std::numeric_limits<int>::max()) ||
          map->info.height >
              static_cast<uint32_t>(std::numeric_limits<int>::max()))
        throw std::invalid_argument("Map too large");
      grid = std::make_unique<Grid>(
          map->info.width, map->info.height, map->info.resolution,
          Pose2d{map->info.origin.position.x, map->info.origin.position.y,
                 yaw(q)},
          map->data, match_distance, occupied_threshold);
    } catch (const std::invalid_argument &exc) {
      reason = exc.what();
    }
  }

  void invalidate(const std::string &why) {
    window.reset();
    processed_stamp.reset();
    agreement.reset();
    reason = why;
  }
  template <class Msg>
  bool sensorFresh(const std::shared_ptr<const Msg> &msg,
                   Steady::time_point received, double now) const {
    return msg && elapsed(received) <= sensor_timeout &&
           fresh(seconds(msg->header.stamp), now, sensor_timeout,
                 future_tolerance);
  }
  void evaluate(double now) {
    if (!grid) {
      invalidate("waiting_for_valid_map");
      return;
    }
    if (!sensorFresh(scan, scan_received, now)) {
      invalidate("waiting_for_fresh_scan");
      return;
    }
    if (!sensorFresh(odom, odom_received, now)) {
      invalidate("waiting_for_fresh_odom");
      return;
    }
    if (backend == "cartographer_localization" && require_imu &&
        !sensorFresh(imu, imu_received, now)) {
      invalidate("waiting_for_fresh_imu");
      return;
    }
    if (odom->header.frame_id != odom_frame ||
        odom->child_frame_id != base_frame) {
      invalidate("odometry_frame_mismatch");
      return;
    }
    const auto &p = odom->pose.pose.position;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      invalidate("invalid_odometry");
      return;
    }
    try {
      quaternionMatrix(quaternion(odom->pose.pose.orientation));
    } catch (const std::invalid_argument &) {
      invalidate("invalid_odometry_orientation");
      return;
    }
    const double stamp = seconds(scan->header.stamp);
    try {
      const auto correction =
          buffer->lookupTransform(map_frame, odom_frame, tf2::TimePointZero);
      if (!fresh(seconds(correction.header.stamp), now, sensor_timeout,
                 future_tolerance)) {
        invalidate("waiting_for_fresh_map_odom_tf");
        return;
      }
      if (processed_stamp && stamp == *processed_stamp)
        return;
      // Query BufferCore directly without the timeout overload. Humble checks
      // for a dedicated TF thread even when that overload's timeout is zero;
      // this node receives TF in its executor and retries on the next tick.
      const auto scan_time = tf2::TimePoint(std::chrono::nanoseconds(
          rclcpp::Time(scan->header.stamp).nanoseconds()));
      const auto transform = buffer->lookupTransform(
          map_frame, scan->header.frame_id, scan_time);
      const auto &t = transform.transform.translation;
      const auto points = scanEndpoints(
          scan->ranges, scan->angle_min, scan->angle_increment, scan->range_min,
          scan->range_max,
          quaternionMatrix(quaternion(transform.transform.rotation)),
          {t.x, t.y, t.z}, max_endpoints, max_scan_range);
      agreement = grid->agreement(points);
      const bool good =
          agreement->endpoints >= static_cast<std::size_t>(min_endpoints) &&
          agreement->known_ratio >= min_known_ratio &&
          agreement->match_ratio >= min_match_ratio;
      const auto &c = correction.transform.translation;
      window.update(stamp, good,
                    {c.x, c.y, yaw(quaternion(correction.transform.rotation))});
      processed_stamp = stamp;
      reason = window.ready()
                   ? "ready"
                   : (good ? "waiting_for_stability" : "scan_map_mismatch");
    } catch (const tf2::TransformException &exc) {
      invalidate(std::string("waiting_for_tf_or_valid_scan: ") + exc.what());
    } catch (const std::invalid_argument &exc) {
      invalidate(std::string("waiting_for_tf_or_valid_scan: ") + exc.what());
    }
  }
  void tick() {
    const double now = node.now().seconds();
    evaluate(now);
    const bool ready = window.ready();
    ever_ready |= ready;
    std::string state =
        ready ? "ready" : (ever_ready ? "lost" : "initializing");
    if (!ever_ready && elapsed(started) > startup_timeout)
      state = "timeout";
    nlohmann::json status = {{"stamp", now},
                             {"backend", backend},
                             {"ready", ready},
                             {"state", state},
                             {"reason", reason}};
    if (agreement) {
      status["endpoints"] = agreement->endpoints;
      status["known_ratio"] = agreement->known_ratio;
      status["match_ratio"] = agreement->match_ratio;
    }
    std_msgs::msg::Bool ready_msg;
    ready_msg.data = ready;
    ready_pub->publish(ready_msg);
    std_msgs::msg::String status_msg;
    status_msg.data = status.dump();
    status_pub->publish(status_msg);
  }
};

LocalizationReadiness::LocalizationReadiness(const rclcpp::NodeOptions &options)
    : Node("localization_readiness", options),
      impl_(std::make_unique<Impl>(*this)) {}
LocalizationReadiness::~LocalizationReadiness() = default;
} // namespace wbmm_localization
