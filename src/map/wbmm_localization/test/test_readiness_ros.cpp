#include "wbmm_localization/nodes.hpp"
#include <cmath>
#include <gtest/gtest.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>

using namespace std::chrono_literals;
using namespace wbmm_localization;
namespace {
class ReadinessRos : public ::testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};
geometry_msgs::msg::TransformStamped transform(const std::string &parent,
                                               const std::string &child,
                                               const rclcpp::Time &stamp) {
  geometry_msgs::msg::TransformStamped msg;
  msg.header.frame_id = parent;
  msg.child_frame_id = child;
  msg.header.stamp = stamp;
  msg.transform.rotation.w = 1.;
  return msg;
}
} // namespace

TEST_F(ReadinessRos, StableMatchExpiryAndWaiterSuccess) {
  auto readiness = std::make_shared<LocalizationReadiness>(
      rclcpp::NodeOptions().parameter_overrides(
          {rclcpp::Parameter("stable_duration", .6),
           rclcpp::Parameter("min_samples", 3),
           rclcpp::Parameter("sensor_timeout", .5)}));
  auto waiter = std::make_shared<WaitForLocalization>(
      rclcpp::NodeOptions().parameter_overrides(
          {rclcpp::Parameter("backend", "cartographer_localization"),
           rclcpp::Parameter("timeout", 3.)}));
  auto source = std::make_shared<rclcpp::Node>("readiness_test_source");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(readiness);
  executor.add_node(waiter);
  executor.add_node(source);
  nlohmann::json last;
  auto subscription = source->create_subscription<std_msgs::msg::String>(
      "/localization/status", 10,
      [&last](std_msgs::msg::String::ConstSharedPtr msg) {
        last = nlohmann::json::parse(msg->data);
      });
  auto map_pub = source->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local());
  auto scan_pub =
      source->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  auto odom_pub = source->create_publisher<nav_msgs::msg::Odometry>(
      "/odometry/filtered", 10);
  auto imu_pub =
      source->create_publisher<sensor_msgs::msg::Imu>("/imu/data", 10);
  tf2_ros::TransformBroadcaster dynamic(source);
  tf2_ros::StaticTransformBroadcaster fixed(source);
  fixed.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
      transform("base_footprint", "laser_link", source->now()),
      transform("base_footprint", "imu_link", source->now())});
  nav_msgs::msg::OccupancyGrid map;
  map.header.frame_id = "map";
  map.info.width = map.info.height = 100;
  map.info.resolution = .05;
  map.info.origin.position.x = map.info.origin.position.y = -2.5;
  map.info.origin.orientation.w = 1.;
  map.data.assign(10000, 0);
  sensor_msgs::msg::LaserScan scan;
  constexpr double pi = 3.14159265358979323846;
  scan.header.frame_id = "laser_link";
  scan.angle_min = -pi;
  scan.angle_increment = pi / 180;
  scan.range_min = .08;
  scan.range_max = 30;
  for (int i = 0; i < 360; ++i) {
    const double angle = -pi + i * pi / 180,
                 distance = 2 / std::max(std::abs(std::cos(angle)),
                                         std::abs(std::sin(angle)));
    scan.ranges.push_back(distance);
    const int x = std::floor((distance * std::cos(angle) + 2.5) / .05),
              y = std::floor((distance * std::sin(angle) + 2.5) / .05);
    map.data[y * 100 + x] = 100;
  }
  map_pub->publish(map);
  auto run_until = [&](const auto &predicate, double timeout,
                       bool publish_scan) {
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration<double>(timeout);
    while (std::chrono::steady_clock::now() < deadline) {
      const auto now = source->now();
      dynamic.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
          transform("map", "odom", now),
          transform("odom", "base_footprint", now)});
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = now;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "base_footprint";
      odom.pose.pose.orientation.w = 1.;
      odom_pub->publish(odom);
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = now;
      imu.header.frame_id = "imu_link";
      imu.linear_acceleration.z = 9.81;
      imu_pub->publish(imu);
      if (publish_scan) {
        scan.header.stamp = now - rclcpp::Duration::from_seconds(.1);
        scan_pub->publish(scan);
      }
      for (int i = 0; i < 10; ++i) {
        executor.spin_some();
        std::this_thread::sleep_for(5ms);
      }
      if (predicate())
        return true;
    }
    return false;
  };
  ASSERT_TRUE(run_until(
      [&] { return waiter->result() == 0 && last.value("ready", false); }, 5,
      true))
      << last.dump();
  EXPECT_GT(last["match_ratio"].get<double>(), .95);
  ASSERT_TRUE(
      run_until([&] { return last.value("state", "") == "lost"; }, 2, false))
      << last.dump();
  EXPECT_FALSE(last["ready"].get<bool>());
}

TEST_F(ReadinessRos, PausedSimulationClockStillTimesOut) {
  auto waiter = std::make_shared<WaitForLocalization>(
      rclcpp::NodeOptions().parameter_overrides(
          {rclcpp::Parameter("use_sim_time", true),
           rclcpp::Parameter("timeout", .5)}));
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(waiter);
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline && waiter->result() == -1)
    executor.spin_once(50ms);
  EXPECT_EQ(waiter->result(), 1);
}
