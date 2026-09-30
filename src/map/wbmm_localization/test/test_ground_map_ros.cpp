#include "wbmm_localization/ground_map.hpp"
#include <gtest/gtest.h>
#include <tf2_ros/transform_broadcaster.h>
#include <thread>

using namespace std::chrono_literals;
namespace {
geometry_msgs::msg::TransformStamped tf(const std::string &parent,
                                        const std::string &child, double z) {
  geometry_msgs::msg::TransformStamped out;
  out.header.frame_id = parent;
  out.child_frame_id = child;
  out.transform.translation.z = z;
  out.transform.rotation.w = 1.;
  return out;
}
class GroundMapRos : public testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
};
}  // namespace

TEST_F(GroundMapRos, WaitForExtrinsicsProjectGridComposeGroundTFAndLatchForLateSubscriber) {
  auto bridge = std::make_shared<wbmm_localization::CartographerGroundMap>();
  auto source = std::make_shared<rclcpp::Node>("ground_map_test");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(bridge); executor.add_node(source);
  tf2_ros::Buffer buffer(source->get_clock());
  tf2_ros::TransformListener listener(buffer, source, false);
  tf2_ros::StaticTransformBroadcaster fixed(source);
  tf2_ros::TransformBroadcaster dynamic(source);
  auto input = source->create_publisher<nav_msgs::msg::OccupancyGrid>(
      "/cartographer/map", rclcpp::QoS(1).transient_local());
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr output;
  auto sub = source->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(), [&output](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { output = msg; });
  auto spin = [&](const auto &done, double seconds) {
    const auto end = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    do {
      executor.spin_some();
      if (done()) return true;
      std::this_thread::sleep_for(10ms);
    } while (std::chrono::steady_clock::now() < end);
    return done();
  };
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "cartographer_map";
  grid.header.stamp = source->now();
  grid.info.width = grid.info.height = 2; grid.info.resolution = .05;
  grid.info.origin.position.x = -1.25; grid.info.origin.position.y = 2.5;
  grid.info.origin.position.z = .1; grid.info.origin.orientation.w = 1.;
  grid.data = {-1, 0, 50, 100};
  input->publish(grid);
  EXPECT_FALSE(spin([&] { return bool(output); }, .4));
  auto imu = tf("base_link", "imu_link", .23);
  imu.transform.translation.x = imu.transform.translation.y = .185;
  fixed.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{
      tf("base_footprint", "base_link", .147), imu});
  ASSERT_TRUE(spin([&] { return bool(output); }, 3.));
  EXPECT_EQ(output->header.frame_id, "map");
  EXPECT_EQ(output->header.stamp, grid.header.stamp);
  EXPECT_EQ(output->data, grid.data);
  EXPECT_EQ(output->info.origin.position.x, grid.info.origin.position.x);
  EXPECT_EQ(output->info.origin.position.y, grid.info.origin.position.y);
  EXPECT_EQ(output->info.origin.position.z, 0.);
  auto carto = tf("cartographer_map", "odom", -.377);
  carto.header.stamp = source->now();
  auto odom = tf("odom", "base_footprint", 0.); odom.header.stamp = carto.header.stamp;
  dynamic.sendTransform(std::vector<geometry_msgs::msg::TransformStamped>{carto, odom});
  ASSERT_TRUE(spin([&] { return buffer.canTransform("map", "base_footprint", tf2::TimePointZero); }, 2.));
  EXPECT_NEAR(buffer.lookupTransform("map", "cartographer_map", tf2::TimePointZero)
                  .transform.translation.z, .377, 1e-9);
  EXPECT_NEAR(buffer.lookupTransform("map", "base_footprint", tf2::TimePointZero)
                  .transform.translation.z, 0., 1e-9);
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr late;
  auto late_sub = source->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map", rclcpp::QoS(1).transient_local(), [&late](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { late = msg; });
  ASSERT_TRUE(spin([&] { return bool(late); }, 2.));
  EXPECT_EQ(late->data, grid.data);
  output.reset(); grid.header.frame_id = "map"; input->publish(grid);
  EXPECT_FALSE(spin([&] { return bool(output); }, .3));
}
