#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

using namespace std::chrono_literals;
class PreflightRos : public testing::Test {
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }
  int run(bool start_ekf, int odom_count, bool map, int tf_count = 0) {
    auto source = std::make_shared<rclcpp::Node>("preflight_test_source");
    std::vector<rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr> odoms;
    std::vector<rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr> transforms;
    for (int i = 0; i < odom_count; ++i)
      odoms.push_back(source->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10));
    for (int i = 0; i < tf_count; ++i)
      transforms.push_back(source->create_publisher<tf2_msgs::msg::TFMessage>("/tf", 100));
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr maps;
    if (map) maps = source->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 1);
    const char *parameter = start_ekf ? "start_ekf:=true" : "start_ekf:=false";
    const pid_t child = fork();
    if (child == 0) {
      execl(PREFLIGHT_PATH, PREFLIGHT_PATH, "--ros-args", "-p", parameter, nullptr);
      _exit(127);
    }
    if (child < 0) return -1;
    struct Cleanup {
      pid_t child;
      bool reaped = false;
      ~Cleanup() { if (!reaped) { kill(child, SIGKILL); waitpid(child, nullptr, 0); } }
    } cleanup{child};
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(source);
    const auto deadline = std::chrono::steady_clock::now() + 4s;
    while (std::chrono::steady_clock::now() < deadline) {
      tf2_msgs::msg::TFMessage msg;
      geometry_msgs::msg::TransformStamped tf;
      tf.header.frame_id = "odom"; tf.child_frame_id = "base_footprint";
      tf.header.stamp = source->now(); tf.transform.rotation.w = 1.;
      msg.transforms.push_back(tf);
      for (const auto &pub : transforms) pub->publish(msg);
      executor.spin_once(10ms);
      int status = 0;
      if (waitpid(child, &status, WNOHANG) == child) {
        cleanup.reaped = true;
        return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
      }
    }
    return -1;
  }
};

TEST_F(PreflightRos, StartsNewEkfInEmptyGraph) { EXPECT_EQ(run(true, 0, false), 0); }
TEST_F(PreflightRos, ReusesOneExistingEkf) { EXPECT_EQ(run(false, 1, false), 0); }
TEST_F(PreflightRos, RejectsSecondEkf) { EXPECT_EQ(run(true, 1, false), 1); }
TEST_F(PreflightRos, RejectsDuplicateOdometry) { EXPECT_EQ(run(false, 2, false), 1); }
TEST_F(PreflightRos, RejectsAnotherMapPublisher) { EXPECT_EQ(run(false, 0, true), 1); }
TEST_F(PreflightRos, RejectsDuplicateBaseTfEvenWithoutOdometryMessages) {
  EXPECT_EQ(run(false, 0, false, 2), 1);
}
