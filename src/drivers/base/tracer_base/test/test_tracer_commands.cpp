#include "tracer_base/tracer_messenger.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <utility>
#include <vector>

namespace {
// No CAN socket or real SDK transport is constructed by these tests.
struct FakeTracer {
  std::vector<std::pair<double, double>> commands;
  void SetMotionCommand(double v, double w) { commands.emplace_back(v, w); }
  void SetLightCommand(AgxLightMode, uint8_t) {}
  auto GetRobotState() -> decltype(std::declval<westonrobot::TracerRobot>().GetRobotState()) { return {}; }
  auto GetActuatorState() -> decltype(std::declval<westonrobot::TracerRobot>().GetActuatorState()) { return {}; }
};
}

TEST(TracerCommands, LatestQueueDispatchAndInvalidCommandStop)
{
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("tracer_command_test");
  node->declare_parameter("publish_command_timing", true);
  auto fake = std::make_shared<FakeTracer>();
  westonrobot::TracerMessenger<FakeTracer> messenger(fake, node.get());
  messenger.SetBaseFrame("base_footprint");
  messenger.SetOdometryFrame("odom");
  messenger.SetOdometryTopicName("/test_odom");
  messenger.SetupSubscription();
  auto pub = node->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 1);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  geometry_msgs::msg::Twist command;
  command.linear.x = 0.01;
  pub->publish(command);
  for (int i = 0; i < 100 && fake->commands.empty(); ++i) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_FALSE(fake->commands.empty());
  EXPECT_DOUBLE_EQ(fake->commands.back().first, 0.01);
  fake->commands.clear();
  // Let DDS receive a burst while the executor is not processing callbacks.
  for (int i = 1; i <= 10; ++i) {
    command.linear.x = i * 0.01;
    pub->publish(command);
  }
  rclcpp::sleep_for(std::chrono::milliseconds(50));
  executor.spin_some();
  ASSERT_EQ(fake->commands.size(), 1U);
  EXPECT_DOUBLE_EQ(fake->commands.back().first, 0.1);
  command.linear.x = std::numeric_limits<double>::quiet_NaN();
  pub->publish(command);
  for (int i = 0; i < 100 && fake->commands.size() < 2; ++i) {
    executor.spin_some();
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  ASSERT_EQ(fake->commands.size(), 2U);
  EXPECT_DOUBLE_EQ(fake->commands.back().first, 0.0);
  EXPECT_DOUBLE_EQ(fake->commands.back().second, 0.0);
  rclcpp::shutdown();
}
