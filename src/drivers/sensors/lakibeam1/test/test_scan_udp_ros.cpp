#include "msop_fixture.hpp"
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <arpa/inet.h>
#include <signal.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <unistd.h>
#include <thread>
#include <cmath>

using namespace std::chrono_literals;
TEST(ScanUdpRos, ActualDriverRejectsLatePacketsAndExitsWithoutSensorTraffic) {
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    execl(SCAN_NODE_PATH, SCAN_NODE_PATH, "--ros-args", "-p", "hostip:=127.0.0.1",
          "-p", "port:=\"32368\"", "-p", "scanfreq:=\"25\"", "-p", "configure_sensor:=false",
          "-p", "inverted:=true", "-p", "output_topic:=/synthetic_scan", nullptr);
    _exit(127);
  }
  struct Cleanup {
    pid_t child;
    int socket = -1;
    ~Cleanup() { kill(child, SIGKILL); waitpid(child, nullptr, 0); if (socket >= 0) close(socket); }
  } cleanup{child};
  rclcpp::init(0, nullptr);
  auto node = std::make_shared<rclcpp::Node>("udp_scan_test");
  std::vector<sensor_msgs::msg::LaserScan> scans;
  auto sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "/synthetic_scan", rclcpp::SensorDataQoS(),
      [&scans](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) { scans.push_back(*msg); });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + 4s;
  while (node->count_publishers("/synthetic_scan") == 0 && std::chrono::steady_clock::now() < deadline)
    executor.spin_once(20ms);
  ASSERT_EQ(node->count_publishers("/synthetic_scan"), 1u);
  cleanup.socket = socket(AF_INET, SOCK_DGRAM, 0);
  ASSERT_GE(cleanup.socket, 0);
  sockaddr_in destination{};
  destination.sin_family = AF_INET;
  destination.sin_port = htons(32368);
  destination.sin_addr.s_addr = inet_addr("127.0.0.1");
  auto send = [&](const auto &packet, size_t length) {
    EXPECT_EQ(sendto(cleanup.socket, packet.data(), length, 0,
                     reinterpret_cast<sockaddr *>(&destination), sizeof(destination)),
              static_cast<ssize_t>(length));
  };
  uint32_t timestamp = 100000;
  msop_test::Packet late{};
  for (int turn = 0; turn < 5; ++turn) {
    for (int i = 0; i < 8; ++i) {
      auto packet = msop_test::packet(i * 4800, timestamp, i == 7 ? 6 : 12);
      if (turn == 0 && i == 0) late = packet;
      if (turn == 1 && i == 2) {
        send(late, late.size());
        send(packet, packet.size() - 1);
      }
      send(packet, packet.size());
      timestamp += i == 7 ? 2667 : 5333;
      executor.spin_some();
      std::this_thread::sleep_for(i == 7 ? 3ms : 5ms);
    }
  }
  for (int i = 0; i < 30; ++i) executor.spin_once(10ms);
  ASSERT_EQ(scans.size(), 4u);
  for (size_t i = 0; i < scans.size(); ++i) {
    const auto &scan = scans[i];
    EXPECT_EQ(scan.ranges.size(), 1440u);
    EXPECT_EQ(scan.header.frame_id, "laser_link");
    EXPECT_NEAR(scan.scan_time, .04, 1e-5);
    EXPECT_NEAR(scan.ranges.front(), 1., 1e-6);
    EXPECT_NEAR(scan.ranges.back(), 2.439, 1e-6);
    EXPECT_LT(scan.angle_increment, 0.f);
    EXPECT_NEAR(scan.angle_max, scan.angle_min + 1439 * scan.angle_increment, 1e-6);
    if (i) { EXPECT_GT(rclcpp::Time(scan.header.stamp).nanoseconds(),
                    rclcpp::Time(scans[i-1].header.stamp).nanoseconds() +
                    std::llround(scans[i-1].time_increment * 1439 * 1e9)); }
  }
  // No recvfrom deadlock when Ctrl+C arrives and the sensor is silent.
  kill(child, SIGINT);
  int status = 0;
  bool exited = false;
  for (int i = 0; i < 100; ++i) {
    if (waitpid(child, &status, WNOHANG) == child) { exited = true; break; }
    std::this_thread::sleep_for(10ms);
  }
  EXPECT_TRUE(exited);
  if (exited) { EXPECT_TRUE(WIFEXITED(status)); EXPECT_EQ(WEXITSTATUS(status), 0); }
  rclcpp::shutdown();
}
