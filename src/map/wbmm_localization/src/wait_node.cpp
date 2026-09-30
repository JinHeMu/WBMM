#include "wbmm_localization/nodes.hpp"
#include "wbmm_localization/readiness.hpp"

#include <chrono>
#include <cmath>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>

namespace wbmm_localization {
struct WaitForLocalization::Impl {
  rclcpp::Node &node;
  std::string backend;
  double timeout;
  int result = -1;
  std::chrono::steady_clock::time_point started =
      std::chrono::steady_clock::now();
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription;
  rclcpp::TimerBase::SharedPtr timer;

  explicit Impl(rclcpp::Node &owner) : node(owner) {
    backend = node.declare_parameter<std::string>("backend", "");
    timeout = node.declare_parameter<double>("timeout", 30.0);
    const auto topic = node.declare_parameter<std::string>(
        "status_topic", "/localization/status");
    if (!std::isfinite(timeout) || timeout <= 0)
      throw std::invalid_argument("timeout must be finite and positive");
    subscription = node.create_subscription<std_msgs::msg::String>(
        topic, 10, [this](std_msgs::msg::String::ConstSharedPtr msg) {
          if (result != -1)
            return;
          const auto status = nlohmann::json::parse(msg->data, nullptr, false);
          if (statusIsReady(status, node.now().seconds(), backend)) {
            RCLCPP_INFO(node.get_logger(),
                        "Localization ready; releasing planner startup.");
            result = 0;
          }
        });
    // Wall time continues when the simulation clock is paused or absent.
    timer = node.create_wall_timer(std::chrono::milliseconds(100), [this] {
      if (result == -1 && std::chrono::duration<double>(
                              std::chrono::steady_clock::now() - started)
                                  .count() >= timeout) {
        RCLCPP_ERROR(
            node.get_logger(),
            "Localization startup timed out; planner remains unstarted.");
        result = 1;
      }
    });
  }
};
WaitForLocalization::WaitForLocalization(const rclcpp::NodeOptions &options)
    : Node("wait_for_localization", options),
      impl_(std::make_unique<Impl>(*this)) {}
WaitForLocalization::~WaitForLocalization() = default;
int WaitForLocalization::result() const { return impl_->result; }
} // namespace wbmm_localization
