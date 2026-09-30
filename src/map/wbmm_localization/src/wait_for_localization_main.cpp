#include "wbmm_localization/nodes.hpp"
#include <iostream>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 1;
  try {
    auto node = std::make_shared<wbmm_localization::WaitForLocalization>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    while (rclcpp::ok() && node->result() == -1)
      executor.spin_once(std::chrono::milliseconds(100));
    if (node->result() != -1)
      result = node->result();
  } catch (const std::exception &exc) {
    std::cerr << "wait_for_localization: " << exc.what() << '\n';
  }
  if (rclcpp::ok())
    rclcpp::shutdown();
  return result;
}
