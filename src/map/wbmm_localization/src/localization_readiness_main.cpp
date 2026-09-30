#include "wbmm_localization/nodes.hpp"
#include <iostream>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<wbmm_localization::LocalizationReadiness>());
  } catch (const std::exception &exc) {
    std::cerr << "localization_readiness: " << exc.what() << '\n';
    result = 1;
  }
  if (rclcpp::ok())
    rclcpp::shutdown();
  return result;
}
