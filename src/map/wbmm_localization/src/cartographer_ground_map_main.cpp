#include "wbmm_localization/ground_map.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<wbmm_localization::CartographerGroundMap>());
  rclcpp::shutdown();
  return 0;
}
