#include <rclcpp/rclcpp.hpp>
#include <plan_manage/remani_replan_fsm.h>

using namespace remani_planner;

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("remani_planner_node");

  REMANIReplanFSM remani_replan;

  remani_replan.init(node);
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
