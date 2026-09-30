#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace wbmm_localization {
// Only owns map -> cartographer_map; Cartographer and EKF own the other edges.
class CartographerGroundMap : public rclcpp::Node {
public:
  explicit CartographerGroundMap(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void anchor();
  void on_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void publish(const nav_msgs::msg::OccupancyGrid &msg);
  std::string ground_frame_, imu_frame_, internal_frame_, public_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  tf2_ros::StaticTransformBroadcaster broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr pending_;
  bool ready_ = false;
};
}  // namespace wbmm_localization
