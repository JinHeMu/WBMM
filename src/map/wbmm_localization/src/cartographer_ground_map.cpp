#include "wbmm_localization/ground_map.hpp"

#include <cmath>
#include <stdexcept>
#include <tf2/exceptions.h>

namespace wbmm_localization {
CartographerGroundMap::CartographerGroundMap(const rclcpp::NodeOptions &options)
    : Node("cartographer_ground_map", options), buffer_(get_clock()),
      listener_(buffer_, this, false), broadcaster_(this) {
  ground_frame_ = declare_parameter<std::string>("ground_frame", "base_footprint");
  imu_frame_ = declare_parameter<std::string>("imu_frame", "imu_link");
  internal_frame_ = declare_parameter<std::string>("internal_frame", "cartographer_map");
  public_frame_ = declare_parameter<std::string>("public_frame", "map");
  if (ground_frame_.empty() || imu_frame_.empty() || internal_frame_.empty() ||
      public_frame_.empty() || public_frame_ == internal_frame_)
    throw std::invalid_argument("Ground-map frames must be nonempty and map frames distinct");
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);
  subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/cartographer/map", qos,
      [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) { on_map(msg); });
  timer_ = create_wall_timer(std::chrono::milliseconds(200), [this] { anchor(); });
}

void CartographerGroundMap::anchor() {
  if (ready_) return;
  try {
    const auto extrinsic = buffer_.lookupTransform(ground_frame_, imu_frame_, tf2::TimePointZero);
    // This offset is frozen for a rigid IMU extrinsic, never taken from map/odom.
    // A dynamic extrinsic cannot safely anchor a static map transform.
    if (rclcpp::Time(extrinsic.header.stamp).nanoseconds() != 0) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Ground-to-IMU extrinsic must come entirely from static TF");
      return;
    }
    const double height = extrinsic.transform.translation.z;
    if (!std::isfinite(height)) return;
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = public_frame_;
    transform.child_frame_id = internal_frame_;
    transform.transform.translation.z = height;
    transform.transform.rotation.w = 1.;
    broadcaster_.sendTransform(transform);
    ready_ = true;
    timer_->cancel();
    RCLCPP_INFO(get_logger(), "Ground map anchored %.3f m below Cartographer origin", height);
    if (pending_) {
      publish(*pending_);
      pending_.reset();
    }
  } catch (const tf2::TransformException &error) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                         "Waiting for static %s -> %s: %s", ground_frame_.c_str(),
                         imu_frame_.c_str(), error.what());
  }
}

void CartographerGroundMap::on_map(nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg) {
  if (msg->header.frame_id != internal_frame_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Expected internal map frame %s; got %s", internal_frame_.c_str(),
                          msg->header.frame_id.c_str());
    return;
  }
  if (!ready_) pending_ = msg;
  else publish(*msg);
}

void CartographerGroundMap::publish(const nav_msgs::msg::OccupancyGrid &msg) {
  auto out = msg;
  // Parallel map frames share x/y/yaw. Project only the raster onto ground;
  // submaps, tracking poses and 3D point clouds keep their real heights.
  out.header.frame_id = public_frame_;
  out.info.origin.position.z = 0.;
  publisher_->publish(out);
}
}  // namespace wbmm_localization
