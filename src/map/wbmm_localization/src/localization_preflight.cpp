#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <chrono>
#include <set>
#include <string>

// Read-only ROS graph check before launching another localization stack.
// Never changes or terminates any existing process.
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("localization_preflight");
  const bool start_ekf = node->declare_parameter<bool>("start_ekf", false);
  const bool global_backend = node->declare_parameter<bool>("global_backend", true);
  const auto odom_topic = node->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
  const auto base_frame = node->declare_parameter<std::string>("base_frame", "base_footprint");
  const auto odom_frame = node->declare_parameter<std::string>("odom_frame", "odom");
  std::set<std::string> odom_tf_publishers;
  bool existing_global_tf = false;
  auto subscription = node->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", rclcpp::QoS(100),
      [&](tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo &info) {
        for (const auto &tf : msg->transforms) {
          if (tf.child_frame_id == odom_frame &&
              (tf.header.frame_id == "map" || tf.header.frame_id == "cartographer_map"))
            existing_global_tf = true;
          if (tf.header.frame_id == odom_frame && tf.child_frame_id == base_frame) {
            const auto &gid = info.get_rmw_message_info().publisher_gid;
            odom_tf_publishers.emplace(reinterpret_cast<const char *>(gid.data), sizeof(gid.data));
          }
        }
      });
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(1200);
  while (rclcpp::ok() && std::chrono::steady_clock::now() < end)
    executor.spin_once(std::chrono::milliseconds(20));
  const auto odom_publishers = node->count_publishers(odom_topic);
  std::string error;
  if (start_ekf && (odom_publishers > 0 || !odom_tf_publishers.empty()))
    error = "An odometry/TF publisher already exists. Reuse it with start_ekf:=false; "
            "do not start a second EKF.";
  else if (odom_publishers > 1 || odom_tf_publishers.size() > 1)
    error = "Multiple odometry or odom->base_footprint TF publishers exist. "
            "Stop the duplicate localization stack before retrying.";
  else if (global_backend && (node->count_publishers("/map") > 0 ||
                             node->count_publishers("/cartographer/map") > 0 ||
                             existing_global_tf))
    error = "A map or global TF publisher already exists. Stop the previous mapping/localization "
            "stack before starting a new one.";
  if (error.empty())
    RCLCPP_INFO(node->get_logger(), "Publisher check passed; starting localization.");
  else
    RCLCPP_ERROR(node->get_logger(), "%s", error.c_str());
  rclcpp::shutdown();
  return error.empty() ? 0 : 1;
}
