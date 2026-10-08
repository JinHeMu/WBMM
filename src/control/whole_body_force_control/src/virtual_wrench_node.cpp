#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Geometry>

#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

// C++ manual simulation source. It has no hardware interface and publishes
// zero force until explicitly changed through its ROS parameters.
class VirtualWrenchNode final : public rclcpp::Node {
public:
  VirtualWrenchNode() : Node("virtual_force_publisher") {
    const auto topic = declare_parameter<std::string>(
        "topic", "/whole_body_force_control/fake_wrench");
    frame_ = declare_parameter<std::string>("frame_id", "jk_se_vi_200_link");
    const auto rate = declare_parameter<double>("rate", 125.0);
    force_ = declare_parameter<std::vector<double>>("force", {0., 0., 0.});
    torque_ = declare_parameter<std::vector<double>>("torque", {0., 0., 0.});
    enabled_ = declare_parameter<bool>("publish_enabled", true);
    const auto commandTopic = declare_parameter<std::string>(
        "command_topic", "/whole_body_force_control/virtual_wrench_command");
    commandFrame_ = declare_parameter<std::string>("command_frame", "tool0");
    commandTimeout_ = declare_parameter<double>("command_timeout", 0.15);
    if (topic.empty() || frame_.empty() || !std::isfinite(rate) || rate <= 0.0 ||
        !valid(force_) || !valid(torque_) || commandTopic.empty() || commandFrame_.empty() ||
        !std::isfinite(commandTimeout_) || commandTimeout_ <= 0.0) {
      throw std::invalid_argument("invalid virtual wrench configuration");
    }
    publisher_ = create_publisher<geometry_msgs::msg::WrenchStamped>(topic, 1);
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    commandSub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        commandTopic, rclcpp::QoS(1), [this](const geometry_msgs::msg::WrenchStamped::SharedPtr message) {
          Eigen::Vector3d force(message->wrench.force.x, message->wrench.force.y, message->wrench.force.z);
          Eigen::Vector3d torque(message->wrench.torque.x, message->wrench.torque.y, message->wrench.torque.z);
          if (!force.allFinite() || !torque.allFinite() ||
              (message->header.frame_id != frame_ && message->header.frame_id != commandFrame_)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Rejected invalid virtual wrench command/frame");
            return;
          }
          try {
            if (message->header.frame_id != frame_) {
              const auto tf = tfBuffer_->lookupTransform(frame_, message->header.frame_id, tf2::TimePointZero);
              const auto &q = tf.transform.rotation;
              const Eigen::Matrix3d R = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
              const auto &p = tf.transform.translation;
              force = (R * force).eval();
              torque = R * torque + Eigen::Vector3d(p.x, p.y, p.z).cross(force);
            }
            commandForce_ = force; commandTorque_ = torque;
            lastCommand_ = std::chrono::steady_clock::now();
            commandReceived_ = true;
          } catch (const tf2::TransformException &error) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Virtual command TF not ready: %s", error.what());
          }
        });
    callback_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter> &parameters) {
          auto force = force_, torque = torque_;
          bool enabled = enabled_;
          rcl_interfaces::msg::SetParametersResult result;
          try {
            for (const auto &parameter : parameters) {
              if (parameter.get_name() == "force") { force = parameter.as_double_array(); }
              else if (parameter.get_name() == "torque") { torque = parameter.as_double_array(); }
              else if (parameter.get_name() == "publish_enabled") { enabled = parameter.as_bool(); }
              else { throw std::invalid_argument("only force, torque, publish_enabled are mutable"); }
            }
            if (!valid(force) || !valid(torque)) { throw std::invalid_argument("expected finite 3-vector"); }
            force_ = force; torque_ = torque; enabled_ = enabled;
            for (const auto &parameter : parameters) {
              if (parameter.get_name() == "force" || parameter.get_name() == "torque") {
                commandReceived_ = false;  // Explicit parameter update returns to manual mode.
              }
            }
            result.successful = true;
          } catch (const std::exception &error) {
            result.successful = false; result.reason = error.what();
          }
          return result;
        });
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / rate)), [this]() {
          if (!enabled_) { return; }
          geometry_msgs::msg::WrenchStamped message;
          // Zero stamp requests latest TF, as in WBMM's existing sim source.
          message.header.frame_id = frame_;
          message.wrench.force.x = force_[0]; message.wrench.force.y = force_[1];
          message.wrench.force.z = force_[2]; message.wrench.torque.x = torque_[0];
          message.wrench.torque.y = torque_[1]; message.wrench.torque.z = torque_[2];
          if (commandReceived_) {
            const bool fresh = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - lastCommand_).count() <= commandTimeout_;
            // A lost keyboard never falls back to a previous nonzero parameter.
            const Eigen::Vector3d force = fresh ? commandForce_ : Eigen::Vector3d::Zero();
            const Eigen::Vector3d torque = fresh ? commandTorque_ : Eigen::Vector3d::Zero();
            message.wrench.force.x = force.x(); message.wrench.force.y = force.y();
            message.wrench.force.z = force.z(); message.wrench.torque.x = torque.x();
            message.wrench.torque.y = torque.y(); message.wrench.torque.z = torque.z();
          }
          publisher_->publish(message);
        });
    RCLCPP_INFO(get_logger(), "Virtual sensor publishing %s in %s; keyboard command=%s (%s), timeout=%.3f s",
                topic.c_str(), frame_.c_str(), commandTopic.c_str(), commandFrame_.c_str(), commandTimeout_);
  }
private:
  static bool valid(const std::vector<double> &values) {
    return values.size() == 3 && std::isfinite(values[0]) &&
        std::isfinite(values[1]) && std::isfinite(values[2]);
  }
  std::string frame_;
  std::vector<double> force_, torque_;
  bool enabled_;
  std::string commandFrame_;
  double commandTimeout_;
  bool commandReceived_{false};
  Eigen::Vector3d commandForce_{Eigen::Vector3d::Zero()}, commandTorque_{Eigen::Vector3d::Zero()};
  std::chrono::steady_clock::time_point lastCommand_{};
  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr commandSub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr publisher_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VirtualWrenchNode>());
  rclcpp::shutdown();
  return 0;
}
