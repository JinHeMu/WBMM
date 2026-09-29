#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <sstream>
#include <wbmm_robot_model/wbmm_robot_model.hpp>
#include <wbmm_visualization/trajectory_display.hpp>

namespace wbmm::visualization {
class VisualizationNode : public rclcpp::Node {
  using PlannerMsg = wbmm_planner_ros::msg::WholeBodyTrajectory;
  using PolicyMsg = ocs2_msgs::msg::MpcFlattenedController;
  using MarkerArray = visualization_msgs::msg::MarkerArray;
  using Clock = std::chrono::steady_clock;

public:
  VisualizationNode() : Node("wbmm_visualization") {
    const auto urdf_file = declare_parameter<std::string>("urdf_file", "");
    const auto xml = declare_parameter<std::string>("robot_description", "");
    auto mesh_directory = declare_parameter<std::string>("mesh_directory", "");
    if (xml.empty() && urdf_file.empty())
      throw std::invalid_argument(
          "Provide readable urdf_file or robot_description XML.");
    if (mesh_directory.empty() && !urdf_file.empty())
      mesh_directory =
          std::filesystem::absolute(urdf_file).parent_path().string();
    // 统一读取：显示与规划/控制使用同一个 URDF 解析入口。
    const auto description = xml.empty()
      ? wbmm::robot_model::loadRobotDescription(urdf_file)
      : wbmm::robot_model::parseRobotDescription(xml, "robot_description");
    const auto names = declare_parameter<std::vector<std::string>>(
        "joint_names",
        {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    const auto base = declare_parameter<std::string>("base_frame", "");
    const auto default_names = declare_parameter<std::vector<std::string>>(
        "default_joint_names", std::vector<std::string>{});
    const auto default_values = declare_parameter<std::vector<double>>(
        "default_joint_positions", std::vector<double>{});
    if (default_names.size() != default_values.size())
      throw std::invalid_argument(
          "Auxiliary joint defaults have mismatched sizes.");
    std::map<std::string, double> defaults;
    for (std::size_t i = 0; i < default_names.size(); ++i)
      if (!defaults.emplace(default_names[i], default_values[i]).second)
        throw std::invalid_argument("Duplicate auxiliary default joint.");
    auto config = wbmm::robot_model::RobotModelConfig::defaultsFor(description);
    config.state_base_frame = base.empty() ? description.root_link : base;
    config.controlled_joints = names;
    config.locked_joints = std::move(defaults);
    model_ = std::make_unique<RobotVisualModel>(
        std::make_shared<const wbmm::robot_model::RobotDescription>(description),
        std::move(config), mesh_directory);
    mpc_frame_ = declare_parameter<std::string>("mpc_frame", "odom");
    if (mpc_frame_.empty())
      throw std::invalid_argument(
          "MPC policy has no header; configure mpc_frame explicitly.");
    planner_.name = "planner";
    mpc_.name = "mpc";
    planner_.options = options("planner", {0.10, 0.80, 0.55, 0.17}, 0.5, 12);
    mpc_.options = options("mpc", {1.0, 0.45, 0.12, 0.25}, 0.2, 8);
    planner_.timeout = declare_parameter<double>("planner.timeout", 0.0);
    mpc_.timeout = declare_parameter<double>("mpc.timeout", 1.0);
    for (double v : {planner_.timeout, mpc_.timeout})
      if (!std::isfinite(v) || v < 0)
        throw std::invalid_argument("Display timeouts must be >= 0.");
    const auto point_limit =
        declare_parameter<int>("max_trajectory_points", 100000);
    if (point_limit < 1 || point_limit > 1000000)
      throw std::invalid_argument(
          "max_trajectory_points must be in [1,1000000].");
    max_points_ = static_cast<std::size_t>(point_limit);
    const auto rate = declare_parameter<double>("publish_rate", 5.0);
    if (!std::isfinite(rate) || rate <= 0 || rate > 60)
      throw std::invalid_argument("publish_rate must be in (0,60].");
    const auto planner_output = declare_parameter<std::string>(
        "planner_marker_topic", "/wbmm/visualization/planner");
    const auto mpc_output = declare_parameter<std::string>(
        "mpc_marker_topic", "/wbmm/visualization/mpc");
    if (planner_output == mpc_output)
      throw std::invalid_argument(
          "Planner and MPC require separate snapshot topics.");
    const auto marker_qos = rclcpp::QoS(1).reliable().transient_local();
    planner_.publisher =
        create_publisher<MarkerArray>(planner_output, marker_qos);
    mpc_.publisher = create_publisher<MarkerArray>(mpc_output, marker_qos);
    // Keep only the newest message. Conversion/FK runs at the display rate,
    // independent of the MPC solve rate; this node never subscribes controller
    // gains for execution.
    planner_sub_ = create_subscription<PlannerMsg>(
        declare_parameter<std::string>("planner_topic",
                                       "/wbmm/whole_body_trajectory"),
        rclcpp::QoS(1).reliable(), [this](PlannerMsg::ConstSharedPtr msg) {
          planner_msg_ = msg;
          planner_.dirty = true;
          planner_.received = Clock::now();
        });
    mpc_sub_ = create_subscription<PolicyMsg>(
        declare_parameter<std::string>("mpc_policy_topic",
                                       "/mobile_manipulator_mpc_policy"),
        rclcpp::QoS(1).best_effort(), [this](PolicyMsg::ConstSharedPtr msg) {
          mpc_msg_ = msg;
          mpc_.dirty = true;
          mpc_.received = Clock::now();
        });
    timer_ =
        create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
                              std::chrono::duration<double>(1 / rate)),
                          [this]() { tick(); });
    RCLCPP_INFO(get_logger(),
                "Display only: %zu URDF visuals, base=%s; planner frame from "
                "message, MPC frame=%s; outputs %s and %s",
                model_->visualCount(), model_->baseFrame().c_str(),
                mpc_frame_.c_str(), planner_output.c_str(), mpc_output.c_str());
  }

private:
  struct Layer {
    std::string name;
    DisplayOptions options;
    MarkerLayer markers;
    rclcpp::Publisher<MarkerArray>::SharedPtr publisher;
    Clock::time_point received;
    double timeout{0};
    bool dirty{false}, visible{false};
  };
  DisplayOptions options(const std::string &name, std::vector<double> rgba,
                         double dt, int count) {
    DisplayOptions o;
    o.sample_interval =
        declare_parameter<double>(name + ".sample_interval", dt);
    int poses = declare_parameter<int>(name + ".max_robot_poses", count);
    int markers = declare_parameter<int>(name + ".max_markers", 5000);
    if (poses < 2 || poses > 500 || markers < 1 || markers > 100000 ||
        !std::isfinite(o.sample_interval) || o.sample_interval <= 0)
      throw std::invalid_argument(
          "Invalid ghost sample interval/count/marker budget.");
    o.max_robot_poses = poses;
    o.max_markers = markers;
    rgba = declare_parameter<std::vector<double>>(name + ".rgba", rgba);
    if (rgba.size() != 4)
      throw std::invalid_argument("RGBA needs four entries.");
    for (double v : rgba)
      if (!std::isfinite(v) || v < 0 || v > 1)
        throw std::invalid_argument("RGBA must be in [0,1].");
    o.style.red = rgba[0];
    o.style.green = rgba[1];
    o.style.blue = rgba[2];
    o.style.alpha = rgba[3];
    o.style.use_urdf_materials =
        declare_parameter<bool>(name + ".use_urdf_materials", false);
    o.style.use_embedded_materials =
        declare_parameter<bool>(name + ".use_embedded_materials", false);
    o.show_base_path = declare_parameter<bool>(name + ".show_base_path", false);
    o.show_ee_path = declare_parameter<bool>(name + ".show_ee_path", false);
    o.ee_frame = declare_parameter<std::string>(name + ".ee_frame", "tool0");
    o.line_width = declare_parameter<double>(name + ".line_width", 0.015);
    if (!std::isfinite(o.line_width) || o.line_width <= 0 ||
        (o.show_ee_path && !model_->hasLink(o.ee_frame)))
      throw std::invalid_argument("Invalid line width or EE path link.");
    return o;
  }
  void clear(Layer &layer) {
    layer.publisher->publish(layer.markers.replace(MarkerArray{}));
    layer.visible = false;
    layer.dirty = false;
  }
  template <class Convert> void update(Layer &layer, Convert convert) {
    if (layer.dirty) {
      layer.dirty = false;
      try {
        auto trajectory = convert();
        auto markers =
            renderTrajectory(*model_, trajectory, layer.name, layer.options);
        layer.visible = !markers.markers.empty();
        layer.publisher->publish(layer.markers.replace(std::move(markers)));
      } catch (const std::exception &e) {
        clear(layer);
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Cleared %s display: %s", layer.name.c_str(),
                             e.what());
      }
    }
    if (layer.visible && layer.timeout > 0 &&
        std::chrono::duration<double>(Clock::now() - layer.received).count() >
            layer.timeout)
      clear(layer);
  }
  void tick() {
    update(planner_, [&]() { return fromPlanner(*planner_msg_, max_points_); });
    update(mpc_, [&]() {
      return fromMpc(*mpc_msg_, mpc_frame_, model_->jointNames(), max_points_);
    });
  }
  std::unique_ptr<RobotVisualModel> model_;
  std::string mpc_frame_;
  std::size_t max_points_;
  Layer planner_, mpc_;
  PlannerMsg::ConstSharedPtr planner_msg_;
  PolicyMsg::ConstSharedPtr mpc_msg_;
  rclcpp::Subscription<PlannerMsg>::SharedPtr planner_sub_;
  rclcpp::Subscription<PolicyMsg>::SharedPtr mpc_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace wbmm::visualization
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<wbmm::visualization::VisualizationNode>());
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("wbmm_visualization"), "Display failed: %s",
                 e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
