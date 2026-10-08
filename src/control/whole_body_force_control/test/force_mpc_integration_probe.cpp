// Manual closed-loop regression against force_mpc.launch.py, simulation only.
// This probe changes parameters on the C++ virtual sensor; it never connects
// to a robot SDK or supplies hardware joint/base commands.
#include "wbmm_pinocchio/pinocchio_robot_model.hpp"
#include "wbmm_robot_model/wbmm_robot_model.hpp"
#include "wbmm_ros_interfaces/wbmm_conversions.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Geometry>
#include <chrono>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

class Probe {
public:
  Probe() : node_(std::make_shared<rclcpp::Node>("force_mpc_integration_probe")) {
    report_ = node_->declare_parameter<std::string>(
        "report", "/tmp/wbmm_force_mpc_report.json");
    const auto description = wbmm::robot_model::loadRobotDescription(
        ament_index_cpp::get_package_share_directory("tracer_jaka_description") +
        "/urdf/tracer_jaka_zu5.urdf");
    auto config = wbmm::robot_model::RobotModelConfig::defaultsFor(description);
    config.state_base_frame = description.root_link;
    model_ = std::make_shared<wbmm::pinocchio::PinocchioRobotModel>(
        wbmm::pinocchio::KinematicModel::create(
            std::make_shared<const wbmm::robot_model::RobotDescription>(description), config));
    const auto statusQos = rclcpp::QoS(1).reliable().transient_local();
    observation_ = node_->create_subscription<ocs2_msgs::msg::MpcObservation>(
        "/mobile_manipulator_mpc_observation", rclcpp::SensorDataQoS(),
        [this](const ocs2_msgs::msg::MpcObservation::SharedPtr msg) {
          if (msg->state.value.size() == 9) {
            for (int i = 0; i < 9; ++i) { state_[i] = msg->state.value[i]; }
            gotState_ = true;
          }
        });
    forceStateSub_ = node_->create_subscription<std_msgs::msg::String>(
        "/whole_body_force_control/states", statusQos,
        [this](const std_msgs::msg::String::SharedPtr msg) { forceState_ = msg->data; });
    gateStateSub_ = node_->create_subscription<std_msgs::msg::String>(
        "/mobile_manipulator_force_execution_state", statusQos,
        [this](const std_msgs::msg::String::SharedPtr msg) { gateState_ = msg->data; });
    collisionSub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "/mujoco/unexpected_collision", 10,
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          collisionSeen_ = true; collision_ = collision_ || msg->data;
        });
    baseCommandSub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "/base_controller/cmd_vel", 10,
        [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
          ++baseCommandCount_;
          lastBaseSpeed_ = std::hypot(msg->linear.x, msg->angular.z);
          if (checkStop_) { maximumStoppedBaseSpeed_ = std::max(maximumStoppedBaseSpeed_, lastBaseSpeed_); }
        });
    armCommandSub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/arm_controller/commands", 10,
        [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
          ++armCommandCount_;
          if (msg->data.size() != 6) { return; }
          Eigen::Matrix<double, 6, 1> command;
          for (int i = 0; i < 6; ++i) { command[i] = msg->data[i]; }
          if (checkStop_) {
            if (!gotHold_) { hold_ = command; gotHold_ = true; }
            maximumHoldVariation_ = std::max(maximumHoldVariation_, (command - hold_).norm());
          }
        });
    parameters_ = std::make_shared<rclcpp::AsyncParametersClient>(node_, "/virtual_force_publisher");
  }

  ~Probe() {
    // A failed assertion must not leave a nonzero manual force injection.
    try {
      if (rclcpp::ok() && parameters_ && parameters_->wait_for_service(0s)) {
        setForce(Eigen::Vector3d::Zero());
        setParameter(rclcpp::Parameter("publish_enabled", true));
      }
    } catch (...) {
      std::cerr << "Could not restore zero virtual force; stop the simulation." << std::endl;
    }
  }

  void run() {
    require(parameters_->wait_for_service(10s), "C++ virtual sensor is unavailable; launch fake_wrench:=true");
    waitUntil([this] { return gotState_ && forceState_ == "ACTIVE" && gateState_ == "ACTIVE"; }, 120.0);
    step(2.0);
    const auto initialState = state_;
    const Eigen::Vector3d initial = tcpPosition();
    const auto sensorPose = pose("jk_se_vi_200_link");
    const Eigen::Matrix3d worldFromSensor = Eigen::Quaterniond(
        sensorPose.orientation.w, sensorPose.orientation.x,
        sensorPose.orientation.y, sensorPose.orientation.z).toRotationMatrix();
    const Eigen::Vector3d pushSensor = worldFromSensor.transpose() * Eigen::Vector3d(5, 0, 0);
    setForce(pushSensor);
    step(10.0);
    const Eigen::Vector3d pushed = tcpPosition();
    const double baseTravel = (state_.head<2>() - initialState.head<2>()).norm();
    const double armTravel = (state_.tail<6>() - initialState.tail<6>()).norm();
    std::cout << "Push: TCP dx=" << pushed.x() - initial.x()
              << " base=" << baseTravel << " arm=" << armTravel << std::endl;
    require(forceState_ == "ACTIVE" && gateState_ == "ACTIVE", "push did not stay ACTIVE");
    require(pushed.x() - initial.x() > 0.06, "measured TCP did not follow positive force");
    require(baseTravel > 0.001 && armTravel > 0.01, "base/arm participation was not observed");
    setForce(Eigen::Vector3d::Zero());
    step(4.0);
    const Eigen::Vector3d released = tcpPosition();
    step(3.0);
    const double releaseDrift = (tcpPosition() - released).norm();
    std::cout << "Release drift=" << releaseDrift << std::endl;
    require(releaseDrift < 0.01, "zero-stiffness release did not settle");
    require(tcpPosition().x() - initial.x() > 0.06, "release returned to the initial pose");
    setForce(-pushSensor);
    step(10.0);
    const Eigen::Vector3d pulled = tcpPosition();
    std::cout << "Reverse TCP dx=" << pulled.x() - released.x() << std::endl;
    require(released.x() - pulled.x() > 0.06, "measured TCP did not reverse with negative force");
    setForce(Eigen::Vector3d::Zero());
    step(2.0);
    setParameter(rclcpp::Parameter("publish_enabled", false));
    waitUntil([this] { return forceState_.rfind("FAULT_", 0) == 0 && gateState_ == "FAULT_INTERLOCK"; }, 3.0);
    const std::string disconnectFault = forceState_;
    step(0.3, false);
    checkStop_ = true;
    const auto beforeBase = baseCommandCount_, beforeArm = armCommandCount_;
    step(1.0, false);
    setParameter(rclcpp::Parameter("publish_enabled", true));
    step(1.0, false);
    require(baseCommandCount_ > beforeBase && armCommandCount_ > beforeArm, "stop commands were not observed");
    require(maximumStoppedBaseSpeed_ < 1e-12 && gotHold_ && maximumHoldVariation_ < 1e-12,
            "fault did not stop base and freeze the arm command");
    require(forceState_.rfind("FAULT_", 0) == 0 && gateState_ == "FAULT_INTERLOCK",
            "restoring sensor data silently resumed motion");
    checkStop_ = false;
    trigger("/whole_body_force_control/force_sensor/reset");
    trigger("/whole_body_force_control/reset");
    waitUntil([this] { return forceState_ == "ACTIVE"; }, 5.0);
    require(gateState_ == "FAULT_INTERLOCK", "MRT resumed without explicit interlock reset");
    const Eigen::Vector3d beforeRecovery = tcpPosition();
    trigger("/mobile_manipulator/force_control/reset_interlock");
    waitUntil([this] { return gateState_ == "ACTIVE"; }, 5.0);
    step(2.0);
    const double recoveryDisplacement = (tcpPosition() - beforeRecovery).norm();
    require(recoveryDisplacement < 0.02, "recovery replayed an old force offset");
    require(collisionSeen_ && !collision_, "collision monitor absent or unexpected collision detected");
    std::ofstream report(report_);
    require(report.good(), "cannot open report: " + report_);
    report << "{\n  \"passed\": true,\n  \"backend\": \"MuJoCo\",\n"
           << "  \"positive_tcp_dx_m\": " << pushed.x() - initial.x()
           << ",\n  \"base_travel_m\": " << baseTravel
           << ",\n  \"arm_joint_change_norm_rad\": " << armTravel
           << ",\n  \"release_drift_m\": " << releaseDrift
           << ",\n  \"negative_tcp_dx_m\": " << pulled.x() - released.x()
           << ",\n  \"disconnect_fault\": \"" << disconnectFault
           << "\",\n  \"stopped_base_command_max\": " << maximumStoppedBaseSpeed_
           << ",\n  \"hold_command_variation_rad\": " << maximumHoldVariation_
           << ",\n  \"recovery_displacement_m\": " << recoveryDisplacement
           << ",\n  \"unexpected_collision\": false\n}\n";
    require(report.good(), "failed to write report: " + report_);
    std::cout << "PASS: " << report_ << std::endl;
  }

private:
  static void require(bool ok, const std::string &message) {
    if (!ok) { throw std::runtime_error(message); }
  }
  void step(double seconds, bool checkFault = true) {
    const auto end = Clock::now() + std::chrono::duration<double>(seconds);
    while (rclcpp::ok() && Clock::now() < end) {
      rclcpp::spin_some(node_);
      if (checkFault) {
        require(forceState_.rfind("FAULT_", 0) != 0, "force controller: " + forceState_);
        require(gateState_ != "FAULT_INTERLOCK", "MRT execution interlock faulted");
      }
      require(!collision_, "unexpected MuJoCo collision");
      std::this_thread::sleep_for(5ms);
    }
  }
  void waitUntil(const std::function<bool()> &condition, double timeout) {
    const auto end = Clock::now() + std::chrono::duration<double>(timeout);
    while (rclcpp::ok() && Clock::now() < end && !condition()) { step(0.02, false); }
    require(condition(), "timeout; force=" + forceState_ + " MRT=" + gateState_);
  }
  void setParameter(const rclcpp::Parameter &parameter) {
    auto future = parameters_->set_parameters({parameter});
    require(rclcpp::spin_until_future_complete(node_, future, 3s) ==
                rclcpp::FutureReturnCode::SUCCESS, "virtual sensor parameter timeout");
    require(future.get().at(0).successful, "virtual sensor parameter rejected");
  }
  void setForce(const Eigen::Vector3d &force) {
    setParameter(rclcpp::Parameter("force", std::vector<double>{force.x(), force.y(), force.z()}));
  }
  void trigger(const std::string &service) {
    auto client = node_->create_client<std_srvs::srv::Trigger>(service);
    require(client->wait_for_service(3s), "service unavailable: " + service);
    auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    require(rclcpp::spin_until_future_complete(node_, future, 3s) ==
                rclcpp::FutureReturnCode::SUCCESS, "service timeout: " + service);
    require(future.get()->success, "service rejected: " + service);
  }
  wbmm::core::Pose pose(const std::string &frame) {
    wbmm::core::Header header; header.frame_id = "odom";
    const auto state = wbmm::ros_interfaces::toCoreState(state_, model_->jointNames(), header);
    wbmm::core::Pose result;
    require(state.has_value() && model_->forwardKinematics(*state, frame, result), "FK failed");
    return result;
  }
  Eigen::Vector3d tcpPosition() {
    const auto result = pose("tool0");
    return {result.position.x, result.position.y, result.position.z};
  }
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<wbmm::pinocchio::PinocchioRobotModel> model_;
  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_;
  Eigen::Matrix<double, 9, 1> state_{Eigen::Matrix<double, 9, 1>::Zero()};
  Eigen::Matrix<double, 6, 1> hold_;
  bool gotState_{false}, collisionSeen_{false}, collision_{false}, checkStop_{false}, gotHold_{false};
  double lastBaseSpeed_{0}, maximumStoppedBaseSpeed_{0}, maximumHoldVariation_{0};
  size_t baseCommandCount_{0}, armCommandCount_{0};
  std::string forceState_, gateState_, report_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observation_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr forceStateSub_, gateStateSub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collisionSub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr baseCommandSub_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr armCommandSub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try { Probe().run(); }
  catch (const std::exception &error) {
    std::cerr << "FAIL: " << error.what() << std::endl;
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
