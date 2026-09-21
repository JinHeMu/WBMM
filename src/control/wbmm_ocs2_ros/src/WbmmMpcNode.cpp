// =============================================================================
//  WbmmMpcNode.cpp
//
//  OCS2 MPC 求解器节点 (ROS 2)。
//
//  双参考模式接口:
//    - mobile_manipulator_whole_body_target : 9D 全身目标
//    - mobile_manipulator_ee_target         : 7D 末端位姿目标
//    - mobile_manipulator_task_phase_state  : 当前 phase 状态 (latched)
//    - mobile_manipulator_set_task_phase    : 切换 phase service
//
//  modeSwitch.activate=false 时仍使用同样的双参考订阅，但 WbmmInterface
//  内部走旧的单 cost/约束路径；modeSwitch.activate=true 时两路参考由
//  PhaseWeightedStateCost 按 phase 加权。
// =============================================================================

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <wbmm_ocs2/WbmmInterface.h>
#include <wbmm_ocs2_ros/msg/task_phase_state.hpp>
#include <wbmm_ocs2_ros/srv/set_task_phase.hpp>

namespace
{

constexpr char kDefaultRobotName[] = "mobile_manipulator";
constexpr std::size_t kEndEffectorStateDim = 7;

std::string resolveTopic(
    const rclcpp::Node::SharedPtr& node,
    const std::string& parameterName,
    const std::string& fallback)
{
  const std::string value = node->get_parameter(parameterName).as_string();
  return value.empty() ? fallback : value;
}

bool isValidTarget(
    const ocs2::TargetTrajectories& target, std::size_t expectedDim)
{
  const auto& timeTrajectory = target.timeTrajectory;
  const auto& stateTrajectory = target.stateTrajectory;

  if (stateTrajectory.empty() ||
      timeTrajectory.size() != stateTrajectory.size() ||
      !std::all_of(
          timeTrajectory.begin(), timeTrajectory.end(),
          [](ocs2::scalar_t time) { return std::isfinite(time); }))
  {
    return false;
  }

  if (timeTrajectory.size() > 1U &&
      std::adjacent_find(
          timeTrajectory.begin(), timeTrajectory.end(),
          [](ocs2::scalar_t lhs, ocs2::scalar_t rhs)
          { return !(lhs < rhs); }) != timeTrajectory.end())
  {
    return false;
  }

  return std::all_of(
      stateTrajectory.begin(), stateTrajectory.end(),
      [expectedDim](const ocs2::vector_t& state)
      {
        return static_cast<std::size_t>(state.size()) == expectedDim &&
               state.allFinite();
      });
}

}  // namespace

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto nodeHandle = std::make_shared<rclcpp::Node>("wbmm_mpc_node");

  // -- ROS parameters --------------------------------------------------------
  nodeHandle->declare_parameter<std::string>("taskFile", "");
  nodeHandle->declare_parameter<std::string>("libFolder", "");
  nodeHandle->declare_parameter<std::string>("urdfFile", "");
  nodeHandle->declare_parameter<std::string>("robot_name", kDefaultRobotName);
  nodeHandle->declare_parameter<std::string>("whole_body_target_topic", "");
  nodeHandle->declare_parameter<std::string>("ee_target_topic", "");
  nodeHandle->declare_parameter<std::string>("task_phase_state_topic", "");
  nodeHandle->declare_parameter<std::string>("set_task_phase_service", "");
  nodeHandle->declare_parameter<int>("initial_task_phase", -1);

  const auto taskFile = nodeHandle->get_parameter("taskFile").as_string();
  const auto libFolder = nodeHandle->get_parameter("libFolder").as_string();
  const auto urdfFile = nodeHandle->get_parameter("urdfFile").as_string();
  const auto robotName =
      nodeHandle->get_parameter("robot_name").as_string();

  if (taskFile.empty() || libFolder.empty() || urdfFile.empty())
  {
    RCLCPP_FATAL(
        nodeHandle->get_logger(),
        "taskFile / libFolder / urdfFile parameters must all be set.");
    rclcpp::shutdown();
    return 1;
  }
  if (robotName.empty())
  {
    RCLCPP_FATAL(nodeHandle->get_logger(), "robot_name must not be empty.");
    rclcpp::shutdown();
    return 1;
  }

  const std::string wholeBodyTargetTopic = resolveTopic(
      nodeHandle, "whole_body_target_topic",
      robotName + "_whole_body_target");
  const std::string eeTargetTopic = resolveTopic(
      nodeHandle, "ee_target_topic", robotName + "_ee_target");
  const std::string taskPhaseStateTopic = resolveTopic(
      nodeHandle, "task_phase_state_topic",
      robotName + "_task_phase_state");
  const std::string setTaskPhaseService = resolveTopic(
      nodeHandle, "set_task_phase_service",
      robotName + "_set_task_phase");

  RCLCPP_INFO(nodeHandle->get_logger(), "Task file : %s", taskFile.c_str());
  RCLCPP_INFO(nodeHandle->get_logger(), "Lib folder: %s", libFolder.c_str());
  RCLCPP_INFO(nodeHandle->get_logger(), "URDF file : %s", urdfFile.c_str());

  // -- OCS2 problem interface ------------------------------------------------
  wbmm_ocs2::WbmmInterface interface(taskFile, libFolder, urdfFile);
  const bool modeSwitchEnabled = interface.isModeSwitchEnabled();
  const std::size_t wholeBodyStateDim =
      interface.getWbmmModelInfo().stateDim;

  // Optional deterministic start phase for force-control profiles. The task
  // file remains the source of truth when this parameter is -1.
  const int initialTaskPhase =
      nodeHandle->get_parameter("initial_task_phase").as_int();
  if (initialTaskPhase < -1 || initialTaskPhase > 3)
  {
    RCLCPP_FATAL(
        nodeHandle->get_logger(),
        "initial_task_phase must be in [-1, 3], got %d.", initialTaskPhase);
    rclcpp::shutdown();
    return 1;
  }
  if (initialTaskPhase >= 0)
  {
    if (modeSwitchEnabled)
    {
      interface.setTaskPhase(
          static_cast<wbmm_ocs2::TaskPhase>(initialTaskPhase));
    }
    else
    {
      RCLCPP_WARN(
          nodeHandle->get_logger(),
          "initial_task_phase=%d is ignored because modeSwitch.activate=false.",
          initialTaskPhase);
    }
  }

  // -- dual reference subscribers -------------------------------------------
  const auto targetQos = rclcpp::QoS(1).reliable();
  const rclcpp::Logger logger = nodeHandle->get_logger();
  rclcpp::Clock::SharedPtr clock = nodeHandle->get_clock();

  [[maybe_unused]] auto wholeBodyTargetSub =
      nodeHandle->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
          wholeBodyTargetTopic, targetQos,
          [&interface, wholeBodyStateDim, logger, clock](
              const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg)
          {
            try
            {
              auto target =
                  ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
              if (!isValidTarget(target, wholeBodyStateDim))
              {
                RCLCPP_WARN_THROTTLE(
                    logger, *clock, 2000,
                    "Dropping whole-body target with invalid state dimension.");
                return;
              }
              interface.setWholeBodyTarget(target);
            }
            catch (const std::exception& error)
            {
              RCLCPP_WARN_THROTTLE(
                  logger, *clock, 2000,
                  "Failed to parse whole-body target: %s", error.what());
            }
          });

  [[maybe_unused]] auto eeTargetSub =
      nodeHandle->create_subscription<ocs2_msgs::msg::MpcTargetTrajectories>(
          eeTargetTopic, targetQos,
          [&interface, logger, clock](
              const ocs2_msgs::msg::MpcTargetTrajectories::SharedPtr msg)
          {
            try
            {
              auto target =
                  ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
              if (!isValidTarget(target, kEndEffectorStateDim))
              {
                RCLCPP_WARN_THROTTLE(
                    logger, *clock, 2000,
                    "Dropping end-effector target with invalid dimension "
                    "(expected 7D).");
                return;
              }
              interface.setEndEffectorTarget(target);
            }
            catch (const std::exception& error)
            {
              RCLCPP_WARN_THROTTLE(
                  logger, *clock, 2000,
                  "Failed to parse end-effector target: %s", error.what());
            }
          });

  // -- MPC solver ------------------------------------------------------------
  ocs2::GaussNewtonDDP_MPC mpc(
      interface.mpcSettings(), interface.ddpSettings(),
      interface.getRollout(), interface.getOptimalControlProblem(),
      interface.getInitializer());

  // The OCP costs read the two reference buffers directly through
  // WbmmReferenceManager. The solver must see the same manager for reset
  // target routing and mode schedule publication.
  mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());

  ocs2::MPC_ROS_Interface mpcNode(mpc, robotName);

  // -- phase service / state publisher --------------------------------------
  std::shared_ptr<rclcpp::Publisher<wbmm_ocs2_ros::msg::TaskPhaseState>>
      phaseStatePublisher;
  [[maybe_unused]] rclcpp::TimerBase::SharedPtr phaseStateTimer;
  [[maybe_unused]]
  rclcpp::Service<wbmm_ocs2_ros::srv::SetTaskPhase>::SharedPtr phaseService;

  if (modeSwitchEnabled)
  {
    phaseStatePublisher =
        nodeHandle->create_publisher<wbmm_ocs2_ros::msg::TaskPhaseState>(
            taskPhaseStateTopic,
            rclcpp::QoS(1).transient_local().reliable());

    rclcpp::Node* nodeRaw = nodeHandle.get();
    auto publishPhaseState = [nodeRaw, phaseStatePublisher, &interface,
                              modeSwitchEnabled]()
    {
      wbmm_ocs2_ros::msg::TaskPhaseState stateMsg;
      stateMsg.header.stamp = nodeRaw->now();
      stateMsg.requested_phase = static_cast<int32_t>(
          interface.getWbmmReferenceManagerPtr()->getRequestedTaskPhase());
      stateMsg.active_phase = static_cast<int32_t>(interface.getTaskPhase());
      stateMsg.mode_switch_enabled = modeSwitchEnabled;
      phaseStatePublisher->publish(stateMsg);
    };
    publishPhaseState();

    phaseStateTimer = nodeHandle->create_wall_timer(
        std::chrono::milliseconds(200), publishPhaseState);

    phaseService =
        nodeHandle->create_service<wbmm_ocs2_ros::srv::SetTaskPhase>(
            setTaskPhaseService,
            [&interface, publishPhaseState](
                const std::shared_ptr<
                    wbmm_ocs2_ros::srv::SetTaskPhase::Request> request,
                std::shared_ptr<
                    wbmm_ocs2_ros::srv::SetTaskPhase::Response> response)
            {
              response->success = false;
              response->requested_phase = static_cast<int32_t>(
                  interface.getWbmmReferenceManagerPtr()
                      ->getRequestedTaskPhase());
              response->active_phase =
                  static_cast<int32_t>(interface.getTaskPhase());

              if (request->phase < 0 || request->phase > 3)
              {
                response->message =
                    "phase must be in [0, 3]: 0=Navigation, 1=Transition, "
                    "2=Execution, 3=Retract";
                return;
              }

              try
              {
                interface.setTaskPhase(
                    static_cast<wbmm_ocs2::TaskPhase>(request->phase));
                response->success = true;
                response->message =
                    "Task phase accepted; active phase updates at the next "
                    "MPC solve.";
              }
              catch (const std::exception& error)
              {
                response->message =
                    std::string("setTaskPhase failed: ") + error.what();
              }

              response->requested_phase = static_cast<int32_t>(
                  interface.getWbmmReferenceManagerPtr()
                      ->getRequestedTaskPhase());
              response->active_phase =
                  static_cast<int32_t>(interface.getTaskPhase());
              publishPhaseState();
            });

    RCLCPP_INFO(
        nodeHandle->get_logger(),
        "Task phase service : %s", setTaskPhaseService.c_str());
    RCLCPP_INFO(
        nodeHandle->get_logger(),
        "Task phase state   : %s (latched)", taskPhaseStateTopic.c_str());
  }
  else
  {
    RCLCPP_WARN(
        nodeHandle->get_logger(),
        "modeSwitch.activate=false: phase service and state publisher are "
        "disabled.");
  }

  RCLCPP_INFO(
      nodeHandle->get_logger(), "Whole-body target: %s",
      wholeBodyTargetTopic.c_str());
  RCLCPP_INFO(
      nodeHandle->get_logger(), "EE target       : %s",
      eeTargetTopic.c_str());

  // -- spin ------------------------------------------------------------------
  mpcNode.launchNodes(nodeHandle);

  rclcpp::shutdown();
  return 0;
}
