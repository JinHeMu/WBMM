// =============================================================================
//  WbmmMrtNode.cpp
//
//  OCS2 MRT execution node for the WBMM differential-drive mobile manipulator.
//  It reads robot state, evaluates the latest MPC policy, checks the result,
//  and publishes base velocity plus arm position commands.
// =============================================================================

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <wbmm_ocs2/WbmmInterface.h>
#include <wbmm_ocs2_ros/msg/task_phase_state.hpp>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include "WbmmVisualization.h"

using namespace std::chrono_literals;

namespace
{

  constexpr char kDefaultRobotName[] = "mobile_manipulator";
  constexpr int kPlanExpiredMaxLog = 5;
  constexpr double kMaxCommandDt = 0.05;

} // namespace

class WbmmMrtNode : public rclcpp::Node
{
public:
  WbmmMrtNode() : Node("wbmm_mrt_node")
  {
    declareParameters();
    readParameters();
    validateParameters();
    setupRobotModel();
    setupRosInterfaces();
  }

  void initMrt()
  {
    // Ignore launch-level remaps for this private helper node.
    rclcpp::NodeOptions options;
    options.use_global_arguments(false);
    ocs2Node_ = std::make_shared<rclcpp::Node>(std::string(get_name()) + "_ocs2_internal", options);

    mrt_ = std::make_unique<ocs2::MRT_ROS_Interface>(robotName_);
    mrt_->initRollout(&interface_->getRollout());
    mrt_->launchNodes(ocs2Node_);

    if (!enableViz_)
    {
      return;
    }

    viz_ = std::make_unique<wbmm::WbmmVisualization>(shared_from_this(), *interface_, worldFrame_, vizSelfCollision_);

    vizEveryN_ = std::max(1, static_cast<int>(std::round(mrtRate_ / std::max(1.0, vizRate_))));

    RCLCPP_INFO(get_logger(), "Visualization enabled: every %d cycles (approximately %.1f Hz)", vizEveryN_,
                mrtRate_ / static_cast<double>(vizEveryN_));
  }

  void run()
  {
    if (!waitForRobotState())
    {
      return;
    }

    auto observation = makeObservation(0.0);
    {
      std::lock_guard<std::mutex> lock(stateMutex_);
      lastGoodArmQ_ = armQ_;
    }

    resetMpc(observation);
    if (!waitForFirstPolicy(observation))
    {
      return;
    }

    RCLCPP_INFO(get_logger(), "Got first MPC policy. Entering MRT loop at %.1f Hz", mrtRate_);
    runControlLoop();
  }

  void shutdownOcs2()
  {
    viz_.reset();
    mrt_.reset();
    ocs2Node_.reset();
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("taskFile", "");
    declare_parameter<std::string>("libFolder", "");
    declare_parameter<std::string>("urdfFile", "");
    declare_parameter<std::string>("esdfFile", "");
    declare_parameter<std::string>("robot_name", kDefaultRobotName);
    declare_parameter<std::string>("task_phase_state_topic", "");
    declare_parameter<int>("initial_task_phase", -1);
    declare_parameter<double>("mrt_loop_rate", 100.0);
    declare_parameter<double>("traj_horizon", 0.05);

    declare_parameter<std::string>("base_cmd_topic", "/diff_drive_controller/cmd_vel");
    declare_parameter<std::string>("arm_cmd_topic", "/arm_controller/commands");
    declare_parameter<std::string>("odom_topic", "/wheel/odometry");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<bool>("use_stamped_cmd", true);
    declare_parameter<bool>("command_output_enabled", false);
    declare_parameter<std::vector<std::string>>("arm_joint_names",
                                                {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});

    declare_parameter<std::string>("base_frame", "base_footprint");
    declare_parameter<std::string>("world_frame", "odom");
    declare_parameter<std::string>("ee_frame", "tool0");
    declare_parameter<bool>("use_whole_body_target", true);

    declare_parameter<double>("arm_max_delta_per_step", 0.50);
    declare_parameter<bool>("arm_use_velocity_integrator", false);
    declare_parameter<double>("arm_max_command_velocity", 0.50);

    declare_parameter<bool>("enable_visualization", true);
    declare_parameter<bool>("viz_self_collision", true);
    declare_parameter<double>("viz_rate", 20.0);
  }

  void readParameters()
  {
    taskFile_ = get_parameter("taskFile").as_string();
    libFolder_ = get_parameter("libFolder").as_string();
    urdfFile_ = get_parameter("urdfFile").as_string();
    esdfFile_ = get_parameter("esdfFile").as_string();
    robotName_ = get_parameter("robot_name").as_string();
    initialTaskPhase_ = get_parameter("initial_task_phase").as_int();

    taskPhaseStateTopic_ = get_parameter("task_phase_state_topic").as_string();
    if (taskPhaseStateTopic_.empty())
    {
      taskPhaseStateTopic_ = robotName_ + "_task_phase_state";
    }

    mrtRate_ = get_parameter("mrt_loop_rate").as_double();
    trajHorizon_ = get_parameter("traj_horizon").as_double();
    armJointNames_ = get_parameter("arm_joint_names").as_string_array();

    baseFrame_ = get_parameter("base_frame").as_string();
    worldFrame_ = get_parameter("world_frame").as_string();
    eeFrame_ = get_parameter("ee_frame").as_string();
    useStampedCmd_ = get_parameter("use_stamped_cmd").as_bool();
    commandOutputEnabled_ = get_parameter("command_output_enabled").as_bool();
    useWholeBodyTarget_ = get_parameter("use_whole_body_target").as_bool();

    armMaxDeltaPerStep_ = get_parameter("arm_max_delta_per_step").as_double();
    armUseVelocityIntegrator_ = get_parameter("arm_use_velocity_integrator").as_bool();
    armMaxCommandVelocity_ = get_parameter("arm_max_command_velocity").as_double();

    enableViz_ = get_parameter("enable_visualization").as_bool();
    vizSelfCollision_ = get_parameter("viz_self_collision").as_bool();
    vizRate_ = get_parameter("viz_rate").as_double();
  }

  void validateParameters() const
  {
    if (taskFile_.empty() || libFolder_.empty() || urdfFile_.empty())
    {
      throw std::runtime_error("taskFile / libFolder / urdfFile parameters must all be set.");
    }
    if (robotName_.empty())
    {
      throw std::runtime_error("robot_name must not be empty.");
    }
    if (initialTaskPhase_ < -1 || initialTaskPhase_ > 3)
    {
      throw std::runtime_error("initial_task_phase must be in [-1, 3].");
    }
    if (!std::isfinite(mrtRate_) || mrtRate_ <= 0.0)
    {
      throw std::runtime_error("mrt_loop_rate must be positive.");
    }
    if (!std::isfinite(trajHorizon_) || trajHorizon_ < 0.0)
    {
      throw std::runtime_error("traj_horizon must not be negative.");
    }
    if (!std::isfinite(armMaxDeltaPerStep_) || armMaxDeltaPerStep_ <= 0.0)
    {
      throw std::runtime_error("arm_max_delta_per_step must be positive.");
    }
    if (!std::isfinite(armMaxCommandVelocity_) || armMaxCommandVelocity_ <= 0.0)
    {
      throw std::runtime_error("arm_max_command_velocity must be positive.");
    }
    if (!std::isfinite(vizRate_) || vizRate_ <= 0.0)
    {
      throw std::runtime_error("viz_rate must be positive.");
    }
    if (baseFrame_.empty() || worldFrame_.empty() || eeFrame_.empty())
    {
      throw std::runtime_error("base_frame / world_frame / ee_frame must not be empty.");
    }
    const std::unordered_set<std::string> uniqueJointNames(armJointNames_.begin(), armJointNames_.end());
    if (uniqueJointNames.size() != armJointNames_.size() ||
        std::any_of(armJointNames_.begin(), armJointNames_.end(), [](const std::string &name)
                    { return name.empty(); }))
    {
      throw std::runtime_error("arm_joint_names must be non-empty and unique.");
    }
  }

  void setupRobotModel()
  {
    interface_ = std::make_unique<wbmm_ocs2::WbmmInterface>(
      taskFile_, libFolder_, urdfFile_, esdfFile_, worldFrame_);

    const auto &info = interface_->getWbmmModelInfo();
    stateDim_ = info.stateDim;
    inputDim_ = info.inputDim;
    armDim_ = info.armDim;
    armQ_.assign(armJointNames_.size(), 0.0);

    RCLCPP_INFO(get_logger(), "OCS2 model dims: state=%zu input=%zu arm=%zu", stateDim_, inputDim_, armDim_);
    if (stateDim_ != 3 + armDim_)
    {
      throw std::runtime_error("OCS2 state dimension must equal base plus arm.");
    }
    if (inputDim_ != 2 + armDim_)
    {
      throw std::runtime_error("OCS2 input dimension must equal base inputs plus arm.");
    }
    if (armDim_ != armJointNames_.size())
    {
      throw std::runtime_error("OCS2 arm dimension does not match arm_joint_names. Check removeJoints.");
    }

    modeSwitchEnabled_ = interface_->isModeSwitchEnabled();
    int initialPhase = modeSwitchEnabled_
                           ? static_cast<int>(interface_->getTaskPhase())
                           : 0;
    if (modeSwitchEnabled_ && initialTaskPhase_ >= 0)
    {
      initialPhase = initialTaskPhase_;
    }
    currentPhase_.store(initialPhase);
    currentPhaseReceived_.store(false);
    RCLCPP_INFO(
        get_logger(), "Task phase mode: %s, initial phase=%d",
        modeSwitchEnabled_ ? "ENABLED" : "disabled", initialPhase);
    if (!modeSwitchEnabled_ && initialTaskPhase_ >= 0)
    {
      RCLCPP_WARN(
          get_logger(),
          "initial_task_phase is ignored because modeSwitch.activate=false.");
    }
  }

  void setupRosInterfaces()
  {
    tfBuffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tfListener_ = std::make_unique<tf2_ros::TransformListener>(*tfBuffer_);
    setupPublishers();
    setupSubscribers();
    logJointOrder();
  }

  void setupPublishers()
  {
    if (!commandOutputEnabled_)
    {
      RCLCPP_WARN(get_logger(), "DRY-RUN safety gate active: command publishers are disabled. "
                                "MPC/MRT computation and visualization remain active.");
      return;
    }

    const auto baseTopic = get_parameter("base_cmd_topic").as_string();
    if (useStampedCmd_)
    {
      baseStampedPub_ = create_publisher<geometry_msgs::msg::TwistStamped>(baseTopic, 10);
      RCLCPP_INFO(get_logger(), "Publishing base commands as TwistStamped on %s", baseTopic.c_str());
    }
    else
    {
      basePub_ = create_publisher<geometry_msgs::msg::Twist>(baseTopic, 10);
      RCLCPP_INFO(get_logger(), "Publishing base commands as Twist on %s", baseTopic.c_str());
    }

    const auto armTopic = get_parameter("arm_cmd_topic").as_string();
    armPub_ = create_publisher<std_msgs::msg::Float64MultiArray>(armTopic, rclcpp::QoS(10).reliable());
    RCLCPP_INFO(get_logger(), "Publishing arm position commands on %s", armTopic.c_str());
  }

  void setupSubscribers()
  {
    odomSub_ =
        create_subscription<nav_msgs::msg::Odometry>(get_parameter("odom_topic").as_string(), rclcpp::SensorDataQoS(),
                                                     [this](const nav_msgs::msg::Odometry::SharedPtr msg)
                                                     { odomCallback(msg); });
    jointSub_ = create_subscription<sensor_msgs::msg::JointState>(
        get_parameter("joint_state_topic").as_string(), rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg)
        { jointCallback(msg); });

    if (modeSwitchEnabled_)
    {
      phaseStateSub_ =
          create_subscription<wbmm_ocs2_ros::msg::TaskPhaseState>(
              taskPhaseStateTopic_,
              rclcpp::QoS(1).transient_local().reliable(),
              [this](
                  const wbmm_ocs2_ros::msg::TaskPhaseState::SharedPtr msg)
              { taskPhaseStateCallback(msg); });
      RCLCPP_INFO(
          get_logger(), "Subscribing task phase state on %s",
          taskPhaseStateTopic_.c_str());
    }
  }

  void logJointOrder() const
  {
    std::ostringstream text;
    for (size_t i = 0; i < armJointNames_.size(); ++i)
    {
      text << (i == 0 ? "" : ", ") << armJointNames_[i];
    }
    RCLCPP_INFO(get_logger(), "Forward controller joint command order: [%s]", text.str().c_str());
  }

  bool waitForRobotState()
  {
    RCLCPP_INFO(get_logger(), "Waiting for odom and complete joint_states...");
    rclcpp::Rate rate(10.0);
    while (rclcpp::ok() && (!gotOdom_.load() || !gotJoints_.load()))
    {
      rate.sleep();
    }
    return rclcpp::ok();
  }

  ocs2::SystemObservation makeObservation(double time)
  {
    ocs2::SystemObservation observation;
    observation.state.setZero(stateDim_);
    observation.input.setZero(inputDim_);
    observation.time = time;
    observation.mode = modeSwitchEnabled_
                          ? static_cast<std::size_t>(
                                std::clamp(currentPhase_.load(), 0, 3))
                          : 0;

    std::lock_guard<std::mutex> lock(stateMutex_);
    fillStateLocked(observation.state);
    return observation;
  }

  void resetMpc(const ocs2::SystemObservation &observation)
  {
    const int phaseValue = modeSwitchEnabled_
                               ? std::clamp(currentPhase_.load(), 0, 3)
                               : 0;
    const bool useEeTarget =
        modeSwitchEnabled_
            ? phaseValue ==
                  static_cast<int>(wbmm_ocs2::TaskPhase::kExecution)
            : !useWholeBodyTarget_;

    ocs2::vector_t target;
    if (!useEeTarget)
    {
      target = observation.state;
      std::ostringstream text;
      text << target.transpose();
      RCLCPP_INFO(get_logger(), "Initial whole-body target: [%s]", text.str().c_str());
    }
    else
    {
      target = lookupCurrentEePose();
      RCLCPP_INFO(get_logger(),
                  "Initial EE target: pos=(%.3f, %.3f, %.3f), "
                  "quat=(%.3f, %.3f, %.3f, %.3f)",
                  target(0), target(1), target(2), target(3), target(4), target(5), target(6));
    }
    mrt_->resetMpcNode(ocs2::TargetTrajectories({0.0}, {target}, {ocs2::vector_t::Zero(inputDim_)}));
  }

  bool waitForFirstPolicy(const ocs2::SystemObservation &observation)
  {
    RCLCPP_INFO(get_logger(), "Waiting for first MPC policy...");
    while (rclcpp::ok() && !mrt_->initialPolicyReceived())
    {
      mrt_->setCurrentObservation(observation);
      mrt_->spinMRT();
      std::this_thread::sleep_for(50ms);
    }
    return rclcpp::ok();
  }

  void runControlLoop()
  {
    using SteadyClock = std::chrono::steady_clock;

    rclcpp::Rate rate(mrtRate_);
    const auto startTime = now();
    lastReport_ = startTime;
    int expiredCount = 0;

    while (rclcpp::ok())
    {
      const auto workBegin = SteadyClock::now();
      mrt_->spinMRT();

      auto observation = makeObservation((now() - startTime).seconds());
      mrt_->setCurrentObservation(observation);
      if (mrt_->updatePolicy())
      {
        ++policyUpdateCount_;
      }

      executePolicy(observation, expiredCount);
      updateVisualization(observation);
      updateTiming(observation, workBegin);
      rate.sleep();
    }
  }

  bool readPlanEnd(double time, double &planEnd)
  {
    planEnd = std::numeric_limits<double>::quiet_NaN();
    try
    {
      const auto &policy = mrt_->getPolicy();
      if (policy.timeTrajectory_.empty())
      {
        return false;
      }
      planEnd = policy.timeTrajectory_.back();
      return std::isfinite(planEnd) && time <= planEnd;
    }
    catch (const std::exception &error)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to inspect MPC policy: %s", error.what());
      return false;
    }
    catch (...)
    {
      return false;
    }
  }

  bool evaluateCurrentPolicy(const ocs2::SystemObservation &observation, ocs2::vector_t &state,
                             ocs2::vector_t &input, size_t &mode)
  {
    try
    {
      mrt_->evaluatePolicy(observation.time, observation.state, state, input, mode);
      return isPolicyVectorValid(state, input);
    }
    catch (const std::exception &error)
    {
      RCLCPP_ERROR(get_logger(), "[SAFETY] Current policy evaluation failed: %s", error.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(get_logger(), "[SAFETY] Current policy evaluation failed with unknown exception.");
    }
    return false;
  }

  void executePolicy(const ocs2::SystemObservation &observation, int &expiredCount)
  {
    double planEnd = std::numeric_limits<double>::quiet_NaN();
    if (!readPlanEnd(observation.time, planEnd))
    {
      logExpiredPlan(observation.time, planEnd, ++expiredCount);
      stopAndHold();
      return;
    }
    expiredCount = 0;

    ocs2::vector_t policyState;
    ocs2::vector_t policyInput;
    size_t policyMode = 0;
    if (!evaluateCurrentPolicy(observation, policyState, policyInput, policyMode))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "[SAFETY] MPC current output is invalid. Stopping base and holding arm.");
      stopAndHold();
      return;
    }

    if (modeSwitchEnabled_ &&
        policyMode != static_cast<size_t>(std::clamp(currentPhase_.load(), 0, 3)))
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[SAFETY] MPC policy mode (%zu) does not match requested task phase (%d). "
          "Stopping base and holding arm until a new policy arrives.",
          policyMode, currentPhase_.load());
      stopAndHold();
      return;
    }

    std::vector<double> armCommand;
    if (!computeSafeArmCommand(observation.time, observation.state, armCommand))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                            "[SAFETY] Predicted arm command is unsafe. Stopping base and holding arm.");
      stopAndHold();
      return;
    }

    publishBaseCommand(policyInput);
    publishArmPositions(armCommand);
    lastGoodArmQ_ = armCommand;
  }

  void logExpiredPlan(double time, double planEnd, int count)
  {
    if (count > kPlanExpiredMaxLog + 1)
    {
      return;
    }
    if (count == kPlanExpiredMaxLog + 1)
    {
      RCLCPP_ERROR(get_logger(), "[SAFETY] MPC plan remains invalid; suppressing repeated logs.");
      return;
    }
    if (std::isfinite(planEnd))
    {
      RCLCPP_ERROR(get_logger(),
                   "[SAFETY] MPC plan expired: currentTime=%.3f, planEnd=%.3f, count=%d. "
                   "Stopping base and holding arm.",
                   time, planEnd, count);
    }
    else
    {
      RCLCPP_ERROR(get_logger(),
                   "[SAFETY] MPC policy is empty or invalid, count=%d. "
                   "Stopping base and holding arm.",
                   count);
    }
  }

  void updateVisualization(const ocs2::SystemObservation &observation)
  {
    if (!viz_ || ++vizCounter_ % vizEveryN_ != 0)
    {
      return;
    }
    try
    {
      viz_->update(observation, mrt_->getPolicy(), mrt_->getCommand());
    }
    catch (const std::exception &error)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Visualization update failed: %s", error.what());
    }
  }

  void updateTiming(const ocs2::SystemObservation &observation, const std::chrono::steady_clock::time_point &workBegin)
  {
    const double workMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - workBegin).count();
    loopWorkSumMs_ += workMs;
    loopWorkMaxMs_ = std::max(loopWorkMaxMs_, workMs);
    ++loopCount_;

    try
    {
      const double ageMs = 1000.0 * (observation.time - mrt_->getCommand().mpcInitObservation_.time);
      if (std::isfinite(ageMs))
      {
        planAgeSumMs_ += ageMs;
        planAgeMaxMs_ = std::max(planAgeMaxMs_, ageMs);
      }
    }
    catch (...)
    {
      // Timing diagnostics must never interrupt control.
    }

    const double reportDt = (now() - lastReport_).seconds();
    if (reportDt < 2.0 || loopCount_ == 0)
    {
      return;
    }
    RCLCPP_INFO(get_logger(),
                "[timing] ctrl_loop=%.1f Hz (target %.1f) | work avg=%.2f ms max=%.2f ms | "
                "MPC_policy_seen=%.1f Hz | plan_age avg=%.1f ms max=%.1f ms",
                static_cast<double>(loopCount_) / reportDt, mrtRate_, loopWorkSumMs_ / static_cast<double>(loopCount_),
                loopWorkMaxMs_, static_cast<double>(policyUpdateCount_) / reportDt,
                planAgeSumMs_ / static_cast<double>(loopCount_), planAgeMaxMs_);

    lastReport_ = now();
    loopCount_ = 0;
    policyUpdateCount_ = 0;
    loopWorkSumMs_ = 0.0;
    loopWorkMaxMs_ = 0.0;
    planAgeSumMs_ = 0.0;
    planAgeMaxMs_ = 0.0;
  }

  void taskPhaseStateCallback(
      const wbmm_ocs2_ros::msg::TaskPhaseState::SharedPtr msg)
  {
    if (!msg->mode_switch_enabled)
    {
      return;
    }
    if (msg->requested_phase < 0 || msg->requested_phase > 3)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Ignoring invalid requested_phase=%d from task phase state.",
          msg->requested_phase);
      return;
    }

    const int previous = currentPhase_.exchange(msg->requested_phase);
    if (!currentPhaseReceived_.exchange(true))
    {
      RCLCPP_INFO(
          get_logger(), "Received initial task phase: %d (active=%d)",
          msg->requested_phase, msg->active_phase);
    }
    else if (previous != msg->requested_phase)
    {
      RCLCPP_INFO(
          get_logger(), "Task phase changed: %d -> %d (MPC active=%d)",
          previous, msg->requested_phase, msg->active_phase);
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    tf2::Quaternion quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
                               msg->pose.pose.orientation.w);
    if (!std::isfinite(msg->pose.pose.position.x) || !std::isfinite(msg->pose.pose.position.y) ||
        !std::isfinite(quaternion.x()) || !std::isfinite(quaternion.y()) || !std::isfinite(quaternion.z()) ||
        !std::isfinite(quaternion.w()) || quaternion.length2() < 1.0e-12)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Ignoring invalid odometry pose.");
      return;
    }
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    if (!std::isfinite(yaw))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Ignoring invalid odometry pose.");
      return;
    }

    std::lock_guard<std::mutex> lock(stateMutex_);
    baseX_ = msg->pose.pose.position.x;
    baseY_ = msg->pose.pose.position.y;
    baseYaw_ = yaw;
    gotOdom_.store(true);
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::vector<double> nextArmQ(armJointNames_.size(), 0.0);
    bool complete = true;
    for (size_t i = 0; i < armJointNames_.size(); ++i)
    {
      const auto match = std::find(msg->name.begin(), msg->name.end(), armJointNames_[i]);
      if (match == msg->name.end())
      {
        complete = false;
        continue;
      }
      const auto index = static_cast<size_t>(std::distance(msg->name.begin(), match));
      if (index >= msg->position.size() || !std::isfinite(msg->position[index]))
      {
        complete = false;
        continue;
      }
      nextArmQ[i] = msg->position[index];
    }
    if (!complete)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(stateMutex_);
    armQ_ = std::move(nextArmQ);
    gotJoints_.store(true);
  }

  void fillStateLocked(ocs2::vector_t &state) const
  {
    state.resize(stateDim_);
    state.setZero();
    state(0) = baseX_;
    state(1) = baseY_;
    state(2) = baseYaw_;
    for (size_t i = 0; i < armDim_ && i < armQ_.size(); ++i)
    {
      state(static_cast<Eigen::Index>(3 + i)) = armQ_[i];
    }
  }

  ocs2::vector_t lookupCurrentEePose()
  {
    rclcpp::Rate retryRate(10.0);
    int retries = 30;
    while (rclcpp::ok() && retries-- > 0 &&
           !tfBuffer_->canTransform(worldFrame_, eeFrame_, tf2::TimePointZero, tf2::durationFromSec(0.1)))
    {
      retryRate.sleep();
    }

    try
    {
      const auto transform = tfBuffer_->lookupTransform(worldFrame_, eeFrame_, tf2::TimePointZero, tf2::durationFromSec(1.0));
      ocs2::vector_t pose(7);
      pose << transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z,
          transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z,
          transform.transform.rotation.w;
      return pose;
    }
    catch (const tf2::TransformException &error)
    {
      throw std::runtime_error("TF lookup " + worldFrame_ + " -> " + eeFrame_ + " failed: " + error.what());
    }
  }

  bool isPolicyVectorValid(const ocs2::vector_t &state, const ocs2::vector_t &input) const
  {
    return state.size() == static_cast<Eigen::Index>(stateDim_) && input.size() == static_cast<Eigen::Index>(inputDim_) &&
           state.allFinite() && input.allFinite();
  }

  bool computeSafeArmCommand(double time, const ocs2::vector_t &currentState, std::vector<double> &command)
  {
    command.clear();
    try
    {
      ocs2::vector_t predictedState;
      ocs2::vector_t predictedInput;
      if (!evaluateFuturePolicy(time, currentState, predictedState, predictedInput))
      {
        return false;
      }

      std::vector<double> measured;
      {
        std::lock_guard<std::mutex> lock(stateMutex_);
        measured = armQ_;
      }
      if (measured.size() != armDim_)
      {
        return false;
      }

      if (armUseVelocityIntegrator_)
      {
        return integrateArmCommand(time, measured, predictedInput, command);
      }
      return makePositionArmCommand(measured, predictedState, command);
    }
    catch (const std::exception &error)
    {
      RCLCPP_ERROR(get_logger(), "[SAFETY] Future policy evaluation failed: %s", error.what());
    }
    catch (...)
    {
      RCLCPP_ERROR(get_logger(), "[SAFETY] Future policy evaluation failed with unknown exception.");
    }
    command.clear();
    return false;
  }

  bool evaluateFuturePolicy(double time, const ocs2::vector_t &currentState, ocs2::vector_t &predictedState,
                            ocs2::vector_t &predictedInput)
  {
    const auto &policy = mrt_->getPolicy();
    if (policy.timeTrajectory_.empty())
    {
      return false;
    }
    const double planEnd = policy.timeTrajectory_.back();
    if (!std::isfinite(planEnd) || time > planEnd)
    {
      return false;
    }

    constexpr double kEndSafety = 1e-3;
    const double latestTime = std::max(time, planEnd - kEndSafety);
    const double queryTime = std::min(time + trajHorizon_, latestTime);
    size_t mode = 0;
    mrt_->evaluatePolicy(queryTime, currentState, predictedState, predictedInput, mode);
    if (predictedState.size() < static_cast<Eigen::Index>(3 + armDim_))
    {
      RCLCPP_ERROR(get_logger(), "[SAFETY] Predicted state dimension is %ld, expected >= %zu",
                   static_cast<long>(predictedState.size()), 3 + armDim_);
      return false;
    }
    return true;
  }

  bool integrateArmCommand(double time, const std::vector<double> &measured, const ocs2::vector_t &predictedInput,
                           std::vector<double> &command)
  {
    if (predictedInput.size() < static_cast<Eigen::Index>(2 + armDim_))
    {
      RCLCPP_ERROR(get_logger(), "[SAFETY] Predicted input dimension is %ld, expected >= %zu",
                   static_cast<long>(predictedInput.size()), 2 + armDim_);
      return false;
    }

    if (integratedArmCommand_.size() != armDim_)
    {
      integratedArmCommand_ = measured;
      lastArmCommandTime_ = time;
    }
    const double dt = std::clamp(time - lastArmCommandTime_, 0.0, kMaxCommandDt);
    lastArmCommandTime_ = time;

    command.resize(armDim_);
    for (size_t i = 0; i < armDim_; ++i)
    {
      const double velocity =
          std::clamp(predictedInput(static_cast<Eigen::Index>(2 + i)), -armMaxCommandVelocity_, armMaxCommandVelocity_);
      const double next = std::clamp(integratedArmCommand_[i] + dt * velocity, measured[i] - armMaxDeltaPerStep_,
                                     measured[i] + armMaxDeltaPerStep_);
      if (!std::isfinite(next))
      {
        command.clear();
        return false;
      }
      command[i] = next;
    }
    integratedArmCommand_ = command;
    return true;
  }

  bool makePositionArmCommand(const std::vector<double> &measured, const ocs2::vector_t &predictedState,
                              std::vector<double> &command)
  {
    command.resize(armDim_);
    for (size_t i = 0; i < armDim_; ++i)
    {
      const double target = predictedState(static_cast<Eigen::Index>(3 + i));
      if (!std::isfinite(target) || !std::isfinite(measured[i]))
      {
        RCLCPP_ERROR(get_logger(), "[SAFETY] Arm joint %zu contains NaN/Inf: command=%.6f measured=%.6f", i + 1, target,
                     measured[i]);
        command.clear();
        return false;
      }

      const double delta = target - measured[i];
      if (std::abs(delta) > armMaxDeltaPerStep_)
      {
        // Rate-limit instead of rejecting the whole command. Rejecting made
        // the MRT safety gate hold the arm and could leave the MPC plan
        // expired whenever the new EE target required a large joint step.
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[SAFETY] Arm joint %zu target is rate-limited: "
            "target=%.3f measured=%.3f delta=%.3f limit=%.3f",
            i + 1, target, measured[i], delta, armMaxDeltaPerStep_);
      }
      command[i] = std::clamp(
          target, measured[i] - armMaxDeltaPerStep_,
          measured[i] + armMaxDeltaPerStep_);
    }
    return true;
  }

  void publishBaseCommand(const ocs2::vector_t &input)
  {
    if (input.size() < 2 || !std::isfinite(input(0)) || !std::isfinite(input(1)))
    {
      publishZeroBaseCommand();
      return;
    }
    publishBase(input(0), input(1));
  }

  void publishZeroBaseCommand() { publishBase(0.0, 0.0); }

  void publishBase(double speed, double yawRate)
  {
    if (!commandOutputEnabled_)
    {
      return;
    }
    if (useStampedCmd_)
    {
      geometry_msgs::msg::TwistStamped msg;
      msg.header.stamp = now();
      msg.header.frame_id = baseFrame_;
      msg.twist.linear.x = speed;
      msg.twist.angular.z = yawRate;
      baseStampedPub_->publish(msg);
    }
    else
    {
      geometry_msgs::msg::Twist msg;
      msg.linear.x = speed;
      msg.angular.z = yawRate;
      basePub_->publish(msg);
    }
  }

  void publishArmPositions(const std::vector<double> &positions)
  {
    if (!commandOutputEnabled_)
    {
      return;
    }
    if (positions.size() != armJointNames_.size())
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Arm command size mismatch: command=%zu, joints=%zu",
                            positions.size(), armJointNames_.size());
      return;
    }
    if (!std::all_of(positions.begin(), positions.end(), [](double value)
                     { return std::isfinite(value); }))
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Arm command contains NaN/Inf");
      return;
    }

    std_msgs::msg::Float64MultiArray msg;
    msg.data = positions;
    armPub_->publish(msg);
  }

  void publishHoldArmCommand()
  {
    std::vector<double> hold;
    if (lastGoodArmQ_.size() == armJointNames_.size())
    {
      hold = lastGoodArmQ_;
    }
    else
    {
      std::lock_guard<std::mutex> lock(stateMutex_);
      hold = armQ_;
    }
    if (hold.size() != armJointNames_.size())
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, "Cannot publish arm hold command: invalid command size.");
      return;
    }
    publishArmPositions(hold);
  }

  void stopAndHold()
  {
    publishZeroBaseCommand();
    publishHoldArmCommand();
  }

  std::string taskFile_;
  std::string libFolder_;
  std::string urdfFile_;
  std::string esdfFile_;
  std::string robotName_{kDefaultRobotName};
  std::string taskPhaseStateTopic_;
  std::string baseFrame_;
  std::string worldFrame_;
  std::string eeFrame_;

  int initialTaskPhase_{-1};
  bool modeSwitchEnabled_{false};
  std::atomic<int> currentPhase_{0};
  std::atomic<bool> currentPhaseReceived_{false};

  double mrtRate_{100.0};
  double trajHorizon_{0.05};
  bool useStampedCmd_{true};
  bool commandOutputEnabled_{false};
  bool useWholeBodyTarget_{true};

  double armMaxDeltaPerStep_{0.50};
  bool armUseVelocityIntegrator_{false};
  double armMaxCommandVelocity_{0.50};
  double lastArmCommandTime_{0.0};
  std::vector<std::string> armJointNames_;

  bool enableViz_{true};
  bool vizSelfCollision_{true};
  double vizRate_{20.0};
  int vizEveryN_{5};
  long vizCounter_{0};

  rclcpp::Time lastReport_;
  size_t loopCount_{0};
  size_t policyUpdateCount_{0};
  double loopWorkSumMs_{0.0};
  double loopWorkMaxMs_{0.0};
  double planAgeSumMs_{0.0};
  double planAgeMaxMs_{0.0};

  std::unique_ptr<wbmm_ocs2::WbmmInterface> interface_;
  rclcpp::Node::SharedPtr ocs2Node_;
  std::unique_ptr<ocs2::MRT_ROS_Interface> mrt_;
  std::unique_ptr<wbmm::WbmmVisualization> viz_;

  size_t stateDim_{0};
  size_t inputDim_{0};
  size_t armDim_{0};

  std::unique_ptr<tf2_ros::Buffer> tfBuffer_;
  std::unique_ptr<tf2_ros::TransformListener> tfListener_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr baseStampedPub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr basePub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr armPub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSub_;
  rclcpp::Subscription<wbmm_ocs2_ros::msg::TaskPhaseState>::SharedPtr
      phaseStateSub_;

  std::mutex stateMutex_;
  std::atomic<bool> gotOdom_{false};
  std::atomic<bool> gotJoints_{false};
  double baseX_{0.0};
  double baseY_{0.0};
  double baseYaw_{0.0};
  std::vector<double> armQ_;
  std::vector<double> lastGoodArmQ_;
  std::vector<double> integratedArmCommand_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<WbmmMrtNode> node;
  try
  {
    node = std::make_shared<WbmmMrtNode>();
    node->initMrt();
  }
  catch (const std::exception &error)
  {
    if (node)
    {
      RCLCPP_FATAL(node->get_logger(), "MRT initialization failed: %s", error.what());
    }
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]()
                      {
    try {
      executor.spin();
    } catch (...) {
      // The control thread owns shutdown and reports its own failures.
    } });

  try
  {
    node->run();
  }
  catch (const std::exception &error)
  {
    RCLCPP_ERROR(node->get_logger(), "MRT loop exception: %s", error.what());
  }
  catch (...)
  {
    RCLCPP_ERROR(node->get_logger(), "MRT loop stopped by an unknown exception.");
  }

  executor.cancel();
  if (spinner.joinable())
  {
    spinner.join();
  }
  node->shutdownOcs2();
  rclcpp::shutdown();
  return 0;
}
