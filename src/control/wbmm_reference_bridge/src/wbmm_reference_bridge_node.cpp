// =============================================================================
//  wbmm_reference_bridge_node.cpp
//
//  Turns a planned nominal whole-body trajectory into the rolling OCS2
//  reference window the MPC consumes.
//
//      wbmm_planning_msgs/WholeBodyTrajectory
//              -> ocs2_msgs/MpcTargetTrajectories
//
//  Replaces remani_to_ocs2_reference_bridge.cpp. That bridge had to decode
//  MINCO polynomial coefficients and rebuild yaw from the path tangent, which
//  made the reference yaw rate grow as 1/t near the trajectory start (about
//  4.4 rad/s against a 1.0 rad/s controller limit) and left the reference
//  permanently faster than the base. Here the planner publishes explicit
//  samples, so this node only interpolates and re-times.
//
//  Rest-to-rest playback starts at the planner's publication stamp. The
//  observation clock maps this into OCS2 time. Each window is transformed from
//  the planning frame to the explicit controller world frame using current TF.
// =============================================================================

#include "wbmm_reference_bridge/trajectory_sampler.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_msgs/msg/mpc_target_trajectories.hpp>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <wbmm_planning_msgs/msg/whole_body_trajectory.hpp>

namespace
{

constexpr char kDefaultRobotName[] = "mobile_manipulator";

// A stamp further in the future than this is treated as a clock-domain
// mismatch rather than ordinary jitter, and the arrival time is used instead.
constexpr double kFutureStampTolerance = 0.5;

std::string resolveTopic(
  const rclcpp::Node & node, const std::string & parameter_name,
  const std::string & fallback)
{
  const std::string value = node.get_parameter(parameter_name).as_string();
  return value.empty() ? fallback : value;
}

}  // namespace

class WbmmReferenceBridge : public rclcpp::Node
{
public:
  WbmmReferenceBridge() : Node("wbmm_reference_bridge")
  {
    declareParameters();
    readParameters();
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    setupInterfaces();
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("robot_name", kDefaultRobotName);
    declare_parameter<std::string>("trajectory_topic", "/wbmm/whole_body_trajectory");
    declare_parameter<std::string>("observation_topic", "");
    declare_parameter<std::string>("target_topic", "");
    declare_parameter<std::string>("reference_owner_service", "");

    // Must equal the OCS2 MRT world_frame. A trajectory stamped in another
    // frame is rejected rather than silently reinterpreted.
    declare_parameter<std::string>("world_frame", "odom");
    // Guard against a planner that silently changes the joint set.
    declare_parameter<int>("expected_joint_count", 6);
    declare_parameter<std::vector<std::string>>("joint_names",
      {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    declare_parameter<double>("observation_timeout", 0.5);

    declare_parameter<double>("publish_rate", 20.0);
    declare_parameter<double>("reference_horizon", 3.0);
    declare_parameter<double>("sample_dt", 0.04);
  }

  void readParameters()
  {
    robotName_ = get_parameter("robot_name").as_string();
    worldFrame_ = get_parameter("world_frame").as_string();
    expectedJointCount_ = get_parameter("expected_joint_count").as_int();
    jointNames_ = get_parameter("joint_names").as_string_array();
    observationTimeout_ = get_parameter("observation_timeout").as_double();
    if (worldFrame_.empty() || jointNames_.size() != static_cast<std::size_t>(expectedJointCount_) ||
        !std::isfinite(observationTimeout_) || observationTimeout_ <= 0.0) {
      throw std::runtime_error("Explicit controller frame, joint order and positive observation timeout required.");
    }

    trajectoryTopic_ = get_parameter("trajectory_topic").as_string();
    observationTopic_ =
      resolveTopic(*this, "observation_topic", robotName_ + "_mpc_observation");
    targetTopic_ =
      resolveTopic(*this, "target_topic", robotName_ + "_whole_body_target");
    referenceOwnerService_ = resolveTopic(
      *this, "reference_owner_service",
      "/wbmm_reference_bridge/set_reference_enabled");

    publishRate_ = get_parameter("publish_rate").as_double();
    referenceHorizon_ = get_parameter("reference_horizon").as_double();
    sampleDt_ = get_parameter("sample_dt").as_double();

    if (robotName_.empty())
    {
      throw std::runtime_error("robot_name must not be empty.");
    }
    if (expectedJointCount_ <= 0)
    {
      throw std::runtime_error("expected_joint_count must be positive.");
    }
    if (!std::isfinite(publishRate_) || publishRate_ <= 0.0)
    {
      throw std::runtime_error("publish_rate must be positive.");
    }
    if (!std::isfinite(sampleDt_) || sampleDt_ <= 0.0)
    {
      throw std::runtime_error("sample_dt must be positive.");
    }
    sampleDt_ = std::max(sampleDt_, 0.005);
    if (!std::isfinite(referenceHorizon_) || referenceHorizon_ <= 0.0 ||
        referenceHorizon_ / sampleDt_ > 100000.0) {
      throw std::runtime_error("Invalid reference horizon or sampling budget.");
    }
    referenceHorizon_ = std::max(referenceHorizon_, sampleDt_);
  }

  void setupInterfaces()
  {
    const auto targetQos = rclcpp::QoS(1).reliable();

    targetPublisher_ =
      create_publisher<ocs2_msgs::msg::MpcTargetTrajectories>(targetTopic_, targetQos);

    trajectorySub_ =
      create_subscription<wbmm_planning_msgs::msg::WholeBodyTrajectory>(
        trajectoryTopic_, targetQos,
        [this](const wbmm_planning_msgs::msg::WholeBodyTrajectory::SharedPtr msg)
        {trajectoryCallback(msg);});

    // Best effort: the MRT publishes this at its control rate and a dropped
    // sample only costs a slightly staler time mapping.
    observationSub_ = create_subscription<ocs2_msgs::msg::MpcObservation>(
      observationTopic_, rclcpp::QoS(1).best_effort(),
      [this](const ocs2_msgs::msg::MpcObservation::SharedPtr msg)
      {observationCallback(msg);});

    referenceOwnerService_handle_ = create_service<std_srvs::srv::SetBool>(
      referenceOwnerService_,
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
      {
        setReferenceEnabled(request->data);
        response->success = true;
        response->message = request->data
                              ? "wbmm_reference_bridge acquired MPC reference ownership"
                              : "wbmm_reference_bridge released MPC reference ownership";
        RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
      });

    cancelSub_ = create_subscription<std_msgs::msg::Header>(
        "/wbmm/planning/cancel", targetQos, [this](std_msgs::msg::Header::SharedPtr msg) {
          const std::lock_guard<std::mutex> lock(mutex_);
          const rclcpp::Time stamp(msg->stamp, get_clock()->get_clock_type());
          if (stamp <= lastCancelStamp_)
            return;
          lastCancelStamp_ = stamp;
          // Different DDS topics may be delivered out of order. A newer plan
          // supersedes a delayed cancellation sent before that plan was built.
          if (haveTrajectory_ && trajectoryStartStamp_ > stamp)
            return;
          haveTrajectory_ = false;
          holdPending_ = true;
          if (haveObservation_ && (now() - observationRosStamp_).seconds() >= 0 &&
              (now() - observationRosStamp_).seconds() <= observationTimeout_)
            installMeasuredHold();
        });
    const auto period = std::chrono::duration<double>(1.0 / publishRate_);
    publishTimer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {publishReference();});

    RCLCPP_INFO(
      get_logger(), "Reference bridge: %s -> %s (frame %s, %d joints).",
      trajectoryTopic_.c_str(), targetTopic_.c_str(), worldFrame_.c_str(),
      expectedJointCount_);
    RCLCPP_INFO(
      get_logger(), "MPC clock mapping learned from %s.",
      observationTopic_.c_str());
  }

  // Hold is built in the controller frame from measured MPC state. It is a
  // zero-feedforward reference, not a claim of instantaneous physical braking.
  void installMeasuredHold() {
    if (!referenceEnabled_ || observationState_.size() != jointNames_.size() + 3U)
      return;
    wbmm_planning_msgs::msg::WholeBodyTrajectory hold;
    hold.header.frame_id = worldFrame_;
    hold.trajectory_id = "cancel_hold";
    hold.joint_names = jointNames_;
    hold.time_from_start = {0.0, sampleDt_};
    for (int k = 0; k < 2; ++k) {
      hold.base_x.push_back(observationState_[0]);
      hold.base_y.push_back(observationState_[1]);
      hold.base_yaw.push_back(observationState_[2]);
      hold.base_linear_velocity.push_back(0);
      hold.base_yaw_rate.push_back(0);
      hold.phase.push_back(0);
      for (std::size_t j = 0; j < jointNames_.size(); ++j) {
        hold.joint_positions.push_back(observationState_[3 + j]);
        hold.joint_velocities.push_back(0);
      }
    }
    sampler_ = wbmm::reference_bridge::TrajectorySampler(hold);
    trajectoryStartStamp_ = now();
    haveTrajectory_ = true;
    holdPending_ = false;
  }

  void setReferenceEnabled(bool enabled)
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if (!enabled) {
      haveTrajectory_ = false;
      holdPending_ = false;
    }
    referenceEnabled_ = enabled;
  }

  void trajectoryCallback(
    const wbmm_planning_msgs::msg::WholeBodyTrajectory::SharedPtr msg)
  {
    std::string reason;
    if (!wbmm::reference_bridge::TrajectorySampler::validate(*msg, &reason))
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected whole-body trajectory: %s", reason.c_str());
      return;
    }

    if (static_cast<int>(msg->joint_names.size()) != expectedJointCount_)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Rejected whole-body trajectory: %zu joints, expected %d.",
        msg->joint_names.size(), expectedJointCount_);
      return;
    }

    if (msg->joint_names != jointNames_) {
      RCLCPP_ERROR(get_logger(), "Rejected trajectory: joint order differs from the controller.");
      return;
    }

    // header.stamp is the instant the first sample describes. Fall back to the
    // arrival time only when the stamp is missing or implausibly in the future.
    rclcpp::Time startStamp(msg->header.stamp, get_clock()->get_clock_type());
    const rclcpp::Time arrival = now();
    if (startStamp.nanoseconds() == 0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Trajectory has a zero header stamp; using the arrival time as t = 0.");
      startStamp = arrival;
    }
    else if ((startStamp - arrival).seconds() > kFutureStampTolerance)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Trajectory header stamp is %.3f s in the future; using the arrival "
        "time as t = 0. Check that the planner and this node share a clock.",
        (startStamp - arrival).seconds());
      startStamp = arrival;
    }

    const std::lock_guard<std::mutex> lock(mutex_);
    if (!referenceEnabled_ || startStamp <= lastCancelStamp_) {
      return;
    }
    holdPending_ = false;
    sampler_ = wbmm::reference_bridge::TrajectorySampler(*msg);
    trajectoryStartStamp_ = startStamp;
    haveTrajectory_ = true;
    ++generation_;

    RCLCPP_INFO(
      get_logger(), "Trajectory '%s' (generation %lu): %zu samples, %.3f s.",
      sampler_.trajectoryId().c_str(), static_cast<unsigned long>(generation_),
      sampler_.sampleCount(), sampler_.duration());
  }

  void observationCallback(const ocs2_msgs::msg::MpcObservation::SharedPtr msg)
  {
    if (!std::isfinite(msg->time) || msg->time < 0.0 ||
        msg->state.value.size() != jointNames_.size() + 3U ||
        !std::all_of(msg->state.value.begin(), msg->state.value.end(),
          [](double v) { return std::isfinite(v); }))
    {
      return;
    }
    const std::lock_guard<std::mutex> lock(mutex_);
    if (haveObservation_ && msg->time < observationTime_) {
      haveTrajectory_ = false;  // An MPC reset requires a new plan.
      holdPending_ = false;
      lastCancelStamp_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    }
    observationState_.assign(msg->state.value.begin(), msg->state.value.end());
    observationTime_ = msg->time;
    observationRosStamp_ = now();
    haveObservation_ = true;
    if (holdPending_)
      installMeasuredHold();
  }

  void publishReference()
  {
    ocs2::scalar_array_t timeTrajectory;
    ocs2::vector_array_t stateTrajectory;
    ocs2::vector_array_t inputTrajectory;

    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (!referenceEnabled_ || !haveTrajectory_ || !haveObservation_)
      {
        return;
      }

      const double observationAge = (now() - observationRosStamp_).seconds();
      if (observationAge < 0.0 || observationAge > observationTimeout_) {
        haveTrajectory_ = false;
        haveObservation_ = false;
        return;
      }
      double tx = 0.0, ty = 0.0, rotation = 0.0;
      if (sampler_.frameId() != worldFrame_) {
        try {
          const auto tf = tfBuffer_->lookupTransform(worldFrame_, sampler_.frameId(), tf2::TimePointZero);
          const auto & r = tf.transform.rotation;
          double roll, pitch;
          tf2::Matrix3x3(tf2::Quaternion(r.x, r.y, r.z, r.w)).getRPY(roll, pitch, rotation);
          tx = tf.transform.translation.x; ty = tf.transform.translation.y;
          if (!std::isfinite(rotation) || !std::isfinite(tx) || !std::isfinite(ty) ||
              std::abs(roll) > 1e-3 || std::abs(pitch) > 1e-3) { return; }
        } catch (const tf2::TransformException & error) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Reference transform unavailable: %s", error.what());
          return;
        }
      }
      const std::size_t joints = sampler_.jointCount();
      const std::size_t stateDim = 3U + joints;
      const std::size_t inputDim = 2U + joints;

      // Elapsed time inside the trajectory, measured on the planner's clock.
      double elapsed = (now() - trajectoryStartStamp_).seconds();
      if (!std::isfinite(elapsed))
      {
        return;
      }
      elapsed = std::max(elapsed, 0.0);

      // Rolling window: OCS2 time obsTimeNow + i * sampleDt maps to ROS time
      // now + i * sampleDt, and to trajectory time elapsed + i * sampleDt.
      const double obsTimeNow =
        observationTime_ + (now() - observationRosStamp_).seconds();
      const auto sampleCount =
        static_cast<std::size_t>(std::ceil(referenceHorizon_ / sampleDt_)) + 1U;

      timeTrajectory.reserve(sampleCount);
      stateTrajectory.reserve(sampleCount);
      inputTrajectory.reserve(sampleCount);

      for (std::size_t i = 0U; i < sampleCount; ++i)
      {
        const double offset = static_cast<double>(i) * sampleDt_;
        const auto sample = sampler_.sample(elapsed + offset);

        ocs2::vector_t state(stateDim);
        state << std::cos(rotation) * sample.base_x - std::sin(rotation) * sample.base_y + tx,
          std::sin(rotation) * sample.base_x + std::cos(rotation) * sample.base_y + ty,
          sample.base_yaw + rotation,
          Eigen::Map<const Eigen::VectorXd>(
            sample.joint_positions.data(), static_cast<Eigen::Index>(joints));

        ocs2::vector_t input(inputDim);
        input << sample.base_linear_velocity, sample.base_yaw_rate,
          Eigen::Map<const Eigen::VectorXd>(
            sample.joint_velocities.data(), static_cast<Eigen::Index>(joints));

        timeTrajectory.push_back(obsTimeNow + offset);
        stateTrajectory.push_back(std::move(state));
        inputTrajectory.push_back(std::move(input));
      }
    }

    ocs2::TargetTrajectories target(
      std::move(timeTrajectory), std::move(stateTrajectory),
      std::move(inputTrajectory));
    targetPublisher_->publish(
      ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(target));
  }

  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr cancelSub_;
  rclcpp::Time lastCancelStamp_{0, 0, RCL_ROS_TIME};
  std::vector<double> observationState_;
  bool holdPending_{false};
  std::string robotName_;
  std::string worldFrame_;
  std::string trajectoryTopic_;
  std::string observationTopic_;
  std::string targetTopic_;
  std::string referenceOwnerService_;
  int expectedJointCount_{6};
  double publishRate_{20.0};
  double referenceHorizon_{3.0};
  double sampleDt_{0.04};

  rclcpp::Publisher<ocs2_msgs::msg::MpcTargetTrajectories>::SharedPtr targetPublisher_;
  rclcpp::Subscription<wbmm_planning_msgs::msg::WholeBodyTrajectory>::SharedPtr trajectorySub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr observationSub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr referenceOwnerService_handle_;
  rclcpp::TimerBase::SharedPtr publishTimer_;

  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::vector<std::string> jointNames_;
  double observationTimeout_{0.5};
  std::mutex mutex_;
  wbmm::reference_bridge::TrajectorySampler sampler_;
  rclcpp::Time trajectoryStartStamp_{0, 0, RCL_ROS_TIME};
  double observationTime_{0.0};
  rclcpp::Time observationRosStamp_{0, 0, RCL_ROS_TIME};
  bool haveTrajectory_{false};
  bool haveObservation_{false};
  bool referenceEnabled_{true};
  std::uint64_t generation_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<WbmmReferenceBridge>());
  }
  catch (const std::exception & error)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("wbmm_reference_bridge"),
      "Reference bridge failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
