// REMANI PolynomialTraj -> OCS2 whole-body TargetTrajectories bridge.
//
// REMANI plans the flat output
//   p = [x, y, q1, ..., q6]
// and publishes one PolynomialTraj message for every constant-gear (singul)
// section.  OCS2 tracks
//   x = [x, y, yaw, q1, ..., q6]
// with velocity inputs
//   u = [v, omega, qdot1, ..., qdot6].
//
// This node assembles the REMANI sections, evaluates their polynomials and
// publishes a short rolling reference window in the OCS2 observation clock.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_msgs/msg/mpc_observation.hpp>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>

#include <quadrotor_msgs/msg/polynomial_matrix.hpp>
#include <quadrotor_msgs/msg/polynomial_traj.hpp>

namespace
{
double wrapToPi(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double unwrapNear(double angle, double reference)
{
  return reference + wrapToPi(angle - reference);
}

struct PolynomialSample
{
  std::vector<double> position;
  std::vector<double> velocity;
  std::vector<double> acceleration;
  int singul = 1;
};

struct TrajectorySection
{
  uint32_t id = 0;
  int singul = 1;
  std::vector<quadrotor_msgs::msg::PolynomialMatrix> pieces;

  double duration() const
  {
    double total = 0.0;
    for (const auto &piece : pieces)
    {
      total += std::max(0.0, piece.duration);
    }
    return total;
  }
};

struct AssembledTrajectory
{
  rclcpp::Time startStamp{0, 0, RCL_ROS_TIME};
  std::vector<TrajectorySection> sections;
  double totalDuration = 0.0;
  uint64_t generation = 0;

  bool empty() const
  {
    return sections.empty() || totalDuration <= 0.0;
  }
};

bool evaluatePiece(
    const quadrotor_msgs::msg::PolynomialMatrix &piece,
    double time,
    PolynomialSample &sample)
{
  const size_t dim = static_cast<size_t>(piece.num_dim);
  const size_t degree = static_cast<size_t>(piece.num_order);
  const size_t expected = dim * (degree + 1U);
  if (dim == 0U || piece.data.size() != expected)
  {
    return false;
  }

  const double t = std::clamp(time, 0.0, std::max(0.0, piece.duration));
  sample.position.assign(dim, 0.0);
  sample.velocity.assign(dim, 0.0);
  sample.acceleration.assign(dim, 0.0);

  // REMANI copies an Eigen column-major coefficient matrix into data.
  // Column 0 is the highest power and column degree is the constant term.
  for (size_t column = 0; column <= degree; ++column)
  {
    const size_t power = degree - column;
    const double tPower = std::pow(t, static_cast<int>(power));
    for (size_t d = 0; d < dim; ++d)
    {
      const double c = piece.data[column * dim + d];
      sample.position[d] += c * tPower;
      if (power >= 1U)
      {
        sample.velocity[d] +=
            static_cast<double>(power) * c *
            std::pow(t, static_cast<int>(power - 1U));
      }
      if (power >= 2U)
      {
        sample.acceleration[d] +=
            static_cast<double>(power * (power - 1U)) * c *
            std::pow(t, static_cast<int>(power - 2U));
      }
    }
  }
  return true;
}

bool sampleTrajectory(
    const AssembledTrajectory &trajectory,
    double relativeTime,
    PolynomialSample &sample)
{
  if (trajectory.empty())
  {
    return false;
  }

  double t = std::clamp(relativeTime, 0.0, trajectory.totalDuration);
  for (size_t sectionIndex = 0;
       sectionIndex < trajectory.sections.size();
       ++sectionIndex)
  {
    const auto &section = trajectory.sections[sectionIndex];
    const double sectionDuration = section.duration();
    const bool lastSection = sectionIndex + 1U == trajectory.sections.size();
    if (t <= sectionDuration || lastSection)
    {
      double pieceTime = std::clamp(t, 0.0, sectionDuration);
      for (size_t pieceIndex = 0; pieceIndex < section.pieces.size(); ++pieceIndex)
      {
        const auto &piece = section.pieces[pieceIndex];
        const bool lastPiece = pieceIndex + 1U == section.pieces.size();
        if (pieceTime <= piece.duration || lastPiece)
        {
          if (!evaluatePiece(piece, pieceTime, sample))
          {
            return false;
          }
          sample.singul = section.singul >= 0 ? 1 : -1;
          return true;
        }
        pieceTime -= piece.duration;
      }
      return false;
    }
    t -= sectionDuration;
  }
  return false;
}
}  // namespace

class RemaniToOcs2ReferenceBridge final : public rclcpp::Node
{
public:
  RemaniToOcs2ReferenceBridge()
      : Node("remani_to_ocs2_reference_bridge")
  {
    robotName_ = declare_parameter<std::string>(
        "robot_name", "mobile_manipulator");
    trajectoryTopic_ = declare_parameter<std::string>(
        "trajectory_topic", "/planning/trajectory");
    stateDim_ = declare_parameter<int>("state_dim", 9);
    inputDim_ = declare_parameter<int>("input_dim", 8);
    armDim_ = declare_parameter<int>("arm_dim", 6);
    sampleDt_ = declare_parameter<double>("sample_dt", 0.04);
    referenceHorizon_ = declare_parameter<double>("reference_horizon", 3.0);
    startLead_ = declare_parameter<double>("start_lead", 0.05);
    publishRate_ = declare_parameter<double>("publish_rate", 20.0);
    assemblyTimeout_ = declare_parameter<double>(
        "assembly_timeout", 0.04);
    zeroVelocityThreshold_ = declare_parameter<double>(
        "zero_velocity_threshold", 1.0e-4);
    holdAtEnd_ = declare_parameter<double>("hold_at_end", 2.0);
    transformX_ = declare_parameter<double>("planner_to_ocs2_x", 0.0);
    transformY_ = declare_parameter<double>("planner_to_ocs2_y", 0.0);
    transformYaw_ = declare_parameter<double>("planner_to_ocs2_yaw", 0.0);

    if (stateDim_ != armDim_ + 3 || inputDim_ != armDim_ + 2)
    {
      throw std::runtime_error(
          "Expected state_dim=arm_dim+3 and input_dim=arm_dim+2.");
    }
    sampleDt_ = std::max(sampleDt_, 0.005);
    referenceHorizon_ = std::max(referenceHorizon_, sampleDt_);
    publishRate_ = std::max(publishRate_, 1.0);
    assemblyTimeout_ = std::max(assemblyTimeout_, 0.005);

    const auto trajectoryQos =
        rclcpp::QoS(rclcpp::KeepLast(50)).reliable();
    trajectorySub_ =
        create_subscription<quadrotor_msgs::msg::PolynomialTraj>(
            trajectoryTopic_, trajectoryQos,
            std::bind(
                &RemaniToOcs2ReferenceBridge::trajectoryCallback,
                this, std::placeholders::_1));

    const std::string observationTopic =
        robotName_ + "_mpc_observation";
    observationSub_ =
        create_subscription<ocs2_msgs::msg::MpcObservation>(
            observationTopic, rclcpp::QoS(1).best_effort(),
            std::bind(
                &RemaniToOcs2ReferenceBridge::observationCallback,
                this, std::placeholders::_1));

    assemblyTimer_ = create_wall_timer(
        std::chrono::duration<double>(assemblyTimeout_),
        std::bind(
            &RemaniToOcs2ReferenceBridge::finishAssembly, this));
    assemblyTimer_->cancel();

    publishTimer_ = create_wall_timer(
        std::chrono::duration<double>(1.0 / publishRate_),
        std::bind(
            &RemaniToOcs2ReferenceBridge::publishReference, this));

    RCLCPP_INFO(
        get_logger(),
        "Bridge ready: %s -> %s_mpc_target, sample_dt=%.3f s, "
        "horizon=%.2f s.",
        trajectoryTopic_.c_str(), robotName_.c_str(), sampleDt_,
        referenceHorizon_);
  }

  void init()
  {
    targetPublisher_ =
        std::make_unique<ocs2::TargetTrajectoriesRosPublisher>(
            shared_from_this(), robotName_);
  }

private:
  void observationCallback(
      const ocs2_msgs::msg::MpcObservation::ConstSharedPtr msg)
  {
    if (static_cast<int>(msg->state.value.size()) != stateDim_)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Ignoring MPC observation with state dimension %zu (expected %d).",
          msg->state.value.size(), stateDim_);
      return;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    observationTime_ = msg->time;
    observationState_.resize(stateDim_);
    for (int i = 0; i < stateDim_; ++i)
    {
      observationState_(i) =
          static_cast<double>(msg->state.value[static_cast<size_t>(i)]);
    }
    observationRosStamp_ = now();
    haveObservation_ = true;
  }

  void trajectoryCallback(
      const quadrotor_msgs::msg::PolynomialTraj::ConstSharedPtr msg)
  {
    using Message = quadrotor_msgs::msg::PolynomialTraj;
    if (msg->action == Message::ACTION_ABORT ||
        msg->action == Message::ACTION_WARN_IMPOSSIBLE)
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        assembling_.clear();
        active_ = AssembledTrajectory{};
        pending_ = AssembledTrajectory{};
        havePending_ = false;
      }
      assemblyTimer_->cancel();
      RCLCPP_WARN(
          get_logger(), "REMANI aborted the trajectory; publishing hold target.");
      publishReference();
      return;
    }

    if (msg->action != Message::ACTION_ADD || msg->trajectory.empty())
    {
      return;
    }

    TrajectorySection section;
    section.id = msg->trajectory_id;
    section.singul = msg->singul >= 0 ? 1 : -1;
    section.pieces = msg->trajectory;

    for (const auto &piece : section.pieces)
    {
      if (static_cast<int>(piece.num_dim) != armDim_ + 2 ||
          piece.data.size() !=
              static_cast<size_t>(piece.num_dim) *
                  (static_cast<size_t>(piece.num_order) + 1U) ||
          piece.duration <= 0.0)
      {
        RCLCPP_ERROR(
            get_logger(),
            "Rejected REMANI section %u: invalid dimension, coefficient "
            "count or duration.",
            msg->trajectory_id);
        return;
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (msg->trajectory_id == 1U)
      {
        assembling_.clear();
        assemblyStartStamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
        if (assemblyStartStamp_.nanoseconds() == 0)
        {
          assemblyStartStamp_ = now();
        }
        ++assemblyGeneration_;
      }
      else if (assembling_.empty())
      {
        RCLCPP_WARN(
            get_logger(),
            "Ignoring REMANI section %u because section 1 has not arrived.",
            msg->trajectory_id);
        return;
      }
      assembling_[section.id] = std::move(section);
    }

    assemblyTimer_->reset();
  }

  void finishAssembly()
  {
    assemblyTimer_->cancel();

    AssembledTrajectory assembled;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (assembling_.empty())
      {
        return;
      }

      uint32_t expectedId = 1U;
      assembled.startStamp = assemblyStartStamp_;
      assembled.generation = assemblyGeneration_;
      for (const auto &[id, section] : assembling_)
      {
        if (id != expectedId)
        {
          RCLCPP_ERROR(
              get_logger(),
              "REMANI trajectory has a segment gap: expected %u, received %u.",
              expectedId, id);
          assembling_.clear();
          return;
        }
        assembled.totalDuration += section.duration();
        assembled.sections.push_back(section);
        ++expectedId;
      }
      assembling_.clear();

      pending_ = assembled;
      havePending_ = true;
    }

    RCLCPP_INFO(
        get_logger(),
        "Assembled REMANI generation %lu: %zu sections, %.3f s.",
        static_cast<unsigned long>(assembled.generation),
        assembled.sections.size(), assembled.totalDuration);
    publishReference();
  }

  bool sampleAt(
      const rclcpp::Time &rosStamp,
      const ocs2::vector_t &holdState,
      double &yawReference,
      ocs2::vector_t &state,
      ocs2::vector_t &input)
  {
    AssembledTrajectory selected;
    bool haveSelected = false;
    {
      // mutex_ is already held by publishReference().
      if (havePending_ && rosStamp >= pending_.startStamp)
      {
        active_ = pending_;
        pending_ = AssembledTrajectory{};
        havePending_ = false;
      }

      if (!active_.empty() && rosStamp >= active_.startStamp)
      {
        selected = active_;
        haveSelected = true;
      }
    }

    if (!haveSelected)
    {
      state = holdState;
      input = ocs2::vector_t::Zero(inputDim_);
      yawReference = holdState(2);
      return true;
    }

    const double relativeTime =
        (rosStamp - selected.startStamp).seconds();
    PolynomialSample sample;
    if (!sampleTrajectory(selected, relativeTime, sample) ||
        static_cast<int>(sample.position.size()) != armDim_ + 2)
    {
      return false;
    }

    const double c = std::cos(transformYaw_);
    const double s = std::sin(transformYaw_);
    const double px = sample.position[0];
    const double py = sample.position[1];
    const double vx = sample.velocity[0];
    const double vy = sample.velocity[1];
    const double ax = sample.acceleration[0];
    const double ay = sample.acceleration[1];

    const double x = transformX_ + c * px - s * py;
    const double y = transformY_ + s * px + c * py;
    const double vxWorld = c * vx - s * vy;
    const double vyWorld = s * vx + c * vy;
    const double axWorld = c * ax - s * ay;
    const double ayWorld = s * ax + c * ay;
    const double speed2 = vxWorld * vxWorld + vyWorld * vyWorld;

    double yaw = yawReference;
    double forwardVelocity = 0.0;
    double omega = 0.0;
    if (speed2 >
        zeroVelocityThreshold_ * zeroVelocityThreshold_)
    {
      const double rawYaw = std::atan2(
          static_cast<double>(sample.singul) * vyWorld,
          static_cast<double>(sample.singul) * vxWorld);
      yaw = unwrapNear(rawYaw, yawReference);
      forwardVelocity =
          static_cast<double>(sample.singul) * std::sqrt(speed2);
      omega =
          (vxWorld * ayWorld - vyWorld * axWorld) / speed2;
    }

    state = ocs2::vector_t::Zero(stateDim_);
    input = ocs2::vector_t::Zero(inputDim_);
    state(0) = x;
    state(1) = y;
    state(2) = yaw;
    input(0) = forwardVelocity;
    input(1) = omega;
    for (int joint = 0; joint < armDim_; ++joint)
    {
      state(3 + joint) =
          sample.position[static_cast<size_t>(2 + joint)];
      input(2 + joint) =
          sample.velocity[static_cast<size_t>(2 + joint)];
    }

    if (relativeTime >= selected.totalDuration)
    {
      input.setZero();
      if (relativeTime > selected.totalDuration + holdAtEnd_)
      {
        // Keep publishing the terminal state.  The active trajectory remains
        // available until REMANI supplies a replacement or aborts.
      }
    }
    yawReference = yaw;
    return true;
  }

  void publishReference()
  {
    if (!targetPublisher_)
    {
      return;
    }

    ocs2::scalar_array_t timeTrajectory;
    ocs2::vector_array_t stateTrajectory;
    ocs2::vector_array_t inputTrajectory;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!haveObservation_)
      {
        return;
      }

      const ocs2::vector_t holdState = observationState_;
      const double obsTimeNow =
          observationTime_ + (now() - observationRosStamp_).seconds();
      const rclcpp::Time rosNow = now();
      double yawReference = holdState(2);

      // Anchor the reference at the latest measured state.  This prevents a
      // discontinuity while a future-starting replanned trajectory is pending.
      timeTrajectory.push_back(obsTimeNow);
      stateTrajectory.push_back(holdState);
      inputTrajectory.push_back(ocs2::vector_t::Zero(inputDim_));

      const int sampleCount =
          static_cast<int>(std::ceil(referenceHorizon_ / sampleDt_)) + 1;
      timeTrajectory.reserve(static_cast<size_t>(sampleCount + 1));
      stateTrajectory.reserve(static_cast<size_t>(sampleCount + 1));
      inputTrajectory.reserve(static_cast<size_t>(sampleCount + 1));

      for (int i = 0; i < sampleCount; ++i)
      {
        const double offset = startLead_ + static_cast<double>(i) * sampleDt_;
        ocs2::vector_t state;
        ocs2::vector_t input;
        if (!sampleAt(
                rosNow + rclcpp::Duration::from_seconds(offset),
                holdState, yawReference, state, input))
        {
          RCLCPP_ERROR_THROTTLE(
              get_logger(), *get_clock(), 2000,
              "Failed to evaluate REMANI polynomial reference.");
          return;
        }
        timeTrajectory.push_back(obsTimeNow + offset);
        stateTrajectory.push_back(std::move(state));
        inputTrajectory.push_back(std::move(input));
      }
    }

    ocs2::TargetTrajectories target(
        std::move(timeTrajectory),
        std::move(stateTrajectory),
        std::move(inputTrajectory));
    targetPublisher_->publishTargetTrajectories(target);
  }

  std::string robotName_;
  std::string trajectoryTopic_;
  int stateDim_ = 9;
  int inputDim_ = 8;
  int armDim_ = 6;
  double sampleDt_ = 0.04;
  double referenceHorizon_ = 3.0;
  double startLead_ = 0.05;
  double publishRate_ = 20.0;
  double assemblyTimeout_ = 0.04;
  double zeroVelocityThreshold_ = 1.0e-4;
  double holdAtEnd_ = 2.0;
  double transformX_ = 0.0;
  double transformY_ = 0.0;
  double transformYaw_ = 0.0;

  std::mutex mutex_;
  bool haveObservation_ = false;
  double observationTime_ = 0.0;
  rclcpp::Time observationRosStamp_{0, 0, RCL_ROS_TIME};
  ocs2::vector_t observationState_;

  std::map<uint32_t, TrajectorySection> assembling_;
  rclcpp::Time assemblyStartStamp_{0, 0, RCL_ROS_TIME};
  uint64_t assemblyGeneration_ = 0;
  AssembledTrajectory active_;
  AssembledTrajectory pending_;
  bool havePending_ = false;

  rclcpp::Subscription<quadrotor_msgs::msg::PolynomialTraj>::SharedPtr
      trajectorySub_;
  rclcpp::Subscription<ocs2_msgs::msg::MpcObservation>::SharedPtr
      observationSub_;
  rclcpp::TimerBase::SharedPtr assemblyTimer_;
  rclcpp::TimerBase::SharedPtr publishTimer_;
  std::unique_ptr<ocs2::TargetTrajectoriesRosPublisher> targetPublisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RemaniToOcs2ReferenceBridge>();
  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
