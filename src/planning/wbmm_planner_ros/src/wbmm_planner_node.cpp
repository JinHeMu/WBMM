// =============================================================================
//  wbmm_planner_node.cpp
//
//  Thin ROS wrapper around the ROS-free wbmm::planning::WholeBodyPlanner.
//
//      /goal_pose + odom + /joint_states
//              -> wbmm_planner_ros/WholeBodyTrajectory
//
//  Everything algorithmic lives in wbmm_planner; this file only owns
//  parameters, the ESDF and URDF collision model, and the ROS interfaces.
//  It replaces the REMANI remani_planner_node (FSM + manager + visualisation).
// =============================================================================

#include <wbmm_collision/esdf_checker.hpp>
#include <wbmm_robot_model/wbmm_robot_model.hpp>
#include <wbmm_environment/esdf_loader.hpp>
#include <wbmm_pinocchio/pinocchio_robot_model.hpp>
#include <wbmm_planner/whole_body_planner.hpp>
#include <wbmm_planner_ros/msg/whole_body_trajectory.hpp>

#include <chrono>
#include <future>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/string.hpp>
#include <wbmm_planner_ros/msg/whole_body_goal.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

class WbmmPlannerNode : public rclcpp::Node
{
public:
  WbmmPlannerNode() : Node("wbmm_planner_node")
  {
    declareParameters();
    loadEnvironment();
    tfBuffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tfBuffer_);
    setupInterfaces();
  }

private:
  void declareParameters()
  {
    declare_parameter<std::string>("urdf_file", "");
    declare_parameter<std::string>("esdf_file", "");
    declare_parameter<std::string>("world_frame", "map");
    declare_parameter<std::string>("base_collision_link", "base_link");
    declare_parameter<std::vector<std::string>>(
      "joint_names",
      {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"});

    declare_parameter<std::string>("goal_topic", "/goal_pose");
    declare_parameter<std::string>("odom_topic", "/wheel/odometry");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<std::string>(
      "trajectory_topic", "/wbmm/whole_body_trajectory");

    declare_parameter<double>("cruise_speed", 0.35);
    declare_parameter<double>("min_segment_duration", 0.4);
    declare_parameter<double>("max_joint_speed", 1.57);
    declare_parameter<double>("max_trajectory_duration", 60.0);
    declare_parameter<double>("max_linear_velocity", 0.5);
    declare_parameter<double>("max_yaw_rate", 1.0);
    declare_parameter<double>("max_joint_velocity", 2.0);
    declare_parameter<double>("max_heading_step", 0.35);
    declare_parameter<double>("sample_dt", 0.05);
    declare_parameter<double>("tangent_chord_length", 0.15);
    declare_parameter<int>("minco_waypoint_stride", 3);

    declare_parameter<bool>("enable_optimization", true);
    declare_parameter<bool>("optimizer.optimize_durations", true);
    declare_parameter<double>("optimizer.max_solve_time", 1.0);
    declare_parameter<int>("optimizer.max_iterations", 100);
    declare_parameter<int>("optimizer.samples_per_piece", 12);
    declare_parameter<double>("optimizer.time_weight", 5.0);
    declare_parameter<double>("optimizer.obstacle_margin", 0.10);
    declare_parameter<double>("base_search_resolution", 0.15);
    declare_parameter<double>("base_search_position_tolerance", 0.20);
    declare_parameter<double>("base_search_yaw_tolerance", 0.30);
    declare_parameter<double>("base_search_max_time", 2.0);
    declare_parameter<double>("arm_seed_waypoint_spacing", 0.25);
    declare_parameter<int>("arm_seed_candidates", 64);
    declare_parameter<double>("arm_seed_max_joint_step", 0.6);
    declare_parameter<double>("arm_seed_max_time", 1.0);

    declare_parameter<double>("collision_safety_margin", 0.0);
    declare_parameter<bool>("treat_unknown_as_occupied", false);

    declare_parameter<std::string>("base_model", "differential");
    declare_parameter<double>("max_base_speed", 0.5);
    declare_parameter<double>("max_base_yaw_rate", 1.0);
    declare_parameter<std::vector<double>>(
      "joint_min", {-6.28, -1.48, -3.05, -1.48, -6.28, -6.28});
    declare_parameter<std::vector<double>>(
      "joint_max", {6.28, 4.62, 3.05, 4.62, 6.28, 6.28});

    declare_parameter<int>("environment_revision", 1);
    declare_parameter<int>("collision_model_revision", 1);
    declare_parameter<bool>("plan_on_goal", true);
    declare_parameter<double>("state_timeout", 0.5);
    declare_parameter<std::string>("whole_body_goal_topic", "/wbmm/whole_body_goal");
    declare_parameter<std::string>("cancel_topic", "/wbmm/planning/cancel");
    declare_parameter<double>("sample_rrt_max_time", 2.0);
    declare_parameter<double>("whole_body_rrt_max_time", 3.0);
    declare_parameter<int>("rrt_max_nodes", 12000);
    declare_parameter<int>("rrt_random_seed", 1);
    declare_parameter<bool>("enable_whole_body_rrt", true);
    declare_parameter<bool>("enable_primitive_fallback", true);
    declare_parameter<double>("goal_wait_timeout", 10.0);
    declare_parameter<double>("arrival_position_tolerance", 0.20);
    declare_parameter<double>("arrival_yaw_tolerance", 0.30);
  }

  void loadEnvironment()
  {
    urdfFile_ = get_parameter("urdf_file").as_string();
    esdfFile_ = get_parameter("esdf_file").as_string();
    worldFrame_ = get_parameter("world_frame").as_string();
    baseCollisionLink_ = get_parameter("base_collision_link").as_string();
    jointNames_ = get_parameter("joint_names").as_string_array();

    if (urdfFile_.empty() || esdfFile_.empty()) {
      throw std::runtime_error("urdf_file and esdf_file must be set.");
    }
    if (jointNames_.empty()) {
      throw std::runtime_error("joint_names must not be empty.");
    }

    // ---- Environment ------------------------------------------------------
    const auto loaded = wbmm::environment::NpzEsdfLoader::load(esdfFile_);
    if (loaded.status != wbmm::environment::LoadStatus::kSuccess ||
      loaded.grid == nullptr)
    {
      throw std::runtime_error("Cannot load ESDF '" + esdfFile_ + "': " + loaded.message);
    }
    if (worldFrame_.empty()) {
      // The ESDF and the planning frame must agree by construction, so adopting
      // the map's own frame removes a way to misconfigure the pair.
      worldFrame_ = loaded.grid->info().frame_id;
      RCLCPP_INFO(
        get_logger(), "world_frame not set; adopting the ESDF frame '%s'.",
        worldFrame_.c_str());
    } else if (loaded.grid->info().frame_id != worldFrame_) {
      throw std::runtime_error(
        "ESDF frame '" + loaded.grid->info().frame_id +
        "' does not match world_frame '" + worldFrame_ + "'.");
    }
    environment_ = loaded.grid;
    RCLCPP_INFO(
      get_logger(), "ESDF '%s': %dx%dx%d voxels of %.3f m in frame '%s'.",
      esdfFile_.c_str(), loaded.grid->info().shape.x(),
      loaded.grid->info().shape.y(), loaded.grid->info().shape.z(),
      loaded.grid->info().voxel_size, worldFrame_.c_str());

    // ---- Robot model and collision spheres --------------------------------
    // 统一来源：urdfdom 读取 URDF 描述，配置显式声明受控关节顺序与
    // 底盘碰撞 link，共享碰撞球模型由同一个描述构建。所有消费者（运动学、
    // 碰撞检查、优化器）都使用这一个对象。
    auto robot_description = wbmm::robot_model::loadRobotDescription(urdfFile_);
    auto config =
      wbmm::robot_model::RobotModelConfig::defaultsFor(robot_description);
    config.state_base_frame = robot_description.root_link;
    config.controlled_joints = jointNames_;
    config.base_collision_link = baseCollisionLink_;
    auto built =
      wbmm::robot_model::buildRobotModelDescription(robot_description, config);
    if (!built.success) {
      throw std::runtime_error(
        "Cannot build the robot model description: " + built.message);
    }
    collisionModel_ =
      std::make_shared<const wbmm::robot_model::RobotModelDescription>(
        std::move(built.model));

    const auto kinematic_model =
      wbmm::pinocchio::KinematicModel::create(collisionModel_);
    robotModel_ = std::make_shared<wbmm::pinocchio::PinocchioRobotModel>(
      kinematic_model, get_parameter("max_base_speed").as_double(),
      get_parameter("max_base_yaw_rate").as_double());

    wbmm::collision::CollisionCheckOptions options;
    options.safety_margin = get_parameter("collision_safety_margin").as_double();
    options.treat_unknown_as_occupied =
      get_parameter("treat_unknown_as_occupied").as_bool();

    // 碰撞检查复用同一个 KinematicModel：一次查询只更新一次 Context 并批量
    // 取回全部球心，不再逐 link 重算整树 FK。
    checker_ = std::make_shared<wbmm::collision::EsdfChecker>(
      kinematic_model, environment_, collisionModel_->collision_spheres, options);

    RCLCPP_INFO(
      get_logger(), "Robot model %s: %zu base + %zu arm spheres, margin %.3f m.",
      collisionModel_->contentIdHex().c_str(),
      collisionModel_->collision_spheres.baseSphereCount(),
      collisionModel_->collision_spheres.armSphereCount(),
      options.safety_margin);

    // The configured base model must agree with the URDF, otherwise the
    // planner would shape a trajectory for the wrong kinematics.
    const std::string configured = get_parameter("base_model").as_string();
    const bool differential =
      robotModel_->baseModel() == wbmm::core::BaseModel::kDifferentialDrive;
    if ((configured == "differential") != differential) {
      throw std::runtime_error(
        "base_model '" + configured + "' disagrees with the URDF-derived model.");
    }
  }

  void setupInterfaces()
  {
    const auto trajectoryQos = rclcpp::QoS(1).reliable();

    trajectoryPublisher_ =
      create_publisher<wbmm_planner_ros::msg::WholeBodyTrajectory>(
        get_parameter("trajectory_topic").as_string(), trajectoryQos);

    goalSub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("goal_topic").as_string(), trajectoryQos,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
      {goalCallback(msg);});

    odomSub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("odom_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg)
      {odomCallback(msg);});

    jointSub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("joint_state_topic").as_string(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg)
      {jointCallback(msg);});

    statusPublisher_ = create_publisher<std_msgs::msg::String>("/wbmm/planning/status",
                                                               rclcpp::QoS(1).transient_local());
    finishPublisher_ = create_publisher<std_msgs::msg::Bool>("/planning/finish", trajectoryQos);
    cancelPublisher_ = create_publisher<std_msgs::msg::Header>(
        get_parameter("cancel_topic").as_string(), trajectoryQos);
    cancelSub_ = create_subscription<std_msgs::msg::Header>(
        get_parameter("cancel_topic").as_string(), trajectoryQos,
        [this](std_msgs::msg::Header::SharedPtr msg) {
          const rclcpp::Time stamp(msg->stamp, get_clock()->get_clock_type());
          if (stamp <= lastCancel_)
            return;
          lastCancel_ = stamp;
          ++generation_;
          pending_.reset();
          executing_.reset();
          setStatus("CANCELLED");
        });
    wholeGoalSub_ = create_subscription<wbmm_planner_ros::msg::WholeBodyGoal>(
        get_parameter("whole_body_goal_topic").as_string(), trajectoryQos,
        [this](wbmm_planner_ros::msg::WholeBodyGoal::SharedPtr msg) {
          std::optional<wbmm::core::JointState> joints;
          if (!msg->joint_names.empty() || !msg->joint_positions.empty()) {
            if (msg->joint_names.size() != jointNames_.size() ||
                msg->joint_positions.size() != jointNames_.size()) {
              setStatus("REJECTED: whole-body goal joint dimensions");
              return;
            }
            wbmm::core::JointState q;
            q.names = jointNames_;
            q.velocities.assign(jointNames_.size(), 0);
            for (const auto &name : jointNames_) {
              const auto i = std::find(msg->joint_names.begin(), msg->joint_names.end(), name);
              if (i == msg->joint_names.end() ||
                  std::count(msg->joint_names.begin(), msg->joint_names.end(), name) != 1) {
                setStatus("REJECTED: whole-body goal joint names");
                return;
              }
              const double value = msg->joint_positions[std::distance(msg->joint_names.begin(), i)];
              if (!std::isfinite(value)) {
                setStatus("REJECTED: non-finite goal joint");
                return;
              }
              q.positions.push_back(value);
            }
            joints = q;
          }
          queueGoal(msg->header, msg->base_pose, joints);
        });
    planningTimer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { planningTick(); });
    setStatus("WAITING_FOR_GOAL");
    RCLCPP_INFO(
      get_logger(), "Planner ready: %s -> %s.",
      get_parameter("goal_topic").as_string().c_str(),
      get_parameter("trajectory_topic").as_string().c_str());
  }

  bool poseInWorld(const std_msgs::msg::Header & header,
                   const geometry_msgs::msg::Pose & pose,
                   wbmm::core::BaseState & state)
  {
    tf2::Quaternion q(pose.orientation.x, pose.orientation.y,
      pose.orientation.z, pose.orientation.w);
    if (header.frame_id.empty() || !std::isfinite(pose.position.x) ||
        !std::isfinite(pose.position.y) || !std::isfinite(q.length2()) ||
        q.length2() < 1e-12) { return false; }
    q.normalize();
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    state.x = pose.position.x; state.y = pose.position.y; state.yaw = yaw;
    if (header.frame_id == worldFrame_) { return true; }
    try {
      const auto tf = tfBuffer_->lookupTransform(worldFrame_, header.frame_id,
        rclcpp::Time(header.stamp, get_clock()->get_clock_type()));
      const auto & r = tf.transform.rotation;
      tf2::Quaternion rotation(r.x, r.y, r.z, r.w);
      tf2::Matrix3x3(rotation).getRPY(roll, pitch, yaw);
      if (!std::isfinite(yaw) || std::abs(roll) > 1e-3 || std::abs(pitch) > 1e-3) {
        return false;
      }
      const double x = state.x, y = state.y;
      state.x = std::cos(yaw) * x - std::sin(yaw) * y + tf.transform.translation.x;
      state.y = std::sin(yaw) * x + std::cos(yaw) * y + tf.transform.translation.y;
      state.yaw += yaw;
      return std::isfinite(state.x) && std::isfinite(state.y);
    } catch (const tf2::TransformException & error) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Cannot transform planning input: %s", error.what());
      return false;
    }
  }

  bool freshStamp(const builtin_interfaces::msg::Time & stamp) const
  {
    const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
    return age >= -0.05 && age <= get_parameter("state_timeout").as_double();
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    wbmm::core::BaseState state;
    if (!freshStamp(msg->header.stamp) || !poseInWorld(msg->header, msg->pose.pose, state) ||
        !std::isfinite(msg->twist.twist.linear.x) || !std::isfinite(msg->twist.twist.angular.z)) {
      return;
    }
    state.linear_velocity = msg->twist.twist.linear.x;
    state.yaw_rate = msg->twist.twist.angular.z;
    const std::lock_guard<std::mutex> lock(stateMutex_);
    start_ = state;
    odomStamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    haveOdom_ = true;
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!freshStamp(msg->header.stamp)) { return; }
    std::vector<double> positions(jointNames_.size(), 0.0);
    std::vector<double> velocities(jointNames_.size(), 0.0);
    for (std::size_t i = 0U; i < jointNames_.size(); ++i) {
      const auto match =
        std::find(msg->name.begin(), msg->name.end(), jointNames_[i]);
      if (match == msg->name.end()) {
        return;  // incomplete: keep the previous state
      }
      const auto index = static_cast<std::size_t>(
        std::distance(msg->name.begin(), match));
      if (index >= msg->position.size() || !std::isfinite(msg->position[index])) {
        return;
      }
      positions[i] = msg->position[index];
      if (index < msg->velocity.size() && std::isfinite(msg->velocity[index])) {
        velocities[i] = msg->velocity[index];
      }
    }

    const std::lock_guard<std::mutex> lock(stateMutex_);
    // The planner contract requires names, positions and velocities to agree in
    // size; a state carrying only positions is rejected downstream.
    startJoints_.names = jointNames_;
    startJoints_.positions = std::move(positions);
    startJoints_.velocities = std::move(velocities);
    startJoints_.efforts.assign(jointNames_.size(), 0.0);
    jointStamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    haveJoints_ = true;
  }

  wbmm::planning::PlannerConfig buildConfig() const
  {
    wbmm::planning::PlannerConfig config;
    config.cruise_speed = get_parameter("cruise_speed").as_double();
    config.min_segment_duration = get_parameter("min_segment_duration").as_double();
    config.max_joint_speed = get_parameter("max_joint_speed").as_double();
    config.max_trajectory_duration = get_parameter("max_trajectory_duration").as_double();
    config.minco_waypoint_stride =
      static_cast<std::size_t>(std::max(1, static_cast<int>(
        get_parameter("minco_waypoint_stride").as_int())));

    config.builder.sample_dt = get_parameter("sample_dt").as_double();
    config.builder.tangent_chord_length =
      get_parameter("tangent_chord_length").as_double();
    config.builder.max_linear_velocity = get_parameter("max_linear_velocity").as_double();
    config.builder.max_yaw_rate = get_parameter("max_yaw_rate").as_double();
    config.builder.max_joint_velocity = get_parameter("max_joint_velocity").as_double();
    config.builder.max_heading_step = get_parameter("max_heading_step").as_double();

    config.enable_optimization = get_parameter("enable_optimization").as_bool();
    config.optimizer.optimize_durations =
        get_parameter("optimizer.optimize_durations").as_bool();
    config.optimizer.max_solve_time =
        get_parameter("optimizer.max_solve_time").as_double();
    config.optimizer.max_iterations =
        get_parameter("optimizer.max_iterations").as_int();
    const auto samples = get_parameter("optimizer.samples_per_piece").as_int();
    config.optimizer.samples_per_piece =
        samples > 0 ? static_cast<std::size_t>(samples) : 0;
    config.optimizer.time_weight =
        get_parameter("optimizer.time_weight").as_double();
    config.optimizer.obstacle_margin =
        get_parameter("optimizer.obstacle_margin").as_double();
    config.optimizer.treat_unknown_as_occupied =
        get_parameter("treat_unknown_as_occupied").as_bool();
    config.base_search.position_resolution =
      get_parameter("base_search_resolution").as_double();
    config.base_search.position_tolerance =
      get_parameter("base_search_position_tolerance").as_double();
    config.base_search.yaw_tolerance =
      get_parameter("base_search_yaw_tolerance").as_double();
    // Leave half of the measured-arrival tolerance for tracking error. A
    // search endpoint on the arrival boundary otherwise never finishes after
    // even a sub-millimetre controller error.
    config.base_search.position_tolerance =
        std::min(config.base_search.position_tolerance,
                 0.5 * get_parameter("arrival_position_tolerance").as_double());
    config.base_search.yaw_tolerance = std::min(
        config.base_search.yaw_tolerance, 0.5 * get_parameter("arrival_yaw_tolerance").as_double());
    config.base_search.max_search_time =
      get_parameter("base_search_max_time").as_double();

    config.arm_seed.waypoint_spacing =
      get_parameter("arm_seed_waypoint_spacing").as_double();
    config.arm_seed.candidates_per_waypoint = static_cast<std::size_t>(
      std::max(1, static_cast<int>(get_parameter("arm_seed_candidates").as_int())));
    config.arm_seed.max_joint_step =
      get_parameter("arm_seed_max_joint_step").as_double();
    config.arm_seed.max_search_time =
      get_parameter("arm_seed_max_time").as_double();
    config.sample_rrt.max_search_time = get_parameter("sample_rrt_max_time").as_double();
    config.whole_body_rrt.max_search_time = get_parameter("whole_body_rrt_max_time").as_double();
    config.sample_rrt.max_nodes = config.whole_body_rrt.max_nodes =
        static_cast<std::size_t>(std::max<int64_t>(2, get_parameter("rrt_max_nodes").as_int()));
    config.sample_rrt.random_seed = config.whole_body_rrt.random_seed =
        static_cast<std::uint32_t>(get_parameter("rrt_random_seed").as_int());
    config.enable_whole_body_rrt = get_parameter("enable_whole_body_rrt").as_bool();
    config.enable_primitive_fallback = get_parameter("enable_primitive_fallback").as_bool();
    // Search bounds follow the loaded map, including custom deployment maps.
    const auto &info = environment_->info();
    config.base_search.min_x = info.origin.x();
    config.base_search.min_y = info.origin.y();
    config.base_search.max_x = info.origin.x() + info.shape.x() * info.voxel_size;
    config.base_search.max_y = info.origin.y() + info.shape.y() * info.voxel_size;
    return config;
  }

  void setStatus(const std::string &value) {
    std_msgs::msg::String msg;
    msg.data = value;
    statusPublisher_->publish(msg);
    RCLCPP_INFO(get_logger(), "Planning state: %s", value.c_str());
  }

  bool freshState() const {
    return haveOdom_ && haveJoints_ && (now() - odomStamp_).seconds() >= -0.05 &&
           (now() - jointStamp_).seconds() >= -0.05 &&
           (now() - odomStamp_).seconds() <= get_parameter("state_timeout").as_double() &&
           (now() - jointStamp_).seconds() <= get_parameter("state_timeout").as_double();
  }
  bool stationary() const {
    return std::abs(start_.linear_velocity) <= 0.02 && std::abs(start_.yaw_rate) <= 0.02 &&
           std::all_of(startJoints_.velocities.begin(), startJoints_.velocities.end(),
                       [](double v) { return std::isfinite(v) && std::abs(v) <= 0.02; });
  }
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    queueGoal(msg->header, msg->pose, std::nullopt);
  }
  void queueGoal(const std_msgs::msg::Header &header, const geometry_msgs::msg::Pose &pose,
                 const std::optional<wbmm::core::JointState> &joints) {
    if (!get_parameter("plan_on_goal").as_bool())
      return;
    wbmm::planning::PlanRequest request;
    request.header.frame_id = worldFrame_;
    if (!poseInWorld(header, pose, request.goal)) {
      setStatus("REJECTED: goal pose/TF");
      return;
    }
    request.goal_joints = joints;
    ++generation_;
    executing_.reset();
    pending_ = request;
    pendingSince_ = now();
    lastCancel_ = now();
    std_msgs::msg::Header cancel;
    cancel.stamp = lastCancel_;
    cancelPublisher_->publish(cancel);
    std_msgs::msg::Bool finish;
    finish.data = false;
    finishPublisher_->publish(finish);
    setStatus("WAITING_FOR_STOP");
  }
  void planningTick() {
    if (future_.valid()) {
      if (future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready)
        return;
      auto result = future_.get();
      RCLCPP_INFO(get_logger(),
                  "PLAN_METRICS base=%.6f arm=%.6f whole_rrt=%.6f build=%.6f "
                  "optimization=%.6f applied=%d evaluations=%d "
                  "cost_before=%.8g cost_after=%.8g; %s",
                  result.base_search_time, result.arm_seed_time,
                  result.whole_body_rrt_time, result.build_time,
                  result.optimization_time, result.optimization_applied,
                  result.optimization_evaluations, result.initial_cost,
                  result.final_cost, result.optimization_message.c_str());
      if (planningGeneration_ != generation_)
        return;
      if (!result.success) {
        pending_.reset();
        setStatus("FAILED: " + result.message);
        return;
      }
      bool unchanged =
          freshState() && stationary() &&
          std::hypot(start_.x - inflight_.start.x, start_.y - inflight_.start.y) < 0.03 &&
          std::abs(std::remainder(start_.yaw - inflight_.start.yaw, 2 * M_PI)) < 0.03;
      for (std::size_t j = 0; unchanged && j < jointNames_.size(); ++j)
        unchanged =
            std::abs(startJoints_.positions[j] - inflight_.start_joints.positions[j]) < 0.03;
      if (!unchanged) {
        setStatus("WAITING_FOR_STOP: state changed during planning");
        return;
      }
      auto message = toMessage(result.trajectory);
      message.header.stamp = now();
      message.trajectory_id += "_" + std::to_string(generation_);
      trajectoryPublisher_->publish(message);
      executing_ = inflight_;
      pending_.reset();
      executionStart_ = now();
      executionDuration_ = result.trajectory.points.back().time_from_start;
      arrivalSince_.reset();
      setStatus("EXECUTING: " + result.search_backend + " / " + result.trajectory_backend);
      RCLCPP_INFO(get_logger(), "%zu samples, %.2f s; fallback: %s", message.time_from_start.size(),
                  executionDuration_, result.fallback_reason.c_str());
      return;
    }
    if (executing_) {
      if (!freshState()) {
        cancelExecution(
            "FAULT: stale state (odom age=" + std::to_string((now() - odomStamp_).seconds()) +
            ", joint age=" + std::to_string((now() - jointStamp_).seconds()) + ")");
        return;
      }
      bool arrived = (now() - executionStart_).seconds() >= executionDuration_ && stationary() &&
                     std::hypot(start_.x - executing_->goal.x, start_.y - executing_->goal.y) <=
                         get_parameter("arrival_position_tolerance").as_double() &&
                     std::abs(std::remainder(start_.yaw - executing_->goal.yaw, 2 * M_PI)) <=
                         get_parameter("arrival_yaw_tolerance").as_double();
      if (executing_->goal_joints)
        for (std::size_t j = 0; arrived && j < jointNames_.size(); ++j)
          arrived =
              std::abs(startJoints_.positions[j] - executing_->goal_joints->positions[j]) < 0.05;
      if (arrived && !arrivalSince_)
        arrivalSince_ = now();
      if (!arrived)
        arrivalSince_.reset();
      if (arrivalSince_ && (now() - *arrivalSince_).seconds() > 0.5) {
        std_msgs::msg::Bool msg;
        msg.data = true;
        finishPublisher_->publish(msg);
        executing_.reset();
        setStatus("SUCCEEDED");
      } else if ((now() - executionStart_).seconds() > executionDuration_ + 10)
        cancelExecution("FAULT: arrival timeout");
    }
    if (!pending_)
      return;
    if ((now() - pendingSince_).seconds() > get_parameter("goal_wait_timeout").as_double()) {
      pending_.reset();
      setStatus("FAILED: fresh stationary state timeout");
      return;
    }
    if (!freshState() || !stationary())
      return;
    auto request = *pending_;
    request.start = start_;
    request.start_joints = startJoints_;
    request.limits = robotModel_->limits();
    request.limits.joint_min = get_parameter("joint_min").as_double_array();
    request.limits.joint_max = get_parameter("joint_max").as_double_array();
    request.limits.max_base_speed = get_parameter("max_base_speed").as_double();
    request.limits.max_base_yaw_rate = get_parameter("max_base_yaw_rate").as_double();
    request.environment_revision = static_cast<std::uint64_t>(
      std::max(0, static_cast<int>(get_parameter("environment_revision").as_int())));
    request.collision_model_revision = static_cast<std::uint64_t>(
      std::max(0, static_cast<int>(get_parameter("collision_model_revision").as_int())));

    if (request.limits.joint_min.size() != jointNames_.size() ||
      request.limits.joint_max.size() != jointNames_.size())
    {
      RCLCPP_ERROR(
        get_logger(), "joint_min/joint_max must have %zu entries.",
        jointNames_.size());
      return;
    }

    const auto baseChecker = [this](
                               const wbmm::core::Header & header,
                               const wbmm::core::BaseState & base)
    {return checker_->checkBase(header, base).isFree();};
    const auto wholeBodyChecker = [this](
                                    const wbmm::core::Header &,
                                    const wbmm::core::WholeBodyState & state)
    {
      return checker_->check(state, wbmm::collision::CheckScope::kWholeBody)
        .isFree();
    };

    inflight_ = request;
    planningGeneration_ = generation_;
    const auto config = buildConfig();
    setStatus("PLANNING");
    const auto optimize = [environment = environment_, model = collisionModel_](
                              const wbmm::traj_opt::OptimizerInput &input,
                              const wbmm::traj_opt::OptimizerConfig &options) {
      wbmm::traj_opt::WholeBodyOptimizer optimizer(options, environment, *model);
      if (!optimizer.prepare(input)) {
        wbmm::traj_opt::OptimizerResult failure;
        failure.message = optimizer.message();
        return failure;
      }
      return optimizer.optimize();
    };
    future_ = std::async(std::launch::async, [request, config, baseChecker,
                                              wholeBodyChecker, optimize]() {
      try {
        return wbmm::planning::WholeBodyPlanner(config).plan(
            request, baseChecker, wholeBodyChecker, optimize);
      } catch (const std::exception &e) {
        wbmm::planning::PlanResult result;
        result.message = e.what();
        return result;
      } catch (...) {
        wbmm::planning::PlanResult result;
        result.message = "Unknown planning exception";
        return result;
      }
    });
  }
  void cancelExecution(const std::string &reason) {
    ++generation_;
    pending_.reset();
    executing_.reset();
    lastCancel_ = now();
    std_msgs::msg::Header cancel;
    cancel.stamp = lastCancel_;
    cancelPublisher_->publish(cancel);
    setStatus(reason);
  }

  static wbmm_planner_ros::msg::WholeBodyTrajectory toMessage(
    const wbmm::core::WholeBodyTrajectory & trajectory)
  {
    wbmm_planner_ros::msg::WholeBodyTrajectory message;
    message.header.frame_id =
      trajectory.points.empty() ? std::string{} : trajectory.points.front().state.header.frame_id;
    message.trajectory_id = trajectory.trajectory_id;
    message.environment_revision = trajectory.environment_revision;
    message.collision_model_revision = trajectory.collision_model_revision;

    const std::size_t samples = trajectory.points.size();
    const std::size_t joints =
      samples == 0U ? 0U : trajectory.points.front().state.joints.names.size();
    if (samples > 0U) {
      message.joint_names = trajectory.points.front().state.joints.names;
    }

    message.time_from_start.reserve(samples);
    message.base_x.reserve(samples);
    message.base_y.reserve(samples);
    message.base_yaw.reserve(samples);
    message.base_linear_velocity.reserve(samples);
    message.base_yaw_rate.reserve(samples);
    message.phase.reserve(samples);
    message.joint_positions.reserve(samples * joints);
    message.joint_velocities.reserve(samples * joints);

    for (const auto & point : trajectory.points) {
      message.time_from_start.push_back(point.time_from_start);
      message.base_x.push_back(point.state.base.x);
      message.base_y.push_back(point.state.base.y);
      message.base_yaw.push_back(point.state.base.yaw);
      // The message contract puts the feedforward base input here, not a
      // measured velocity; fall back to the state when no input was attached.
      const bool hasInput = point.feedforward_input.has_value() &&
        point.feedforward_input->base_command.size() >= 2U;
      message.base_linear_velocity.push_back(
        hasInput ? point.feedforward_input->base_command[0]
                 : point.state.base.linear_velocity);
      message.base_yaw_rate.push_back(
        hasInput ? point.feedforward_input->base_command[1]
                 : point.state.base.yaw_rate);
      message.phase.push_back(static_cast<std::uint8_t>(point.phase));

      for (std::size_t j = 0U; j < joints; ++j) {
        message.joint_positions.push_back(point.state.joints.positions[j]);
        message.joint_velocities.push_back(point.state.joints.velocities[j]);
      }
    }
    return message;
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr finishPublisher_;
  rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr cancelPublisher_;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr cancelSub_;
  rclcpp::Subscription<wbmm_planner_ros::msg::WholeBodyGoal>::SharedPtr wholeGoalSub_;
  rclcpp::TimerBase::SharedPtr planningTimer_;
  std::optional<wbmm::planning::PlanRequest> pending_, executing_;
  wbmm::planning::PlanRequest inflight_;
  std::uint64_t generation_{0}, planningGeneration_{0};
  rclcpp::Time lastCancel_{0, 0, RCL_ROS_TIME}, pendingSince_{0, 0, RCL_ROS_TIME},
      executionStart_{0, 0, RCL_ROS_TIME};
  std::optional<rclcpp::Time> arrivalSince_;
  double executionDuration_{0};
  std::string urdfFile_;
  std::string esdfFile_;
  std::string worldFrame_;
  std::string baseCollisionLink_;
  std::vector<std::string> jointNames_;

  std::shared_ptr<const wbmm::environment::EsdfGrid> environment_;
  wbmm::core::RobotModelPtr robotModel_;
  wbmm::robot_model::RobotModelDescriptionPtr collisionModel_;
  std::shared_ptr<wbmm::collision::EsdfChecker> checker_;

  rclcpp::Publisher<wbmm_planner_ros::msg::WholeBodyTrajectory>::SharedPtr
    trajectoryPublisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goalSub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointSub_;

  std::shared_ptr<tf2_ros::Buffer> tfBuffer_;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  rclcpp::Time odomStamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time jointStamp_{0, 0, RCL_ROS_TIME};
  std::mutex stateMutex_;
  wbmm::core::BaseState start_;
  wbmm::core::JointState startJoints_;
  bool haveOdom_{false};
  bool haveJoints_{false};
  std::future<wbmm::planning::PlanResult> future_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<WbmmPlannerNode>());
  }
  catch (const std::exception & error)
  {
    RCLCPP_FATAL(
      rclcpp::get_logger("wbmm_planner_node"), "Planner failed to start: %s",
      error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
