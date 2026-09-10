#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <memory>
#include <string>
#include <vector>

namespace ta_wbmp
{

enum class TaskTrajectoryType
{
  SURFACE_RASTER,
  RAS_DRAWING,
  WAYPOINT_SEQUENCE
};

struct TaskWaypoint
{
  double progress{0.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
  Eigen::Vector3d tangent{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d surface_normal{Eigen::Vector3d::UnitY()};
  double nominal_speed{0.05};
  bool contact{true};
  std::string label;
};

struct TaskGeometry
{
  std::string surface_type{"planar"};
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d normal{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d axis_u{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d axis_v{Eigen::Vector3d::UnitZ()};
  Eigen::Vector2d u_limits{Eigen::Vector2d::Zero()};
  Eigen::Vector2d v_limits{Eigen::Vector2d::Zero()};
  double radius{0.0};
  bool local_coordinates{false};
};

struct MpcExecutionConfig
{
  double reference_rate{20.0};
  double reference_horizon{3.0};
  double reference_dt{0.08};
  double tracking_slow_squared_tolerance{0.04};
  double tracking_stop_squared_tolerance{0.16};
};

struct ForceExecutionConfig
{
  bool enabled{false};
  std::string mode{"constant_force"};
  std::string wrench_topic{"/fts_broadcaster/wrench"};
  std::string force_axis{"z"};
  bool absolute_force{true};
  double desired_force{12.0};
  double mass{3.0};
  double damping{45.0};
  double stiffness{300.0};
  double filter_alpha{0.25};
  double max_offset{0.015};
  double max_velocity{0.010};
  double base_share{0.0};
  double max_base_delta{0.02};
  double max_joint_delta{0.10};
  double sensor_timeout{0.20};
  double progress_full_speed_error{2.0};
  double progress_pause_error{8.0};
  double progress_min_scale{0.10};
  double hard_limit{35.0};
  double spike_rejection_n{8.0};
  int spike_confirm_samples{3};
};

struct TaskExecutionConfig
{
  MpcExecutionConfig mpc;
  ForceExecutionConfig force;
};

struct TaskTrajectory
{
  std::string name;
  std::string frame_id{"odom"};
  TaskTrajectoryType type{TaskTrajectoryType::SURFACE_RASTER};
  TaskGeometry geometry;
  TaskExecutionConfig execution;
  std::vector<TaskWaypoint> points;
};

class TaskTrajectoryProvider
{
public:
  virtual ~TaskTrajectoryProvider() = default;
  virtual TaskTrajectory generate() const = 0;
};

// YAML-backed default provider. A new task (inspection, spraying, picking,
// polishing, etc.) can either use the generic waypoint_sequence schema or
// implement TaskTrajectoryProvider without changing the whole-body planner.
class TaskTrajectoryGenerator final : public TaskTrajectoryProvider
{
public:
  explicit TaskTrajectoryGenerator(std::string task_file);
  TaskTrajectory generate() const override;

private:
  std::string task_file_;
};

std::string toString(TaskTrajectoryType type);

}  // namespace ta_wbmp
