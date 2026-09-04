#pragma once

#include "ta_wbmp/cost.hpp"
#include "ta_wbmp/extensions.hpp"
#include "ta_wbmp/task_trajectory.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace ta_wbmp
{

inline constexpr char kPhaseNavigate[] = "NAVIGATE";
inline constexpr char kPhasePrecontactAlign[] = "PRECONTACT_ALIGN";
inline constexpr char kPhasePrecontactApproach[] = "PRECONTACT_APPROACH";
inline constexpr char kPhaseTask[] = "TASK_CONSTRAINED";

double wrapAngle(double angle);

struct Waypoint
{
  double time{0.0};
  Eigen::VectorXd state;
  std::string phase;
  Eigen::Vector3d task_target{Eigen::Vector3d::Zero()};
  bool has_task_target{false};
  Eigen::Vector3d task_normal{Eigen::Vector3d::UnitY()};
  bool contact{false};
};

struct PlanReport
{
  bool success{false};
  std::string task_name;
  std::string task_type;
  int state_dimension{0};
  std::size_t waypoint_count{0};
  double duration{0.0};
  int candidate_count{0};
  int feasible_candidate_count{0};
  int selected_candidate_id{-1};
  double selected_standoff{0.0};
  double selected_longitudinal_offset{0.0};
  double selected_future_task_score{0.0};
  double max_contact_position_error{0.0};
  double max_tool_axis_error{0.0};
  double minimum_joint_margin{0.0};
  double minimum_manipulability{0.0};
  double minimum_sigma{0.0};
  double task_base_path_length{0.0};
  double max_lateral_velocity{0.0};
  double max_base_speed{0.0};
  double max_angular_speed{0.0};
  double max_joint_speed{0.0};
  double navigation_clearance{0.0};
  std::map<std::string, int> phase_counts;
  std::map<std::string, bool> constraints;
};

struct Plan
{
  std::vector<Waypoint> waypoints;
  std::vector<Eigen::Vector3d> task_targets;
  std::vector<Eigen::Vector3d> task_normals;
  std::vector<Eigen::Vector3d> obstacles;  // x, y, radius
  Eigen::Vector3d surface_center{Eigen::Vector3d::Zero()};
  Eigen::Vector3d surface_normal{Eigen::Vector3d::UnitY()};
  Eigen::Vector3d surface_axis_u{Eigen::Vector3d::UnitX()};
  Eigen::Vector3d surface_axis_v{Eigen::Vector3d::UnitZ()};
  Eigen::Vector2d surface_u_limits{Eigen::Vector2d::Zero()};
  Eigen::Vector2d surface_v_limits{Eigen::Vector2d::Zero()};
  bool surface_local_coordinates{false};
  Eigen::Vector2d x_limits{Eigen::Vector2d::Zero()};
  Eigen::Vector2d z_limits{Eigen::Vector2d::Zero()};
  Eigen::Vector2d surface_parameter_limits{Eigen::Vector2d::Zero()};
  std::string surface_type{"planar"};
  double surface_radius{0.0};
  std::string frame_id{"odom"};
  TaskTrajectory task_trajectory;
  std::vector<CandidateMetrics> candidate_evaluations;
  Eigen::VectorXd remani_navigation_goal;
  Eigen::VectorXd task_entry_state;
  std::size_t execution_start_index{0};
  std::size_t task_start_index{0};
  PlanReport report;
};

struct VisualGeometry
{
  std::string name;
  std::string mesh_path;
  Eigen::Vector3d mesh_scale{Eigen::Vector3d::Ones()};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

struct CollisionSphereGeometry
{
  std::string name;
  Eigen::Vector3d center{Eigen::Vector3d::Zero()};
  double radius{0.0};
  pinocchio::JointIndex parent_joint{0};
};

class TaskAwarePlanner
{
public:
  TaskAwarePlanner(const std::string & urdf_file, const std::string & ee_frame,
                   const std::string & task_file,
                   std::shared_ptr<const CandidateCostEvaluator> cost = nullptr,
                   std::shared_ptr<const WholeBodyStateValidityChecker> validity = nullptr,
                   std::shared_ptr<const NavigationCostEstimator> nav_cost = nullptr);

  Plan plan();
  Eigen::Vector3d toolPosition(const Eigen::VectorXd & state) const;
  Eigen::Matrix3d toolRotation(const Eigen::VectorXd & state) const;
  std::vector<Eigen::Vector3d> jointPoints(const Eigen::VectorXd & state) const;
  std::vector<VisualGeometry> visualGeometry(
    const Eigen::VectorXd & state) const;
  std::vector<CollisionSphereGeometry> collisionSpheres(
    const Eigen::VectorXd & state) const;
  bool environmentCollisionChecked() const;

private:
  struct TaskSample
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d normal{Eigen::Vector3d::UnitY()};
    Eigen::Matrix3d desired_rotation{Eigen::Matrix3d::Identity()};
    Eigen::Vector3d tangent{Eigen::Vector3d::UnitX()};
    double nominal_speed{0.05};
    bool contact{true};
  };

  struct IkSolution
  {
    Eigen::VectorXd q;
    double position_error{0.0};
    double axis_error{0.0};
  };

  struct Candidate
  {
    int id{-1};
    double standoff{0.0};
    double offset{0.0};
    std::vector<Eigen::Vector3d> bases;
    std::vector<Eigen::VectorXd> joints;
    Eigen::VectorXd precontact_joint;
    std::vector<Eigen::VectorXd> approach_joints;
    std::vector<Eigen::Vector3d> approach_targets;
    double score{0.0};
    double max_position_error{0.0};
    double max_axis_error{0.0};
    double minimum_joint_margin{0.0};
    double minimum_manipulability{0.0};
    double minimum_sigma{0.0};
    double base_path_length{0.0};
    double arm_path_length{0.0};
    CandidateMetrics metrics;
  };

  Eigen::Matrix3d rotationZ(double yaw) const;
  Eigen::Vector3d surfacePoint(double parameter, double z) const;
  Eigen::Vector3d surfaceNormal(double parameter) const;
  Eigen::Matrix3d desiredRotation(
    const Eigen::Vector3d & normal) const;
  std::vector<TaskSample> taskSamples() const;
  std::optional<Candidate> selectCandidate(
      const std::vector<TaskSample> & samples,
      int & candidate_count, int & feasible_count,
      std::vector<CandidateMetrics> & evaluations) const;
  std::optional<Candidate> evaluateCandidate(
      int candidate_id, const std::vector<TaskSample> & samples, double standoff,
      double offset, double yaw, std::string & failure_reason) const;
  std::optional<IkSolution> solveIk(
    const Eigen::Vector3d & base, const Eigen::Vector3d & target,
    const Eigen::Matrix3d & desired_rotation,
    const Eigen::VectorXd & seed) const;
  double jointMargin(const Eigen::VectorXd & q) const;
  double manipulability(const Eigen::VectorXd & q) const;
  double minimumSingularValue(const Eigen::VectorXd & q) const;

  std::vector<Eigen::Vector2d> gridAstar(
    const Eigen::Vector2d & start, const Eigen::Vector2d & goal) const;
  std::vector<Eigen::Vector2d> simplifyPolyline(
    const std::vector<Eigen::Vector2d> & points) const;
  bool segmentIsFree(
    const Eigen::Vector2d & start, const Eigen::Vector2d & goal) const;
  double pointClearance(const Eigen::Vector2d & point) const;

  std::vector<Waypoint> timeParameterizeNavigation(
    const std::vector<Eigen::Vector2d> & points,
    const Eigen::Vector3d & goal_base) const;
  void appendAlignment(
    std::vector<Waypoint> & waypoints,
    const Eigen::VectorXd & target_joint) const;
  void appendApproach(
    std::vector<Waypoint> & waypoints, const Eigen::Vector3d & base,
    const std::vector<Eigen::VectorXd> & joints,
    const std::vector<Eigen::Vector3d> & targets) const;
  void appendTask(
    std::vector<Waypoint> & waypoints, const Candidate & candidate,
    const std::vector<TaskSample> & samples) const;
  PlanReport validate(
    const std::vector<Waypoint> & waypoints, const Candidate & candidate,
    int candidate_count, int feasible_count) const;
  bool precontactClearanceOk(const std::vector<Waypoint> & waypoints) const;

  std::string task_name_;
  std::string frame_id_;
  std::string urdf_file_;
  pinocchio::Model model_;
  pinocchio::GeometryModel visual_model_;
  pinocchio::GeometryModel collision_model_;
  pinocchio::FrameIndex ee_frame_id_;

  Eigen::Vector3d surface_center_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d surface_normal_{Eigen::Vector3d::UnitY()};
  Eigen::Vector2d x_limits_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d z_limits_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d surface_parameter_limits_{Eigen::Vector2d::Zero()};
  std::string surface_type_{"planar"};
  double surface_radius_{0.0};
  Eigen::Matrix3d desired_rotation_{Eigen::Matrix3d::Identity()};
  TaskTrajectory task_trajectory_;
  Eigen::Vector3d base_standoff_direction_{Eigen::Vector3d::Zero()};
  bool fixed_base_standoff_direction_{false};
  std::string base_policy_{"follow_task"};
  std::shared_ptr<const CandidateCostEvaluator> cost_evaluator_;
  std::shared_ptr<const WholeBodyStateValidityChecker> validity_checker_;
  std::shared_ptr<const NavigationCostEstimator> navigation_cost_estimator_;

  int raster_rows_{3};
  int raster_columns_{6};
  double sample_spacing_{0.08};
  double tangential_speed_{0.06};
  double row_change_speed_{0.035};
  double approach_clearance_{0.12};
  double approach_speed_{0.025};
  double hold_duration_{0.6};

  std::vector<double> standoff_samples_;
  std::vector<double> offset_samples_;
  double task_yaw_{0.0};
  double position_tolerance_{0.012};
  double axis_tolerance_{0.12};
  double minimum_margin_{0.035};
  double minimum_manipulability_{1.0e-5};

  Eigen::VectorXd initial_state_;
  double grid_resolution_{0.10};
  double robot_radius_{0.31};
  double base_speed_{0.25};
  double angular_speed_{0.55};
  double joint_speed_{0.12};
  double sample_period_{0.10};
  std::vector<Eigen::Vector3d> obstacles_;

  int max_iterations_{90};
  double damping_{1.0e-4};
  double max_joint_step_{0.18};
  Eigen::VectorXd nominal_seed_;
  Eigen::VectorXd search_seed_;
  Eigen::VectorXd lower_;
  Eigen::VectorXd upper_;
};

}  // namespace ta_wbmp
