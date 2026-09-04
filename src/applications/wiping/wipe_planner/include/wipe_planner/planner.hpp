#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <yaml-cpp/yaml.h>

#include <limits>
#include <string>
#include <vector>

namespace wipe_planner
{

double wrapAngle(double angle);
double wholeBodySquaredError(const Eigen::VectorXd & current,
                             const Eigen::VectorXd & goal);
double adaptiveProgressRate(double tracking_error, double lag_error,
                            double previous_rate, double min_rate,
                            double max_rate, double filter,
                            double lag_gain, double ahead_gain,
                            double soft_error, double hard_error);
double rateLimitedStep(double current, double target, double max_rate,
                       double dt);
double forceProgressScale(double force_error, double full_speed_error,
                          double pause_error, double minimum_scale);

struct Waypoint
{
  double time{0.0};
  Eigen::VectorXd state;
  Eigen::VectorXd input;
  Eigen::Vector3d contact_target{Eigen::Vector3d::Zero()};
  bool in_contact{false};
};

struct PlanReport
{
  std::size_t points{0};
  double duration{0.0};
  double max_position_error{0.0};
  double max_axis_error{0.0};
  double max_lateral_velocity{0.0};
  double max_wheel_speed{0.0};
  int ik_failures{0};
  int hybrid_expanded_nodes{0};
  int reachability_rejections{0};
  int collision_rejections{0};
};

// Geometry/kinematics quality evaluated over every waypoint of a planned
// rollout.  Collision clearance is self-collision clearance between the same
// non-adjacent URDF geometry pairs used by the planner; it deliberately does
// not pretend to be an ESDF/environment distance.
struct TrajectoryMetrics
{
  std::size_t evaluated_points{0};
  double min_joint_margin{std::numeric_limits<double>::infinity()};
  double min_normalized_joint_margin{std::numeric_limits<double>::infinity()};
  double min_manipulability{std::numeric_limits<double>::infinity()};
  double min_self_collision_clearance{std::numeric_limits<double>::infinity()};
  double max_joint_speed{0.0};
};

struct VisualGeometry
{
  std::string name;
  std::string mesh_path;
  Eigen::Vector3d mesh_scale{Eigen::Vector3d::Ones()};
  Eigen::Vector4d color{0.7, 0.7, 0.7, 1.0};
  Eigen::Vector3d position{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
};

class Planner
{
public:
  Planner(const std::string & urdf_file, const std::string & ee_frame,
          const std::string & task_file);

  std::vector<Waypoint> plan(const Eigen::VectorXd & measured_seed,
                             PlanReport & report) const;
  // Continue the wipe task from an already reached whole-body pre-contact
  // state.  Unlike plan()+prependMeasuredAlignment(), the returned first
  // waypoint is exactly precontact_state and no motion back to a nominal
  // entry configuration is inserted.
  std::vector<Waypoint> planFromPrecontact(
    const Eigen::VectorXd & precontact_state, PlanReport & report) const;
  // Enumerate collision-free whole-body states whose tool pose satisfies the
  // first coverage point plus the configured surface-normal clearance.
  std::vector<Eigen::VectorXd> precontactCandidates() const;
  std::vector<Waypoint> prependMeasuredAlignment(
    const std::vector<Waypoint> & task_trajectory,
    const Eigen::VectorXd & measured_state, double max_joint_speed,
    PlanReport & report) const;
  Eigen::VectorXd normalOffsetCorrectedState(const Eigen::VectorXd & state,
                                      double normal_offset,
                                      double max_joint_delta) const;
  Eigen::Vector3d framePosition(const Eigen::VectorXd & state,
                                const std::string & frame) const;
  Eigen::Matrix3d frameRotation(const Eigen::VectorXd & state,
                                const std::string & frame) const;
  TrajectoryMetrics evaluateTrajectory(
    const std::vector<Waypoint> & trajectory) const;
  bool isArmConfigurationCollisionFree(const Eigen::VectorXd & q) const;
  Eigen::VectorXd jointLowerLimits() const {return model_.lowerPositionLimit;}
  Eigen::VectorXd jointUpperLimits() const {return model_.upperPositionLimit;}
  std::vector<VisualGeometry> visualGeometry(const Eigen::VectorXd & state) const;
  Eigen::Vector3d surfaceCenter() const {return surface_center_;}
  Eigen::Vector3d surfaceNormal() const {return normal_;}
  Eigen::Vector3d precontactTarget() const;
  // Ordered surface-contact targets before IK/base planning. The sequence is a
  // single continuous stroke for both the legacy raster and the RAS pattern.
  std::vector<Eigen::Vector3d> coverageTargets() const;
  std::string coveragePattern() const {return coverage_pattern_;}
  std::string surfaceType() const {return surface_type_;}
  Eigen::Vector2d surfaceXLimits() const {return x_limits_;}
  Eigen::Vector2d surfaceYLimits() const {return y_limits_;}
  Eigen::Vector2d surfaceZLimits() const {return z_limits_;}
  // Wall-parallel horizontal unit vector for vertical surfaces. Derived from
  // normal_into_room, so the coverage follows a wall running along ANY odom
  // axis (e.g. plane X=const) without rotating the map. x_limits are then
  // coordinates along this direction.
  Eigen::Vector3d wallDirection() const;
  double desiredForce() const {return desired_force_;}
  int armDof() const {return model_.nq;}

  static Eigen::Vector3d propagateDifferentialDrive(
    const Eigen::Vector3d & pose, double v, double omega, double dt);

private:
  struct IkResult
  {
    Eigen::VectorXd q;
    double position_error{std::numeric_limits<double>::infinity()};
    double axis_error{std::numeric_limits<double>::infinity()};
    bool success{false};
  };

  struct BaseCandidate
  {
    double cost{std::numeric_limits<double>::infinity()};
    std::vector<Eigen::Vector3d> bases;
    std::vector<Eigen::VectorXd> joints;
    std::vector<Eigen::Vector3d> contact_targets;
    double max_position_error{0.0};
    double max_axis_error{0.0};
    int failures{0};
  };

  struct SearchResult
  {
    bool success{false};
    double cost{std::numeric_limits<double>::infinity()};
    std::vector<Eigen::Vector3d> bases;
    std::vector<Eigen::VectorXd> joints;
    std::vector<Eigen::Vector3d> contact_targets;
    double max_position_error{0.0};
    double max_axis_error{0.0};
  };

  Eigen::Matrix3d rotationZ(double yaw) const;
  void forward(const Eigen::Vector3d & base, const Eigen::VectorXd & q,
               Eigen::Vector3d & position, Eigen::Matrix3d & rotation) const;
  IkResult solveIk(const Eigen::Vector3d & base, const Eigen::Vector3d & target,
                   const Eigen::VectorXd & previous) const;
  std::vector<Eigen::Vector3d> rasterTargets() const;
  std::vector<Eigen::Vector3d> rasTargets() const;
  std::vector<Eigen::Vector3d> densifyTargets(
    const std::vector<Eigen::Vector3d> & coarse) const;
  std::vector<Eigen::Vector3d> reachableBaseCandidates(
    const Eigen::Vector3d & target) const;
  SearchResult hybridAstarSegment(
    const Eigen::Vector3d & start_base, const Eigen::VectorXd & start_q,
    const Eigen::Vector3d & start_target, const Eigen::Vector3d & goal_base,
    const Eigen::Vector3d & goal_target, PlanReport & report) const;
  BaseCandidate searchWholeBodyPath(
    const std::vector<Eigen::Vector3d> & targets,
    const Eigen::VectorXd & arm_seed, PlanReport & report) const;
  BaseCandidate continueWholeBodyPath(
    const std::vector<Eigen::Vector3d> & targets,
    const Eigen::Vector3d & start_base, const Eigen::VectorXd & start_q,
    PlanReport & report) const;
  bool basePoseValid(const Eigen::Vector3d & base,
                     const Eigen::Vector2d & lower,
                     const Eigen::Vector2d & upper) const;
  bool armCollisionFree(const Eigen::VectorXd & q) const;
  bool armMotionCollisionFree(const Eigen::VectorXd & first_q,
                              const Eigen::VectorXd & second_q,
                              bool allow_colliding_start = false) const;
  std::vector<Eigen::VectorXd> collisionFreeJointPath(
    const Eigen::VectorXd & start_q, const Eigen::VectorXd & goal_q) const;
  bool wholeBodyMotionCollisionFree(
    const Eigen::Vector3d & first_base, const Eigen::VectorXd & first_q,
    const Eigen::Vector3d & second_base, const Eigen::VectorXd & second_q,
    const Eigen::Vector2d & lower, const Eigen::Vector2d & upper) const;
  static Eigen::Vector3d propagateArc(
    const Eigen::Vector3d & pose, double curvature, double arc);
  std::vector<Waypoint> buildContactTrajectory(
    const std::vector<Eigen::Vector3d> & targets,
    const BaseCandidate & candidate, PlanReport & report) const;
  std::vector<Waypoint> prependNormalApproach(
    const std::vector<Waypoint> & contact_trajectory,
    PlanReport & report) const;
  void fillFeedforward(std::vector<Waypoint> & trajectory,
                       PlanReport & report) const;

  std::string urdf_file_;
  pinocchio::Model model_;
  pinocchio::GeometryModel visual_model_;
  pinocchio::GeometryModel collision_model_;
  pinocchio::FrameIndex ee_frame_id_;

  Eigen::Vector3d surface_center_;
  Eigen::Vector3d normal_;
  std::string surface_type_{"vertical"};
  Eigen::Vector2d x_limits_;
  Eigen::Vector2d y_limits_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d z_limits_;
  Eigen::Matrix3d desired_tool_rotation_{Eigen::Matrix3d::Identity()};
  std::string coverage_pattern_{"raster"};
  double ras_width_{0.90};
  double ras_height_{0.60};
  int raster_rows_{4};
  int raster_columns_{5};
  double solver_spacing_{0.03};
  double tangential_speed_{0.06};
  double corner_speed_{0.025};
  double contact_offset_{0.0};
  double desired_force_{12.0};
  double approach_clearance_{0.12};
  double approach_speed_{0.02};
  double approach_touch_distance_{0.02};
  double approach_touch_speed_{0.001};
  double precontact_hold_duration_{1.0};
  double preferred_standoff_{0.80};
  Eigen::Vector2d standoff_limits_{0.72, 0.88};
  double candidate_lateral_step_{0.04};
  double candidate_longitudinal_range_{0.16};
  double candidate_longitudinal_step_{0.08};
  Eigen::Vector3d base_standoff_direction_{0.0, -1.0, 0.0};
  double cleaning_yaw_{0.0};
  double hybrid_xy_resolution_{0.05};
  double hybrid_yaw_resolution_{0.15};
  double hybrid_primitive_length_{0.06};
  double hybrid_collision_check_step_{0.02};
  double hybrid_goal_position_tolerance_{0.025};
  double hybrid_goal_yaw_tolerance_{0.08};
  double hybrid_max_curvature_{2.0};
  double hybrid_heuristic_weight_{4.0};
  double hybrid_reverse_penalty_{1.05};
  double hybrid_gear_switch_penalty_{3.0};
  double hybrid_curvature_penalty_{0.15};
  double hybrid_curvature_change_penalty_{0.08};
  double hybrid_joint_motion_penalty_{0.20};
  double hybrid_search_margin_{0.30};
  double hybrid_standoff_margin_{0.05};
  double collision_joint_step_{0.04};
  int hybrid_max_nodes_{8000};
  int hybrid_yaw_samples_{0};
  double hybrid_yaw_sample_step_{0.08};
  double max_base_speed_{0.25};
  double max_base_acceleration_{0.35};
  double max_angular_speed_{0.55};
  double max_angular_acceleration_{0.8};
  double wheel_separation_{0.34};
  double wheel_radius_{0.07};
  double max_wheel_angular_speed_{10.0};
  double max_joint_speed_{0.55};
  double ik_position_tolerance_{0.012};
  double ik_axis_tolerance_{0.12};
  int ik_iterations_{120};
  Eigen::VectorXd nominal_ik_seed_;
  Eigen::VectorXd search_ik_seed_;
};

}  // namespace wipe_planner
