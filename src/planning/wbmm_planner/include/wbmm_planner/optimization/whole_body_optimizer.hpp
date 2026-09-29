#pragma once

#include <wbmm_collision/trajectory_spheres.hpp>
#include <wbmm_robot_model/wbmm_robot_model.hpp>
#include <wbmm_core/trajectory.hpp>
#include <wbmm_environment/esdf_grid.hpp>
#include <wbmm_planner/optimization/minco.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace wbmm::traj_opt
{

struct OptimizerConfig
{
  bool optimize_durations{true};
  double time_weight{5.0};
  double min_piece_duration{0.05};
  double max_total_duration{60.0};
  double max_solve_time{1.0};
  double max_yaw_rate{1.0};
  double max_base_acceleration{1.0};
  bool treat_unknown_as_occupied{false};
  // ---- Cost weights --------------------------------------------------------
  double obstacle_weight{50000.0};
  double ground_weight{5000.0};
  double self_collision_weight{1000.0};
  double feasibility_weight{100.0};

  // ---- Clearances ----------------------------------------------------------
  double obstacle_margin{0.10};
  double ground_margin{0.03};
  double self_collision_margin{0.10};

  // ---- Base and joint envelope --------------------------------------------
  double max_base_speed{0.5};
  double max_joint_speed{1.57};
  double max_joint_acceleration{3.14};
  // Relative threshold for envelope diagnostics only. Envelope excess does not
  // reject a candidate here; the caller must time-scale and validate the final
  // reference. ESDF collision at a sweep sample rejects the candidate.
  double envelope_tolerance{0.05};

  // Compatibility parameter: heading at zero speed uses initial_yaw. Cost
  // quadrature uses interior midpoints, avoiding atan2 at fixed rest endpoints.
  double heading_hold_speed{1.0e-6};

  // Heading used before the first sample fast enough to define one.
  double initial_yaw{0.0};
  // Sized to the joint count; empty skips the position-limit term.
  Eigen::VectorXd joint_min;
  Eigen::VectorXd joint_max;

  // ---- Sampling ------------------------------------------------------------
  // Cost samples per polynomial piece. The obstacle and self-collision terms
  // are only as good as this sampling: a thin obstacle between two samples is
  // invisible to the optimizer, which is why the caller also runs a post-hoc
  // safety sweep before publishing.
  std::size_t samples_per_piece{8};

  // ---- L-BFGS --------------------------------------------------------------
  int max_iterations{200};
  double convergence_delta{1.0e-3};
  int memory_size{256};

  std::string trajectory_id{"optimized"};
};

struct OptimizerInput
{
  wbmm::core::Header header;
  // 8 x 4 columns of (position, velocity, acceleration, jerk).
  Eigen::MatrixXd head_state;
  Eigen::MatrixXd tail_state;
  // 8 x (pieces - 1) interior waypoints.
  Eigen::MatrixXd inner_points;
  Eigen::VectorXd durations;
  int gear{1};
  std::vector<std::string> joint_names;
  std::uint64_t environment_revision{0};
  std::uint64_t collision_model_revision{0};
};

struct OptimizerResult
{
  bool success{false};
  std::string message;
  Eigen::MatrixXd inner_points;
  Eigen::VectorXd durations;
  int solver_status{0};

  wbmm::core::WholeBodyTrajectory trajectory;

  int evaluations{0};
  double initial_cost{0.0};
  double final_cost{0.0};
  double solve_time{0.0};

  double max_linear_velocity{0.0};
  double max_yaw_rate{0.0};
  double max_joint_velocity{0.0};
  double min_obstacle_clearance{0.0};

  // Collision-only sampled diagnostic. This does NOT certify the controller
  // envelope, continuous collision freedom, or readiness for publication.
  bool safe{false};
};

// MINCO-based whole-body trajectory optimizer.
//
// Decision vector: column-major interior waypoints followed by virtual times.
// Positive real durations use REMANI's C2 virtual-time map plus a minimum
// duration. Gradients include real-time quadrature and the MINCO adjoint.
// Cost quadrature excludes fixed endpoints; publication is separately checked
// at endpoints and along the sampled/interpolated controller reference.
// Success is a candidate, not a continuous-time safety certificate.
//
// The objective is the sum of the snap cost, obstacle clearance against the
// ESDF, ground clearance, self-collision between sphere groups, and base/joint
// envelope penalties. Gradients are analytic: sphere positions and their
// Jacobians come from wbmm::collision::evaluateTrajectorySpheres,
// which is itself verified against finite differences. Geometry comes from the
// shared wbmm_robot_model CollisionSphereModel.
//
// ROS-free, like the rest of wbmm_planner.
class WholeBodyOptimizer
{
public:
  WholeBodyOptimizer(
    OptimizerConfig config,
    std::shared_ptr<const wbmm::environment::EsdfGrid> environment,
    wbmm::robot_model::RobotModelDescription model);

  // Validates the input and sizes the decision vector. Must be called before
  // cost() or optimize().
  [[nodiscard]] bool prepare(const OptimizerInput & input);

  [[nodiscard]] std::size_t decisionDimension() const noexcept
  {
    return decision_dimension_;
  }

  [[nodiscard]] Eigen::VectorXd initialGuess() const;

  // Objective and its analytic gradient at a decision vector.
  //
  // This is a pure function of `x`: every call rebuilds the MINCO from the
  // decision vector. That costs a banded factorisation per call, which the
  // optimizer amortises, but it is what makes the gradient verifiable against
  // finite differences in the test suite.
  [[nodiscard]] double cost(
    const Eigen::VectorXd & x, Eigen::VectorXd & gradient);

  [[nodiscard]] OptimizerResult optimize();

  [[nodiscard]] const std::string & message() const noexcept {return message_;}

  // Diagnostic accessors: the polynomial coefficients after the last cost()
  // call and the raw coefficient gradient it accumulated. Exposed so the
  // gradient chain can be split into "is gdC right" and "is the adjoint right".
  [[nodiscard]] const Eigen::MatrixXd & coefficientsForDiagnosis() const
  {return minco_.get_b();}
  [[nodiscard]] const Eigen::MatrixXd & coefficientGradientForDiagnosis() const
  {return gdC_snapshot_;}
  [[nodiscard]] const Trajectory<7> & trajectoryForDiagnosis() const
  {return trajectory_;}

private:
  [[nodiscard]] bool rebuild(const Eigen::VectorXd & x);
  // Accumulates every non-smoothness term directly into the MINCO's own
  // coefficient gradient, because that is what getGrad2TP() reads back.
  void accumulateCosts(double &cost, Eigen::VectorXd &gradient_durations);
  [[nodiscard]] bool safetySweep(
    double sample_dt, std::string & reason, double & min_clearance,
    double & max_base_speed, double & max_yaw_rate,
    double & max_joint_speed) const;

  OptimizerConfig config_;
  std::shared_ptr<const wbmm::environment::EsdfGrid> environment_;
  wbmm::robot_model::RobotModelDescription model_;

  OptimizerInput input_;
  MinSnapOpt<8> minco_;
  Trajectory<7> trajectory_;
  // gdC captured before the adjoint solve mutates it in place.
  Eigen::MatrixXd gdC_snapshot_;
  // Cost evaluations, used as a cheap progress signal.
  int evaluations_{0};

  std::size_t decision_dimension_{0};
  int piece_count_{0};
  int joint_count_{0};
  Eigen::VectorXd initial_durations_;
  std::string message_;
  bool prepared_{false};
  double best_cost_{0.0};
  Eigen::VectorXd best_x_;
};

}  // namespace wbmm::traj_opt
