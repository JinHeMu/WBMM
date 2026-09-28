#pragma once

#include <wbmm_collision/urdf_collision_model.hpp>
#include <wbmm_core/trajectory.hpp>
#include <wbmm_environment/esdf_grid.hpp>
#include <wbmm_traj_opt/minco.hpp>

#include <Eigen/Core>

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace wbmm::traj_opt
{

struct OptimizerConfig
{
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
  // The envelope terms are soft penalties, so the optimizer may settle a little
  // outside the limit. The safety sweep treats that as acceptable up to this
  // relative tolerance; collisions are always fatal.
  double envelope_tolerance{0.05};

  // Experimental: holding heading changes the objective/gradient coupling.
  // A chord is not a general fix: it violates differential-drive translation
  // unless geometry and its full derivative chain are changed consistently.
  // Negative keeps the existing velocity-direction reconstruction.
  double heading_hold_speed{-1.0};

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
// Decision vector: the interior waypoints, flattened column-major
// (traj_dim x (pieces - 1) values). Durations are taken from the input and held
// fixed.
//
// Known limitation: the objective is not differentiable at the trajectory
// endpoints. Their velocity is exactly zero by the boundary conditions, so the
// reconstructed heading degenerates to atan2(0, 0) and a vanishing perturbation
// picks an arbitrary direction, which moves the arm spheres and jumps the
// collision cost. The finite-difference test therefore measures the gradient at
// a step above that scale. Fixing it means giving the heading the same
// held-at-rest behaviour the trajectory builder already uses.
//
// Scope note: REMANI also optimizes the durations. That is deliberately NOT
// done here yet. The explicit duration derivative and MINCO's KKT adjoint
// propagation interact in a way this implementation has not reproduced
// correctly, and an unverified timing gradient is worse than none. The caller
// already enforces the controller envelope by time-scaling the finished
// reference, so timing is covered; only the geometric refinement is delegated
// to this optimizer. The transverse gradient remains unverified; its current loose test is a
// regression diagnostic, not a correctness certificate. This optimizer is not
// connected to WholeBodyPlanner and requires external acceptance checks.
//
// The objective is the sum of the snap cost, obstacle clearance against the
// ESDF, ground clearance, self-collision between sphere groups, and base/joint
// envelope penalties. Gradients are analytic: sphere positions and their
// Jacobians come from wbmm::collision::evaluateWholeBodyKinematics, which is
// itself verified against finite differences.
//
// ROS-free, like the rest of wbmm_traj_opt.
class WholeBodyOptimizer
{
public:
  WholeBodyOptimizer(
    OptimizerConfig config,
    std::shared_ptr<const wbmm::environment::EsdfGrid> environment,
    wbmm::collision::UrdfCollisionModel collision_model);

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
  void accumulateCosts(double & cost);
  [[nodiscard]] bool safetySweep(
    double sample_dt, std::string & reason, double & min_clearance,
    double & max_base_speed, double & max_yaw_rate,
    double & max_joint_speed) const;

  OptimizerConfig config_;
  std::shared_ptr<const wbmm::environment::EsdfGrid> environment_;
  wbmm::collision::UrdfCollisionModel collision_model_;

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
};

}  // namespace wbmm::traj_opt
