#include "wbmm_traj_opt/whole_body_optimizer.hpp"

#include "wbmm_traj_opt/lbfgs.hpp"
#include "wbmm_traj_opt/whole_body_trajectory_builder.hpp"

#include <wbmm_collision/whole_body_kinematics.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace wbmm::traj_opt
{
namespace
{

using Clock = std::chrono::steady_clock;

constexpr int kDim = 8;
// Same positive C2 time map as REMANI (with a caller-selected lower bound).
double realTime(double t) {
  return t > 0 ? 0.5 * t * t + t + 1 : 1 / (0.5 * t * t - t + 1);
}
double virtualTime(double t) {
  return t > 1 ? std::sqrt(2 * t - 1) - 1 : 1 - std::sqrt(2 / t - 1);
}
double timeDerivative(double t) {
  const double d = 0.5 * t * t - t + 1;
  return t > 0 ? t + 1 : (1 - t) / (d * d);
}

// C1 penalty on a scalar excess. Returns the cost and d(cost)/d(value).
double penalty(double excess, double & derivative)
{
  if (excess <= 0.0) {
    derivative = 0.0;
    return 0.0;
  }
  derivative = 2.0 * excess;
  return excess * excess;
}

// C1 penalty on a clearance shortfall, matching the shape REMANI uses for
// obstacle terms.
double clearancePenalty(double shortfall, double & derivative)
{
  if (shortfall <= 0.0) {
    derivative = 0.0;
    return 0.0;
  }
  derivative = 3.0 * shortfall * shortfall;
  return shortfall * shortfall * shortfall;
}

}  // namespace

WholeBodyOptimizer::WholeBodyOptimizer(
  OptimizerConfig config,
  std::shared_ptr<const wbmm::environment::EsdfGrid> environment,
  wbmm::collision::UrdfCollisionModel collision_model)
: config_(std::move(config)),
  environment_(std::move(environment)),
  collision_model_(std::move(collision_model))
{
}

bool WholeBodyOptimizer::prepare(const OptimizerInput & input)
{
  message_.clear();
  prepared_ = false;
  decision_dimension_ = 0;
  evaluations_ = 0;
  best_cost_ = std::numeric_limits<double>::infinity();
  best_x_.resize(0);
  input_ = input;

  if (environment_ == nullptr) {
    message_ = "An ESDF environment is required.";
    return false;
  }
  if (!collision_model_.success) {
    message_ = "The URDF collision model is not valid: " + collision_model_.message;
    return false;
  }
  if (input_.head_state.rows() != kDim || input_.head_state.cols() != 4 ||
    input_.tail_state.rows() != kDim || input_.tail_state.cols() != 4)
  {
    message_ = "head_state and tail_state must be 8 x 4.";
    return false;
  }
  if (input_.gear != 1 && input_.gear != -1) {
    message_ = "gear must be +1 or -1.";
    return false;
  }
  joint_count_ = static_cast<int>(input_.joint_names.size());
  if (joint_count_ != kDim - 2) {
    message_ = "This optimizer requires exactly six named arm joints.";
    return false;
  }
  if (input_.durations.size() <= 0 ||
    input_.inner_points.cols() != input_.durations.size() - 1 ||
    input_.inner_points.rows() != kDim)
  {
    message_ = "inner_points must be 8 x (durations.size() - 1).";
    return false;
  }
  if (!input_.head_state.allFinite() || !input_.tail_state.allFinite() ||
    !input_.inner_points.allFinite() || !input_.durations.allFinite() ||
    (input_.durations.array() <= 0.0).any())
  {
    message_ = "Non-finite or non-positive input.";
    return false;
  }
  if (static_cast<std::size_t>(joint_count_) != collision_model_.joints.size()) {
    message_ = "joint_names does not match the collision model's joints.";
    return false;
  }
  if (config_.samples_per_piece == 0U) {
    message_ = "samples_per_piece must be positive.";
    return false;
  }

  if (input_.header.frame_id != environment_->info().frame_id) {
    message_ = "Optimizer input frame does not match ESDF.";
    return false;
  }
  for (int j = 0; j < joint_count_; ++j)
    if (input_.joint_names[j] != collision_model_.joints[j].name) {
      message_ = "Optimizer joint names/order do not match collision model.";
      return false;
    }
  for (double v : {config_.min_piece_duration, config_.max_total_duration,
                   config_.max_solve_time, config_.max_base_speed,
                   config_.max_joint_speed, config_.max_joint_acceleration,
                   config_.max_yaw_rate, config_.max_base_acceleration})
    if (!std::isfinite(v) || v <= 0) {
      message_ = "Optimizer limits and budgets must be positive and finite.";
      return false;
    }
  for (double v : {config_.time_weight, config_.obstacle_weight,
                   config_.ground_weight, config_.self_collision_weight,
                   config_.feasibility_weight, config_.obstacle_margin,
                   config_.ground_margin, config_.self_collision_margin})
    if (!std::isfinite(v) || v < 0) {
      message_ = "Invalid optimizer weight/margin.";
      return false;
    }
  if (config_.max_iterations < 1 || config_.memory_size < 1 ||
      !std::isfinite(config_.initial_yaw) ||
      !std::isfinite(config_.convergence_delta) ||
      config_.convergence_delta <= 0 || input_.durations.size() > 1000 ||
      config_.samples_per_piece > 10000 ||
      (input_.durations.array() <= config_.min_piece_duration).any() ||
      input_.durations.sum() > config_.max_total_duration) {
    message_ = "Invalid optimizer iteration/time/sample budget.";
    return false;
  }
  if ((config_.joint_min.size() != 0 || config_.joint_max.size() != 0) &&
      (config_.joint_min.size() != joint_count_ ||
       config_.joint_max.size() != joint_count_ ||
       !config_.joint_min.allFinite() || !config_.joint_max.allFinite() ||
       (config_.joint_min.array() > config_.joint_max.array()).any())) {
    message_ = "Invalid optimizer joint limits.";
    return false;
  }
  piece_count_ = static_cast<int>(input_.durations.size());
  initial_durations_ = input_.durations;
  decision_dimension_ =
      static_cast<std::size_t>(kDim * (piece_count_ - 1) +
                               (config_.optimize_durations ? piece_count_ : 0));
  minco_.reset(input_.head_state, input_.tail_state, piece_count_);
  prepared_ = true;
  return true;
}

Eigen::VectorXd WholeBodyOptimizer::initialGuess() const
{
  const Eigen::Index inner_count = kDim * (piece_count_ - 1);
  Eigen::VectorXd x(decision_dimension_);
  x.head(inner_count) = Eigen::Map<const Eigen::VectorXd>(
      input_.inner_points.data(), inner_count);
  if (config_.optimize_durations)
    for (int i = 0; i < piece_count_; ++i)
      x(inner_count + i) =
          virtualTime(initial_durations_(i) - config_.min_piece_duration);
  return x;
}

bool WholeBodyOptimizer::rebuild(const Eigen::VectorXd & x)
{
  if (!prepared_ || static_cast<std::size_t>(x.size()) != decision_dimension_ ||
      !x.allFinite()) {
    message_ = "Decision vector has the wrong size or is not finite.";
    return false;
  }

  const Eigen::Index inner_count = kDim * (piece_count_ - 1);
  Eigen::MatrixXd inner(kDim, piece_count_ - 1);
  Eigen::Map<Eigen::VectorXd>(inner.data(), inner_count) = x.head(inner_count);
  Eigen::VectorXd durations = initial_durations_;
  if (config_.optimize_durations)
    for (int i = 0; i < piece_count_; ++i)
      durations(i) = config_.min_piece_duration + realTime(x(inner_count + i));
  if (!durations.allFinite() || durations.sum() > config_.max_total_duration)
    return false;

  minco_.reset(input_.head_state, input_.tail_state, piece_count_);
  minco_.generate(inner, durations);
  trajectory_ = minco_.getTraj(input_.gear);
  return true;
}

double WholeBodyOptimizer::cost(
  const Eigen::VectorXd & x, Eigen::VectorXd & gradient)
{
  ++evaluations_;
  gradient = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(decision_dimension_));
  if (!rebuild(x)) {
    // A non-finite or malformed trial point must not silently become a cheap
    // one; report a large cost with a zero gradient so the line search backs off.
    return std::numeric_limits<double>::max() / 1e3;
  }

  // Seed smoothness derivatives; add quadrature derivatives before the adjoint.
  Eigen::VectorXd gradient_durations = Eigen::VectorXd::Zero(piece_count_);
  double total = 0.0;
  minco_.initGradCost(gradient_durations, total);
  accumulateCosts(total, gradient_durations);
  total += config_.time_weight * minco_.get_T1().sum();
  gradient_durations.array() += config_.time_weight;

  // Snapshot before the adjoint solve, which mutates gdC in place.
  gdC_snapshot_ = minco_.get_gdC();

  Eigen::MatrixXd gradient_inner(kDim, piece_count_ - 1);
  Eigen::MatrixXd gradient_head(kDim, 4);
  Eigen::MatrixXd gradient_tail(kDim, 4);
  minco_.getGrad2TP(
    gradient_durations, gradient_inner, gradient_head, gradient_tail);

  const Eigen::Index inner_count = kDim * (piece_count_ - 1);
  gradient.head(inner_count) =
      Eigen::Map<const Eigen::VectorXd>(gradient_inner.data(), inner_count);
  if (config_.optimize_durations)
    for (int i = 0; i < piece_count_; ++i)
      gradient(inner_count + i) =
          gradient_durations(i) * timeDerivative(x(inner_count + i));

  if (!gradient.allFinite() || !std::isfinite(total)) {
    gradient.setZero();
    return std::numeric_limits<double>::max() / 1e3;
  }
  if (total < best_cost_) {
    best_cost_ = total;
    best_x_ = x;
  }
  return total;
}

void WholeBodyOptimizer::accumulateCosts(double &cost,
                                         Eigen::VectorXd &gradient_durations) {
  // initGradCost() has already seeded this with the smoothness gradient.
  Eigen::MatrixXd & gradient_coefficients = minco_.get_gdC();

  const int samples = static_cast<int>(config_.samples_per_piece);
  const auto & b = minco_.get_b();
  const auto & durations = minco_.get_T1();
  const std::string & frame = input_.header.frame_id;

  Eigen::Matrix<double, kDim, 1> beta0;
  Eigen::Matrix<double, kDim, 1> beta1;
  Eigen::Matrix<double, kDim, 1> beta2;
  Eigen::Matrix<double, kDim, 1> beta3;

  for (int piece = 0; piece < piece_count_; ++piece) {
    const Eigen::MatrixXd coefficients =
      b.block(piece * kDim, 0, kDim, kDim);
    const double duration = durations(piece);
    const double step = duration / static_cast<double>(samples);

    for (int sample = 0; sample < samples; ++sample) {
      const double s =
          (static_cast<double>(sample) + 0.5) / static_cast<double>(samples);
      // MINCO stores REAL-TIME polynomial coefficients: Piece::getPos evaluates
      // sum_k c_k * t^k with the real time t. The basis must therefore be
      // evaluated at tau = s * T, and beta_k = d^k(beta_0)/dtau are the real
      // derivatives with no further scaling. Getting this wrong shifts every
      // sample to the wrong instant and breaks the duration gradient.
      const double tau = s * duration;
      const double t2 = tau * tau;
      const double t3 = t2 * tau;
      const double t4 = t2 * t2;
      const double t5 = t4 * tau;
      const double t6 = t3 * t3;
      const double t7 = t6 * tau;
      beta0 << 1.0, tau, t2, t3, t4, t5, t6, t7;
      beta1 << 0.0, 1.0, 2.0 * tau, 3.0 * t2, 4.0 * t3, 5.0 * t4, 6.0 * t5, 7.0 * t6;
      beta2 << 0.0, 0.0, 2.0, 6.0 * tau, 12.0 * t2, 20.0 * t3, 30.0 * t4, 42.0 * t5;
      beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * tau, 60.0 * t2, 120.0 * t3, 210.0 * t4;

      const Eigen::VectorXd position = coefficients.transpose() * beta0;
      const Eigen::VectorXd velocity = coefficients.transpose() * beta1;
      const Eigen::VectorXd acceleration = coefficients.transpose() * beta2;
      const Eigen::VectorXd jerk = coefficients.transpose() * beta3;

      // Midpoint quadrature avoids undefined headings at fixed rest boundaries.
      const double weight = 1.0;
      double sample_cost = 0.0;

      Eigen::VectorXd grad_position = Eigen::VectorXd::Zero(kDim);
      Eigen::VectorXd grad_velocity = Eigen::VectorXd::Zero(kDim);
      Eigen::VectorXd grad_acceleration = Eigen::VectorXd::Zero(kDim);

      // ---- Collision spheres: obstacle, ground, self-collision -------------
      const auto kinematics = wbmm::collision::evaluateWholeBodyKinematics(
          collision_model_, position.head<2>(), velocity.head<2>(), input_.gear,
          position.tail(joint_count_), config_.initial_yaw);
      if (!kinematics.success)
        throw std::runtime_error(kinematics.message);

      if (kinematics.success) {
        // Chains a gradient expressed at a sphere position back onto the
        // decision variables, through the analytic sphere Jacobian.
        const auto chain = [&](const wbmm::collision::SphereKinematics & sphere,
                               const Eigen::Vector3d & gradient_at_sphere,
                               Eigen::VectorXd & out_position,
                               Eigen::VectorXd & out_velocity)
        {
          out_position(0) += sphere.jacobian.col(0).dot(gradient_at_sphere);
          out_position(1) += sphere.jacobian.col(1).dot(gradient_at_sphere);
          for (int j = 0; j < joint_count_; ++j) {
            out_position(2 + j) +=
              sphere.jacobian.col(4 + j).dot(gradient_at_sphere);
          }
          out_velocity(0) += sphere.jacobian.col(2).dot(gradient_at_sphere);
          out_velocity(1) += sphere.jacobian.col(3).dot(gradient_at_sphere);
        };

        const auto handleSphere = [&](const wbmm::collision::SphereKinematics & sphere)
        {
          // Obstacle.
          //
          // A sphere outside the grid is queried at the nearest point inside it
          // rather than being skipped. Skipping made the objective
          // discontinuous: a sphere crossing the grid edge dropped its whole
          // penalty in one step, which the finite-difference test detects as a
          // cost jump and which would break the line search in practice. The
          // clamped query is continuous and stays conservative because the
          // ESDF is clamped at its boundary too.
          auto query = environment_->query(frame, sphere.position);
          Eigen::Vector3d outside = Eigen::Vector3d::Zero();
          if (query.status == wbmm::environment::QueryStatus::kOutOfBounds) {
            const auto & info = environment_->info();
            const Eigen::Vector3d lower = info.origin;
            const Eigen::Vector3d upper =
              info.origin + info.voxel_size * info.shape.cast<double>();
            Eigen::Vector3d clamped = sphere.position;
            for (int axis = 0; axis < 3; ++axis) {
              clamped(axis) = std::clamp(clamped(axis), lower(axis), upper(axis));
            }
            query = environment_->query(frame, clamped);
            outside = sphere.position - clamped;
            for (int axis = 0; axis < 3; ++axis)
              if (outside(axis) != 0)
                query.gradient(axis) = 0;
            if (outside.norm() > 0) {
              query.distance -= outside.norm();
              query.gradient -= outside.normalized();
            }
          }
          if (query.status != wbmm::environment::QueryStatus::kSuccess ||
              !query.gradient_valid ||
              (config_.treat_unknown_as_occupied && !query.fully_observed))
            throw std::runtime_error(
                "Optimizer ESDF query is unavailable or unknown: " +
                query.message);
          if (query.status == wbmm::environment::QueryStatus::kSuccess &&
            query.gradient_valid)
          {
            const double shortfall =
              sphere.radius + config_.obstacle_margin - query.distance;
            double derivative = 0.0;
            const double term = clearancePenalty(shortfall, derivative);
            if (term > 0.0) {
              sample_cost += config_.obstacle_weight * term;
              // d(shortfall)/d(p) = -gradient(distance).
              const Eigen::Vector3d gradient_at_sphere =
                (-config_.obstacle_weight * derivative) * query.gradient;
              chain(sphere, gradient_at_sphere, grad_position, grad_velocity);
            }
          }

          // Ground: only the z component matters, so only row 2 of the
          // Jacobian contributes.
          const double ground_shortfall =
            sphere.radius + config_.ground_margin - sphere.position.z();
          double ground_derivative = 0.0;
          const double ground_term =
            penalty(ground_shortfall, ground_derivative);
          if (ground_term > 0.0) {
            sample_cost += config_.ground_weight * ground_term;
            Eigen::Vector3d gradient_at_sphere = Eigen::Vector3d::Zero();
            gradient_at_sphere.z() = -config_.ground_weight * ground_derivative;
            chain(sphere, gradient_at_sphere, grad_position, grad_velocity);
          }
        };

        for (const auto & sphere : kinematics.base) {
          handleSphere(sphere);
        }
        for (const auto & sphere : kinematics.arm) {
          handleSphere(sphere);
        }

        // Self-collision between sphere groups. Same-group pairs are excluded
        // (they are rigidly linked) and arm pairs closer than two joints apart
        // are excluded because the links are adjacent.
        const auto pairPenalty = [&](
                                   const wbmm::collision::SphereKinematics & a,
                                   const wbmm::collision::SphereKinematics & b)
        {
          const Eigen::Vector3d delta = a.position - b.position;
          const double distance = delta.norm();
          if (distance < 1e-9) {
            return;
          }
          const double shortfall =
            a.radius + b.radius + config_.self_collision_margin - distance;
          double derivative = 0.0;
          const double term = clearancePenalty(shortfall, derivative);
          if (term <= 0.0) {
            return;
          }
          sample_cost += config_.self_collision_weight * term;
          const Eigen::Vector3d direction = delta / distance;
          // shortfall = ra + rb + margin - |a - b|, so
          //   d(shortfall)/d(a) = -direction   and   d(shortfall)/d(b) = +direction
          // and d(cost)/d(.) = weight * derivative * d(shortfall)/d(.).
          chain(
            a, -config_.self_collision_weight * derivative * direction,
            grad_position, grad_velocity);
          chain(
            b, config_.self_collision_weight * derivative * direction,
            grad_position, grad_velocity);
        };

        for (std::size_t i = 0; i < kinematics.arm.size(); ++i) {
          for (const auto & base_sphere : kinematics.base) {
            pairPenalty(kinematics.arm[i], base_sphere);
          }
          const std::size_t group_i = kinematics.arm_group[i];
          for (std::size_t j = i + 1U; j < kinematics.arm.size(); ++j) {
            const std::size_t group_j = kinematics.arm_group[j];
            const std::size_t difference =
              group_i > group_j ? group_i - group_j : group_j - group_i;
            if (difference < 2U) {
              continue;  // adjacent links
            }
            pairPenalty(kinematics.arm[i], kinematics.arm[j]);
          }
        }
      }

      // ---- Base velocity envelope -----------------------------------------
      {
        const double speed_squared =
          velocity(0) * velocity(0) + velocity(1) * velocity(1);
        const double excess =
          speed_squared - config_.max_base_speed * config_.max_base_speed;
        double derivative = 0.0;
        const double term = penalty(excess, derivative);
        if (term > 0.0) {
          sample_cost += config_.feasibility_weight * term;
          grad_velocity(0) +=
            config_.feasibility_weight * derivative * 2.0 * velocity(0);
          grad_velocity(1) +=
            config_.feasibility_weight * derivative * 2.0 * velocity(1);
        }
      }

      // Differential-drive yaw rate = cross(v,a)/|v|^2. The final emitted
      // reference is checked again by the builder and the planner.
      const Eigen::Vector2d v = velocity.head<2>(), a = acceleration.head<2>();
      const double speed2 = v.squaredNorm();
      if (speed2 > 1e-12) {
        const double cross = v.x() * a.y() - v.y() * a.x();
        const double omega = cross / speed2;
        double derivative = 0;
        const double term =
            penalty(omega * omega - config_.max_yaw_rate * config_.max_yaw_rate,
                    derivative);
        sample_cost += config_.feasibility_weight * term;
        const double factor =
            config_.feasibility_weight * derivative * 2 * omega;
        grad_velocity.head<2>() +=
            factor * (Eigen::Vector2d(a.y(), -a.x()) / speed2 -
                      2 * cross * v / (speed2 * speed2));
        grad_acceleration.head<2>() +=
            factor * Eigen::Vector2d(-v.y(), v.x()) / speed2;
      }
      double acc_derivative = 0;
      const double acc_term =
          penalty(a.squaredNorm() - config_.max_base_acceleration *
                                        config_.max_base_acceleration,
                  acc_derivative);
      sample_cost += config_.feasibility_weight * acc_term;
      grad_acceleration.head<2>() +=
          2 * config_.feasibility_weight * acc_derivative * a;

      // ---- Joint position, velocity and acceleration envelopes ------------
      for (int j = 0; j < joint_count_; ++j) {
        const int index = 2 + j;
        if (config_.joint_min.size() == joint_count_ &&
          config_.joint_max.size() == joint_count_) {
          double derivative = 0.0;
          double term = penalty(config_.joint_min(j) - position(index), derivative);
          if (term > 0.0) {
            sample_cost += config_.feasibility_weight * term;
            grad_position(index) -= config_.feasibility_weight * derivative;
          }
          term = penalty(position(index) - config_.joint_max(j), derivative);
          if (term > 0.0) {
            sample_cost += config_.feasibility_weight * term;
            grad_position(index) += config_.feasibility_weight * derivative;
          }
        }

        double derivative = 0.0;
        double term = penalty(
          std::abs(velocity(index)) - config_.max_joint_speed, derivative);
        if (term > 0.0) {
          sample_cost += config_.feasibility_weight * term;
          grad_velocity(index) += config_.feasibility_weight * derivative *
            (velocity(index) >= 0.0 ? 1.0 : -1.0);
        }

        term = penalty(
          std::abs(acceleration(index)) - config_.max_joint_acceleration,
          derivative);
        if (term > 0.0) {
          sample_cost += config_.feasibility_weight * term;
          grad_acceleration(index) += config_.feasibility_weight * derivative *
            (acceleration(index) >= 0.0 ? 1.0 : -1.0);
        }
      }

      // The quadrature weight belongs to the cost, not only to the gradient.
      // Adding the raw per-sample cost here made the objective an unweighted
      // sum while the gradient integrated with quadrature weights, so the two
      // disagreed by exactly the ratio of the two weightings. The
      // finite-difference test caught it.
      cost += weight * step * sample_cost;
      // Explicit d/dT: changing quadrature width and evaluation time s*T.
      // The coefficient dependence is handled separately by MINCO's adjoint.
      gradient_durations(piece) +=
          weight *
          (sample_cost / samples +
           step * s *
               (grad_position.dot(velocity) + grad_velocity.dot(acceleration) +
                grad_acceleration.dot(jerk)));

      // ---- Accumulate into the coefficient and duration gradients ---------
      // Real-time coefficients: d(sample)/d(coefficients) = beta_k(t).
      gradient_coefficients.block(piece * kDim, 0, kDim, kDim) +=
        weight * step * beta0 * grad_position.transpose();
      gradient_coefficients.block(piece * kDim, 0, kDim, kDim) +=
        weight * step * beta1 * grad_velocity.transpose();
      gradient_coefficients.block(piece * kDim, 0, kDim, kDim) +=
        weight * step * beta2 * grad_acceleration.transpose();
    }
  }
}

OptimizerResult WholeBodyOptimizer::optimize()
{
  OptimizerResult result;
  if (!prepared_) {
    result.message = message_.empty() ? "prepare() was not called." : message_;
    return result;
  }

  const auto started = Clock::now();
  Eigen::VectorXd x = initialGuess();
  Eigen::VectorXd gradient(static_cast<Eigen::Index>(decision_dimension_));

  result.initial_cost = cost(x, gradient);

  lbfgs::lbfgs_parameter_t parameters;
  parameters.mem_size = config_.memory_size;
  parameters.g_epsilon = 0.0;  // nonsmooth penalties: rely on the delta test
  parameters.delta = config_.convergence_delta;
  parameters.max_iterations = config_.max_iterations;

  double final_cost = result.initial_cost;
  struct Progress {
    WholeBodyOptimizer *self;
    Clock::time_point start;
    double budget;
  } progress{this, started, config_.max_solve_time};
  const int status =
      decision_dimension_ == 0
          ? 0
          : lbfgs::lbfgs_optimize(
                x, final_cost,
                [](void *instance, const Eigen::VectorXd &trial,
                   Eigen::VectorXd &out) {
                  return static_cast<Progress *>(instance)->self->cost(trial,
                                                                       out);
                },
                nullptr,
                [](void *instance, const Eigen::VectorXd &,
                   const Eigen::VectorXd &, double, double, int, int) {
                  auto *p = static_cast<Progress *>(instance);
                  return std::chrono::duration<double>(Clock::now() - p->start)
                                     .count() > p->budget
                             ? 1
                             : 0;
                },
                &progress, parameters);

  result.solver_status = status;
  if (best_x_.size() != x.size() || !std::isfinite(best_cost_)) {
    result.message = "No finite optimizer iterate.";
    return result;
  }
  x = best_x_;
  result.final_cost = best_cost_;
  result.solve_time =
    std::chrono::duration<double>(Clock::now() - started).count();
  result.evaluations = evaluations_;

  if (!rebuild(x)) {
    result.message = "The optimizer returned an unusable decision vector: " + message_;
    return result;
  }

  std::string reason;
  double min_clearance = 0.0;
  result.safe = safetySweep(
    0.02, reason, min_clearance, result.max_linear_velocity,
    result.max_yaw_rate, result.max_joint_velocity);
  result.min_obstacle_clearance = min_clearance;
  if (!result.safe) {
    result.message = "Optimized candidate rejected by collision sweep: " + reason;
    return result;
  }

  TrajectoryBuilderConfig builder;
  builder.gear = input_.gear;
  builder.initial_yaw = config_.initial_yaw;
  builder.trajectory_id = config_.trajectory_id;
  builder.environment_revision = input_.environment_revision;
  builder.collision_model_revision = input_.collision_model_revision;

  const auto built = buildWholeBodyTrajectory(
    trajectory_, input_.joint_names, input_.header.frame_id, builder);
  if (!built.success) {
    result.message = "Trajectory shaping failed: " + built.message;
    return result;
  }

  result.inner_points =
      Eigen::Map<const Eigen::Matrix<double, kDim, Eigen::Dynamic>>(
          x.data(), kDim, piece_count_ - 1);
  result.durations = minco_.get_T1();
  result.trajectory = built.trajectory;
  result.success = true;
  result.message = (reason.empty() ? "ok" : "Optimized; note: " + reason) +
                   " (L-BFGS status " + std::to_string(status) + ")";
  return result;
}

bool WholeBodyOptimizer::safetySweep(
  double sample_dt, std::string & reason, double & min_clearance,
  double & max_base_speed, double & max_yaw_rate, double & max_joint_speed) const
{
  std::string envelope_note;
  min_clearance = std::numeric_limits<double>::max();
  max_base_speed = 0.0;
  max_yaw_rate = 0.0;
  max_joint_speed = 0.0;

  const double duration = trajectory_.getTotalDuration();
  if (!(duration > 0.0)) {
    reason = "The trajectory has no duration.";
    return false;
  }
  if (!std::isfinite(duration) || duration > config_.max_total_duration) {
    reason = "Trajectory exceeds optimizer duration budget.";
    return false;
  }
  const int samples = std::max(2, static_cast<int>(std::ceil(duration / sample_dt)));
  double previous_yaw = config_.initial_yaw;
  bool first = true;

  for (int i = 0; i <= samples; ++i) {
    const double t = duration * static_cast<double>(i) / static_cast<double>(samples);
    const Eigen::VectorXd position = trajectory_.getPos(t);
    const Eigen::VectorXd velocity = trajectory_.getVel(t);
    if (!position.allFinite() || !velocity.allFinite()) {
      reason = "The trajectory produced a non-finite sample.";
      return false;
    }

    const auto kinematics = wbmm::collision::evaluateWholeBodyKinematics(
        collision_model_, position.head<2>(), velocity.head<2>(), input_.gear,
        position.tail(joint_count_), previous_yaw);
    if (!kinematics.success) {
      reason = "Kinematics failed during the safety sweep: " + kinematics.message;
      return false;
    }

    const auto check = [&](const wbmm::collision::SphereKinematics & sphere)
    {
      auto query = environment_->query(input_.header.frame_id, sphere.position);
      if (query.status != wbmm::environment::QueryStatus::kSuccess ||
          !std::isfinite(query.distance) ||
          (config_.treat_unknown_as_occupied && !query.fully_observed)) {
        reason = "Cannot validate sphere '" + sphere.name + "': " + query.message;
        return;
      }
      if (query.status == wbmm::environment::QueryStatus::kSuccess) {
        min_clearance = std::min(
          min_clearance, query.distance - sphere.radius);
        if (query.distance - sphere.radius <= 0.0) {
          reason = "Sphere '" + sphere.name + "' is in collision at t=" +
            std::to_string(t) + ".";
        }
      }
    };
    for (const auto & sphere : kinematics.base) {
      check(sphere);
    }
    for (const auto & sphere : kinematics.arm) {
      check(sphere);
    }
    if (!reason.empty()) {
      return false;
    }

    const double speed = std::hypot(velocity(0), velocity(1));
    max_base_speed = std::max(max_base_speed, speed);
    // Envelope excess is recorded, not fatal. The envelope terms are soft
    // penalties, so a small overshoot is expected, and the hard guarantee is the
    // caller's time-scaling loop. Only a collision makes the trajectory unsafe.
    if (speed > config_.max_base_speed * (1.0 + config_.envelope_tolerance) &&
      envelope_note.empty())
    {
      envelope_note = "base speed " + std::to_string(speed) +
        " m/s exceeds the limit";
    }

    for (int j = 0; j < joint_count_; ++j) {
      max_joint_speed = std::max(
        max_joint_speed, std::abs(velocity(2 + j)));
    }

    // Yaw rate is the derivative of the emitted heading, consistent with the
    // trajectory builder.
    const double yaw = kinematics.yaw;
    if (!first && speed > 1e-6) {
      const double dt = duration / static_cast<double>(samples);
      double delta = yaw - previous_yaw;
      while (delta > M_PI) {
        delta -= 2.0 * M_PI;
      }
      while (delta < -M_PI) {
        delta += 2.0 * M_PI;
      }
      max_yaw_rate = std::max(max_yaw_rate, std::abs(delta / dt));
    }
    previous_yaw = yaw;
    first = false;
  }

  if (max_joint_speed >
    config_.max_joint_speed * (1.0 + config_.envelope_tolerance) &&
    envelope_note.empty())
  {
    envelope_note = "joint speed " + std::to_string(max_joint_speed) +
      " rad/s exceeds the limit";
  }
  if (min_clearance == std::numeric_limits<double>::max()) {
    min_clearance = 0.0;
  }
  if (!envelope_note.empty()) {
    reason = envelope_note;
  }
  return true;
}

}  // namespace wbmm::traj_opt
