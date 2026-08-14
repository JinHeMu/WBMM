#include "wipe_planner/planner.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <map>
#include <queue>
#include <random>
#include <stdexcept>
#include <tuple>

namespace wipe_planner
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

Eigen::VectorXd yamlVector(const YAML::Node & node)
{
  Eigen::VectorXd result(node.size());
  for (std::size_t i = 0; i < node.size(); ++i) {
    result[static_cast<Eigen::Index>(i)] = node[i].as<double>();
  }
  return result;
}
}  // namespace

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

double wholeBodySquaredError(const Eigen::VectorXd & current,
                             const Eigen::VectorXd & goal)
{
  if (current.size() != goal.size() || current.size() < 3) {
    return std::numeric_limits<double>::infinity();
  }
  Eigen::VectorXd error = current - goal;
  error[2] = wrapAngle(error[2]);
  return error.squaredNorm();
}

double adaptiveProgressRate(double tracking_error, double lag_error,
                            double previous_rate, double min_rate,
                            double max_rate, double filter,
                            double lag_gain, double ahead_gain,
                            double soft_error, double hard_error)
{
  hard_error = std::max(soft_error + 1.0e-6, hard_error);
  const double error_rate = tracking_error <= soft_error ? 1.0 :
    std::clamp((hard_error - tracking_error) /
               (hard_error - soft_error), 0.0, 1.0);
  double raw_rate = 1.0 - lag_gain * std::max(0.0, lag_error) +
    ahead_gain * std::max(0.0, -lag_error);
  if (tracking_error > soft_error) {
    raw_rate = std::min(raw_rate, error_rate);
  }
  raw_rate = std::clamp(raw_rate, min_rate, max_rate);
  filter = std::clamp(filter, 0.0, 1.0);
  return (1.0 - filter) * previous_rate + filter * raw_rate;
}

ForceAdmittance::ForceAdmittance(double desired_force, double gain, double leak,
                                 double max_offset, double filter_alpha)
: desired_force_(desired_force), gain_(gain), leak_(leak),
  max_offset_(std::abs(max_offset)), alpha_(std::clamp(filter_alpha, 0.0, 1.0))
{}

double ForceAdmittance::update(double measured_force, double dt)
{
  measured_force = std::max(0.0, measured_force);
  if (!initialized_) {
    filtered_force_ = measured_force;
    initialized_ = true;
  } else {
    filtered_force_ = alpha_ * measured_force + (1.0 - alpha_) * filtered_force_;
  }
  const double velocity = gain_ * (filtered_force_ - desired_force_) - leak_ * offset_;
  offset_ = std::clamp(offset_ + std::clamp(dt, 0.0, 0.1) * velocity,
                       -max_offset_, max_offset_);
  return offset_;
}

Planner::Planner(const std::string & urdf_file, const std::string & ee_frame,
                 const std::string & task_file)
: urdf_file_(urdf_file)
{
  pinocchio::urdf::buildModel(urdf_file_, model_);
  ee_frame_id_ = model_.getFrameId(ee_frame);
  if (ee_frame_id_ >= model_.frames.size()) {
    throw std::runtime_error("End-effector frame not found: " + ee_frame);
  }
  std::vector<std::string> package_dirs{
    std::filesystem::path(urdf_file_).parent_path().parent_path().parent_path().string()};
  pinocchio::urdf::buildGeom(
    model_, urdf_file_, pinocchio::GeometryType::VISUAL, visual_model_, package_dirs);
  pinocchio::urdf::buildGeom(
    model_, urdf_file_, pinocchio::GeometryType::COLLISION,
    collision_model_, package_dirs);
  // Match REMANI's self-collision philosophy: do not test geometry on the
  // same rigid body or directly connected bodies, but test all other pairs.
  for (std::size_t first = 0; first < collision_model_.geometryObjects.size(); ++first) {
    for (std::size_t second = first + 1;
         second < collision_model_.geometryObjects.size(); ++second)
    {
      const auto first_joint = collision_model_.geometryObjects[first].parentJoint;
      const auto second_joint = collision_model_.geometryObjects[second].parentJoint;
      const bool adjacent = first_joint == second_joint ||
        model_.parents[first_joint] == second_joint ||
        model_.parents[second_joint] == first_joint;
      if (!adjacent) {
        collision_model_.addCollisionPair(
          pinocchio::CollisionPair(first, second));
      }
    }
  }

  const YAML::Node root = YAML::LoadFile(task_file)["wipe_task"];
  const auto surface = root["surface"];
  surface_center_ = yamlVector(surface["center"]);
  normal_ = yamlVector(surface["normal_into_room"]);
  normal_.normalize();
  x_limits_ = yamlVector(surface["x_limits"]);
  z_limits_ = yamlVector(surface["z_limits"]);

  const auto raster = root["raster"];
  raster_rows_ = raster["rows"].as<int>();
  raster_columns_ = raster["columns"].as<int>();
  solver_spacing_ = raster["solver_spacing"].as<double>();
  tangential_speed_ = raster["tangential_speed"].as<double>();
  corner_speed_ = raster["corner_speed"].as<double>();

  const auto contact = root["contact"];
  contact_offset_ = contact["offset"].as<double>();
  desired_force_ = contact["desired_normal_force"].as<double>();

  const auto approach = root["approach"];
  if (approach) {
    approach_clearance_ = approach["clearance"].as<double>();
    approach_speed_ = approach["speed"].as<double>();
    precontact_hold_duration_ = approach["precontact_hold_duration"].as<double>();
  }
  if (approach_clearance_ <= 0.0 || approach_speed_ <= 0.0 ||
      precontact_hold_duration_ < 0.0)
  {
    throw std::runtime_error(
      "Approach clearance/speed must be positive and hold duration non-negative");
  }

  const auto whole_body = root["whole_body"];
  preferred_standoff_ = whole_body["preferred_standoff"].as<double>();
  standoff_limits_ = yamlVector(whole_body["standoff_limits"]);
  candidate_lateral_step_ = whole_body["candidate_lateral_step"].as<double>();
  candidate_longitudinal_range_ = whole_body["candidate_longitudinal_range"].as<double>();
  candidate_longitudinal_step_ = whole_body["candidate_longitudinal_step"].as<double>();
  cleaning_yaw_ = whole_body["cleaning_yaw"].as<double>();
  const Eigen::VectorXd rotation_values = yamlVector(whole_body["desired_tool_rotation"]);
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      desired_tool_rotation_(row, column) = rotation_values[row * 3 + column];
    }
  }

  const auto hybrid = root["hybrid_astar"];
  if (hybrid) {
    hybrid_xy_resolution_ = hybrid["xy_resolution"].as<double>();
    hybrid_yaw_resolution_ = hybrid["yaw_resolution"].as<double>();
    hybrid_primitive_length_ = hybrid["primitive_length"].as<double>();
    hybrid_collision_check_step_ = hybrid["collision_check_step"].as<double>();
    hybrid_goal_position_tolerance_ =
      hybrid["goal_position_tolerance"].as<double>();
    hybrid_goal_yaw_tolerance_ = hybrid["goal_yaw_tolerance"].as<double>();
    hybrid_max_curvature_ = hybrid["max_curvature"].as<double>();
    hybrid_heuristic_weight_ = hybrid["heuristic_weight"].as<double>();
    hybrid_reverse_penalty_ = hybrid["reverse_penalty"].as<double>();
    hybrid_gear_switch_penalty_ = hybrid["gear_switch_penalty"].as<double>();
    hybrid_curvature_penalty_ = hybrid["curvature_penalty"].as<double>();
    hybrid_curvature_change_penalty_ =
      hybrid["curvature_change_penalty"].as<double>();
    hybrid_joint_motion_penalty_ = hybrid["joint_motion_penalty"].as<double>();
    hybrid_search_margin_ = hybrid["search_margin"].as<double>();
    hybrid_standoff_margin_ = hybrid["standoff_margin"].as<double>();
    collision_joint_step_ = hybrid["collision_joint_step"].as<double>();
    hybrid_max_nodes_ = hybrid["max_nodes"].as<int>();
    hybrid_yaw_samples_ = hybrid["yaw_samples"].as<int>();
    hybrid_yaw_sample_step_ = hybrid["yaw_sample_step"].as<double>();
  }

  const auto limits = root["limits"];
  max_base_speed_ = limits["base_speed"].as<double>();
  max_base_acceleration_ = limits["base_acceleration"].as<double>();
  max_angular_speed_ = limits["angular_speed"].as<double>();
  max_angular_acceleration_ = limits["angular_acceleration"].as<double>();
  wheel_separation_ = limits["wheel_separation"].as<double>();
  wheel_radius_ = limits["wheel_radius"].as<double>();
  max_wheel_angular_speed_ = limits["wheel_angular_speed"].as<double>();
  max_joint_speed_ = limits["joint_speed"].as<double>();

  const auto solver = root["solver"];
  ik_position_tolerance_ = solver["position_tolerance"].as<double>();
  ik_axis_tolerance_ = solver["axis_tolerance"].as<double>();
  ik_iterations_ = solver["max_iterations"].as<int>();
  nominal_ik_seed_ = yamlVector(solver["nominal_arm_seed"]);
  if (nominal_ik_seed_.size() != model_.nq) {
    throw std::runtime_error("solver.nominal_arm_seed must match the arm DOF");
  }
  search_ik_seed_ = solver["search_arm_seed"] ?
    yamlVector(solver["search_arm_seed"]) : nominal_ik_seed_;
  if (search_ik_seed_.size() != model_.nq) {
    throw std::runtime_error("solver.search_arm_seed must match the arm DOF");
  }
}

Eigen::Matrix3d Planner::rotationZ(double yaw) const
{
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

void Planner::forward(const Eigen::Vector3d & base, const Eigen::VectorXd & q,
                      Eigen::Vector3d & position, Eigen::Matrix3d & rotation) const
{
  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, q);
  pinocchio::updateFramePlacements(model_, data);
  const auto & placement = data.oMf[ee_frame_id_];
  const Eigen::Matrix3d base_rotation = rotationZ(base.z());
  position = Eigen::Vector3d(base.x(), base.y(), 0.0) +
             base_rotation * placement.translation();
  rotation = base_rotation * placement.rotation();
}

Planner::IkResult Planner::solveIk(const Eigen::Vector3d & base,
                                   const Eigen::Vector3d & target,
                                   const Eigen::VectorXd & previous) const
{
  IkResult result;
  result.q = previous.cwiseMax(model_.lowerPositionLimit +
                               Eigen::VectorXd::Constant(model_.nq, 1.0e-5))
                     .cwiseMin(model_.upperPositionLimit -
                               Eigen::VectorXd::Constant(model_.nq, 1.0e-5));
  const Eigen::VectorXd center = 0.5 * (model_.lowerPositionLimit +
                                        model_.upperPositionLimit);
  const Eigen::VectorXd range = (model_.upperPositionLimit -
                                 model_.lowerPositionLimit).cwiseMax(1.0);
  pinocchio::Data data(model_);
  const Eigen::Matrix3d base_rotation = rotationZ(base.z());
  const Eigen::Vector3d base_translation(base.x(), base.y(), 0.0);
  auto linearize = [&](const Eigen::VectorXd & q, Eigen::VectorXd & value,
                       Eigen::MatrixXd * jacobian) {
      pinocchio::forwardKinematics(model_, data, q);
      pinocchio::computeJointJacobians(model_, data, q);
      pinocchio::updateFramePlacements(model_, data);
      const auto & placement = data.oMf[ee_frame_id_];
      const Eigen::Vector3d position = base_translation +
        base_rotation * placement.translation();
      const Eigen::Matrix3d actual_rotation =
        base_rotation * placement.rotation();
      const Eigen::Vector3d actual_x_axis = actual_rotation.col(0);
      const Eigen::Vector3d actual_z_axis = actual_rotation.col(2);
      value.resize(15);
      value.segment<3>(0) = 35.0 * (position - target);
      value.segment<3>(3) = 18.0 *
        desired_tool_rotation_.col(2).cross(actual_z_axis);
      value.segment<3>(6) = 12.0 *
        desired_tool_rotation_.col(0).cross(actual_x_axis);
      value.segment(9, model_.nq) = 0.035 * (q - previous) +
        0.004 * (q - center).cwiseQuotient(range);
      if (jacobian == nullptr) {
        return;
      }
      Eigen::MatrixXd frame_jacobian = Eigen::MatrixXd::Zero(6, model_.nv);
      pinocchio::getFrameJacobian(model_, data, ee_frame_id_,
        pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, frame_jacobian);
      jacobian->setZero(15, model_.nq);
      jacobian->topRows<3>() = 35.0 * base_rotation * frame_jacobian.topRows(3);
      const Eigen::MatrixXd angular = base_rotation * frame_jacobian.bottomRows(3);
      for (int column = 0; column < model_.nq; ++column) {
        const Eigen::Vector3d angular_column = angular.col(column);
        const Eigen::Vector3d z_axis_derivative =
          angular_column.cross(actual_z_axis);
        jacobian->block<3, 1>(3, column) =
          18.0 * desired_tool_rotation_.col(2).cross(z_axis_derivative);
        const Eigen::Vector3d x_axis_derivative =
          angular_column.cross(actual_x_axis);
        jacobian->block<3, 1>(6, column) =
          12.0 * desired_tool_rotation_.col(0).cross(x_axis_derivative);
      }
      jacobian->bottomRows(model_.nq) =
        0.035 * Eigen::MatrixXd::Identity(model_.nq, model_.nq);
      jacobian->bottomRows(model_.nq).diagonal().array() +=
        0.004 * range.cwiseInverse().array();
    };

  Eigen::VectorXd initial_residual;
  linearize(result.q, initial_residual, nullptr);
  double damping = 1.0e-4;
  double best_cost = initial_residual.squaredNorm();
  for (int iteration = 0; iteration < ik_iterations_; ++iteration) {
    Eigen::VectorXd r;
    Eigen::MatrixXd jacobian;
    linearize(result.q, r, &jacobian);
    const Eigen::MatrixXd normal_matrix = jacobian.transpose() * jacobian +
      damping * Eigen::MatrixXd::Identity(model_.nq, model_.nq);
    const Eigen::VectorXd delta = -normal_matrix.ldlt().solve(jacobian.transpose() * r);
    if (!delta.allFinite()) {
      break;
    }
    Eigen::VectorXd candidate = result.q + delta.cwiseMax(-0.18).cwiseMin(0.18);
    candidate = candidate.cwiseMax(model_.lowerPositionLimit +
                                    Eigen::VectorXd::Constant(model_.nq, 1.0e-5))
                         .cwiseMin(model_.upperPositionLimit -
                                    Eigen::VectorXd::Constant(model_.nq, 1.0e-5));
    Eigen::VectorXd candidate_residual;
    linearize(candidate, candidate_residual, nullptr);
    const double candidate_cost = candidate_residual.squaredNorm();
    if (candidate_cost < best_cost) {
      result.q = candidate;
      best_cost = candidate_cost;
      damping = std::max(1.0e-7, damping * 0.5);
    } else {
      damping = std::min(1.0e3, damping * 10.0);
    }
    if (delta.norm() < 1.0e-7) {
      break;
    }
  }
  Eigen::Vector3d position;
  Eigen::Matrix3d rotation;
  forward(base, result.q, position, rotation);
  result.position_error = (position - target).norm();
  const Eigen::Matrix3d rotation_error =
    desired_tool_rotation_.transpose() * rotation;
  result.axis_error = std::acos(std::clamp(
    0.5 * (rotation_error.trace() - 1.0), -1.0, 1.0));
  result.success = result.position_error <= ik_position_tolerance_ &&
                   result.axis_error <= ik_axis_tolerance_;
  return result;
}

std::vector<Eigen::Vector3d> Planner::rasterTargets() const
{
  std::vector<Eigen::Vector3d> coarse;
  const Eigen::Vector3d plane = surface_center_ - contact_offset_ * normal_;
  for (int row = 0; row < raster_rows_; ++row) {
    const double z_ratio = raster_rows_ == 1 ? 0.0 :
      static_cast<double>(row) / static_cast<double>(raster_rows_ - 1);
    const double z = z_limits_.x() + z_ratio * (z_limits_.y() - z_limits_.x());
    for (int column = 0; column < raster_columns_; ++column) {
      const int ordered_column = row % 2 == 0 ? column : raster_columns_ - 1 - column;
      const double x_ratio = raster_columns_ == 1 ? 0.0 :
        static_cast<double>(ordered_column) /
        static_cast<double>(raster_columns_ - 1);
      Eigen::Vector3d point = plane;
      point.x() = x_limits_.x() + x_ratio * (x_limits_.y() - x_limits_.x());
      point.z() = z;
      coarse.push_back(point);
    }
  }
  std::vector<Eigen::Vector3d> dense;
  dense.push_back(coarse.front());
  for (std::size_t i = 1; i < coarse.size(); ++i) {
    const double distance = (coarse[i] - coarse[i - 1]).norm();
    const int subdivisions = std::max(1, static_cast<int>(std::ceil(
      distance / std::max(0.02, solver_spacing_))));
    for (int j = 1; j <= subdivisions; ++j) {
      dense.push_back(coarse[i - 1] +
        static_cast<double>(j) / subdivisions * (coarse[i] - coarse[i - 1]));
    }
  }
  return dense;
}

std::vector<Eigen::Vector3d> Planner::reachableBaseCandidates(
  const Eigen::Vector3d & target) const
{
  std::vector<Eigen::Vector3d> candidates;
  for (double standoff = standoff_limits_.x();
       standoff <= standoff_limits_.y() + 1.0e-9;
       standoff += std::max(1.0e-3, candidate_lateral_step_))
  {
    for (double lag = -candidate_longitudinal_range_;
         lag <= candidate_longitudinal_range_ + 1.0e-9;
         lag += std::max(1.0e-3, candidate_longitudinal_step_))
    {
      for (int yaw_sample = -hybrid_yaw_samples_;
           yaw_sample <= hybrid_yaw_samples_; ++yaw_sample)
      {
        candidates.emplace_back(
          target.x() + lag,
          surface_center_.y() + normal_.y() * standoff,
          wrapAngle(cleaning_yaw_ + yaw_sample * hybrid_yaw_sample_step_));
      }
    }
  }
  return candidates;
}

Eigen::Vector3d Planner::propagateArc(
  const Eigen::Vector3d & pose, double curvature, double arc)
{
  Eigen::Vector3d result = pose;
  if (std::abs(curvature) < 1.0e-9) {
    result.x() += arc * std::cos(pose.z());
    result.y() += arc * std::sin(pose.z());
  } else {
    const double next_yaw = pose.z() + curvature * arc;
    result.x() += (std::sin(next_yaw) - std::sin(pose.z())) / curvature;
    result.y() -= (std::cos(next_yaw) - std::cos(pose.z())) / curvature;
    result.z() = next_yaw;
  }
  result.z() = wrapAngle(result.z());
  return result;
}

bool Planner::basePoseValid(const Eigen::Vector3d & base,
                            const Eigen::Vector2d & lower,
                            const Eigen::Vector2d & upper) const
{
  if ((base.head<2>().array() < lower.array()).any() ||
      (base.head<2>().array() > upper.array()).any())
  {
    return false;
  }
  const Eigen::Vector3d base_point(base.x(), base.y(), surface_center_.z());
  const double standoff = normal_.dot(base_point - surface_center_);
  return standoff >= standoff_limits_.x() - hybrid_standoff_margin_ &&
         standoff <= standoff_limits_.y() + hybrid_standoff_margin_;
}

bool Planner::armCollisionFree(const Eigen::VectorXd & q) const
{
  pinocchio::Data data(model_);
  pinocchio::GeometryData geometry_data(collision_model_);
  return !pinocchio::computeCollisions(
    model_, data, collision_model_, geometry_data, q, true);
}

bool Planner::armMotionCollisionFree(const Eigen::VectorXd & first_q,
                                     const Eigen::VectorXd & second_q,
                                     bool allow_colliding_start) const
{
  const double distance = (second_q - first_q).cwiseAbs().maxCoeff();
  const int samples = std::max(1, static_cast<int>(std::ceil(
    distance / std::max(0.01, collision_joint_step_ * 0.5))));
  for (int sample = 0; sample <= samples; ++sample) {
    if (allow_colliding_start && sample == 0) {
      continue;
    }
    const double ratio = static_cast<double>(sample) / samples;
    if (!armCollisionFree(first_q + ratio * (second_q - first_q))) {
      return false;
    }
  }
  return true;
}

std::vector<Eigen::VectorXd> Planner::collisionFreeJointPath(
  const Eigen::VectorXd & start_q, const Eigen::VectorXd & goal_q) const
{
  if (armMotionCollisionFree(start_q, goal_q, true)) {
    return {start_q, goal_q};
  }

  struct JointNode
  {
    Eigen::VectorXd q;
    int parent{-1};
  };
  std::vector<JointNode> tree{{start_q, -1}};
  tree.reserve(5000);
  std::mt19937 generator(42U);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  constexpr double kStep = 0.14;

  auto randomConfiguration = [&]() {
      Eigen::VectorXd q(model_.nq);
      for (int joint = 0; joint < model_.nq; ++joint) {
        q[joint] = model_.lowerPositionLimit[joint] + unit(generator) *
          (model_.upperPositionLimit[joint] - model_.lowerPositionLimit[joint]);
      }
      return q;
    };

  for (int iteration = 0; iteration < 5000; ++iteration) {
    Eigen::VectorXd sample;
    const double selector = unit(generator);
    if (selector < 0.30) {
      sample = goal_q;
    } else if (selector < 0.45) {
      sample = nominal_ik_seed_;
    } else {
      sample = randomConfiguration();
    }

    int nearest = 0;
    double nearest_distance = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0; index < tree.size(); ++index) {
      const double distance = (tree[index].q - sample).squaredNorm();
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = static_cast<int>(index);
      }
    }
    Eigen::VectorXd direction = sample - tree[static_cast<std::size_t>(nearest)].q;
    const double infinity_norm = direction.cwiseAbs().maxCoeff();
    if (infinity_norm > kStep) {
      direction *= kStep / infinity_norm;
    }
    Eigen::VectorXd candidate = tree[static_cast<std::size_t>(nearest)].q + direction;
    candidate = candidate.cwiseMax(model_.lowerPositionLimit +
      Eigen::VectorXd::Constant(model_.nq, 1.0e-5));
    candidate = candidate.cwiseMin(model_.upperPositionLimit -
      Eigen::VectorXd::Constant(model_.nq, 1.0e-5));
    const bool root_escape = nearest == 0 && !armCollisionFree(start_q);
    if (!armCollisionFree(candidate) ||
        !armMotionCollisionFree(tree[static_cast<std::size_t>(nearest)].q,
          candidate, root_escape))
    {
      continue;
    }
    tree.push_back({candidate, nearest});
    const int added = static_cast<int>(tree.size() - 1);
    if (!armMotionCollisionFree(candidate, goal_q)) {
      continue;
    }

    std::vector<Eigen::VectorXd> path{goal_q};
    for (int node = added; node >= 0; node = tree[static_cast<std::size_t>(node)].parent) {
      path.push_back(tree[static_cast<std::size_t>(node)].q);
    }
    std::reverse(path.begin(), path.end());

    // Deterministic shortcut smoothing removes unnecessary RRT bends while
    // preserving the special escape edge from a conservatively colliding start.
    std::vector<Eigen::VectorXd> shortened{path.front()};
    std::size_t current = 0;
    while (current + 1 < path.size()) {
      std::size_t next = path.size() - 1;
      while (next > current + 1 && !armMotionCollisionFree(
          path[current], path[next], current == 0 && !armCollisionFree(start_q)))
      {
        --next;
      }
      shortened.push_back(path[next]);
      current = next;
    }
    return shortened;
  }
  throw std::runtime_error(
    "Could not find a collision-free joint path to the pre-contact pose");
}

bool Planner::wholeBodyMotionCollisionFree(
  const Eigen::Vector3d & first_base, const Eigen::VectorXd & first_q,
  const Eigen::Vector3d & second_base, const Eigen::VectorXd & second_q,
  const Eigen::Vector2d & lower, const Eigen::Vector2d & upper) const
{
  const double base_distance =
    (second_base.head<2>() - first_base.head<2>()).norm();
  const double yaw_distance = std::abs(wrapAngle(second_base.z() - first_base.z()));
  const double joint_distance = (second_q - first_q).cwiseAbs().maxCoeff();
  const int samples = std::max({
    1,
    static_cast<int>(std::ceil(base_distance /
      std::max(1.0e-3, hybrid_collision_check_step_))),
    static_cast<int>(std::ceil(yaw_distance /
      std::max(1.0e-3, hybrid_yaw_resolution_ * 0.5))),
    static_cast<int>(std::ceil(joint_distance /
      std::max(1.0e-3, collision_joint_step_))) });
  for (int sample = 1; sample <= samples; ++sample) {
    const double ratio = static_cast<double>(sample) / samples;
    Eigen::Vector3d base = first_base + ratio * (second_base - first_base);
    base.z() = wrapAngle(first_base.z() + ratio *
      wrapAngle(second_base.z() - first_base.z()));
    const Eigen::VectorXd q = first_q + ratio * (second_q - first_q);
    if (!basePoseValid(base, lower, upper) || !armCollisionFree(q)) {
      return false;
    }
  }
  return true;
}

Planner::SearchResult Planner::hybridAstarSegment(
  const Eigen::Vector3d & start_base, const Eigen::VectorXd & start_q,
  const Eigen::Vector3d & start_target, const Eigen::Vector3d & goal_base,
  const Eigen::Vector3d & goal_target, PlanReport & report) const
{
  struct Node
  {
    Eigen::Vector3d base;
    Eigen::VectorXd q;
    Eigen::Vector3d target;
    double g{0.0};
    double f{0.0};
    double curvature{0.0};
    int direction{0};
    int parent{-1};
    bool closed{false};
    double position_error{0.0};
    double axis_error{0.0};
  };
  using QueueEntry = std::pair<double, int>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>,
    std::greater<QueueEntry>> open;
  std::vector<Node> nodes;
  nodes.reserve(static_cast<std::size_t>(hybrid_max_nodes_));
  std::map<std::tuple<int, int, int>, int> visited;
  const Eigen::Vector2d lower =
    start_base.head<2>().cwiseMin(goal_base.head<2>()) -
    Eigen::Vector2d::Constant(hybrid_search_margin_);
  const Eigen::Vector2d upper =
    start_base.head<2>().cwiseMax(goal_base.head<2>()) +
    Eigen::Vector2d::Constant(hybrid_search_margin_);
  auto key = [&](const Eigen::Vector3d & base) {
      return std::make_tuple(
        static_cast<int>(std::floor(base.x() / hybrid_xy_resolution_)),
        static_cast<int>(std::floor(base.y() / hybrid_xy_resolution_)),
        static_cast<int>(std::floor(
          (wrapAngle(base.z()) + kPi) / hybrid_yaw_resolution_)));
    };
  auto heuristic = [&](const Eigen::Vector3d & base) {
      return (base.head<2>() - goal_base.head<2>()).norm() +
        0.15 * std::abs(wrapAngle(base.z() - goal_base.z()));
    };
  nodes.push_back(Node{start_base, start_q, start_target, 0.0,
    hybrid_heuristic_weight_ * heuristic(start_base)});
  open.emplace(nodes.front().f, 0);
  visited[key(start_base)] = 0;

  auto finish = [&](int parent_index) {
      SearchResult result;
      const Node & parent = nodes[static_cast<std::size_t>(parent_index)];
      IkResult final_ik = solveIk(goal_base, goal_target, parent.q);
      if (!final_ik.success) {
        ++report.reachability_rejections;
        return result;
      }
      if (!wholeBodyMotionCollisionFree(
          parent.base, parent.q, goal_base, final_ik.q, lower, upper))
      {
        ++report.collision_rejections;
        return result;
      }
      std::vector<int> chain;
      for (int index = parent_index; index > 0;
           index = nodes[static_cast<std::size_t>(index)].parent)
      {
        chain.push_back(index);
      }
      std::reverse(chain.begin(), chain.end());
      for (const int index : chain) {
        const Node & node = nodes[static_cast<std::size_t>(index)];
        result.bases.push_back(node.base);
        result.joints.push_back(node.q);
        result.contact_targets.push_back(node.target);
        result.max_position_error = std::max(
          result.max_position_error, node.position_error);
        result.max_axis_error = std::max(result.max_axis_error, node.axis_error);
      }
      if (result.bases.empty() ||
          (result.bases.back() - goal_base).norm() > 1.0e-8)
      {
        result.bases.push_back(goal_base);
        result.joints.push_back(final_ik.q);
        result.contact_targets.push_back(goal_target);
      } else {
        result.bases.back() = goal_base;
        result.joints.back() = final_ik.q;
        result.contact_targets.back() = goal_target;
      }
      result.max_position_error = std::max(
        result.max_position_error, final_ik.position_error);
      result.max_axis_error = std::max(result.max_axis_error, final_ik.axis_error);
      result.cost = parent.g +
        hybrid_joint_motion_penalty_ * (final_ik.q - parent.q).norm();
      result.success = true;
      return result;
    };

  while (!open.empty() &&
         nodes.size() < static_cast<std::size_t>(hybrid_max_nodes_))
  {
    const int current_index = open.top().second;
    open.pop();
    Node & current = nodes[static_cast<std::size_t>(current_index)];
    if (current.closed) {
      continue;
    }
    current.closed = true;
    ++report.hybrid_expanded_nodes;
    const double position_error =
      (current.base.head<2>() - goal_base.head<2>()).norm();
    const double yaw_error = std::abs(wrapAngle(current.base.z() - goal_base.z()));
    if (position_error <= hybrid_goal_position_tolerance_ &&
        yaw_error <= hybrid_goal_yaw_tolerance_)
    {
      SearchResult result = finish(current_index);
      if (result.success) {
        return result;
      }
    }

    const Eigen::Vector2d segment = goal_base.head<2>() - start_base.head<2>();
    const double segment_norm_squared = segment.squaredNorm();
    const std::array<double, 5> curvatures{
      0.0, -0.5 * hybrid_max_curvature_,
      0.5 * hybrid_max_curvature_, -hybrid_max_curvature_,
      hybrid_max_curvature_};
    for (const int direction : {1, -1}) {
      for (const double curvature : curvatures) {
        const double reference_omega = max_base_speed_ * curvature;
        const double left_wheel =
          (max_base_speed_ - 0.5 * wheel_separation_ * reference_omega) /
          wheel_radius_;
        const double right_wheel =
          (max_base_speed_ + 0.5 * wheel_separation_ * reference_omega) /
          wheel_radius_;
        if (std::max(std::abs(left_wheel), std::abs(right_wheel)) >
            max_wheel_angular_speed_ + 1.0e-9 ||
            std::abs(reference_omega) > max_angular_speed_ + 1.0e-9)
        {
          continue;
        }
        const double arc = direction * hybrid_primitive_length_;
        const Eigen::Vector3d next_base =
          propagateArc(current.base, curvature, arc);
        if (!basePoseValid(next_base, lower, upper)) {
          continue;
        }
        double progress = 1.0;
        if (segment_norm_squared > 1.0e-10) {
          progress = std::clamp(
            (next_base.head<2>() - start_base.head<2>()).dot(segment) /
            segment_norm_squared, 0.0, 1.0);
        }
        const Eigen::Vector3d next_target =
          start_target + progress * (goal_target - start_target);
        const IkResult ik = solveIk(next_base, next_target, current.q);
        if (!ik.success) {
          ++report.reachability_rejections;
          continue;
        }
        // As in REMANI, validate the complete motion primitive along its exact
        // arc, not only the endpoint or the straight chord between endpoints.
        bool primitive_collision_free = true;
        const int primitive_samples = std::max({
          1,
          static_cast<int>(std::ceil(std::abs(arc) /
            std::max(1.0e-3, hybrid_collision_check_step_))),
          static_cast<int>(std::ceil((ik.q - current.q).cwiseAbs().maxCoeff() /
            std::max(1.0e-3, collision_joint_step_))) });
        for (int sample = 1; sample <= primitive_samples; ++sample) {
          const double ratio = static_cast<double>(sample) / primitive_samples;
          const Eigen::Vector3d sampled_base = propagateArc(
            current.base, curvature, ratio * arc);
          const Eigen::VectorXd sampled_q = current.q + ratio * (ik.q - current.q);
          if (!basePoseValid(sampled_base, lower, upper) ||
              !armCollisionFree(sampled_q))
          {
            primitive_collision_free = false;
            break;
          }
        }
        if (!primitive_collision_free) {
          ++report.collision_rejections;
          continue;
        }
        double edge_cost = std::abs(arc) *
          (direction > 0 ? 1.0 : hybrid_reverse_penalty_);
        if (current.direction != 0 && current.direction != direction) {
          edge_cost += hybrid_gear_switch_penalty_;
        }
        edge_cost += hybrid_curvature_penalty_ *
          std::abs(curvature) * std::abs(arc);
        edge_cost += hybrid_curvature_change_penalty_ *
          std::abs(curvature - current.curvature);
        edge_cost += hybrid_joint_motion_penalty_ * (ik.q - current.q).norm();
        const double next_g = current.g + edge_cost;
        const auto next_key = key(next_base);
        const auto existing = visited.find(next_key);
        if (existing != visited.end() &&
            nodes[static_cast<std::size_t>(existing->second)].g <= next_g)
        {
          continue;
        }
        Node next;
        next.base = next_base;
        next.q = ik.q;
        next.target = next_target;
        next.g = next_g;
        next.f = next_g + hybrid_heuristic_weight_ * heuristic(next_base);
        next.curvature = curvature;
        next.direction = direction;
        next.parent = current_index;
        next.position_error = ik.position_error;
        next.axis_error = ik.axis_error;
        const int next_index = static_cast<int>(nodes.size());
        nodes.push_back(std::move(next));
        visited[next_key] = next_index;
        open.emplace(nodes.back().f, next_index);
      }
    }
  }
  return {};
}

Planner::BaseCandidate Planner::searchWholeBodyPath(
  const std::vector<Eigen::Vector3d> & targets,
  const Eigen::VectorXd & arm_seed, PlanReport & report) const
{
  if (targets.empty()) {
    throw std::runtime_error("Coverage discretization produced no targets");
  }
  BaseCandidate best_start;
  for (const auto & base : reachableBaseCandidates(targets.front())) {
    IkResult ik;
    for (const Eigen::VectorXd * seed :
         {&nominal_ik_seed_, &search_ik_seed_, &arm_seed})
    {
      IkResult candidate = solveIk(base, targets.front(), *seed);
      if (!candidate.success) {
        ++report.reachability_rejections;
        continue;
      }
      if (!armCollisionFree(candidate.q)) {
        ++report.collision_rejections;
        continue;
      }
      if (!ik.success ||
          candidate.position_error + candidate.axis_error <
          ik.position_error + ik.axis_error)
      {
        ik = std::move(candidate);
      }
    }
    if (!ik.success) {
      continue;
    }
    const Eigen::Vector3d base_point(base.x(), base.y(), surface_center_.z());
    const double standoff = normal_.dot(base_point - surface_center_);
    const double cost = (ik.q - nominal_ik_seed_).squaredNorm() +
      4.0 * std::pow(standoff - preferred_standoff_, 2) +
      20.0 * std::pow(wrapAngle(base.z() - cleaning_yaw_), 2);
    if (cost < best_start.cost) {
      best_start.cost = cost;
      best_start.bases = {base};
      best_start.joints = {ik.q};
      best_start.contact_targets = {targets.front()};
      best_start.max_position_error = ik.position_error;
      best_start.max_axis_error = ik.axis_error;
    }
  }
  if (best_start.bases.empty()) {
    throw std::runtime_error(
      "No collision-free IK solution exists for the first coverage keypoint "
      "(IK rejections=" + std::to_string(report.reachability_rejections) +
      ", collision rejections=" + std::to_string(report.collision_rejections) + ")");
  }

  BaseCandidate result = std::move(best_start);
  for (std::size_t target_index = 1; target_index < targets.size(); ++target_index) {
    SearchResult best_segment;
    for (const auto & goal_base : reachableBaseCandidates(targets[target_index])) {
      SearchResult segment = hybridAstarSegment(
        result.bases.back(), result.joints.back(),
        result.contact_targets.back(), goal_base, targets[target_index], report);
      if (segment.success && segment.cost < best_segment.cost) {
        best_segment = std::move(segment);
      }
    }
    if (!best_segment.success) {
      throw std::runtime_error(
        "IK-constrained Hybrid A* failed at coverage keypoint " +
        std::to_string(target_index) + "/" +
        std::to_string(targets.size() - 1));
    }
    result.cost += best_segment.cost;
    result.max_position_error = std::max(
      result.max_position_error, best_segment.max_position_error);
    result.max_axis_error = std::max(
      result.max_axis_error, best_segment.max_axis_error);
    result.bases.insert(result.bases.end(),
      best_segment.bases.begin(), best_segment.bases.end());
    result.joints.insert(result.joints.end(),
      best_segment.joints.begin(), best_segment.joints.end());
    result.contact_targets.insert(result.contact_targets.end(),
      best_segment.contact_targets.begin(), best_segment.contact_targets.end());
  }
  return result;
}

std::vector<Waypoint> Planner::buildContactTrajectory(
  const std::vector<Eigen::Vector3d> & targets,
  const BaseCandidate & candidate, PlanReport & report) const
{
  (void)targets;
  if (candidate.bases.size() != candidate.joints.size() ||
      candidate.bases.size() != candidate.contact_targets.size())
  {
    throw std::runtime_error("Hybrid A* returned inconsistent whole-body arrays");
  }
  std::vector<Waypoint> result;
  double time = 0.0;
  for (std::size_t i = 0; i < candidate.bases.size(); ++i) {
    if (i > 0) {
      const double tool_distance =
        (candidate.contact_targets[i] - candidate.contact_targets[i - 1]).norm();
      const bool row_change = std::abs(
        candidate.contact_targets[i].z() -
        candidate.contact_targets[i - 1].z()) > 1.0e-6;
      const double task_duration = tool_distance /
        std::max(1.0e-3, row_change ? corner_speed_ : tangential_speed_);
      const double joint_duration =
        (candidate.joints[i] - candidate.joints[i - 1]).cwiseAbs().maxCoeff() /
        std::max(1.0e-3, max_joint_speed_);
      const double base_duration = std::max(
        (candidate.bases[i].head<2>() - candidate.bases[i - 1].head<2>()).norm() /
          std::max(1.0e-3, max_base_speed_),
        std::abs(wrapAngle(candidate.bases[i].z() - candidate.bases[i - 1].z())) /
          std::max(1.0e-3, max_angular_speed_));
      time += std::max({task_duration, joint_duration, base_duration, 0.05});
    }
    Waypoint point;
    point.time = time;
    point.state.resize(model_.nq + 3);
    point.state << candidate.bases[i], candidate.joints[i];
    point.input = Eigen::VectorXd::Zero(model_.nq + 2);
    point.contact_target = candidate.contact_targets[i];
    point.in_contact = true;
    result.push_back(std::move(point));
  }
  report.max_position_error = candidate.max_position_error;
  report.max_axis_error = candidate.max_axis_error;
  report.ik_failures = candidate.failures;
  return result;
}

std::vector<Waypoint> Planner::prependNormalApproach(
  const std::vector<Waypoint> & contact_trajectory,
  PlanReport & report) const
{
  if (contact_trajectory.empty() || !contact_trajectory.front().in_contact) {
    throw std::runtime_error("Normal approach requires a contact trajectory");
  }

  const Waypoint & contact_start = contact_trajectory.front();
  const Eigen::Vector3d base = contact_start.state.head<3>();
  const Eigen::Vector3d contact_target = contact_start.contact_target;
  const int steps = std::max(2, static_cast<int>(std::ceil(
    approach_clearance_ / std::max(0.01, solver_spacing_))));

  // Solve from contact outwards. This keeps every IK seed in the same branch as
  // the collision-free first wiping pose, then the samples are reversed for
  // execution from the room side towards the wall.
  std::vector<Eigen::VectorXd> outward_joints;
  outward_joints.reserve(static_cast<std::size_t>(steps + 1));
  outward_joints.push_back(contact_start.state.tail(model_.nq));
  Eigen::VectorXd previous = outward_joints.front();
  for (int step = 1; step <= steps; ++step) {
    const double distance = approach_clearance_ *
      static_cast<double>(step) / static_cast<double>(steps);
    const Eigen::Vector3d target = contact_target + distance * normal_;
    const IkResult ik = solveIk(base, target, previous);
    if (!ik.success || !armCollisionFree(ik.q)) {
      throw std::runtime_error(
        "No collision-free wall-normal pre-contact IK at clearance " +
        std::to_string(distance) + " m");
    }
    previous = ik.q;
    outward_joints.push_back(previous);
    report.max_position_error = std::max(report.max_position_error, ik.position_error);
    report.max_axis_error = std::max(report.max_axis_error, ik.axis_error);
  }

  std::vector<Waypoint> result;
  result.reserve(contact_trajectory.size() + outward_joints.size() + 1);
  const double cartesian_segment_duration = approach_clearance_ /
    (static_cast<double>(steps) * approach_speed_);
  double approach_time = 0.0;
  for (int index = steps; index >= 1; --index) {
    if (index < steps) {
      const double joint_duration =
        (outward_joints[static_cast<std::size_t>(index + 1)] -
         outward_joints[static_cast<std::size_t>(index)])
        .cwiseAbs().maxCoeff() / std::max(1.0e-3, max_joint_speed_);
      approach_time += std::max(cartesian_segment_duration, joint_duration);
    }
    Waypoint point;
    point.time = index == steps ? 0.0 :
      precontact_hold_duration_ + approach_time;
    point.state.resize(model_.nq + 3);
    point.state << base, outward_joints[static_cast<std::size_t>(index)];
    point.input = Eigen::VectorXd::Zero(model_.nq + 2);
    point.contact_target = contact_target +
      approach_clearance_ * static_cast<double>(index) /
      static_cast<double>(steps) * normal_;
    point.in_contact = false;
    result.push_back(std::move(point));

    // Duplicate the farthest pre-contact state so the robot settles with the
    // tool already wall-normal before any motion towards the wall begins.
    if (index == steps && precontact_hold_duration_ > 1.0e-6) {
      Waypoint hold = result.back();
      hold.time = precontact_hold_duration_;
      result.push_back(std::move(hold));
    }
  }

  const double final_joint_duration =
    (outward_joints[1] - outward_joints[0]).cwiseAbs().maxCoeff() /
    std::max(1.0e-3, max_joint_speed_);
  const double contact_start_time = precontact_hold_duration_ + approach_time +
    std::max(cartesian_segment_duration, final_joint_duration);
  for (const Waypoint & contact : contact_trajectory) {
    Waypoint shifted = contact;
    shifted.time += contact_start_time;
    result.push_back(std::move(shifted));
  }
  return result;
}

std::vector<Waypoint> Planner::prependMeasuredAlignment(
  const std::vector<Waypoint> & task_trajectory,
  const Eigen::VectorXd & measured_state, double max_joint_speed,
  PlanReport & report) const
{
  if (task_trajectory.empty() || measured_state.size() != model_.nq + 3) {
    throw std::runtime_error("Measured alignment received an invalid trajectory/state");
  }
  max_joint_speed = std::max(1.0e-3, std::abs(max_joint_speed));
  const Eigen::VectorXd target_q = task_trajectory.front().state.tail(model_.nq);
  const Eigen::VectorXd measured_q = measured_state.tail(model_.nq);
  const double max_delta = (target_q - measured_q).cwiseAbs().maxCoeff();
  if (max_delta <= 1.0e-3) {
    return task_trajectory;
  }

  const auto joint_path = collisionFreeJointPath(measured_q, target_q);
  std::vector<Waypoint> result;
  result.reserve(task_trajectory.size() + joint_path.size() * 8U);
  const Eigen::Vector3d target_base = task_trajectory.front().state.head<3>();
  double duration = 0.0;
  for (std::size_t segment = 1; segment < joint_path.size(); ++segment) {
    const Eigen::VectorXd delta = joint_path[segment] - joint_path[segment - 1];
    const double segment_duration = std::max(
      0.20, 1.5 * delta.cwiseAbs().maxCoeff() / max_joint_speed);
    const int steps = std::max(2, static_cast<int>(std::ceil(
      delta.cwiseAbs().maxCoeff() / std::max(0.01, collision_joint_step_))));
    for (int step = segment == 1 ? 0 : 1; step <= steps; ++step) {
      const double u = static_cast<double>(step) / static_cast<double>(steps);
      const double smooth = u * u * (3.0 - 2.0 * u);
      Waypoint point;
      point.time = duration + u * segment_duration;
      point.state.resize(model_.nq + 3);
      point.state << target_base, joint_path[segment - 1] + smooth * delta;
      point.input = Eigen::VectorXd::Zero(model_.nq + 2);
      point.input.tail(model_.nq) =
        (6.0 * u * (1.0 - u) / segment_duration) * delta;
      point.contact_target = task_trajectory.front().contact_target;
      point.in_contact = false;
      result.push_back(std::move(point));
    }
    duration += segment_duration;
  }
  for (std::size_t index = 1; index < task_trajectory.size(); ++index) {
    Waypoint shifted = task_trajectory[index];
    shifted.time += duration;
    result.push_back(std::move(shifted));
  }
  report.points = result.size();
  report.duration = result.back().time;
  return result;
}

void Planner::fillFeedforward(std::vector<Waypoint> & trajectory,
                              PlanReport & report) const
{
  for (std::size_t i = 0; i + 1 < trajectory.size(); ++i) {
    const double dt = std::max(1.0e-6, trajectory[i + 1].time - trajectory[i].time);
    if (trajectory[i].in_contact && trajectory[i + 1].in_contact) {
      const double dx = trajectory[i + 1].state[0] - trajectory[i].state[0];
      const double dy = trajectory[i + 1].state[1] - trajectory[i].state[1];
      const double yaw = trajectory[i].state[2];
      const double delta_yaw = wrapAngle(
        trajectory[i + 1].state[2] - trajectory[i].state[2]);
      const double forward_chord = std::cos(yaw) * dx + std::sin(yaw) * dy;
      const double arc = std::abs(delta_yaw) < 1.0e-7 ? forward_chord :
        forward_chord * delta_yaw / std::sin(delta_yaw);
      trajectory[i].input[0] = arc / dt;
      trajectory[i].input[1] = delta_yaw / dt;
    }
    trajectory[i].input.tail(model_.nq) =
      (trajectory[i + 1].state.tail(model_.nq) -
       trajectory[i].state.tail(model_.nq)) / dt;
    const double v = trajectory[i].input[0];
    const double omega = trajectory[i].input[1];
    const double dx = trajectory[i + 1].state[0] - trajectory[i].state[0];
    const double dy = trajectory[i + 1].state[1] - trajectory[i].state[1];
    // The chord of an exact constant-curvature differential-drive arc points
    // along the midpoint heading. This residual tests lateral slip without
    // mistaking finite-difference error from a varying speed profile for slip.
    const double midpoint_yaw = trajectory[i].state[2] + 0.5 * wrapAngle(
      trajectory[i + 1].state[2] - trajectory[i].state[2]);
    const double lateral = std::abs(
      -std::sin(midpoint_yaw) * dx + std::cos(midpoint_yaw) * dy) / dt;
    report.max_lateral_velocity = std::max(report.max_lateral_velocity,
                                           std::abs(lateral));
    const double left = (v - 0.5 * wheel_separation_ * omega) / wheel_radius_;
    const double right = (v + 0.5 * wheel_separation_ * omega) / wheel_radius_;
    report.max_wheel_speed = std::max(
      report.max_wheel_speed, std::max(std::abs(left), std::abs(right)));
  }
  if (report.max_lateral_velocity > 1.0e-5) {
    throw std::runtime_error("Generated trajectory violates the no-lateral-slip constraint");
  }
  if (report.max_wheel_speed > max_wheel_angular_speed_ + 1.0e-4) {
    throw std::runtime_error("Generated trajectory violates wheel speed limits");
  }
}

std::vector<Waypoint> Planner::plan(const Eigen::VectorXd & measured_seed,
                                    PlanReport & report) const
{
  if (measured_seed.size() != model_.nq + 3) {
    throw std::runtime_error("Measured seed has the wrong state dimension");
  }
  const auto targets = rasterTargets();
  const BaseCandidate candidate = searchWholeBodyPath(
    targets, measured_seed.tail(model_.nq), report);
  auto result = prependNormalApproach(
    buildContactTrajectory(targets, candidate, report), report);
  fillFeedforward(result, report);
  report.points = result.size();
  report.duration = result.empty() ? 0.0 : result.back().time;
  return result;
}

Eigen::Vector3d Planner::propagateDifferentialDrive(
  const Eigen::Vector3d & pose, double v, double omega, double dt)
{
  Eigen::Vector3d result = pose;
  if (std::abs(omega) < 1.0e-9) {
    result.x() += v * dt * std::cos(pose.z());
    result.y() += v * dt * std::sin(pose.z());
  } else {
    const double next_yaw = pose.z() + omega * dt;
    result.x() += v / omega * (std::sin(next_yaw) - std::sin(pose.z()));
    result.y() -= v / omega * (std::cos(next_yaw) - std::cos(pose.z()));
    result.z() = next_yaw;
  }
  result.z() = wrapAngle(result.z());
  return result;
}

Eigen::VectorXd Planner::forceCorrectedState(const Eigen::VectorXd & state,
                                             double normal_offset,
                                             double max_joint_delta) const
{
  Eigen::VectorXd corrected = state;
  pinocchio::Data data(model_);
  const Eigen::VectorXd q = state.tail(model_.nq);
  pinocchio::forwardKinematics(model_, data, q);
  pinocchio::updateFramePlacements(model_, data);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, model_.nv);
  pinocchio::computeFrameJacobian(model_, data, q, ee_frame_id_,
    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
  const Eigen::Vector3d local_displacement =
    rotationZ(-state[2]) * (normal_ * normal_offset);
  const Eigen::MatrixXd position_jacobian = jacobian.topRows(3);
  const Eigen::VectorXd delta = position_jacobian.transpose() *
    (position_jacobian * position_jacobian.transpose() +
     1.0e-4 * Eigen::Matrix3d::Identity()).ldlt().solve(local_displacement);
  corrected.tail(model_.nq) = (q + delta.cwiseMax(-std::abs(max_joint_delta))
    .cwiseMin(std::abs(max_joint_delta)))
    .cwiseMax(model_.lowerPositionLimit).cwiseMin(model_.upperPositionLimit);
  return corrected;
}

Eigen::Vector3d Planner::framePosition(const Eigen::VectorXd & state,
                                       const std::string & frame) const
{
  const pinocchio::FrameIndex frame_id = model_.getFrameId(frame);
  if (frame_id >= model_.frames.size()) {
    return Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, state.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  return Eigen::Vector3d(state[0], state[1], 0.0) +
    rotationZ(state[2]) * data.oMf[frame_id].translation();
}

Eigen::Matrix3d Planner::frameRotation(const Eigen::VectorXd & state,
                                       const std::string & frame) const
{
  const pinocchio::FrameIndex frame_id = model_.getFrameId(frame);
  if (frame_id >= model_.frames.size()) {
    return Eigen::Matrix3d::Constant(std::numeric_limits<double>::quiet_NaN());
  }
  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, state.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  return rotationZ(state[2]) * data.oMf[frame_id].rotation();
}

std::vector<VisualGeometry> Planner::visualGeometry(const Eigen::VectorXd & state) const
{
  pinocchio::Data data(model_);
  pinocchio::GeometryData geometry_data(visual_model_);
  pinocchio::forwardKinematics(model_, data, state.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  pinocchio::updateGeometryPlacements(model_, data, visual_model_, geometry_data);
  const Eigen::Matrix3d base_rotation = rotationZ(state[2]);
  const Eigen::Vector3d base_translation(state[0], state[1], 0.0);
  std::vector<VisualGeometry> result;
  result.reserve(visual_model_.geometryObjects.size());
  for (std::size_t i = 0; i < visual_model_.geometryObjects.size(); ++i) {
    const auto & geometry = visual_model_.geometryObjects[i];
    const auto & local = geometry_data.oMg[i];
    VisualGeometry visual;
    visual.name = geometry.name;
    visual.mesh_path = "file://" + std::filesystem::absolute(geometry.meshPath).string();
    visual.mesh_scale = geometry.meshScale;
    visual.color = geometry.meshColor;
    visual.position = base_translation + base_rotation * local.translation();
    visual.orientation = Eigen::Quaterniond(base_rotation * local.rotation());
    result.push_back(std::move(visual));
  }
  return result;
}

}  // namespace wipe_planner
