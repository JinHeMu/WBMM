#include "ta_wbmp/planner.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <yaml-cpp/yaml.h>

#include <hpp/fcl/shape/geometric_shapes.h>

#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <limits>
#include <map>
#include <queue>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace ta_wbmp
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

Eigen::VectorXd yamlVector(const YAML::Node & node)
{
  Eigen::VectorXd result(static_cast<Eigen::Index>(node.size()));
  for (std::size_t index = 0; index < node.size(); ++index) {
    result[static_cast<Eigen::Index>(index)] = node[index].as<double>();
  }
  return result;
}

std::vector<double> yamlStdVector(const YAML::Node & node)
{
  std::vector<double> result;
  result.reserve(node.size());
  for (const auto & value : node) {
    result.push_back(value.as<double>());
  }
  return result;
}

struct AstarEntry
{
  double priority{0.0};
  double cost{0.0};
  std::pair<int, int> cell;

  bool operator<(const AstarEntry & other) const
  {
    return priority > other.priority;
  }
};
}  // namespace

double wrapAngle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

TaskAwarePlanner::TaskAwarePlanner(
  const std::string & urdf_file, const std::string & ee_frame,
  const std::string & task_file,
  std::shared_ptr<const CandidateCostEvaluator> cost,
  std::shared_ptr<const WholeBodyStateValidityChecker> validity,
  std::shared_ptr<const NavigationCostEstimator> nav_cost)
{
  const bool custom_cost = static_cast<bool>(cost);
  cost_evaluator_ = cost ? std::move(cost) :
    std::make_shared<WeightedCandidateCost>();
  navigation_cost_estimator_ = nav_cost ? std::move(nav_cost) :
    std::make_shared<Se2NavigationCostEstimator>();
  task_trajectory_ = TaskTrajectoryGenerator(task_file).generate();
  urdf_file_ = urdf_file;
  pinocchio::urdf::buildModel(urdf_file_, model_);
  if (model_.nq != 6 || model_.nv != 6) {
    throw std::runtime_error(
      "TA-WBMP demo expects a 6-DOF arm, got nq=" +
      std::to_string(model_.nq) + ", nv=" + std::to_string(model_.nv));
  }
  ee_frame_id_ = model_.getFrameId(ee_frame);
  if (ee_frame_id_ >= model_.frames.size()) {
    throw std::runtime_error("End-effector frame not found: " + ee_frame);
  }
  const std::vector<std::string> package_dirs{
    std::filesystem::path(urdf_file_).parent_path().parent_path()
    .parent_path().string()};
  pinocchio::urdf::buildGeom(
    model_, urdf_file_, pinocchio::GeometryType::VISUAL,
    visual_model_, package_dirs);
  pinocchio::urdf::buildGeom(
    model_, urdf_file_, pinocchio::GeometryType::COLLISION,
    collision_model_, package_dirs);
  validity_checker_ = validity ? std::move(validity) :
    std::make_shared<UrdfSelfCollisionStateValidityChecker>(urdf_file_);

  const YAML::Node task = YAML::LoadFile(task_file)["task"];
  if (!task) {
    throw std::runtime_error("Task YAML must contain a 'task' mapping");
  }
  task_name_ = task_trajectory_.name;
  frame_id_ = task_trajectory_.frame_id;

  const auto surface = task["surface"];
  surface_type_ = task_trajectory_.geometry.surface_type;
  surface_center_ = task_trajectory_.geometry.center;
  surface_normal_ = task_trajectory_.geometry.normal;
  surface_radius_ = task_trajectory_.geometry.radius;
  surface_parameter_limits_ = task_trajectory_.geometry.u_limits;
  double minimum_x = std::numeric_limits<double>::infinity();
  double maximum_x = -std::numeric_limits<double>::infinity();
  double minimum_z = std::numeric_limits<double>::infinity();
  double maximum_z = -std::numeric_limits<double>::infinity();
  for (const TaskWaypoint & point : task_trajectory_.points) {
    minimum_x = std::min(minimum_x, point.position.x());
    maximum_x = std::max(maximum_x, point.position.x());
    minimum_z = std::min(minimum_z, point.position.z());
    maximum_z = std::max(maximum_z, point.position.z());
  }
  x_limits_ << minimum_x, maximum_x;
  z_limits_ << minimum_z, maximum_z;
  if (surface_type_ == "cylindrical") {
    if (surface_radius_ <= 0.0) {
      throw std::runtime_error("Cylindrical surface radius must be positive");
    }
    surface_normal_ = surfaceNormal(0.0);
  } else if (surface_type_ == "planar") {
    if (surface_normal_.norm() < 1.0e-9) {
      throw std::runtime_error("Surface normal must be non-zero");
    }
    surface_normal_.normalize();
  } else {
    throw std::runtime_error("Unsupported surface type: " + surface_type_);
  }

  const auto pattern = task["pattern"];
  raster_rows_ = pattern["rows"] ? pattern["rows"].as<int>() : 1;
  raster_columns_ = pattern["columns"] ? pattern["columns"].as<int>() : 2;
  sample_spacing_ = pattern["sample_spacing"].as<double>();
  tangential_speed_ = pattern["tangential_speed"].as<double>();
  row_change_speed_ = pattern["row_change_speed"] ?
    pattern["row_change_speed"].as<double>() : tangential_speed_;

  const auto approach = task["approach"];
  approach_clearance_ = approach["clearance"].as<double>();
  approach_speed_ = approach["speed"].as<double>();
  hold_duration_ = approach["hold_duration"].as<double>();

  const auto constraints = task["constraints"];
  standoff_samples_ = yamlStdVector(constraints["standoff_samples"]);
  offset_samples_ = yamlStdVector(
    constraints["longitudinal_offset_samples"]);
  task_yaw_ = constraints["task_yaw"].as<double>();
  base_policy_ = constraints["base_policy"] ?
    constraints["base_policy"].as<std::string>() : "follow_task";
  if (base_policy_ != "follow_task" && base_policy_ != "fixed") {
    throw std::runtime_error(
      "constraints.base_policy must be 'follow_task' or 'fixed'");
  }
  if (constraints["base_standoff_direction"]) {
    base_standoff_direction_ = yamlVector(
      constraints["base_standoff_direction"]);
    base_standoff_direction_.z() = 0.0;
    if (base_standoff_direction_.head<2>().norm() < 1.0e-9) {
      throw std::runtime_error(
        "constraints.base_standoff_direction must have a horizontal component");
    }
    base_standoff_direction_.normalize();
    fixed_base_standoff_direction_ = true;
  }
  position_tolerance_ = constraints["max_position_error"].as<double>();
  axis_tolerance_ = constraints["max_axis_error"].as<double>();
  minimum_margin_ =
    constraints["minimum_normalized_joint_margin"].as<double>();
  minimum_manipulability_ =
    constraints["minimum_manipulability"].as<double>();
  const Eigen::VectorXd rotation_values =
    yamlVector(constraints["desired_tool_rotation"]);
  if (rotation_values.size() != 9) {
    throw std::runtime_error("desired_tool_rotation must contain 9 values");
  }
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      desired_rotation_(row, column) = rotation_values[row * 3 + column];
    }
  }

  if (constraints["cost_weights"]) {
    const YAML::Node weights = constraints["cost_weights"];
    CandidateCostWeights values;
    const auto read = [&weights](const char * name, double fallback) {
        return weights[name] ? weights[name].as<double>() : fallback;
      };
    values.position_error = read("position_error", values.position_error);
    values.axis_error = read("axis_error", values.axis_error);
    values.arm_path = read("arm_path", values.arm_path);
    values.base_path = read("base_path", values.base_path);
    values.inverse_joint_margin = read(
      "inverse_joint_margin", values.inverse_joint_margin);
    values.inverse_manipulability = read(
      "inverse_manipulability", values.inverse_manipulability);
    values.inverse_min_sigma = read(
      "inverse_min_sigma", values.inverse_min_sigma);
    values.standoff_deviation = read(
      "standoff_deviation", values.standoff_deviation);
    values.longitudinal_offset = read(
      "longitudinal_offset", values.longitudinal_offset);
    values.navigation_cost = read(
      "navigation_cost", values.navigation_cost);
    values.preferred_standoff = read(
      "preferred_standoff", values.preferred_standoff);
    if (!custom_cost) {
      cost_evaluator_ = std::make_shared<WeightedCandidateCost>(values);
    }
  }

  const auto navigation = task["navigation"];
  initial_state_ = yamlVector(navigation["initial_state"]);
  if (initial_state_.size() != model_.nq + 3) {
    throw std::runtime_error("navigation.initial_state must contain 9 values");
  }
  grid_resolution_ = navigation["grid_resolution"].as<double>();
  robot_radius_ = navigation["robot_radius"].as<double>();
  base_speed_ = navigation["base_speed"].as<double>();
  angular_speed_ = navigation["angular_speed"].as<double>();
  joint_speed_ = navigation["joint_speed"].as<double>();
  sample_period_ = navigation["sample_period"].as<double>();
  for (const auto & obstacle : navigation["obstacles"]) {
    obstacles_.emplace_back(
      obstacle["x"].as<double>(), obstacle["y"].as<double>(),
      obstacle["radius"].as<double>());
  }

  const auto solver = task["solver"];
  max_iterations_ = solver["max_iterations"].as<int>();
  damping_ = solver["damping"].as<double>();
  max_joint_step_ = solver["max_joint_step"].as<double>();
  nominal_seed_ = yamlVector(solver["nominal_arm_seed"]);
  search_seed_ = yamlVector(solver["search_arm_seed"]);
  if (nominal_seed_.size() != model_.nq || search_seed_.size() != model_.nq) {
    throw std::runtime_error("Solver seeds must match the arm DOF");
  }

  lower_ = model_.lowerPositionLimit;
  upper_ = model_.upperPositionLimit;
  for (Eigen::Index index = 0; index < lower_.size(); ++index) {
    if (!std::isfinite(lower_[index])) {
      lower_[index] = -2.0 * kPi;
    }
    if (!std::isfinite(upper_[index])) {
      upper_[index] = 2.0 * kPi;
    }
  }
}

Eigen::Matrix3d TaskAwarePlanner::rotationZ(double yaw) const
{
  const double cosine = std::cos(yaw);
  const double sine = std::sin(yaw);
  Eigen::Matrix3d result;
  result << cosine, -sine, 0.0,
    sine, cosine, 0.0,
    0.0, 0.0, 1.0;
  return result;
}

Plan TaskAwarePlanner::plan()
{
  const auto samples = taskSamples();
  int candidate_count = 0;
  int feasible_count = 0;
  std::vector<CandidateMetrics> evaluations;
  const auto selected = selectCandidate(
    samples, candidate_count, feasible_count, evaluations);
  if (!selected) {
    std::map<std::string, int> failures;
    for (const CandidateMetrics & evaluation : evaluations) {
      ++failures[evaluation.failure_reason];
    }
    std::string detail;
    for (const auto & failure : failures) {
      detail += (detail.empty() ? "" : ", ") + failure.first + "=" +
        std::to_string(failure.second);
    }
    throw std::runtime_error(
      "No task-feasible whole-body candidate satisfies the complete future "
      "contact path (" + detail + ")");
  }

  const auto navigation = gridAstar(
    initial_state_.head<2>(), selected->bases.front().head<2>());
  auto waypoints = timeParameterizeNavigation(
    navigation, selected->bases.front());
  appendAlignment(waypoints, selected->precontact_joint);
  appendApproach(
    waypoints, selected->bases.front(), selected->approach_joints,
    selected->approach_targets);
  appendTask(waypoints, *selected, samples);

  Plan result;
  result.waypoints = std::move(waypoints);
  result.task_trajectory = task_trajectory_;
  result.candidate_evaluations = std::move(evaluations);
  result.remani_navigation_goal.resize(model_.nq + 3);
  result.remani_navigation_goal <<
    selected->bases.front(), selected->precontact_joint;
  result.task_entry_state.resize(model_.nq + 3);
  result.task_entry_state << selected->bases.front(), selected->joints.front();
  const auto task_begin = std::find_if(
    result.waypoints.begin(), result.waypoints.end(),
    [](const Waypoint & waypoint) {return waypoint.phase == kPhaseTask;});
  result.task_start_index = static_cast<std::size_t>(
    std::distance(result.waypoints.begin(), task_begin));
  const auto execution_begin = std::find_if(
    result.waypoints.rbegin(), result.waypoints.rend(),
    [](const Waypoint & waypoint) {
      return waypoint.phase == kPhasePrecontactAlign;
    });
  result.execution_start_index = execution_begin == result.waypoints.rend() ?
    result.task_start_index : static_cast<std::size_t>(
    std::distance(result.waypoints.begin(), execution_begin.base()) - 1);
  result.task_targets.reserve(samples.size());
  result.task_normals.reserve(samples.size());
  for (const auto & sample : samples) {
    result.task_targets.push_back(sample.position);
    result.task_normals.push_back(sample.normal);
  }
  result.obstacles = obstacles_;
  result.surface_center = surface_center_;
  result.surface_normal = surface_normal_;
  result.surface_axis_u = task_trajectory_.geometry.axis_u;
  result.surface_axis_v = task_trajectory_.geometry.axis_v;
  result.surface_u_limits = task_trajectory_.geometry.u_limits;
  result.surface_v_limits = task_trajectory_.geometry.v_limits;
  result.surface_local_coordinates =
    task_trajectory_.geometry.local_coordinates;
  result.x_limits = x_limits_;
  result.z_limits = z_limits_;
  result.surface_parameter_limits = surface_parameter_limits_;
  result.surface_type = surface_type_;
  result.surface_radius = surface_radius_;
  result.frame_id = frame_id_;
  result.report = validate(
    result.waypoints, *selected, candidate_count, feasible_count);
  return result;
}

Eigen::Vector3d TaskAwarePlanner::toolPosition(
  const Eigen::VectorXd & state) const
{
  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, state.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  return Eigen::Vector3d(state[0], state[1], 0.0) +
    rotationZ(state[2]) * data.oMf[ee_frame_id_].translation();
}

Eigen::Matrix3d TaskAwarePlanner::toolRotation(
  const Eigen::VectorXd & state) const
{
  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, state.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  return rotationZ(state[2]) * data.oMf[ee_frame_id_].rotation();
}

std::vector<Eigen::Vector3d> TaskAwarePlanner::jointPoints(
  const Eigen::VectorXd & state) const
{
  pinocchio::Data data(model_);
  pinocchio::forwardKinematics(model_, data, state.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  const Eigen::Matrix3d rotation = rotationZ(state[2]);
  const Eigen::Vector3d origin(state[0], state[1], 0.0);
  std::vector<Eigen::Vector3d> result;
  result.emplace_back(origin + rotation * Eigen::Vector3d(0.0, 0.0, 0.50));
  for (pinocchio::JointIndex joint = 1;
    joint < static_cast<pinocchio::JointIndex>(model_.njoints); ++joint)
  {
    result.emplace_back(origin + rotation * data.oMi[joint].translation());
  }
  result.emplace_back(toolPosition(state));
  return result;
}

std::vector<VisualGeometry> TaskAwarePlanner::visualGeometry(
  const Eigen::VectorXd & state) const
{
  pinocchio::Data data(model_);
  pinocchio::GeometryData geometry_data(visual_model_);
  pinocchio::forwardKinematics(model_, data, state.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  pinocchio::updateGeometryPlacements(
    model_, data, visual_model_, geometry_data);
  const Eigen::Matrix3d base_rotation = rotationZ(state[2]);
  const Eigen::Vector3d base_translation(state[0], state[1], 0.0);
  std::vector<VisualGeometry> result;
  result.reserve(visual_model_.geometryObjects.size());
  for (std::size_t index = 0;
    index < visual_model_.geometryObjects.size(); ++index)
  {
    const auto & geometry = visual_model_.geometryObjects[index];
    const auto & local = geometry_data.oMg[index];
    VisualGeometry visual;
    visual.name = geometry.name;
    visual.mesh_path = "file://" +
      std::filesystem::absolute(geometry.meshPath).string();
    visual.mesh_scale = geometry.meshScale;
    visual.position = base_translation +
      base_rotation * local.translation();
    visual.orientation = Eigen::Quaterniond(
      base_rotation * local.rotation());
    result.push_back(std::move(visual));
  }
  return result;
}

std::vector<CollisionSphereGeometry> TaskAwarePlanner::collisionSpheres(
  const Eigen::VectorXd & state) const
{
  pinocchio::Data data(model_);
  pinocchio::GeometryData geometry_data(collision_model_);
  pinocchio::forwardKinematics(model_, data, state.tail(model_.nq));
  pinocchio::updateFramePlacements(model_, data);
  pinocchio::updateGeometryPlacements(
    model_, data, collision_model_, geometry_data);
  const Eigen::Matrix3d base_rotation = rotationZ(state[2]);
  const Eigen::Vector3d base_translation(state[0], state[1], 0.0);
  std::vector<CollisionSphereGeometry> result;
  result.reserve(collision_model_.geometryObjects.size());
  for (std::size_t index = 0;
    index < collision_model_.geometryObjects.size(); ++index)
  {
    const auto & geometry = collision_model_.geometryObjects[index];
    const auto sphere = std::dynamic_pointer_cast<hpp::fcl::Sphere>(
      geometry.geometry);
    if (!sphere) {
      continue;
    }
    const auto & local = geometry_data.oMg[index];
    CollisionSphereGeometry value;
    value.name = geometry.name;
    value.center = base_translation + base_rotation * local.translation();
    value.radius = sphere->radius;
    value.parent_joint = geometry.parentJoint;
    result.push_back(std::move(value));
  }
  return result;
}

bool TaskAwarePlanner::environmentCollisionChecked() const
{
  return validity_checker_ && validity_checker_->checksEnvironment();
}

Eigen::Vector3d TaskAwarePlanner::surfacePoint(
  double parameter, double z) const
{
  if (surface_type_ == "cylindrical") {
    return Eigen::Vector3d(
      surface_center_.x() + surface_radius_ * std::sin(parameter),
      surface_center_.y() - surface_radius_ * std::cos(parameter), z);
  }
  return Eigen::Vector3d(parameter, surface_center_.y(), z);
}

Eigen::Vector3d TaskAwarePlanner::surfaceNormal(double parameter) const
{
  if (surface_type_ == "cylindrical") {
    return Eigen::Vector3d(
      std::sin(parameter), -std::cos(parameter), 0.0);
  }
  return surface_normal_;
}

Eigen::Matrix3d TaskAwarePlanner::desiredRotation(
  const Eigen::Vector3d & normal) const
{
  if (surface_type_ != "cylindrical") {
    return desired_rotation_;
  }
  const Eigen::Vector3d tool_y(0.0, 0.0, -1.0);
  const Eigen::Vector3d tool_z = -normal.normalized();
  const Eigen::Vector3d tool_x = tool_y.cross(tool_z).normalized();
  Eigen::Matrix3d result;
  result.col(0) = tool_x;
  result.col(1) = tool_y;
  result.col(2) = tool_z;
  return result;
}

std::vector<TaskAwarePlanner::TaskSample> TaskAwarePlanner::taskSamples() const
{
  std::vector<TaskSample> samples;
  samples.reserve(task_trajectory_.points.size());
  for (const TaskWaypoint & waypoint : task_trajectory_.points) {
    TaskSample sample;
    sample.position = waypoint.position;
    sample.normal = waypoint.surface_normal;
    sample.desired_rotation = waypoint.orientation.toRotationMatrix();
    sample.tangent = waypoint.tangent;
    sample.nominal_speed = waypoint.nominal_speed;
    sample.contact = waypoint.contact;
    samples.push_back(std::move(sample));
  }
  return samples;
}

std::optional<TaskAwarePlanner::Candidate> TaskAwarePlanner::selectCandidate(
  const std::vector<TaskSample> & samples,
  int & candidate_count, int & feasible_count,
  std::vector<CandidateMetrics> & evaluations) const
{
  std::optional<Candidate> best;
  candidate_count = 0;
  feasible_count = 0;
  for (const double standoff : standoff_samples_) {
    for (const double offset : offset_samples_) {
      ++candidate_count;
      std::string failure_reason;
      auto candidate = evaluateCandidate(
        candidate_count - 1, samples, standoff, offset, task_yaw_,
        failure_reason);
      if (!candidate) {
        CandidateMetrics rejected;
        rejected.candidate_id = candidate_count - 1;
        rejected.standoff = standoff;
        rejected.longitudinal_offset = offset;
        rejected.yaw_offset = task_yaw_;
        rejected.failure_reason = failure_reason.empty() ?
          "UNKNOWN" : failure_reason;
        evaluations.push_back(std::move(rejected));
        continue;
      }
      ++feasible_count;
      evaluations.push_back(candidate->metrics);
      if (!best || candidate->score < best->score) {
        best = std::move(candidate);
      }
    }
  }
  return best;
}

std::optional<TaskAwarePlanner::Candidate> TaskAwarePlanner::evaluateCandidate(
  int candidate_id, const std::vector<TaskSample> & samples, double standoff,
  double offset, double yaw, std::string & failure_reason) const
{
  Candidate candidate;
  candidate.id = candidate_id;
  candidate.standoff = standoff;
  candidate.offset = offset;
  candidate.bases.reserve(samples.size());
  for (const auto & sample : samples) {
    Eigen::Vector3d standoff_direction = fixed_base_standoff_direction_ ?
      base_standoff_direction_ : sample.normal;
    standoff_direction.z() = 0.0;
    if (standoff_direction.head<2>().norm() < 1.0e-9) {
      standoff_direction = Eigen::Vector3d(0.0, -1.0, 0.0);
    }
    standoff_direction.normalize();
    Eigen::Vector3d tangent;
    if (fixed_base_standoff_direction_) {
      // ``longitudinal_offset`` belongs to the base-entry manifold, not to
      // the tool path. On a horizontal surface the tool tangent may be
      // parallel to the standoff direction; using it here would silently
      // change the actual base/table clearance. Keep the two candidate axes
      // orthogonal whenever the YAML provides an explicit base standoff.
      tangent = Eigen::Vector3d(
        -standoff_direction.y(), standoff_direction.x(), 0.0);
    } else {
      tangent = sample.tangent;
    }
    if (surface_type_ == "cylindrical") {
      // Use the analytic cylinder tangent. A chord between two discretized
      // task samples is not tangent at either endpoint and creates a small
      // artificial lateral-slip residual in the differential-drive path.
      tangent = Eigen::Vector3d(-sample.normal.y(), sample.normal.x(), 0.0);
    }
    tangent.z() = 0.0;
    if (tangent.head<2>().norm() < 1.0e-9) {
      tangent = Eigen::Vector3d(
        -standoff_direction.y(), standoff_direction.x(), 0.0);
    }
    tangent.normalize();
    const Eigen::Vector3d base_position = sample.position +
      standoff * standoff_direction + offset * tangent;
    const double normal_yaw = std::atan2(
      standoff_direction.y(), standoff_direction.x());
    const double curve_tangent_correction =
      surface_type_ == "cylindrical" ?
      std::atan2(offset, surface_radius_ + standoff) : 0.0;
    const Eigen::Vector3d base(
      base_position.x(), base_position.y(),
      wrapAngle(
        normal_yaw + 0.5 * kPi + curve_tangent_correction + yaw));
    candidate.bases.push_back(
      base_policy_ == "fixed" && !candidate.bases.empty() ?
      candidate.bases.front() : base);
  }

  std::vector<IkSolution> first_solutions;
  for (const Eigen::VectorXd * seed :
    std::array<const Eigen::VectorXd *, 3>{
      &search_seed_, &nominal_seed_, &initial_state_})
  {
    const Eigen::VectorXd actual_seed = seed == &initial_state_ ?
      initial_state_.tail(model_.nq) : *seed;
    const auto solution = solveIk(
      candidate.bases.front(), samples.front().position,
      samples.front().desired_rotation, actual_seed);
    if (solution) {
      first_solutions.push_back(*solution);
    }
  }
  if (first_solutions.empty()) {
    failure_reason = "FIRST_TASK_IK";
    return std::nullopt;
  }
  std::sort(
    first_solutions.begin(), first_solutions.end(),
    [](const IkSolution & first, const IkSolution & second) {
      return first.position_error + first.axis_error <
             second.position_error + second.axis_error;
    });

  for (const auto & first_solution : first_solutions) {
    candidate.joints.clear();
    candidate.joints.push_back(first_solution.q);
    candidate.max_position_error = first_solution.position_error;
    candidate.max_axis_error = first_solution.axis_error;
    bool feasible = true;
    for (std::size_t index = 1; index < samples.size(); ++index) {
      const auto solution = solveIk(
        candidate.bases[index], samples[index].position,
        samples[index].desired_rotation, candidate.joints.back());
      if (!solution) {
        feasible = false;
        break;
      }
      candidate.joints.push_back(solution->q);
      candidate.max_position_error = std::max(
        candidate.max_position_error, solution->position_error);
      candidate.max_axis_error = std::max(
        candidate.max_axis_error, solution->axis_error);
    }
    if (!feasible) {
      failure_reason = "TASK_PATH_IK";
      continue;
    }

    const int approach_count = std::max(
      3, static_cast<int>(std::ceil(approach_clearance_ /
      std::max(0.02, sample_spacing_))));
    candidate.approach_targets.clear();
    for (int index = 0; index <= approach_count; ++index) {
      const double ratio = static_cast<double>(approach_count - index) /
        static_cast<double>(approach_count);
      candidate.approach_targets.emplace_back(
        samples.front().position +
        samples.front().normal * approach_clearance_ * ratio);
    }

    std::vector<Eigen::VectorXd> outward_joints{candidate.joints.front()};
    Eigen::VectorXd previous = candidate.joints.front();
    for (int index = approach_count - 1; index >= 0; --index) {
      const auto solution = solveIk(
        candidate.bases.front(),
        candidate.approach_targets[static_cast<std::size_t>(index)],
        samples.front().desired_rotation, previous);
      if (!solution) {
        feasible = false;
        break;
      }
      previous = solution->q;
      outward_joints.push_back(previous);
      candidate.max_position_error = std::max(
        candidate.max_position_error, solution->position_error);
      candidate.max_axis_error = std::max(
        candidate.max_axis_error, solution->axis_error);
    }
    if (!feasible) {
      failure_reason = "PRECONTACT_APPROACH_IK";
      continue;
    }
    candidate.approach_joints.assign(
      outward_joints.rbegin(), outward_joints.rend());
    if (candidate.approach_joints.size() !=
      candidate.approach_targets.size())
    {
      failure_reason = "PRECONTACT_APPROACH_SIZE";
      continue;
    }
    candidate.precontact_joint = candidate.approach_joints.front();

    candidate.minimum_joint_margin = std::numeric_limits<double>::infinity();
    for (const auto & q : candidate.joints) {
      candidate.minimum_joint_margin = std::min(
        candidate.minimum_joint_margin, jointMargin(q));
    }
    for (const auto & q : candidate.approach_joints) {
      candidate.minimum_joint_margin = std::min(
        candidate.minimum_joint_margin, jointMargin(q));
    }
    candidate.minimum_manipulability =
      std::numeric_limits<double>::infinity();
    candidate.minimum_sigma = std::numeric_limits<double>::infinity();
    for (const auto & q : candidate.joints) {
      candidate.minimum_manipulability = std::min(
        candidate.minimum_manipulability, manipulability(q));
      candidate.minimum_sigma = std::min(
        candidate.minimum_sigma, minimumSingularValue(q));
    }
    if (candidate.minimum_joint_margin < minimum_margin_ ||
      candidate.minimum_manipulability < minimum_manipulability_)
    {
      failure_reason = "TASK_QUALITY_HARD_LIMIT";
      continue;
    }

    candidate.arm_path_length = 0.0;
    candidate.base_path_length = 0.0;
    for (std::size_t index = 1; index < candidate.joints.size(); ++index) {
      candidate.arm_path_length +=
        (candidate.joints[index] - candidate.joints[index - 1]).norm();
      candidate.base_path_length +=
        (candidate.bases[index].head<2>() -
        candidate.bases[index - 1].head<2>()).norm();
    }
    candidate.metrics.candidate_id = candidate.id;
    candidate.metrics.standoff = standoff;
    candidate.metrics.longitudinal_offset = offset;
    candidate.metrics.yaw_offset = yaw;
    candidate.metrics.feasible = true;
    candidate.metrics.max_position_error = candidate.max_position_error;
    candidate.metrics.max_axis_error = candidate.max_axis_error;
    candidate.metrics.min_joint_margin = candidate.minimum_joint_margin;
    candidate.metrics.min_manipulability = candidate.minimum_manipulability;
    candidate.metrics.min_sigma = candidate.minimum_sigma;
    candidate.metrics.base_path_length = candidate.base_path_length;
    candidate.metrics.arm_path_length = candidate.arm_path_length;
    Eigen::VectorXd navigation_goal(model_.nq + 3);
    navigation_goal << candidate.bases.front(), candidate.precontact_joint;
    candidate.metrics.navigation_cost_estimate =
      navigation_cost_estimator_->estimate(initial_state_, navigation_goal);

    std::vector<Eigen::VectorXd> checked_states;
    std::vector<std::string> checked_stages;
    checked_states.reserve(
      candidate.approach_joints.size() + candidate.joints.size());
    checked_stages.reserve(checked_states.capacity());
    for (const auto & q : candidate.approach_joints) {
      Eigen::VectorXd state(model_.nq + 3);
      state << candidate.bases.front(), q;
      checked_states.push_back(std::move(state));
      checked_stages.emplace_back("PRECONTACT_APPROACH");
    }
    for (std::size_t index = 0; index < candidate.joints.size(); ++index) {
      Eigen::VectorXd state(model_.nq + 3);
      state << candidate.bases[index], candidate.joints[index];
      checked_states.push_back(std::move(state));
      checked_stages.emplace_back("TASK");
    }
    for (std::size_t index = 0; index < checked_states.size(); ++index) {
      StateValidityResult validity = validity_checker_->check(
        checked_states[index]);
      if (validity.valid && index > 0) {
        validity = checkInterpolatedMotion(
          *validity_checker_, checked_states[index - 1],
          checked_states[index]);
      }
      if (!validity.valid) {
        failure_reason = checked_stages[index] + "_" +
          (validity.reason.empty() ? "STATE_VALIDITY" : validity.reason);
        feasible = false;
        break;
      }
    }
    if (!feasible) {
      continue;
    }
    candidate.score = cost_evaluator_->evaluate(candidate.metrics);
    candidate.metrics.score = candidate.score;
    return candidate;
  }
  if (failure_reason.empty()) {
    failure_reason = "NO_IK_BRANCH_COMPLETED";
  }
  return std::nullopt;
}

std::optional<TaskAwarePlanner::IkSolution> TaskAwarePlanner::solveIk(
  const Eigen::Vector3d & base, const Eigen::Vector3d & target,
  const Eigen::Matrix3d & desired_rotation,
  const Eigen::VectorXd & seed) const
{
  Eigen::VectorXd q = seed.cwiseMax(lower_).cwiseMin(upper_);
  const Eigen::Matrix3d base_rotation = rotationZ(base.z());
  const Eigen::Vector3d target_local = base_rotation.transpose() *
    (target - Eigen::Vector3d(base.x(), base.y(), 0.0));
  const Eigen::Matrix3d desired_local_rotation =
    base_rotation.transpose() * desired_rotation;
  pinocchio::Data data(model_);

  for (int iteration = 0; iteration < max_iterations_; ++iteration) {
    pinocchio::forwardKinematics(model_, data, q);
    pinocchio::updateFramePlacements(model_, data);
    const auto & placement = data.oMf[ee_frame_id_];
    const Eigen::Vector3d position_error_vector =
      target_local - placement.translation();
    const Eigen::Vector3d rotation_error_vector = pinocchio::log3(
      desired_local_rotation * placement.rotation().transpose());
    const double position_error = position_error_vector.norm();
    const double axis_error = std::acos(std::clamp(
      placement.rotation().col(2).dot(desired_local_rotation.col(2)),
      -1.0, 1.0));
    if (position_error <= position_tolerance_ &&
      axis_error <= axis_tolerance_)
    {
      return IkSolution{q, position_error, axis_error};
    }

    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, model_.nv);
    pinocchio::computeFrameJacobian(
      model_, data, q, ee_frame_id_,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
    Eigen::MatrixXd weighted_jacobian = jacobian;
    weighted_jacobian.bottomRows(3) *= 0.45;
    Eigen::VectorXd error(6);
    error << position_error_vector, 0.45 * rotation_error_vector;
    const Eigen::MatrixXd normal =
      weighted_jacobian * weighted_jacobian.transpose() +
      damping_ * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd delta = weighted_jacobian.transpose() *
      normal.ldlt().solve(error);
    const double maximum = delta.cwiseAbs().maxCoeff();
    if (maximum > max_joint_step_) {
      delta *= max_joint_step_ / maximum;
    }
    const Eigen::VectorXd bounded_lower =
      (lower_.array() + 1.0e-5).matrix();
    const Eigen::VectorXd bounded_upper =
      (upper_.array() - 1.0e-5).matrix();
    q = (q + delta).cwiseMax(bounded_lower).cwiseMin(bounded_upper);
  }
  return std::nullopt;
}

double TaskAwarePlanner::jointMargin(const Eigen::VectorXd & q) const
{
  const Eigen::ArrayXd span = (upper_ - lower_).array().max(1.0e-9);
  const Eigen::ArrayXd margin =
    ((q - lower_).array() / span).min((upper_ - q).array() / span);
  return margin.minCoeff();
}

double TaskAwarePlanner::manipulability(const Eigen::VectorXd & q) const
{
  pinocchio::Data data(model_);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, model_.nv);
  pinocchio::computeFrameJacobian(
    model_, data, q, ee_frame_id_,
    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
  const Eigen::VectorXd singular_values =
    jacobian.jacobiSvd().singularValues();
  double result = 1.0;
  for (Eigen::Index index = 0; index < singular_values.size(); ++index) {
    result *= std::max(1.0e-12, singular_values[index]);
  }
  return result;
}

double TaskAwarePlanner::minimumSingularValue(const Eigen::VectorXd & q) const
{
  pinocchio::Data data(model_);
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, model_.nv);
  pinocchio::computeFrameJacobian(
    model_, data, q, ee_frame_id_,
    pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, jacobian);
  const Eigen::VectorXd singular_values =
    jacobian.jacobiSvd().singularValues();
  return singular_values.size() == 0 ? 0.0 : singular_values.minCoeff();
}

std::vector<Eigen::Vector2d> TaskAwarePlanner::gridAstar(
  const Eigen::Vector2d & start, const Eigen::Vector2d & goal) const
{
  double minimum_x = std::min(start.x(), goal.x());
  double maximum_x = std::max(start.x(), goal.x());
  double minimum_y = std::min(start.y(), goal.y());
  double maximum_y = std::max(start.y(), goal.y());
  for (const auto & obstacle : obstacles_) {
    minimum_x = std::min(minimum_x, obstacle.x());
    maximum_x = std::max(maximum_x, obstacle.x());
    minimum_y = std::min(minimum_y, obstacle.y());
    maximum_y = std::max(maximum_y, obstacle.y());
  }
  const Eigen::Vector2d lower(minimum_x - 0.8, minimum_y - 0.8);
  const Eigen::Vector2d upper(maximum_x + 0.8, maximum_y + 0.8);
  const Eigen::Array2i shape =
    ((upper - lower) / grid_resolution_).array().ceil().cast<int>() + 1;

  const auto toCell = [&](const Eigen::Vector2d & point) {
      const Eigen::Array2i cell =
        ((point - lower) / grid_resolution_).array().round().cast<int>();
      return std::make_pair(cell.x(), cell.y());
    };
  const auto toWorld = [&](const std::pair<int, int> & cell) -> Eigen::Vector2d {
      return (lower + grid_resolution_ *
        Eigen::Vector2d(cell.first, cell.second)).eval();
    };
  const auto free = [&](const std::pair<int, int> & cell) {
      if (cell.first < 0 || cell.second < 0 ||
        cell.first >= shape.x() || cell.second >= shape.y())
      {
        return false;
      }
      // Keep a small continuous-path reserve beyond the inflated robot radius.
      // Grid-cell validity alone is insufficient because the rotate-drive
      // parameterization samples between cell centers.
      return pointClearance(toWorld(cell)) > 0.015;
    };

  const auto start_cell = toCell(start);
  const auto goal_cell = toCell(goal);
  if (!free(start_cell) || !free(goal_cell)) {
    throw std::runtime_error(
      "Navigation start or selected task base is occupied");
  }

  std::priority_queue<AstarEntry> frontier;
  frontier.push(AstarEntry{0.0, 0.0, start_cell});
  std::map<std::pair<int, int>, double> costs{{start_cell, 0.0}};
  std::map<std::pair<int, int>, std::pair<int, int>> parents;
  const std::array<std::pair<int, int>, 8> moves{{
    {-1, 0}, {1, 0}, {0, -1}, {0, 1},
    {-1, -1}, {-1, 1}, {1, -1}, {1, 1}}};
  bool reached = false;
  while (!frontier.empty()) {
    const AstarEntry current = frontier.top();
    frontier.pop();
    if (current.cost > costs[current.cell] + 1.0e-9) {
      continue;
    }
    if (current.cell == goal_cell) {
      reached = true;
      break;
    }
    for (const auto & move : moves) {
      const std::pair<int, int> neighbor{
        current.cell.first + move.first,
        current.cell.second + move.second};
      if (!free(neighbor)) {
        continue;
      }
      if (move.first != 0 && move.second != 0 &&
        (!free({current.cell.first + move.first, current.cell.second}) ||
        !free({current.cell.first, current.cell.second + move.second})))
      {
        continue;
      }
      const double new_cost = current.cost +
        std::hypot(move.first, move.second);
      const auto known = costs.find(neighbor);
      if (known != costs.end() && new_cost >= known->second) {
        continue;
      }
      costs[neighbor] = new_cost;
      parents[neighbor] = current.cell;
      const double heuristic = std::hypot(
        neighbor.first - goal_cell.first,
        neighbor.second - goal_cell.second);
      frontier.push(AstarEntry{
        new_cost + heuristic, new_cost, neighbor});
    }
  }
  if (!reached) {
    throw std::runtime_error(
      "A* could not find an obstacle-free navigation path");
  }

  std::vector<std::pair<int, int>> cells{goal_cell};
  while (cells.back() != start_cell) {
    cells.push_back(parents.at(cells.back()));
  }
  std::reverse(cells.begin(), cells.end());
  std::vector<Eigen::Vector2d> points;
  points.reserve(cells.size());
  for (const auto & cell : cells) {
    points.emplace_back(toWorld(cell));
  }
  points.front() = start;
  points.back() = goal;
  return simplifyPolyline(points);
}

std::vector<Eigen::Vector2d> TaskAwarePlanner::simplifyPolyline(
  const std::vector<Eigen::Vector2d> & points) const
{
  if (points.size() <= 2) {
    return points;
  }
  std::vector<Eigen::Vector2d> result{points.front()};
  std::size_t anchor = 0;
  while (anchor + 1 < points.size()) {
    std::size_t selected = anchor + 1;
    for (std::size_t candidate = points.size() - 1;
      candidate > anchor; --candidate)
    {
      if (segmentIsFree(points[anchor], points[candidate])) {
        selected = candidate;
        break;
      }
    }
    result.push_back(points[selected]);
    anchor = selected;
  }
  return result;
}

bool TaskAwarePlanner::segmentIsFree(
  const Eigen::Vector2d & start, const Eigen::Vector2d & goal) const
{
  const Eigen::Vector2d delta = goal - start;
  const int count = std::max(
    1, static_cast<int>(std::ceil(delta.norm() /
    (0.2 * grid_resolution_))));
  for (int index = 0; index <= count; ++index) {
    if (pointClearance(
        start + delta * static_cast<double>(index) /
        static_cast<double>(count)) <= 0.015)
    {
      return false;
    }
  }
  return true;
}

double TaskAwarePlanner::pointClearance(const Eigen::Vector2d & point) const
{
  if (obstacles_.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double clearance = std::numeric_limits<double>::infinity();
  for (const auto & obstacle : obstacles_) {
    clearance = std::min(
      clearance,
      (point - obstacle.head<2>()).norm() - obstacle.z() - robot_radius_);
  }
  return clearance;
}

std::vector<Waypoint> TaskAwarePlanner::timeParameterizeNavigation(
  const std::vector<Eigen::Vector2d> & points,
  const Eigen::Vector3d & goal_base) const
{
  Eigen::VectorXd state = initial_state_;
  std::vector<Waypoint> result{
    Waypoint{0.0, state, kPhaseNavigate, Eigen::Vector3d::Zero(), false}};
  double current_time = 0.0;

  const auto rotateTo = [&](double yaw) {
      const double difference = wrapAngle(yaw - state[2]);
      const double duration = std::abs(difference) /
        std::max(1.0e-6, angular_speed_);
      const int steps = std::max(
        1, static_cast<int>(std::ceil(duration / sample_period_)));
      const double start_yaw = state[2];
      for (int index = 1; index <= steps; ++index) {
        state[2] = wrapAngle(
          start_yaw + difference * static_cast<double>(index) /
          static_cast<double>(steps));
        current_time += duration / static_cast<double>(steps);
        result.push_back(Waypoint{
          current_time, state, kPhaseNavigate,
          Eigen::Vector3d::Zero(), false});
      }
    };

  for (std::size_t point_index = 1; point_index < points.size(); ++point_index) {
    const Eigen::Vector2d delta = points[point_index] - state.head<2>();
    const double distance = delta.norm();
    if (distance < 1.0e-8) {
      continue;
    }
    rotateTo(std::atan2(delta.y(), delta.x()));
    const double duration = distance / std::max(1.0e-6, base_speed_);
    const int steps = std::max(
      1, static_cast<int>(std::ceil(duration / sample_period_)));
    const Eigen::Vector2d start_xy = state.head<2>();
    for (int index = 1; index <= steps; ++index) {
      state.head<2>() = start_xy + delta * static_cast<double>(index) /
        static_cast<double>(steps);
      current_time += duration / static_cast<double>(steps);
      result.push_back(Waypoint{
        current_time, state, kPhaseNavigate,
        Eigen::Vector3d::Zero(), false});
    }
  }
  rotateTo(goal_base.z());
  state.head<3>() = goal_base;
  result.back().state.head<3>() = goal_base;
  return result;
}

void TaskAwarePlanner::appendAlignment(
  std::vector<Waypoint> & waypoints,
  const Eigen::VectorXd & target_joint) const
{
  const Eigen::VectorXd start_state = waypoints.back().state;
  const Eigen::VectorXd delta = target_joint - start_state.tail(model_.nq);
  const double duration = std::max(
    0.2, 1.5 * delta.cwiseAbs().maxCoeff() /
    std::max(1.0e-6, joint_speed_));
  const int steps = std::max(
    2, static_cast<int>(std::ceil(duration / sample_period_)));
  const double start_time = waypoints.back().time;
  for (int index = 1; index <= steps; ++index) {
    const double u = static_cast<double>(index) / static_cast<double>(steps);
    const double smooth = u * u * (3.0 - 2.0 * u);
    Eigen::VectorXd state = start_state;
    state.tail(model_.nq) += smooth * delta;
    waypoints.push_back(Waypoint{
      start_time + duration * u, state, kPhasePrecontactAlign,
      Eigen::Vector3d::Zero(), false});
  }
  if (hold_duration_ > 0.0) {
    Waypoint hold = waypoints.back();
    hold.time += hold_duration_;
    waypoints.push_back(std::move(hold));
  }
}

void TaskAwarePlanner::appendApproach(
  std::vector<Waypoint> & waypoints, const Eigen::Vector3d & base,
  const std::vector<Eigen::VectorXd> & joints,
  const std::vector<Eigen::Vector3d> & targets) const
{
  for (std::size_t index = 1; index < joints.size(); ++index) {
    const double distance = (targets[index] - targets[index - 1]).norm();
    const double joint_delta =
      (joints[index] - joints[index - 1]).cwiseAbs().maxCoeff();
    const double duration = std::max({
      distance / std::max(1.0e-6, approach_speed_),
      joint_delta / std::max(1.0e-6, joint_speed_), sample_period_});
    Eigen::VectorXd state(model_.nq + 3);
    state << base, joints[index];
    waypoints.push_back(Waypoint{
      waypoints.back().time + duration, state, kPhasePrecontactApproach,
      targets[index], true, task_trajectory_.points.front().surface_normal,
      false});
  }
}

void TaskAwarePlanner::appendTask(
  std::vector<Waypoint> & waypoints, const Candidate & candidate,
  const std::vector<TaskSample> & samples) const
{
  for (std::size_t index = 0; index < samples.size(); ++index) {
    double duration = sample_period_;
    if (index > 0) {
      const double target_distance =
        (samples[index].position - samples[index - 1].position).norm();
      const double speed = std::max(
        1.0e-6, samples[index].nominal_speed);
      const double base_distance =
        (candidate.bases[index].head<2>() -
        candidate.bases[index - 1].head<2>()).norm();
      const double joint_delta =
        (candidate.joints[index] - candidate.joints[index - 1])
        .cwiseAbs().maxCoeff();
      duration = std::max({
        target_distance / std::max(1.0e-6, speed),
        base_distance / std::max(1.0e-6, base_speed_),
        joint_delta / std::max(1.0e-6, joint_speed_), sample_period_});
    }
    Eigen::VectorXd state(model_.nq + 3);
    state << candidate.bases[index], candidate.joints[index];
    waypoints.push_back(Waypoint{
      waypoints.back().time + duration, state, kPhaseTask,
      samples[index].position, true, samples[index].normal,
      samples[index].contact});
  }
}

PlanReport TaskAwarePlanner::validate(
  const std::vector<Waypoint> & waypoints, const Candidate & candidate,
  int candidate_count, int feasible_count) const
{
  PlanReport report;
  report.task_name = task_name_;
  report.task_type = toString(task_trajectory_.type);
  report.state_dimension = model_.nq + 3;
  report.waypoint_count = waypoints.size();
  report.duration = waypoints.back().time;
  report.candidate_count = candidate_count;
  report.feasible_candidate_count = feasible_count;
  report.selected_candidate_id = candidate.id;
  report.selected_standoff = candidate.standoff;
  report.selected_longitudinal_offset = candidate.offset;
  report.selected_future_task_score = candidate.score;
  for (const std::string phase : {
      kPhaseNavigate, kPhasePrecontactAlign,
      kPhasePrecontactApproach, kPhaseTask})
  {
    report.phase_counts[phase] = static_cast<int>(std::count_if(
      waypoints.begin(), waypoints.end(),
      [&phase](const Waypoint & point) {return point.phase == phase;}));
  }

  report.minimum_joint_margin = std::numeric_limits<double>::infinity();
  report.minimum_manipulability = std::numeric_limits<double>::infinity();
  report.minimum_sigma = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : waypoints) {
    if (waypoint.phase != kPhaseTask) {
      continue;
    }
    const Eigen::Vector3d position = toolPosition(waypoint.state);
    const Eigen::Matrix3d rotation = toolRotation(waypoint.state);
    report.max_contact_position_error = std::max(
      report.max_contact_position_error,
      (position - waypoint.task_target).norm());
    Eigen::Vector3d expected_normal = surface_normal_;
    if (surface_type_ == "cylindrical") {
      const Eigen::Vector3d radial = waypoint.task_target - surface_center_;
      expected_normal = surfaceNormal(std::atan2(radial.x(), -radial.y()));
    }
    report.max_tool_axis_error = std::max(
      report.max_tool_axis_error,
      std::acos(std::clamp(
        rotation.col(2).dot(desiredRotation(expected_normal).col(2)),
        -1.0, 1.0)));
    report.minimum_joint_margin = std::min(
      report.minimum_joint_margin,
      jointMargin(waypoint.state.tail(model_.nq)));
    report.minimum_manipulability = std::min(
      report.minimum_manipulability,
      manipulability(waypoint.state.tail(model_.nq)));
    report.minimum_sigma = std::min(
      report.minimum_sigma,
      minimumSingularValue(waypoint.state.tail(model_.nq)));
  }
  report.task_base_path_length = candidate.base_path_length;

  for (std::size_t index = 1; index < waypoints.size(); ++index) {
    const Waypoint & first = waypoints[index - 1];
    const Waypoint & second = waypoints[index];
    const double dt = std::max(1.0e-9, second.time - first.time);
    const Eigen::Vector2d delta =
      second.state.head<2>() - first.state.head<2>();
    const double yaw_delta = wrapAngle(second.state[2] - first.state[2]);
    const double midpoint_yaw = first.state[2] + 0.5 * yaw_delta;
    const double lateral = std::abs(
      -std::sin(midpoint_yaw) * delta.x() +
      std::cos(midpoint_yaw) * delta.y()) / dt;
    report.max_lateral_velocity = std::max(
      report.max_lateral_velocity, lateral);
    report.max_base_speed = std::max(
      report.max_base_speed, delta.norm() / dt);
    report.max_angular_speed = std::max(
      report.max_angular_speed, std::abs(yaw_delta) / dt);
    report.max_joint_speed = std::max(
      report.max_joint_speed,
      (second.state.tail(model_.nq) - first.state.tail(model_.nq))
      .cwiseAbs().maxCoeff() / dt);
  }

  report.navigation_clearance = std::numeric_limits<double>::infinity();
  for (const auto & waypoint : waypoints) {
    if (waypoint.phase == kPhaseNavigate) {
      report.navigation_clearance = std::min(
        report.navigation_clearance,
        pointClearance(waypoint.state.head<2>()));
    }
  }
  report.constraints = {
    {"navigation_collision_free", report.navigation_clearance >= -1.0e-9},
    {"differential_drive_no_lateral_slip",
      report.max_lateral_velocity <= 1.0e-5},
    {"base_speed_limit", report.max_base_speed <= base_speed_ + 1.0e-6},
    {"angular_speed_limit",
      report.max_angular_speed <= angular_speed_ + 1.0e-6},
    {"joint_speed_limit", report.max_joint_speed <= joint_speed_ + 1.0e-6},
    {"precontact_clearance_reached", precontactClearanceOk(waypoints)},
    {"contact_position",
      report.max_contact_position_error <= position_tolerance_},
    {"tool_normal_alignment", report.max_tool_axis_error <= axis_tolerance_},
    {"joint_limit_margin", report.minimum_joint_margin >= minimum_margin_},
    {"future_task_manipulability",
      report.minimum_manipulability >= minimum_manipulability_}};
  report.success = std::all_of(
    report.constraints.begin(), report.constraints.end(),
    [](const auto & item) {return item.second;});
  if (!report.success) {
    std::string failed;
    for (const auto & constraint : report.constraints) {
      if (!constraint.second) {
        failed += (failed.empty() ? "" : ", ") + constraint.first;
      }
    }
    throw std::runtime_error(
      "Constraint validation failed: " + failed +
      " (navigation_clearance=" +
      std::to_string(report.navigation_clearance) +
      ", lateral=" + std::to_string(report.max_lateral_velocity) + ")");
  }
  return report;
}

bool TaskAwarePlanner::precontactClearanceOk(
  const std::vector<Waypoint> & waypoints) const
{
  const Waypoint * selected = nullptr;
  for (const auto & waypoint : waypoints) {
    if (waypoint.phase == kPhasePrecontactAlign) {
      selected = &waypoint;
    }
  }
  if (selected == nullptr) {
    return false;
  }
  const Eigen::Vector3d tool = toolPosition(selected->state);
  const Eigen::Matrix3d rotation = toolRotation(selected->state);
  const TaskSample first = taskSamples().front();
  const Eigen::Vector3d expected =
    first.position + first.normal * approach_clearance_;
  const double axis_error = std::acos(std::clamp(
    rotation.col(2).dot(first.desired_rotation.col(2)), -1.0, 1.0));
  return (tool - expected).norm() <= position_tolerance_ &&
    axis_error <= axis_tolerance_;
}

}  // namespace ta_wbmp
