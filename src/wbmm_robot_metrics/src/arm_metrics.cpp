#include "wbmm_robot_metrics/arm_metrics.hpp"
#include "wbmm_robot_metrics/joint_limit_metrics.hpp"

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

namespace wbmm::metrics
{
namespace
{

ArmKinematicMetrics invalidMetrics(
  MetricsStatus status, std::string message, const wbmm::core::Header & header,
  const std::string & link_name, const ArmMetricsOptions & options)
{
  ArmKinematicMetrics result;
  result.status = status;
  result.header = header;
  result.link_name = link_name;
  result.options = options;
  result.message = std::move(message);
  return result;
}

bool validTaskDirection(const ArmMetricsOptions & options, Eigen::Index rows)
{
  if (!options.use_task_direction) {
    return true;
  }
  return options.task_direction.size() == rows &&
         options.task_direction.allFinite() &&
         options.task_direction.norm() > std::numeric_limits<double>::epsilon();
}

}  // namespace

ArmMetrics::ArmMetrics(wbmm::core::RobotModelPtr robot_model)
: robot_model_(std::move(robot_model)) {}

bool ArmMetrics::validateOptions(
  const ArmMetricsOptions & options, std::string * message)
{
  const auto fail = [message](const std::string & text) {
      if (message != nullptr) {
        *message = text;
      }
      return false;
    };

  if (options.scaling == JacobianScaling::kCharacteristicLength &&
    (!std::isfinite(options.characteristic_length) ||
    !(options.characteristic_length > 0.0)))
  {
    return fail(
      "characteristic_length must be positive and finite when scaling is "
      "kCharacteristicLength");
  }
  if (!std::isfinite(options.regularization) ||
    !(options.regularization > 0.0))
  {
    return fail("regularization must be positive and finite");
  }
  if (!std::isfinite(options.singular_value_floor) ||
    !(options.singular_value_floor > 0.0))
  {
    return fail("singular_value_floor must be positive and finite");
  }

  const Eigen::Index task_rows =
    options.task == JacobianTask::kTranslation ? 3 : 6;
  if (!validTaskDirection(options, task_rows)) {
    return fail(
      "task_direction size must match task rows (3 for kTranslation, 6 for "
      "kPose), be finite, and have non-zero norm");
  }
  return true;
}

ArmKinematicMetrics ArmMetrics::evaluate(
  const wbmm::core::WholeBodyState & state,
  const std::string & link_name,
  ArmMetricsOptions options) const
{
  if (robot_model_ == nullptr) {
    return invalidMetrics(
      MetricsStatus::kModelError, "RobotModel pointer is null", state.header,
      link_name, options);
  }

  if (state.joints.names != robot_model_->jointNames()) {
    return invalidMetrics(
      MetricsStatus::kInvalidInput,
      "JointState.names must exactly match RobotModel::jointNames() order",
      state.header, link_name, options);
  }

  std::string model_message;
  if (!robot_model_->validate(state, &model_message)) {
    return invalidMetrics(
      MetricsStatus::kInvalidInput,
      model_message.empty() ? "state is invalid for the RobotModel"
                            : model_message,
      state.header, link_name, options);
  }

  const Eigen::Index input_dimension =
    static_cast<Eigen::Index>(robot_model_->inputDimension());
  if (input_dimension <= 0) {
    return invalidMetrics(
      MetricsStatus::kModelError, "RobotModel::inputDimension() must be positive",
      state.header, link_name, options);
  }

  Eigen::MatrixXd frame_jacobian =
    Eigen::MatrixXd::Zero(6, input_dimension);
  if (!robot_model_->frameJacobian(state, link_name, frame_jacobian) ||
    !frame_jacobian.allFinite())
  {
    return invalidMetrics(
      MetricsStatus::kModelError,
      "RobotModel::frameJacobian() failed for link '" + link_name + "'",
      state.header, link_name, options);
  }

  return ArmMetrics::evaluate(
    frame_jacobian, robot_model_->jointNames().size(), state.joints,
    robot_model_->limits(), state.header, link_name, std::move(options));
}

ArmKinematicMetrics ArmMetrics::evaluate(
  const Eigen::Ref<const Eigen::MatrixXd> & frame_jacobian,
  std::size_t arm_column_count,
  const wbmm::core::JointState & joints,
  const wbmm::core::RobotLimits & limits,
  const wbmm::core::Header & header,
  const std::string & link_name,
  ArmMetricsOptions options)
{
  const auto failInput = [&header, &link_name, &options](std::string message) {
      return invalidMetrics(
        MetricsStatus::kInvalidInput, std::move(message), header, link_name,
        options);
    };

  if (link_name.empty()) {
    return failInput("link_name must not be empty");
  }
  std::string options_message;
  if (!validateOptions(options, &options_message)) {
    return failInput(options_message);
  }
  if (frame_jacobian.rows() != 6 || frame_jacobian.cols() <= 0 ||
    !frame_jacobian.allFinite())
  {
    return failInput(
      "frame_jacobian must be finite with shape 6 x inputDimension");
  }

  Eigen::MatrixXd selected_jacobian;
  if (options.scope == JacobianScope::kArmColumns) {
    const auto joint_count = static_cast<Eigen::Index>(arm_column_count);
    if (joint_count <= 0 || joint_count > frame_jacobian.cols()) {
      return failInput(
        "arm_column_count must be in [1, frame_jacobian.cols()]");
    }
    selected_jacobian = frame_jacobian.rightCols(joint_count);
  } else {
    selected_jacobian = frame_jacobian;
  }

  const Eigen::Index task_rows =
    options.task == JacobianTask::kTranslation ? 3 : 6;
  if (selected_jacobian.rows() < task_rows) {
    return invalidMetrics(
      MetricsStatus::kModelError,
      "frame Jacobian does not contain the requested task rows",
      header, link_name, options);
  }
  Eigen::MatrixXd task_jacobian = selected_jacobian.topRows(task_rows);

  if (options.scaling == JacobianScaling::kCharacteristicLength &&
    options.task == JacobianTask::kPose)
  {
    // Balance metres and radians before SVD. Only rows 3..5 are angular;
    // translation-only tasks must remain unchanged.
    task_jacobian.bottomRows(3) *= options.characteristic_length;
  }

  if (task_jacobian.cols() <= 0 || !task_jacobian.allFinite()) {
    return invalidMetrics(
      MetricsStatus::kModelError, "selected task Jacobian is empty or non-finite",
      header, link_name, options);
  }
  if (options.use_task_direction) {
    options.task_direction.normalize();
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    task_jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const Eigen::VectorXd singular_values = svd.singularValues();
  if (singular_values.size() == 0 || !singular_values.allFinite()) {
    return invalidMetrics(
      MetricsStatus::kModelError, "SVD did not produce finite singular values",
      header, link_name, options);
  }

  ArmKinematicMetrics result;
  result.status = MetricsStatus::kSuccess;
  result.header = header;
  result.link_name = link_name;
  result.options = options;
  result.singular_values = singular_values;
  result.sigma_min = singular_values.minCoeff();
  result.sigma_max = singular_values.maxCoeff();
  result.condition_number =
    result.sigma_max / std::max(result.sigma_min, options.singular_value_floor);
  result.manipulability = singular_values.prod();

  const Eigen::MatrixXd jjt = task_jacobian * task_jacobian.transpose();
  const Eigen::MatrixXd regularized_jjt =
    jjt + options.regularization *
    Eigen::MatrixXd::Identity(jjt.rows(), jjt.cols());
  const Eigen::LDLT<Eigen::MatrixXd> factorization(regularized_jjt);
  if (factorization.info() != Eigen::Success) {
    return invalidMetrics(
      MetricsStatus::kModelError,
      "regularized J J^T factorization failed", header, link_name, options);
  }
  const Eigen::MatrixXd inverse_jjt = factorization.solve(
    Eigen::MatrixXd::Identity(jjt.rows(), jjt.cols()));
  if (factorization.info() != Eigen::Success || !inverse_jjt.allFinite()) {
    return invalidMetrics(
      MetricsStatus::kModelError,
      "regularized J J^T solve failed", header, link_name, options);
  }
  result.inverse_manipulability = inverse_jjt.trace();

  if (options.use_task_direction) {
    const double directional_value =
      options.task_direction.dot(jjt * options.task_direction);
    result.task_direction_manipulability =
      std::sqrt(std::max(0.0, directional_value));
  }

  const double dimension_scale = static_cast<double>(
    std::max(task_jacobian.rows(), task_jacobian.cols()));
  const double rank_tolerance =
    dimension_scale * std::numeric_limits<double>::epsilon() *
    std::max(result.sigma_max, 0.0);
  result.numerical_rank = 0;
  for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
    if (singular_values(i) > rank_tolerance) {
      ++result.numerical_rank;
    }
  }

  result.joint_limits = JointLimitMetrics::evaluate(
    joints, limits);

  const bool finite = std::isfinite(result.sigma_min) &&
    std::isfinite(result.sigma_max) &&
    std::isfinite(result.condition_number) &&
    std::isfinite(result.manipulability) &&
    std::isfinite(result.inverse_manipulability) &&
    (options.use_task_direction ?
      std::isfinite(result.task_direction_manipulability) : true);
  if (!finite) {
    return invalidMetrics(
      MetricsStatus::kModelError, "metrics contain non-finite values",
      header, link_name, options);
  }

  result.message =
    result.joint_limits.status == MetricsStatus::kSuccess ?
    "ok" : "kinematics ok; joint-limit metrics unavailable";
  return result;
}

}  // namespace wbmm::metrics
