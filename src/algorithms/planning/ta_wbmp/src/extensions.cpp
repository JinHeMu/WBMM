#include "ta_wbmp/extensions.hpp"

#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/collision/collision.hpp>
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>
#include <stdexcept>

namespace ta_wbmp
{

class UrdfSelfCollisionStateValidityChecker::Impl
{
public:
  Impl(const std::string & urdf_file, double minimum_clearance)
  : minimum_clearance_(std::max(0.0, minimum_clearance))
  {
    pinocchio::urdf::buildModel(urdf_file, model_);
    const std::vector<std::string> package_dirs{
      std::filesystem::path(urdf_file).parent_path().parent_path()
      .parent_path().string()};
    pinocchio::urdf::buildGeom(
      model_, urdf_file, pinocchio::GeometryType::COLLISION,
      collision_model_, package_dirs);
    for (std::size_t first = 0;
      first < collision_model_.geometryObjects.size(); ++first)
    {
      for (std::size_t second = first + 1;
        second < collision_model_.geometryObjects.size(); ++second)
      {
        const auto first_joint =
          collision_model_.geometryObjects[first].parentJoint;
        const auto second_joint =
          collision_model_.geometryObjects[second].parentJoint;
        const bool adjacent = first_joint == second_joint ||
          model_.parents[first_joint] == second_joint ||
          model_.parents[second_joint] == first_joint;
        if (!adjacent) {
          collision_model_.addCollisionPair(
            pinocchio::CollisionPair(first, second));
        }
      }
    }
    if (collision_model_.collisionPairs.empty()) {
      throw std::runtime_error(
        "URDF collision checker has no non-adjacent collision pairs");
    }
  }

  StateValidityResult check(const Eigen::VectorXd & state) const
  {
    if (state.size() != model_.nq + 3 || !state.allFinite()) {
      return {false, -std::numeric_limits<double>::infinity(),
        "INVALID_WHOLE_BODY_STATE"};
    }
    const Eigen::VectorXd q = state.tail(model_.nq);
    pinocchio::Data data(model_);
    pinocchio::GeometryData geometry_data(collision_model_);
    if (pinocchio::computeCollisions(
        model_, data, collision_model_, geometry_data, q, true))
    {
      return {false, 0.0, "SELF_COLLISION"};
    }
    const std::size_t nearest = pinocchio::computeDistances(
      model_, data, collision_model_, geometry_data, q);
    const double clearance = nearest < geometry_data.distanceResults.size() ?
      geometry_data.distanceResults[nearest].min_distance :
      std::numeric_limits<double>::infinity();
    if (clearance < minimum_clearance_) {
      return {false, clearance, "SELF_COLLISION_CLEARANCE"};
    }
    return {true, clearance, {}};
  }

private:
  pinocchio::Model model_;
  pinocchio::GeometryModel collision_model_;
  double minimum_clearance_{0.0};
};

UrdfSelfCollisionStateValidityChecker::
UrdfSelfCollisionStateValidityChecker(
  const std::string & urdf_file, double minimum_clearance)
: impl_(std::make_unique<Impl>(urdf_file, minimum_clearance)) {}

UrdfSelfCollisionStateValidityChecker::
~UrdfSelfCollisionStateValidityChecker() = default;

StateValidityResult UrdfSelfCollisionStateValidityChecker::check(
  const Eigen::VectorXd & state) const
{
  return impl_->check(state);
}

StateValidityResult checkInterpolatedMotion(
  const WholeBodyStateValidityChecker & checker,
  const Eigen::VectorXd & first, const Eigen::VectorXd & second,
  double base_step, double yaw_step, double joint_step)
{
  if (first.size() != second.size() || first.size() < 3 ||
    base_step <= 0.0 || yaw_step <= 0.0 || joint_step <= 0.0)
  {
    return {false, -std::numeric_limits<double>::infinity(),
      "INVALID_MOTION_CHECK_ARGUMENT"};
  }
  const double yaw_delta = std::atan2(
    std::sin(second[2] - first[2]), std::cos(second[2] - first[2]));
  const double joint_delta = first.size() > 3 ?
    (second.tail(first.size() - 3) - first.tail(first.size() - 3))
    .cwiseAbs().maxCoeff() : 0.0;
  const int samples = std::max({
    1,
    static_cast<int>(std::ceil(
      (second.head<2>() - first.head<2>()).norm() / base_step)),
    static_cast<int>(std::ceil(std::abs(yaw_delta) / yaw_step)),
    static_cast<int>(std::ceil(joint_delta / joint_step))});
  double minimum_clearance = std::numeric_limits<double>::infinity();
  for (int index = 0; index <= samples; ++index) {
    const double ratio = static_cast<double>(index) /
      static_cast<double>(samples);
    Eigen::VectorXd state = first + ratio * (second - first);
    state[2] = std::atan2(
      std::sin(first[2] + ratio * yaw_delta),
      std::cos(first[2] + ratio * yaw_delta));
    StateValidityResult result = checker.check(state);
    minimum_clearance = std::min(minimum_clearance, result.clearance);
    if (!result.valid) {
      return result;
    }
  }
  return {true, minimum_clearance, {}};
}

double Se2NavigationCostEstimator::estimate(
  const Eigen::VectorXd & start, const Eigen::VectorXd & goal) const
{
  if (start.size() < 3 || goal.size() < 3) {
    return std::numeric_limits<double>::infinity();
  }
  const double yaw_delta = std::atan2(
    std::sin(goal[2] - start[2]), std::cos(goal[2] - start[2]));
  return (goal.head<2>() - start.head<2>()).norm() +
    yaw_weight_ * std::abs(yaw_delta);
}

}  // namespace ta_wbmp
