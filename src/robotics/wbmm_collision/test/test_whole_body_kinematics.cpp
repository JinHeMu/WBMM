// Verifies the analytic Jacobian against central differences.
//
// This is the test that makes the optimizer's collision gradients trustworthy.
// A sign error or a missing chain-rule term in the heading reconstruction would
// silently degrade planning quality rather than crash, so the analytic result is
// checked against a numerically differentiated ground truth on the real robot
// model, at several configurations, for every variable.

#include "wbmm_collision/whole_body_kinematics.hpp"

#include <Eigen/Geometry>

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

namespace
{

constexpr std::size_t kJoints = 6;
constexpr double kStep = 1.0e-6;

// Central-difference Jacobian of every sphere position, using the same
// evaluator with perturbed inputs. Slower and less accurate than the analytic
// version, which is exactly why it is a good reference.
struct NumericJacobian
{
  std::vector<Eigen::MatrixXd> base;  // one 3 x n per base sphere
  std::vector<Eigen::MatrixXd> arm;   // one 3 x n per arm sphere
};

NumericJacobian numericJacobian(
  const wbmm::collision::UrdfCollisionModel & model,
  const Eigen::Vector2d & position, const Eigen::Vector2d & velocity, int gear,
  const Eigen::VectorXd & joints)
{
  const std::size_t variables =
    wbmm::collision::WholeBodyVariableLayout::size(kJoints);

  const auto positions = [&](const Eigen::Vector2d & p, const Eigen::Vector2d & v,
                             const Eigen::VectorXd & q)
  {
    const auto evaluated =
      wbmm::collision::evaluateWholeBodyKinematics(model, p, v, gear, q);
    std::vector<Eigen::Vector3d> out;
    for (const auto & sphere : evaluated.base) {
      out.push_back(sphere.position);
    }
    for (const auto & sphere : evaluated.arm) {
      out.push_back(sphere.position);
    }
    return out;
  };

  const auto reference = positions(position, velocity, joints);
  NumericJacobian result;
  result.base.assign(model.base.spheres.size(), Eigen::MatrixXd::Zero(3, variables));
  result.arm.assign(
    model.armSphereCount(), Eigen::MatrixXd::Zero(3, variables));

  for (std::size_t variable = 0U; variable < variables; ++variable) {
    Eigen::Vector2d p_plus = position;
    Eigen::Vector2d p_minus = position;
    Eigen::Vector2d v_plus = velocity;
    Eigen::Vector2d v_minus = velocity;
    Eigen::VectorXd q_plus = joints;
    Eigen::VectorXd q_minus = joints;

    if (variable < 2U) {
      (&p_plus.x())[variable] += kStep;
      (&p_minus.x())[variable] -= kStep;
    } else if (variable < 4U) {
      (&v_plus.x())[variable - 2U] += kStep;
      (&v_minus.x())[variable - 2U] -= kStep;
    } else {
      const auto j = static_cast<Eigen::Index>(
        variable - wbmm::collision::WholeBodyVariableLayout::kFirstJointIndex);
      q_plus(j) += kStep;
      q_minus(j) -= kStep;
    }

    const auto plus = positions(p_plus, v_plus, q_plus);
    const auto minus = positions(p_minus, v_minus, q_minus);
    EXPECT_EQ(plus.size(), reference.size());

    for (std::size_t sphere = 0U; sphere < plus.size(); ++sphere) {
      const Eigen::Vector3d column = (plus[sphere] - minus[sphere]) / (2.0 * kStep);
      if (sphere < model.base.spheres.size()) {
        result.base[sphere].col(static_cast<Eigen::Index>(variable)) = column;
      } else {
        result.arm[sphere - model.base.spheres.size()].col(
          static_cast<Eigen::Index>(variable)) = column;
      }
    }
  }
  return result;
}

wbmm::collision::UrdfCollisionModel loadModel()
{
  const std::vector<std::string> names = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  return wbmm::collision::loadUrdfCollisionModel(
    WBMM_TEST_URDF, names, "base_link");
}

// Compares every entry, reporting the worst offender so a failure is actionable.
void expectClose(
  const Eigen::MatrixXd & analytic, const Eigen::MatrixXd & numeric,
  double tolerance, const std::string & label)
{
  ASSERT_EQ(analytic.rows(), numeric.rows());
  ASSERT_EQ(analytic.cols(), numeric.cols());
  double worst = 0.0;
  Eigen::Index worst_row = -1;
  Eigen::Index worst_col = -1;
  for (Eigen::Index r = 0; r < analytic.rows(); ++r) {
    for (Eigen::Index c = 0; c < analytic.cols(); ++c) {
      const double difference = std::abs(analytic(r, c) - numeric(r, c));
      if (difference > worst) {
        worst = difference;
        worst_row = r;
        worst_col = c;
      }
    }
  }
  EXPECT_LT(worst, tolerance)
    << label << ": worst entry (" << worst_row << "," << worst_col
    << ") analytic=" << analytic(worst_row, worst_col)
    << " numeric=" << numeric(worst_row, worst_col);
}

struct Sample
{
  Eigen::Vector2d position;
  Eigen::Vector2d velocity;
  int gear;
  Eigen::VectorXd joints;
  std::string label;
};

std::vector<Sample> samples()
{
  std::vector<Sample> out;
  const std::vector<Eigen::VectorXd> configurations = {
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(kJoints)),
    (Eigen::VectorXd(static_cast<Eigen::Index>(kJoints)) <<
      0.4, 0.9, -0.6, 1.2, -0.3, 0.7).finished(),
    (Eigen::VectorXd(static_cast<Eigen::Index>(kJoints)) <<
      -1.1, 2.0, 0.8, -1.4, 2.5, -2.0).finished(),
  };
  const std::vector<Eigen::Vector2d> velocities = {
    {0.4, 0.0}, {0.3, 0.2}, {-0.25, 0.35}};

  for (std::size_t c = 0U; c < configurations.size(); ++c) {
    for (std::size_t v = 0U; v < velocities.size(); ++v) {
      for (int gear : {1, -1}) {
        Sample sample;
        sample.position = Eigen::Vector2d(1.5, -0.7);
        sample.velocity = velocities[v];
        sample.gear = gear;
        sample.joints = configurations[c];
        sample.label = "config " + std::to_string(c) + " velocity " +
          std::to_string(v) + " gear " + std::to_string(gear);
        out.push_back(std::move(sample));
      }
    }
  }
  return out;
}

TEST(WholeBodyKinematics, MatchesFiniteDifferencesOnTheRealModel)
{
  const auto model = loadModel();
  ASSERT_TRUE(model.success) << model.message;

  for (const auto & sample : samples()) {
    const auto analytic = wbmm::collision::evaluateWholeBodyKinematics(
      model, sample.position, sample.velocity, sample.gear, sample.joints);
    ASSERT_TRUE(analytic.success) << analytic.message;

    const auto numeric = numericJacobian(
      model, sample.position, sample.velocity, sample.gear, sample.joints);

    ASSERT_EQ(analytic.base.size(), numeric.base.size());
    for (std::size_t i = 0U; i < analytic.base.size(); ++i) {
      expectClose(
        analytic.base[i].jacobian, numeric.base[i], 1e-6,
        sample.label + " base sphere " + std::to_string(i));
    }
    ASSERT_EQ(analytic.arm.size(), numeric.arm.size());
    for (std::size_t i = 0U; i < analytic.arm.size(); ++i) {
      expectClose(
        analytic.arm[i].jacobian, numeric.arm[i], 1e-6,
        sample.label + " arm sphere " + std::to_string(i));
    }
  }
}

TEST(WholeBodyKinematics, PositionGradientIsExactForEverySphere)
{
  const auto model = loadModel();
  ASSERT_TRUE(model.success) << model.message;

  Eigen::VectorXd joints = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(kJoints));
  const auto evaluated = wbmm::collision::evaluateWholeBodyKinematics(
    model, Eigen::Vector2d(0.5, 0.25), Eigen::Vector2d(0.4, 0.1), 1, joints);
  ASSERT_TRUE(evaluated.success) << evaluated.message;

  // Translating the base translates every sphere by the same amount, so the
  // first two columns are exactly [1,0,0] and [0,1,0] everywhere.
  for (const auto & sphere : evaluated.base) {
    EXPECT_DOUBLE_EQ(sphere.jacobian(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(sphere.jacobian(1, 1), 1.0);
    EXPECT_DOUBLE_EQ(sphere.jacobian(2, 0), 0.0);
    EXPECT_DOUBLE_EQ(sphere.jacobian(2, 1), 0.0);
  }
  for (const auto & sphere : evaluated.arm) {
    EXPECT_DOUBLE_EQ(sphere.jacobian(0, 0), 1.0);
    EXPECT_DOUBLE_EQ(sphere.jacobian(1, 1), 1.0);
    EXPECT_DOUBLE_EQ(sphere.jacobian(2, 0), 0.0);
    EXPECT_DOUBLE_EQ(sphere.jacobian(2, 1), 0.0);
  }
}

TEST(WholeBodyKinematics, ArmSpheresDoNotDependOnDistalJoints)
{
  const auto model = loadModel();
  ASSERT_TRUE(model.success) << model.message;

  Eigen::VectorXd joints = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(kJoints));
  const auto evaluated = wbmm::collision::evaluateWholeBodyKinematics(
    model, Eigen::Vector2d::Zero(), Eigen::Vector2d(0.4, 0.0), 1, joints);
  ASSERT_TRUE(evaluated.success) << evaluated.message;

  ASSERT_EQ(evaluated.arm.size(), evaluated.arm_group.size());
  for (std::size_t i = 0U; i < evaluated.arm.size(); ++i) {
    const std::size_t group = evaluated.arm_group[i];
    // A sphere on joint i's child link moves with joints 0..i and no others.
    for (std::size_t j = 0U; j < kJoints; ++j) {
      const auto column = static_cast<Eigen::Index>(
        wbmm::collision::WholeBodyVariableLayout::kFirstJointIndex + j);
      if (j > group) {
        EXPECT_DOUBLE_EQ(evaluated.arm[i].jacobian(0, column), 0.0) << "sphere " << i;
        EXPECT_DOUBLE_EQ(evaluated.arm[i].jacobian(1, column), 0.0) << "sphere " << i;
        EXPECT_DOUBLE_EQ(evaluated.arm[i].jacobian(2, column), 0.0) << "sphere " << i;
      }
    }
  }
  // Base spheres move with the base only.
  for (const auto & sphere : evaluated.base) {
    for (std::size_t j = 0U; j < kJoints; ++j) {
      EXPECT_DOUBLE_EQ(
        sphere.jacobian(0, static_cast<Eigen::Index>(
          wbmm::collision::WholeBodyVariableLayout::kFirstJointIndex + j)), 0.0);
    }
  }
}

TEST(WholeBodyKinematics, RejectsBadInput)
{
  auto model = loadModel();
  ASSERT_TRUE(model.success) << model.message;

  const auto wrong_joints = wbmm::collision::evaluateWholeBodyKinematics(
    model, Eigen::Vector2d::Zero(), Eigen::Vector2d(0.4, 0.0), 1,
    Eigen::VectorXd::Zero(3));
  EXPECT_FALSE(wrong_joints.success);

  const auto bad_gear = wbmm::collision::evaluateWholeBodyKinematics(
    model, Eigen::Vector2d::Zero(), Eigen::Vector2d(0.4, 0.0), 0,
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(kJoints)));
  EXPECT_FALSE(bad_gear.success);

  const auto not_finite = wbmm::collision::evaluateWholeBodyKinematics(
    model, Eigen::Vector2d::Zero(),
    Eigen::Vector2d(std::nan(""), 0.0), 1,
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(kJoints)));
  EXPECT_FALSE(not_finite.success);

  auto invalid = model;
  invalid.success = false;
  const auto invalid_model = wbmm::collision::evaluateWholeBodyKinematics(
    invalid, Eigen::Vector2d::Zero(), Eigen::Vector2d(0.4, 0.0), 1,
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(kJoints)));
  EXPECT_FALSE(invalid_model.success);
}

TEST(WholeBodyKinematics, HeadingGradientVanishesAtRest)
{
  const auto model = loadModel();
  ASSERT_TRUE(model.success) << model.message;

  // With no motion the heading is held, so it has no velocity derivative. The
  // trajectory builder holds it in the same regime.
  const auto at_rest = wbmm::collision::evaluateWholeBodyKinematics(
    model, Eigen::Vector2d::Zero(), Eigen::Vector2d::Zero(), 1,
    Eigen::VectorXd::Zero(static_cast<Eigen::Index>(kJoints)));
  ASSERT_TRUE(at_rest.success) << at_rest.message;
  EXPECT_TRUE(at_rest.yaw_gradient.isZero(0.0));
}

}  // namespace
