#include "wbmm_robot_metrics/arm_metrics.hpp"
#include "wbmm_robot_metrics/joint_limit_metrics.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <string>
#include <utility>
#include <vector>

namespace
{

class FakeRobotModel final : public wbmm::core::RobotModel
{
public:
  FakeRobotModel()
  {
    joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    limits_.joint_min.assign(6, -1.0);
    limits_.joint_max.assign(6, 1.0);
    limits_.max_joint_speed.assign(6, 2.0);
    limits_.max_base_speed = 0.5;
    limits_.max_base_yaw_rate = 1.0;

    // Identity in the last six columns. Arm scope takes the last six columns
    // of the 8D contract; whole-body scope keeps the same six non-zero
    // directions plus two zero base columns.
    jacobian_ = Eigen::MatrixXd::Zero(6, 8);
    jacobian_.rightCols(6).setIdentity();
  }

  [[nodiscard]] std::size_t stateDimension() const override {return 9;}
  [[nodiscard]] std::size_t inputDimension() const override {return 8;}
  [[nodiscard]] wbmm::core::BaseModel baseModel() const override
  {
    return wbmm::core::BaseModel::kDifferentialDrive;
  }
  [[nodiscard]] const std::vector<std::string> & jointNames() const override
  {
    return joint_names_;
  }
  [[nodiscard]] const wbmm::core::RobotLimits & limits() const override
  {
    return limits_;
  }

  bool forwardKinematics(
    const wbmm::core::WholeBodyState &, const std::string &,
    wbmm::core::Pose &) const override
  {
    return false;
  }

  bool frameJacobian(
    const wbmm::core::WholeBodyState &, const std::string &,
    Eigen::Ref<Eigen::MatrixXd> jacobian) const override
  {
    if (jacobian.rows() != jacobian_.rows() ||
      jacobian.cols() != jacobian_.cols())
    {
      return false;
    }
    jacobian = jacobian_;
    return true;
  }

  bool validate(
    const wbmm::core::WholeBodyState &, std::string * = nullptr) const override
  {
    return true;
  }

  bool validate(
    const wbmm::core::WholeBodyInput &, std::string * = nullptr) const override
  {
    return true;
  }

private:
  std::vector<std::string> joint_names_;
  wbmm::core::RobotLimits limits_;
  Eigen::MatrixXd jacobian_;
};

wbmm::core::WholeBodyState makeState()
{
  wbmm::core::WholeBodyState state;
  state.header.frame_id = "odom";
  state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
  state.base.x = 1.0;
  state.base.y = 2.0;
  state.base.yaw = 0.4;
  state.joints.names = {"joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6"};
  state.joints.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return state;
}

}  // namespace

TEST(RobotMetrics, JointLimitMarginsUseMidpointBoundaryConvention)
{
  wbmm::core::JointState joints;
  joints.names = {"j1", "j2", "j3"};
  joints.positions = {0.0, 0.5, 2.0};

  wbmm::core::RobotLimits limits;
  limits.joint_min = {-1.0, -1.0, -1.0};
  limits.joint_max = {1.0, 1.0, 1.0};

  const auto result = wbmm::metrics::JointLimitMetrics::evaluate(joints, limits);
  ASSERT_EQ(result.status, wbmm::metrics::MetricsStatus::kSuccess);
  ASSERT_EQ(result.normalized_margin.size(), 3);
  EXPECT_NEAR(result.normalized_margin(0), 1.0, 1.0e-12);
  EXPECT_NEAR(result.normalized_margin(1), 0.5, 1.0e-12);
  EXPECT_NEAR(result.normalized_margin(2), -1.0, 1.0e-12);
  EXPECT_NEAR(result.min_normalized_margin, -1.0, 1.0e-12);
}

TEST(RobotMetrics, ArmMetricsMatchesKnownJacobian)
{
  auto model = std::make_shared<FakeRobotModel>();
  wbmm::metrics::ArmMetrics metrics(model);

  const auto state = makeState();
  auto result = metrics.evaluate(state, "tool0");
  ASSERT_EQ(result.status, wbmm::metrics::MetricsStatus::kSuccess);
  ASSERT_EQ(result.singular_values.size(), 6);
  EXPECT_NEAR(result.sigma_min, 1.0, 1.0e-12);
  EXPECT_NEAR(result.sigma_max, 1.0, 1.0e-12);
  EXPECT_NEAR(result.condition_number, 1.0, 1.0e-12);
  EXPECT_NEAR(result.manipulability, 1.0, 1.0e-12);
  EXPECT_EQ(result.numerical_rank, 6);
  ASSERT_EQ(result.joint_limits.status, wbmm::metrics::MetricsStatus::kSuccess);
  EXPECT_NEAR(result.joint_limits.min_normalized_margin, 1.0, 1.0e-12);
}

TEST(RobotMetrics, ProviderNeutralEntryPointUsesSuppliedJacobian)
{
  Eigen::MatrixXd jacobian = Eigen::MatrixXd::Zero(6, 8);
  jacobian.rightCols(6).setIdentity();

  const auto state = makeState();
  wbmm::core::RobotLimits limits;
  limits.joint_min.assign(6, -1.0);
  limits.joint_max.assign(6, 1.0);

  const auto result = wbmm::metrics::ArmMetrics::evaluate(
    jacobian, 6, state.joints, limits, state.header, "tool0");

  ASSERT_EQ(result.status, wbmm::metrics::MetricsStatus::kSuccess);
  EXPECT_NEAR(result.sigma_min, 1.0, 1.0e-12);
  EXPECT_NEAR(result.manipulability, 1.0, 1.0e-12);
  EXPECT_EQ(result.header.frame_id, "odom");
  EXPECT_EQ(result.link_name, "tool0");
}

TEST(RobotMetrics, SupportsWholeBodyScopeAndTaskDirection)
{
  auto model = std::make_shared<FakeRobotModel>();
  wbmm::metrics::ArmMetrics metrics(model);

  wbmm::metrics::ArmMetricsOptions options;
  options.scope = wbmm::metrics::JacobianScope::kWholeBodyInput;
  options.use_task_direction = true;
  options.task_direction = Eigen::VectorXd::Ones(6);

  const auto state = makeState();
  const auto result = metrics.evaluate(state, "tool0", options);
  ASSERT_EQ(result.status, wbmm::metrics::MetricsStatus::kSuccess);
  EXPECT_EQ(result.singular_values.size(), 6);
  EXPECT_NEAR(result.options.task_direction.norm(), 1.0, 1.0e-12);
  EXPECT_NEAR(result.task_direction_manipulability, 1.0, 1.0e-12);
  EXPECT_NEAR(result.inverse_manipulability, 6.0 / (1.0 + options.regularization), 1.0e-12);
}

TEST(RobotMetrics, CharacteristicLengthScalesAngularRows)
{
  auto model = std::make_shared<FakeRobotModel>();
  wbmm::metrics::ArmMetrics metrics(model);

  wbmm::metrics::ArmMetricsOptions options;
  options.task = wbmm::metrics::JacobianTask::kPose;
  options.scaling = wbmm::metrics::JacobianScaling::kCharacteristicLength;
  options.characteristic_length = 0.5;

  const auto result = metrics.evaluate(makeState(), "tool0", options);
  ASSERT_EQ(result.status, wbmm::metrics::MetricsStatus::kSuccess);
  ASSERT_EQ(result.singular_values.size(), 6);
  EXPECT_NEAR(result.sigma_min, 0.5, 1.0e-12);
  EXPECT_NEAR(result.sigma_max, 1.0, 1.0e-12);
  EXPECT_NEAR(result.condition_number, 2.0, 1.0e-12);
  EXPECT_NEAR(result.manipulability, 0.125, 1.0e-12);
}

TEST(RobotMetrics, CharacteristicLengthDoesNotScaleTranslationTask)
{
  auto model = std::make_shared<FakeRobotModel>();
  wbmm::metrics::ArmMetrics metrics(model);

  wbmm::metrics::ArmMetricsOptions options;
  options.task = wbmm::metrics::JacobianTask::kTranslation;
  options.scaling = wbmm::metrics::JacobianScaling::kCharacteristicLength;
  options.characteristic_length = 0.5;

  const auto result = metrics.evaluate(makeState(), "tool0", options);
  ASSERT_EQ(result.status, wbmm::metrics::MetricsStatus::kSuccess);
  ASSERT_EQ(result.singular_values.size(), 3);
  EXPECT_NEAR(result.sigma_min, 1.0, 1.0e-12);
  EXPECT_NEAR(result.sigma_max, 1.0, 1.0e-12);
  EXPECT_NEAR(result.manipulability, 1.0, 1.0e-12);
}

TEST(RobotMetrics, RejectsInvalidInputsWithoutInventingValues)
{
  wbmm::metrics::ArmMetrics null_model(nullptr);
  auto result = null_model.evaluate(makeState(), "tool0");
  EXPECT_EQ(result.status, wbmm::metrics::MetricsStatus::kModelError);
  EXPECT_TRUE(std::isnan(result.sigma_min));
  EXPECT_TRUE(std::isnan(result.sigma_max));
  EXPECT_TRUE(std::isnan(result.manipulability));
  EXPECT_EQ(result.numerical_rank, -1);

  auto model = std::make_shared<FakeRobotModel>();
  wbmm::metrics::ArmMetrics metrics(model);
  result = metrics.evaluate(makeState(), "");
  EXPECT_EQ(result.status, wbmm::metrics::MetricsStatus::kInvalidInput);

  wbmm::metrics::ArmMetricsOptions options;
  options.use_task_direction = true;
  options.task_direction = Eigen::VectorXd::Ones(5);
  result = metrics.evaluate(makeState(), "tool0", options);
  EXPECT_EQ(result.status, wbmm::metrics::MetricsStatus::kInvalidInput);

  options.task_direction = Eigen::VectorXd::Zero(6);
  result = metrics.evaluate(makeState(), "tool0", options);
  EXPECT_EQ(result.status, wbmm::metrics::MetricsStatus::kInvalidInput);

  auto state = makeState();
  std::swap(state.joints.names[0], state.joints.names[1]);
  result = metrics.evaluate(state, "tool0");
  EXPECT_EQ(result.status, wbmm::metrics::MetricsStatus::kInvalidInput);
  EXPECT_NE(
    result.message.find("exactly match"), std::string::npos);
}
