#include "wbmm_core/wbmm_core.hpp"

#include <gtest/gtest.h>

#include <limits>
#include <memory>
#include <string>

namespace
{


class FakeRobotModel final : public wbmm::core::RobotModel
{
public:
  FakeRobotModel()
  {
    limits_.joint_min = {-1.0, -1.0, -1.0, -1.0, -1.0, -1.0};
    limits_.joint_max = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    limits_.max_joint_speed = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    limits_.max_base_speed = 1.0;
    limits_.max_base_yaw_rate = 1.0;
    joint_names_ = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  }

  std::size_t stateDimension() const override {return 9;}
  std::size_t inputDimension() const override {return 8;}
  wbmm::core::BaseModel baseModel() const override
  {
    return wbmm::core::BaseModel::kDifferentialDrive;
  }
  const std::vector<std::string> & jointNames() const override {return joint_names_;}
  const wbmm::core::RobotLimits & limits() const override {return limits_;}

  bool forwardKinematics(
    const wbmm::core::WholeBodyState & state,
    const std::string &,
    wbmm::core::Pose & pose) const override
  {
    pose = wbmm::core::Pose{};
    pose.header = state.header;
    return true;
  }

  bool frameJacobian(
    const wbmm::core::WholeBodyState &,
    const std::string &,
    Eigen::Ref<Eigen::MatrixXd> jacobian) const override
  {
    if (
      jacobian.rows() != static_cast<Eigen::Index>(wbmm::core::kSpatialVelocityDim) ||
      jacobian.cols() != static_cast<Eigen::Index>(inputDimension()))
    {
      return false;
    }
    jacobian.setZero();
    return true;
  }

  bool validate(
    const wbmm::core::WholeBodyState &,
    std::string * message = nullptr) const override
  {
    if (message != nullptr) {
      message->clear();
    }
    return true;
  }

  bool validate(
    const wbmm::core::WholeBodyInput &,
    std::string * message = nullptr) const override
  {
    if (message != nullptr) {
      message->clear();
    }
    return true;
  }

private:
  std::vector<std::string> joint_names_;
  wbmm::core::RobotLimits limits_;
};

}  // namespace

TEST(WbmmCore, DifferentialDriveWithSixJointsHasNineStatesAndEightInputs)
{
  wbmm::core::WholeBodyState state;
  state.header.frame_id = "odom";
  state.header.clock = wbmm::core::ClockDomain::kSimulation;
  state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
  state.base.x = 1.0;
  state.base.y = 2.0;
  state.base.yaw = 0.3;
  state.joints.names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  state.joints.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  EXPECT_EQ(
    wbmm::core::kDifferentialBaseStateDim + state.joints.names.size(),
    static_cast<std::size_t>(9));
  EXPECT_EQ(
    wbmm::core::kDifferentialBaseInputDim + state.joints.names.size(),
    static_cast<std::size_t>(8));
  EXPECT_TRUE(wbmm::core::isFinite(state.base.x));
  EXPECT_TRUE(wbmm::core::isUnitQuaternion(wbmm::core::Quaternion{}));
}

TEST(WbmmCore, TrajectoryDurationUsesLastPointTime)
{
  wbmm::core::WholeBodyTrajectory trajectory;
  trajectory.trajectory_id = "test";
  trajectory.points.resize(3);
  trajectory.points[0].time_from_start = 0.0;
  trajectory.points[1].time_from_start = 0.5;
  trajectory.points[2].time_from_start = 1.5;

  EXPECT_FALSE(wbmm::core::empty(trajectory));
  EXPECT_DOUBLE_EQ(wbmm::core::duration(trajectory), 1.5);
}

TEST(WbmmCore, PhaseScheduleUsesExplicitTimeAndTask)
{
  wbmm::core::PhaseSchedule phases{
    {0.0, 1.0, wbmm::core::ExecutionPhase::kNavigate, "navigate", false},
    {1.0, 2.0, wbmm::core::ExecutionPhase::kExecution, "wipe", true}};

  ASSERT_EQ(phases.size(), 2U);
  EXPECT_DOUBLE_EQ(phases.back().start_time, 1.0);
  EXPECT_EQ(phases.back().task_id, "wipe");
  EXPECT_TRUE(phases.back().contact);
}

TEST(WbmmCore, RobotModelJacobianMapsEightInputsToSpatialVelocity)
{
  const auto model = std::make_shared<FakeRobotModel>();

  EXPECT_EQ(model->stateDimension(), static_cast<std::size_t>(9));
  EXPECT_EQ(model->inputDimension(), static_cast<std::size_t>(8));
  EXPECT_EQ(
    model->baseModel(),
    wbmm::core::BaseModel::kDifferentialDrive);
  EXPECT_EQ(model->jointNames().size(), static_cast<std::size_t>(6));

  wbmm::core::WholeBodyState state;
  state.header.frame_id = "odom";
  state.header.clock = wbmm::core::ClockDomain::kSimulation;
  wbmm::core::Pose pose;
  Eigen::MatrixXd jacobian(
    static_cast<Eigen::Index>(wbmm::core::kSpatialVelocityDim),
    static_cast<Eigen::Index>(model->inputDimension()));
  EXPECT_TRUE(model->forwardKinematics(state, "ee_link", pose));
  EXPECT_EQ(pose.header.frame_id, "odom");
  EXPECT_TRUE(model->frameJacobian(state, "ee_link", jacobian));

  Eigen::MatrixXd wrong_jacobian(
    static_cast<Eigen::Index>(wbmm::core::kSpatialVelocityDim),
    static_cast<Eigen::Index>(model->stateDimension()));
  EXPECT_FALSE(model->frameJacobian(state, "ee_link", wrong_jacobian));
}

namespace
{

wbmm::core::WholeBodyState makeValidState()
{
  wbmm::core::WholeBodyState state;
  state.header.frame_id = "odom";
  state.header.clock = wbmm::core::ClockDomain::kSimulation;
  state.base_model = wbmm::core::BaseModel::kDifferentialDrive;
  state.base.x = 0.0;
  state.base.y = 0.0;
  state.base.yaw = 0.0;
  state.joints.names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  state.joints.positions = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return state;
}

}  // namespace

TEST(WbmmCoreValidation, RejectsEmptyFrame)
{
  auto state = makeValidState();
  state.header.frame_id.clear();
  EXPECT_FALSE(wbmm::core::validate(state).ok);
}

TEST(WbmmCoreValidation, RejectsNonFiniteValue)
{
  auto state = makeValidState();
  state.base.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(wbmm::core::validate(state).ok);
}

TEST(WbmmCoreValidation, RejectsDuplicateJointNames)
{
  auto state = makeValidState();
  state.joints.names[1] = state.joints.names[0];
  EXPECT_FALSE(wbmm::core::validate(state).ok);
}

TEST(WbmmCoreValidation, RejectsJointArrayDimensionMismatch)
{
  auto state = makeValidState();
  state.joints.positions.pop_back();
  EXPECT_FALSE(wbmm::core::validate(state).ok);
}

TEST(WbmmCoreValidation, RejectsNonMonotonicTrajectoryTime)
{
  wbmm::core::WholeBodyTrajectory trajectory;
  trajectory.trajectory_id = "test";
  trajectory.environment_revision = 1;
  trajectory.collision_model_revision = 1;
  trajectory.points.resize(2);
  trajectory.points[0].time_from_start = 1.0;
  trajectory.points[1].time_from_start = 0.5;
  trajectory.points[0].state = makeValidState();
  trajectory.points[1].state = makeValidState();

  EXPECT_FALSE(wbmm::core::validate(trajectory).ok);
}

TEST(WbmmCoreValidation, RejectsZeroRevision)
{
  wbmm::core::WholeBodyTrajectory trajectory;
  trajectory.trajectory_id = "test";
  trajectory.environment_revision = 0;
  trajectory.collision_model_revision = 1;
  trajectory.points.resize(1);
  trajectory.points[0].state = makeValidState();

  EXPECT_FALSE(wbmm::core::validate(trajectory).ok);
}

TEST(WbmmCoreValidation, SearchResultRequiresParallelArrays)
{
  wbmm::core::SearchResult result;
  result.success = true;
  result.base_path.push_back(wbmm::core::BaseState{});
  result.arm_seed.push_back(makeValidState().joints);
  result.phases.push_back(wbmm::core::ExecutionPhase::kNavigate);
  EXPECT_TRUE(wbmm::core::validate(result).ok);

  result.phases.clear();
  EXPECT_FALSE(wbmm::core::validate(result).ok);
}

TEST(WbmmCoreValidation, RejectsClockDomainMismatch)
{
  wbmm::core::WholeBodyTrajectory trajectory;
  trajectory.trajectory_id = "clock_test";
  trajectory.environment_revision = 1;
  trajectory.collision_model_revision = 1;
  trajectory.points.resize(2);
  trajectory.points[0].time_from_start = 0.0;
  trajectory.points[1].time_from_start = 1.0;
  trajectory.points[0].state = makeValidState();
  trajectory.points[1].state = makeValidState();
  trajectory.points[0].state.header.clock = wbmm::core::ClockDomain::kSimulation;
  trajectory.points[1].state.header.clock = wbmm::core::ClockDomain::kSystem;

  EXPECT_FALSE(wbmm::core::validate(trajectory).ok);
}

TEST(WbmmCoreValidation, RejectsUnspecifiedClock)
{
  auto state = makeValidState();
  state.header.clock = wbmm::core::ClockDomain::kUnspecified;
  EXPECT_FALSE(wbmm::core::validate(state).ok);
}

namespace
{

wbmm::core::TaskTrajectoryPoint makeTaskPoint(
  const std::string & frame_id,
  const wbmm::core::ClockDomain clock,
  const double time)
{
  wbmm::core::TaskTrajectoryPoint point;
  point.time_from_start = time;
  point.pose.header.frame_id = frame_id;
  point.pose.header.clock = clock;
  point.pose.orientation = wbmm::core::Quaternion{};
  point.tangent = wbmm::core::Vector3{1.0, 0.0, 0.0};
  point.surface_normal = wbmm::core::Vector3{0.0, 0.0, 1.0};
  return point;
}

}  // namespace

TEST(WbmmCoreValidation, RejectsDuplicateTrajectoryTime)
{
  wbmm::core::WholeBodyTrajectory trajectory;
  trajectory.trajectory_id = "duplicate_time";
  trajectory.environment_revision = 1;
  trajectory.collision_model_revision = 1;
  trajectory.points.resize(2);
  trajectory.points[0].time_from_start = 0.0;
  trajectory.points[1].time_from_start = 0.0;
  trajectory.points[0].state = makeValidState();
  trajectory.points[1].state = makeValidState();

  EXPECT_FALSE(wbmm::core::validate(trajectory).ok);
}

TEST(WbmmCoreValidation, RejectsTrajectoryFrameMismatch)
{
  wbmm::core::WholeBodyTrajectory trajectory;
  trajectory.trajectory_id = "frame_mismatch";
  trajectory.environment_revision = 1;
  trajectory.collision_model_revision = 1;
  trajectory.points.resize(2);
  trajectory.points[0].time_from_start = 0.0;
  trajectory.points[1].time_from_start = 1.0;
  trajectory.points[0].state = makeValidState();
  trajectory.points[1].state = makeValidState();
  trajectory.points[0].state.header.frame_id = "odom";
  trajectory.points[1].state.header.frame_id = "map";

  EXPECT_FALSE(wbmm::core::validate(trajectory).ok);
}

TEST(WbmmCoreValidation, RejectsTaskTrajectoryFrameMismatch)
{
  wbmm::core::TaskTrajectory trajectory;
  trajectory.task_id = "task_frame_mismatch";
  trajectory.points = {
    makeTaskPoint("task_frame", wbmm::core::ClockDomain::kSimulation, 0.0),
    makeTaskPoint("odom", wbmm::core::ClockDomain::kSimulation, 1.0)};

  EXPECT_FALSE(wbmm::core::validate(trajectory).ok);
}

TEST(WbmmCoreValidation, RejectsNegativeSearchMetadata)
{
  wbmm::core::SearchResult result;
  result.success = true;
  result.base_path.push_back(wbmm::core::BaseState{});
  result.arm_seed.push_back(makeValidState().joints);
  result.phases.push_back(wbmm::core::ExecutionPhase::kNavigate);
  result.path_length = -1.0;

  EXPECT_FALSE(wbmm::core::validate(result).ok);
}

TEST(WbmmCoreValidation, RejectsNegativeFirstTrajectoryTime)
{
  wbmm::core::WholeBodyTrajectory trajectory;
  trajectory.trajectory_id = "negative_time";
  trajectory.environment_revision = 1;
  trajectory.collision_model_revision = 1;
  trajectory.points.resize(1);
  trajectory.points[0].time_from_start = -0.5;
  trajectory.points[0].state = makeValidState();

  EXPECT_FALSE(wbmm::core::validate(trajectory).ok);
}

TEST(WbmmCoreValidation, RejectsTaskReferenceFrameMismatch)
{
  wbmm::core::WholeBodyTrajectory trajectory;
  trajectory.trajectory_id = "task_reference_frame";
  trajectory.environment_revision = 1;
  trajectory.collision_model_revision = 1;
  trajectory.points.resize(1);
  trajectory.points[0].time_from_start = 0.0;
  trajectory.points[0].state = makeValidState();
  trajectory.points[0].task_reference = makeTaskPoint(
    "map", wbmm::core::ClockDomain::kSimulation, 0.0);

  EXPECT_FALSE(wbmm::core::validate(trajectory).ok);
}

TEST(WbmmCoreValidation, ValidatesTwistAndWrench)
{
  wbmm::core::Twist twist;
  twist.header.frame_id = "odom";
  twist.header.clock = wbmm::core::ClockDomain::kSimulation;
  twist.linear = {0.1, 0.2, 0.3};
  twist.angular = {0.01, 0.02, 0.03};
  EXPECT_TRUE(wbmm::core::validate(twist).ok);

  twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(wbmm::core::validate(twist).ok);

  wbmm::core::Wrench wrench;
  wrench.header.frame_id = "tool0";
  wrench.header.clock = wbmm::core::ClockDomain::kSystem;
  wrench.force = {1.0, 2.0, 3.0};
  wrench.torque = {0.1, 0.2, 0.3};
  EXPECT_TRUE(wbmm::core::validate(wrench).ok);
}

TEST(WbmmCoreValidation, ValidatesPhaseSchedule)
{
  wbmm::core::PhaseSchedule schedule{
    {0.0, 1.0, wbmm::core::ExecutionPhase::kNavigate, "navigate", false},
    {1.0, 2.0, wbmm::core::ExecutionPhase::kExecution, "execute", true}};

  EXPECT_TRUE(wbmm::core::validate(schedule, 2.0).ok);

  schedule[1].start_time = 1.1;
  EXPECT_FALSE(wbmm::core::validate(schedule, 2.0).ok);
}

TEST(WbmmCoreValidation, RejectsSearchResultNonFiniteBaseVelocity)
{
  wbmm::core::SearchResult result;
  result.success = true;
  result.base_path.push_back(wbmm::core::BaseState{});
  result.arm_seed.push_back(makeValidState().joints);
  result.phases.push_back(wbmm::core::ExecutionPhase::kNavigate);
  result.base_path[0].linear_velocity = std::numeric_limits<double>::quiet_NaN();

  EXPECT_FALSE(wbmm::core::validate(result).ok);
}
