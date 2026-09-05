#include "wbmm_core/ports.hpp"
#include "wbmm_core/validation.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <string>
#include <type_traits>

namespace wbmm::core
{
namespace
{

class FakeKinematics final : public KinematicsPort
{
public:
  [[nodiscard]] PortContract contract() const noexcept override
  {
    PortContract result;
    result.supports_cancellation = true;
    return result;
  }

  Result<Pose> forwardKinematics(
    const WholeBodyState & state, const std::string & target_frame,
    const OperationContext & context) const override
  {
    if (context.isCancellationRequested()) {
      return Result<Pose>::Failure(Status(ErrorCode::kCancelled, "request cancelled"));
    }
    if (const auto status = validateWholeBodyState(state); !status.ok()) {
      return Result<Pose>::Failure(status);
    }
    if (target_frame.empty()) {
      return Result<Pose>::Failure(Status(ErrorCode::kEmptyFrame, "target frame is empty"));
    }
    Pose pose;
    pose.header = Header{state.header.frame_id, state.header.stamp};
    return Result<Pose>::Success(pose);
  }

  Result<Matrix> frameJacobian(
    const WholeBodyState &, const std::string &,
    const OperationContext &) const override
  {
    Matrix matrix;
    matrix.rows = 6;
    matrix.cols = 9;
    matrix.row_major_data.resize(matrix.rows * matrix.cols, 0.0);
    return Result<Matrix>::Success(std::move(matrix));
  }
};

WholeBodyState validState()
{
  WholeBodyState state;
  state.header = Header{"map", Timestamp{1, ClockType::kSteady}};
  state.base_model = BaseModel::kDifferentialDrive;
  state.joints.names = {"a", "b", "c", "d", "e", "f"};
  state.joints.positions_rad.resize(6, 0.0);
  return state;
}

TEST(PortsTest, PortsRemainAbstractAndBackendIndependent)
{
  static_assert(std::is_abstract_v<KinematicsPort>);
  static_assert(std::is_abstract_v<CollisionCheckerPort>);
  static_assert(std::is_abstract_v<EnvironmentPort>);
  static_assert(std::is_abstract_v<WholeBodyPlannerPort>);
  static_assert(std::is_abstract_v<ContactControllerPort>);
  static_assert(std::is_abstract_v<WholeBodyAllocatorPort>);
  static_assert(std::is_abstract_v<ReferenceOwnershipPort>);
  SUCCEED();
}

TEST(PortsTest, PureCppFakeCanImplementAKinematicsPort)
{
  const FakeKinematics kinematics;
  const auto result = kinematics.forwardKinematics(validState(), "tool", OperationContext{});
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value().header.frame_id, "map");
}

TEST(PortsTest, OperationContextPropagatesCancellation)
{
  const FakeKinematics kinematics;
  std::atomic_bool cancelled{true};
  OperationContext context;
  context.cancellation_requested = &cancelled;
  const auto result = kinematics.forwardKinematics(validState(), "tool", context);
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), ErrorCode::kCancelled);
}

}  // namespace
}  // namespace wbmm::core
