#pragma once

#include "wbmm_core/types.hpp"

#include <string>

namespace wbmm::core
{

enum class CallSemantics
{
  kSynchronous = 0,
  kAsynchronous,
};

enum class ThreadOwnership
{
  kCaller = 0,
  kImplementation,
};

enum class RealtimeSuitability
{
  kNotRealtime = 0,
  kRealtimeSafe,
};

struct PortContract
{
  CallSemantics call_semantics{CallSemantics::kSynchronous};
  ThreadOwnership thread_ownership{ThreadOwnership::kCaller};
  RealtimeSuitability realtime_suitability{RealtimeSuitability::kNotRealtime};
  // Capabilities default to false so an implementation cannot claim them accidentally.
  bool supports_timeout{false};
  bool supports_cancellation{false};
  bool enforces_freshness{false};
};

class KinematicsPort
{
public:
  virtual ~KinematicsPort() = default;
  [[nodiscard]] virtual PortContract contract() const noexcept = 0;
  virtual Result<Pose> forwardKinematics(
    const WholeBodyState & state, const std::string & target_frame,
    const OperationContext & context) const = 0;
  virtual Result<Matrix> frameJacobian(
    const WholeBodyState & state, const std::string & target_frame,
    const OperationContext & context) const = 0;
};

class CollisionCheckerPort
{
public:
  virtual ~CollisionCheckerPort() = default;
  [[nodiscard]] virtual PortContract contract() const noexcept = 0;
  virtual Result<CollisionCheckResult> checkState(
    const WholeBodyState & state, const EnvironmentSnapshot & environment,
    const OperationContext & context) const = 0;
  virtual Result<CollisionCheckResult> checkSegment(
    const WholeBodyState & from, const WholeBodyState & to,
    const EnvironmentSnapshot & environment, const OperationContext & context) const = 0;
};

class EnvironmentPort
{
public:
  virtual ~EnvironmentPort() = default;
  [[nodiscard]] virtual PortContract contract() const noexcept = 0;
  virtual Result<EnvironmentSnapshot> acquireSnapshot(
    const OperationContext & context) const = 0;
  virtual Result<DistanceQueryResult> queryDistance(
    const EnvironmentSnapshot & environment, const DistanceQuery & query,
    const OperationContext & context) const = 0;
};

class TaskTrajectoryProvider
{
public:
  virtual ~TaskTrajectoryProvider() = default;
  [[nodiscard]] virtual PortContract contract() const noexcept = 0;
  virtual Result<TaskTrajectory> createTaskTrajectory(
    const WholeBodyState & start, const OperationContext & context) const = 0;
};

class WholeBodyPlannerPort
{
public:
  virtual ~WholeBodyPlannerPort() = default;
  [[nodiscard]] virtual PortContract contract() const noexcept = 0;
  virtual PlanningResult plan(
    const PlanningRequest & request, const OperationContext & context) const = 0;
};

class ContactControllerPort
{
public:
  virtual ~ContactControllerPort() = default;
  [[nodiscard]] virtual PortContract contract() const noexcept = 0;
  virtual Result<ContactCorrection> update(
    const ContactControlRequest & request, const OperationContext & context) = 0;
};

class WholeBodyAllocatorPort
{
public:
  virtual ~WholeBodyAllocatorPort() = default;
  [[nodiscard]] virtual PortContract contract() const noexcept = 0;
  virtual Result<WholeBodyInput> allocate(
    const AllocationRequest & request, const OperationContext & context) = 0;
};

class NavigationPort
{
public:
  virtual ~NavigationPort() = default;
  [[nodiscard]] virtual PortContract contract() const noexcept = 0;
  virtual PlanningResult planNavigation(
    const NavigationRequest & request, const OperationContext & context) const = 0;
};

class TrajectoryTrackerPort
{
public:
  virtual ~TrajectoryTrackerPort() = default;
  [[nodiscard]] virtual PortContract contract() const noexcept = 0;
  virtual Result<WholeBodyInput> update(
    const TrackingRequest & request, const OperationContext & context) = 0;
};

// This is a domain/test boundary. Real hardware resources remain owned by ros2_control.
class StateProviderPort
{
public:
  virtual ~StateProviderPort() = default;
  [[nodiscard]] virtual PortContract contract() const noexcept = 0;
  virtual Result<WholeBodyState> readState(const OperationContext & context) const = 0;
};

// This is a mock/non-realtime boundary and must not bypass hardware resource ownership.
class CommandSinkPort
{
public:
  virtual ~CommandSinkPort() = default;
  [[nodiscard]] virtual PortContract contract() const noexcept = 0;
  virtual Status writeCommand(
    const WholeBodyInput & command, const OwnershipLease & lease,
    const OperationContext & context) = 0;
};

class ReferenceOwnershipPort
{
public:
  virtual ~ReferenceOwnershipPort() = default;
  [[nodiscard]] virtual PortContract contract() const noexcept = 0;
  virtual Result<OwnershipLease> acquire(
    ReferenceOwner requested_owner, const OperationContext & context) = 0;
  virtual Status release(const OwnershipLease & lease) = 0;
  virtual Result<OwnershipLease> currentOwner() const = 0;
};

}  // namespace wbmm::core
