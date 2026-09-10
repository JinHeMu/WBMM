#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace wbmm::core
{

// Unit convention for all plain scalar fields in this package:
// length: m, angle: rad, linear velocity: m/s, angular velocity: rad/s,
// time: s, effort: N*m.
// Field names intentionally omit unit suffixes; units are part of the contract.

enum class ClockDomain
{
  kUnspecified = 0,
  kSystem,
  kSimulation,
  kOcs2Mpc,
};

struct Header
{
  std::string frame_id;
  double stamp{0.0};
  ClockDomain clock{ClockDomain::kUnspecified};
};

struct Vector3
{
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

// Internal quaternion order is w, x, y, z.
// ROS messages use x, y, z, w and must be converted at the ROS boundary.
struct Quaternion
{
  double w{1.0};
  double x{0.0};
  double y{0.0};
  double z{0.0};
};

struct Pose
{
  Header header;
  Vector3 position;
  Quaternion orientation;
};

struct Twist
{
  Header header;
  Vector3 linear;
  Vector3 angular;
};

struct Wrench
{
  Header header;
  Vector3 force;
  Vector3 torque;
};

enum class BaseModel
{
  kUnspecified = 0,
  kFixed,
  kDifferentialDrive,
  kOmnidirectional,
};

struct BaseState
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};

  double linear_velocity{0.0};
  double lateral_velocity{0.0};
  double yaw_rate{0.0};
};

struct JointState
{
  std::vector<std::string> names;
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;
};

struct WholeBodyState
{
  Header header;
  BaseModel base_model{BaseModel::kUnspecified};
  BaseState base;
  JointState joints;
};

struct WholeBodyInput
{
  double stamp{0.0};
  ClockDomain clock{ClockDomain::kUnspecified};
  BaseModel base_model{BaseModel::kUnspecified};

  // Differential-drive: [v, omega].
  // Fixed: empty.
  // Omnidirectional: [vx, vy, omega].
  std::vector<double> base_command;

  std::vector<std::string> joint_names;
  std::vector<double> joint_velocities;
};

enum class ExecutionPhase
{
  kIdle = 0,
  kNavigate,
  kPreExecution,
  kExecution,
  kTracking,
  kFinish,
  kFault,
};

inline constexpr std::size_t kDifferentialBaseStateDim = 3;
inline constexpr std::size_t kDifferentialBaseInputDim = 2;
inline constexpr std::size_t kSpatialVelocityDim = 6;
inline constexpr std::size_t kWrenchDim = 6;

// First-version WBMM dimension contract: differential-drive base + six joints.
// Concrete joint names are provided by RobotModel/profile adapters, not by core.
inline constexpr std::size_t kSupportedJointCount = 6;

inline bool isFinite(const double value)
{
  return std::isfinite(value);
}

inline bool isFinite(const Vector3 & value)
{
  return isFinite(value.x) && isFinite(value.y) && isFinite(value.z);
}

inline bool isUnitQuaternion(const Quaternion & q, const double tolerance = 1.0e-6)
{
  const double norm_squared = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  return std::abs(norm_squared - 1.0) <= tolerance;
}

}  // namespace wbmm::core
