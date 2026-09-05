#include "wbmm_core/status.hpp"

#include <utility>

namespace wbmm::core
{

const char * toString(const ErrorCode code) noexcept
{
  switch (code) {
    case ErrorCode::kOk: return "ok";
    case ErrorCode::kInvalidArgument: return "invalid_argument";
    case ErrorCode::kInvalidDimension: return "invalid_dimension";
    case ErrorCode::kNonFiniteValue: return "non_finite_value";
    case ErrorCode::kEmptyFrame: return "empty_frame";
    case ErrorCode::kInvalidTimestamp: return "invalid_timestamp";
    case ErrorCode::kDuplicateJointName: return "duplicate_joint_name";
    case ErrorCode::kJointNameMismatch: return "joint_name_mismatch";
    case ErrorCode::kInvalidQuaternion: return "invalid_quaternion";
    case ErrorCode::kUnsupportedBaseModel: return "unsupported_base_model";
    case ErrorCode::kStaleData: return "stale_data";
    case ErrorCode::kTimeout: return "timeout";
    case ErrorCode::kCancelled: return "cancelled";
    case ErrorCode::kNotConfigured: return "not_configured";
    case ErrorCode::kNotActive: return "not_active";
    case ErrorCode::kEnvironmentUnavailable: return "environment_unavailable";
    case ErrorCode::kEnvironmentRevisionMismatch: return "environment_revision_mismatch";
    case ErrorCode::kCollision: return "collision";
    case ErrorCode::kSafetyLimitExceeded: return "safety_limit_exceeded";
    case ErrorCode::kOwnershipConflict: return "ownership_conflict";
    case ErrorCode::kBackendUnsupported: return "backend_unsupported";
    case ErrorCode::kInternalError: return "internal_error";
  }
  return "unknown_error";
}

Status::Status(const ErrorCode code, std::string message)
: code_(code), message_(std::move(message))
{
  if (code_ != ErrorCode::kOk && message_.empty()) {
    message_ = toString(code_);
  }
}

Status Status::Ok()
{
  return {};
}

bool Status::ok() const noexcept
{
  return code_ == ErrorCode::kOk;
}

ErrorCode Status::code() const noexcept
{
  return code_;
}

const std::string & Status::message() const noexcept
{
  return message_;
}

}  // namespace wbmm::core
