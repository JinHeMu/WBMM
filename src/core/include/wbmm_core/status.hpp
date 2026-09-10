#pragma once

#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace wbmm::core
{

enum class ErrorCode
{
  kOk = 0,
  kInvalidArgument,
  kInvalidDimension,
  kNonFiniteValue,
  kEmptyFrame,
  kInvalidTimestamp,
  kDuplicateJointName,
  kJointNameMismatch,
  kInvalidQuaternion,
  kUnsupportedBaseModel,
  kStaleData,
  kTimeout,
  kCancelled,
  kNotConfigured,
  kNotActive,
  kEnvironmentUnavailable,
  kEnvironmentRevisionMismatch,
  kCollision,
  kSafetyLimitExceeded,
  kOwnershipConflict,
  kBackendUnsupported,
  kInternalError,
};

const char * toString(ErrorCode code) noexcept;

class Status final
{
public:
  Status() = default;
  Status(ErrorCode code, std::string message);

  static Status Ok();

  [[nodiscard]] bool ok() const noexcept;
  [[nodiscard]] ErrorCode code() const noexcept;
  [[nodiscard]] const std::string & message() const noexcept;

private:
  ErrorCode code_{ErrorCode::kOk};
  std::string message_;
};

template<typename T>
class Result final
{
public:
  static Result Success(T value)
  {
    return Result(Status::Ok(), std::move(value));
  }

  static Result Failure(Status status)
  {
    if (status.ok()) {
      status = Status(ErrorCode::kInternalError, "failure result requires a non-OK status");
    }
    return Result(std::move(status), std::nullopt);
  }

  [[nodiscard]] bool ok() const noexcept
  {
    return status_.ok() && value_.has_value();
  }

  [[nodiscard]] const Status & status() const noexcept
  {
    return status_;
  }

  [[nodiscard]] const T & value() const
  {
    if (!ok()) {
      throw std::logic_error("attempted to read value from failed Result");
    }
    return *value_;
  }

  [[nodiscard]] T & value()
  {
    if (!ok()) {
      throw std::logic_error("attempted to read value from failed Result");
    }
    return *value_;
  }

  [[nodiscard]] T takeValue()
  {
    if (!ok()) {
      throw std::logic_error("attempted to move value from failed Result");
    }
    return std::move(*value_);
  }

private:
  Result(Status status, std::optional<T> value)
  : status_(std::move(status)), value_(std::move(value)) {}

  Result(Status status, T value)
  : status_(std::move(status)), value_(std::move(value)) {}

  Status status_;
  std::optional<T> value_;
};

}  // namespace wbmm::core
