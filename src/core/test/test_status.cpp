#include "wbmm_core/status.hpp"

#include <gtest/gtest.h>

#include <string>

namespace wbmm::core
{
namespace
{

TEST(StatusTest, OkStatusHasStableSemantics)
{
  const Status status = Status::Ok();
  EXPECT_TRUE(status.ok());
  EXPECT_EQ(status.code(), ErrorCode::kOk);
  EXPECT_STREQ(toString(status.code()), "ok");
}

TEST(StatusTest, FailureCarriesAnActionableCodeAndMessage)
{
  const Status status(ErrorCode::kInvalidDimension, "expected 8 values");
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.code(), ErrorCode::kInvalidDimension);
  EXPECT_EQ(status.message(), "expected 8 values");
}

TEST(ResultTest, SuccessExposesItsValue)
{
  auto result = Result<std::string>::Success("trajectory-ready");
  ASSERT_TRUE(result.ok());
  EXPECT_EQ(result.value(), "trajectory-ready");
}

TEST(ResultTest, FailureRejectsValueAccess)
{
  const auto result = Result<int>::Failure(
    Status(ErrorCode::kEnvironmentUnavailable, "snapshot unavailable"));
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), ErrorCode::kEnvironmentUnavailable);
  EXPECT_THROW(static_cast<void>(result.value()), std::logic_error);
}

TEST(ResultTest, OkStatusCannotConstructAFailure)
{
  const auto result = Result<int>::Failure(Status::Ok());
  EXPECT_FALSE(result.ok());
  EXPECT_EQ(result.status().code(), ErrorCode::kInternalError);
}

}  // namespace
}  // namespace wbmm::core
