#include "wbmm_robot_metrics/arm_metrics.hpp"
#include "wbmm_robot_metrics/joint_limit_metrics.hpp"

#include <gtest/gtest.h>

#include <cmath>

TEST(MetricsSkeleton, PlaceholderDoesNotInventKinematicOrJointLimitValues)
{
  const wbmm::metrics::ArmMetrics evaluator(nullptr);
  wbmm::core::WholeBodyState state;
  state.header.frame_id = "odom";
  const auto arm = evaluator.evaluate(state, "tool0");
  EXPECT_EQ(arm.status, wbmm::metrics::MetricsStatus::kNotImplemented);
  EXPECT_TRUE(std::isnan(arm.sigma_min));
  EXPECT_TRUE(std::isnan(arm.sigma_max));
  EXPECT_TRUE(std::isnan(arm.condition_number));
  EXPECT_TRUE(std::isnan(arm.manipulability));
  EXPECT_EQ(arm.singular_values.size(), 0);
  EXPECT_EQ(arm.numerical_rank, -1);
  EXPECT_EQ(arm.header.frame_id, "odom");
  EXPECT_EQ(arm.link_name, "tool0");
  EXPECT_EQ(arm.joint_limits.status, wbmm::metrics::MetricsStatus::kNotImplemented);

  const auto margins = wbmm::metrics::JointLimitMetrics::evaluate({}, {});
  EXPECT_EQ(margins.status, wbmm::metrics::MetricsStatus::kNotImplemented);
  EXPECT_EQ(margins.normalized_margin.size(), 0);
  EXPECT_TRUE(std::isnan(margins.min_normalized_margin));
}
