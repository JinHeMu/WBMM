#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>

namespace wbmm_localization {

class LocalizationReadiness : public rclcpp::Node {
public:
  explicit LocalizationReadiness(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~LocalizationReadiness() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

class WaitForLocalization : public rclcpp::Node {
public:
  explicit WaitForLocalization(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~WaitForLocalization() override;
  // -1 while waiting, 0 on readiness, 1 on timeout.
  int result() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};
} // namespace wbmm_localization
