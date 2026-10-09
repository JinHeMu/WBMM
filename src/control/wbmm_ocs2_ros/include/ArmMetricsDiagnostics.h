#pragma once

#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <wbmm_ocs2/cost/ArmManipulabilityCost.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace wbmm {

// Metrics are defined in cost/ArmManipulabilityCost.h. This adapter only
// samples/publishes them. postSolverRun receives the COMPLETE solver horizon,
// before MPC_ROS_Interface truncates the policy to solutionTimeWindow.
class ArmMetricsDiagnostics final : public ocs2::SolverSynchronizedModule {
 public:
  ArmMetricsDiagnostics(const wbmm_ocs2::ArmManipulabilityCost& evaluator,
                        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher)
      : evaluator_(evaluator.clone()), publisher_(std::move(publisher)) {}

  void preSolverRun(ocs2::scalar_t, ocs2::scalar_t, const ocs2::vector_t&,
                    const ocs2::ReferenceManagerInterface&) override {
    started_ = std::chrono::steady_clock::now();
  }

  void postSolverRun(const ocs2::PrimalSolution& solution) override {
    const double solveMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - started_).count();
    // Publish at most 10 Hz wall time. This bounds diagnostic FK/SVD overhead
    // independently of adaptive SLQ integration and simulator clock speed.
    if (started_ - lastPublished_ < std::chrono::milliseconds(100)) { return; }
    lastPublished_ = started_;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    std_msgs::msg::Float64MultiArray msg;
    msg.layout.dim.resize(1);
    msg.layout.dim[0].label = evaluator_->metricContract() +
        "time,horizon_s,min_sigma,terminal_sigma,min_yoshikawa,terminal_yoshikawa,time_to_min_s,valid,solve_ms";
    msg.layout.dim[0].size = msg.layout.dim[0].stride = 9;
    msg.data = {nan, nan, nan, nan, nan, nan, nan, 0.0, solveMs};
    const auto& times = solution.timeTrajectory_;
    const auto& states = solution.stateTrajectory_;
    if (times.empty() || times.size() != states.size()) { publisher_->publish(msg); return; }
    msg.data[0] = times.front();
    msg.data[1] = times.back() - times.front();
    double minSigma = std::numeric_limits<double>::infinity();
    double minYoshikawa = minSigma;
    bool valid = true;
    // Sample roughly every 50 ms of predicted time, plus the final state.
    // This is a sampled diagnostic, NOT a continuous-time safety guarantee.
    double nextTime = times.front();
    for (std::size_t i = 0; i < states.size(); ++i) {
      if (i + 1 != states.size() && times[i] < nextTime) { continue; }
      const auto metrics = evaluator_->computeMetrics(states[i]);
      valid = valid && metrics.status == wbmm::metrics::MetricsStatus::kSuccess;
      if (!valid) { break; }
      if (metrics.sigma_min < minSigma) {
        minSigma = metrics.sigma_min;
        msg.data[6] = times[i] - times.front();
      }
      minYoshikawa = std::min(minYoshikawa, metrics.manipulability);
      if (i + 1 == states.size()) {
        msg.data[3] = metrics.sigma_min;
        msg.data[5] = metrics.manipulability;
      }
      nextTime = times[i] + 0.05;
    }
    if (valid) {
      msg.data[2] = minSigma;
      msg.data[4] = minYoshikawa;
      msg.data[7] = 1.0;
    }
    publisher_->publish(msg);
  }

 private:
  std::unique_ptr<wbmm_ocs2::ArmManipulabilityCost> evaluator_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
  std::chrono::steady_clock::time_point started_{}, lastPublished_{};
};

}  // namespace wbmm
