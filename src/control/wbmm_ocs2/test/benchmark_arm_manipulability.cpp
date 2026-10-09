#include "wbmm_ocs2/FactoryFunctions.h"
#include "wbmm_ocs2/cost/ArmManipulabilityCost.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <numeric>
#include <vector>

// Invoke with the robot URDF path; run before/after binaries on the same host.
// Times include the complete value/quadratic callback, not just the SVD.
int main(int argc, char** argv)
{
  if (argc != 2) { std::cerr << "usage: benchmark_arm_manipulability robot.urdf\n"; return 2; }
  const auto pinocchio = wbmm_ocs2::createWbmmPinocchioInterface(argv[1]);
  const auto info = wbmm_ocs2::createWbmmModelInfo(pinocchio, "base_footprint", "tool0");
  const ocs2::PreComputation pre;
  const ocs2::TargetTrajectories targets;
  std::vector<ocs2::vector_t> states;
  for (int i = 0; i < 8; ++i) {
    ocs2::vector_t x(9);
    x << 1.0, 2.0, 0.4, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
    x.tail(6).array() += 0.07 * i;
    states.push_back(x);
  }
  double checksum = 0.0;
  for (const bool yoshikawa : {false, true}) {
    wbmm_ocs2::ArmManipulabilitySettings settings;
    settings.metricsOptions.scaling = wbmm::metrics::JacobianScaling::kCharacteristicLength;
    settings.metricsOptions.characteristic_length = 0.3;
    settings.normalizeMargins = true;
    settings.minSingularRef = 0.12;
    settings.minSingularWeight = 0.5;
    settings.useYoshikawa = yoshikawa;
    settings.yoshikawaRef = 0.0015;
    settings.yoshikawaWeight = 0.005;
    wbmm_ocs2::ArmManipulabilityCost cost(pinocchio, info, settings);
    for (const bool quadratic : {false, true}) {
      std::vector<double> times;
      for (int batch = -2; batch < 25; ++batch) {
        const auto start = std::chrono::steady_clock::now();
        for (int i = 0; i < 800; ++i) {
          const auto& x = states[i % states.size()];
          if (quadratic) {
            const auto result = cost.getQuadraticApproximation(0.0, x, targets, pre);
            checksum += result.f + result.dfdx.sum() + result.dfdxx.trace();
          } else {
            checksum += cost.getValue(0.0, x, targets, pre);
          }
        }
        const double us = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - start).count() / 800;
        if (batch >= 0) { times.push_back(us); }
      }
      std::sort(times.begin(), times.end());
      std::cout << "{\"case\":\"" << (yoshikawa ? "sigma_yoshikawa" : "sigma")
                << "\",\"callback\":\"" << (quadratic ? "quadratic" : "value")
                << "\",\"median_us\":" << times[12] << ",\"p95_us\":" << times[23]
                << ",\"checksum\":" << checksum << "}\n";
    }
  }
}
