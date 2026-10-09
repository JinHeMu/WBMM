// Offline solver comparison only: no ROS initialization or command publishers.
// CSV rows: time, state[9], EE target[7], in the recorded odom frame.
#include "ContinuousYaw.h"
#include <wbmm_ocs2/WbmmInterface.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

int main(int argc, char** argv) {
  if (argc != 7 || (std::string(argv[5]) != "wrapped" && std::string(argv[5]) != "continuous")) {
    std::cerr << "usage: replay_force_mpc task.info robot.urdf input.csv output.csv wrapped|continuous library_folder\n";
    return 2;
  }
  try {
    std::ifstream input(argv[3]);
    std::ofstream output(argv[4]);
    if (!input || !output) { throw std::runtime_error("Cannot open CSV files"); }
    wbmm_ocs2::WbmmInterface interface(argv[1], argv[6], argv[2]);
    interface.setTaskPhase(wbmm_ocs2::TaskPhase::kExecution);
    ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(),
        interface.getRollout(), interface.getOptimalControlProblem(), interface.getInitializer());
    mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
    wbmm::ContinuousYaw yaw;
    output << std::setprecision(17) << "time,yaw,solve_ms,v,w,qdot1,qdot2,qdot3,qdot4,qdot5,qdot6\n";
    std::string line;
    while (std::getline(input, line)) {
      if (line.empty() || line[0] == '#') { continue; }
      std::replace(line.begin(), line.end(), ',', ' ');
      std::istringstream row(line);
      double time;
      ocs2::vector_t state(9), target(7);
      if (!(row >> time)) { throw std::runtime_error("Invalid CSV time"); }
      for (auto* vector : {&state, &target}) {
        for (int i = 0; i < vector->size(); ++i) {
          if (!(row >> (*vector)(i))) { throw std::runtime_error("Invalid CSV state/target"); }
        }
      }
      if (std::string(argv[5]) == "continuous") { state(2) = yaw.update(state(2)); }
      interface.setEndEffectorTarget(ocs2::TargetTrajectories(
          {time}, {target}, {ocs2::vector_t::Zero(8)}));
      const auto start = std::chrono::steady_clock::now();
      if (!mpc.run(time, state)) { throw std::runtime_error("MPC run failed"); }
      const double ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
      const auto solution = mpc.getSolverPtr()->primalSolution(time + 0.2);
      if (solution.inputTrajectory_.empty()) { throw std::runtime_error("Empty MPC policy"); }
      output << time << ',' << state(2) << ',' << ms;
      for (double u : solution.inputTrajectory_.front()) { output << ',' << u; }
      output << '\n';
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
