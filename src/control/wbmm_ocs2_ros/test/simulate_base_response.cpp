// Deterministic offline MPC/plant comparison. No ROS initialization, CAN,
// hardware drivers or command publishers. Plant deliberately differs from MPC.
// Initial CSV: time, geometry[9], unused recorded EE[7].
#include <wbmm_ocs2/WbmmInterface.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>

int main(int argc, char** argv) {
  if (argc != 7) {
    std::cerr << "usage: simulate_base_response task.info robot.urdf initial.csv output.csv library_folder plant_delay_scale\n";
    return 2;
  }
  try {
    wbmm_ocs2::WbmmInterface interface(argv[1], argv[5], argv[2]);
    interface.setTaskPhase(wbmm_ocs2::TaskPhase::kExecution);
    ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(),
        interface.getRollout(), interface.getOptimalControlProblem(), interface.getInitializer());
    mpc.getSolverPtr()->setReferenceManager(interface.getReferenceManagerPtr());
    std::ifstream input(argv[3]);
    std::ofstream output(argv[4]);
    if (!input || !output) { throw std::runtime_error("Cannot open CSV"); }
    std::string line; std::getline(input, line);
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream row(line);
    double ignored;
    ocs2::vector_t geometry(9);
    if (!(row >> ignored)) { throw std::runtime_error("Missing initial state"); }
    for (auto& value : geometry) {
      if (!(row >> value)) { throw std::runtime_error("Invalid initial state"); }
    }
    auto pin = interface.getPinocchioInterface();
    const auto frame = pin.getModel().getFrameId(interface.getWbmmModelInfo().eeFrame);
    auto eePose = [&]() {
      pinocchio::forwardKinematics(pin.getModel(), pin.getData(), geometry);
      pinocchio::updateFramePlacements(pin.getModel(), pin.getData());
      const auto& transform = pin.getData().oMf[frame];
      ocs2::vector_t pose(7);
      pose.head(3) = transform.translation();
      pose.tail(4) = Eigen::Quaterniond(transform.rotation()).coeffs();
      return pose;
    };
    const auto initialTarget = eePose();
    const double dt = .02, duration = 22;
    const double delayScale = std::stod(argv[6]);
    if (!std::isfinite(delayScale) || delayScale < 0) { throw std::runtime_error("Invalid delay scale"); }
    std::deque<std::pair<double, ocs2::vector_t>> history;
    double v = 0, w = 0, offset = 0;
    output << std::setprecision(17)
        << "time,solve_ms,error_m,sigma,v_cmd,w_cmd,v_actual,w_actual,base_x,base_y,yaw,target_x\n";
    for (int step = 0; step <= static_cast<int>(duration / dt); ++step) {
      const double time = step * dt;
      auto target = initialTarget;
      // Smooth bounded force-equivalent target velocity: rest, forward drag,
      // release, reverse drag, release. Same reference for every controller.
      const double speed = time >= 3 && time < 8 ? .025 :
                           time >= 11 && time < 16 ? -.025 : 0;
      offset += dt * speed;
      target(0) += offset;
      interface.setEndEffectorTarget(ocs2::TargetTrajectories({time}, {target}, {ocs2::vector_t::Zero(8)}));
      ocs2::vector_t state = ocs2::vector_t::Zero(interface.getWbmmModelInfo().stateDim);
      state.head(9) = geometry;
      if (state.size() == 11) { state.tail(2) << v, w; }
      const auto start = std::chrono::steady_clock::now();
      if (!mpc.run(time, state)) { throw std::runtime_error("MPC failed"); }
      const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
      const auto solution = mpc.getSolverPtr()->primalSolution(time + .2);
      if (solution.inputTrajectory_.empty()) { throw std::runtime_error("Empty policy"); }
      ocs2::vector_t command = solution.inputTrajectory_.front();
      // Same execution limits as force_mpc, independent of MPC soft bounds.
      command(0) = std::clamp(command(0), -.1, .1);
      command(1) = std::clamp(command(1), -.4, .4);
      for (int j = 2; j < 8; ++j) { command(j) = std::clamp(command(j), -.2, .2); }
      const auto pose = eePose();
      double sigma = 0;
      if (interface.getArmMetricsEvaluator()) {
        sigma = interface.getArmMetricsEvaluator()->computeMetrics(state).sigma_min;
      }
      output << time << ',' << ms << ',' << (pose.head(3) - target.head(3)).norm() << ',' << sigma
          << ',' << command(0) << ',' << command(1) << ',' << v << ',' << w
          << ',' << geometry(0) << ',' << geometry(1) << ',' << geometry(2) << ',' << target(0) << '\n';
      history.emplace_back(time, command);
      auto delayedAxis = [&](int axis, double delay) {
        double value = 0;
        for (const auto& entry : history) {
          if (entry.first > time - delay + 1e-10) { break; }
          value = entry.second(axis);
        }
        return value;
      };
      // diag03 inertia+dead-time fit; arm is an ideal velocity plant here.
      v += -std::expm1(-dt / .1893) * (.8182 * delayedAxis(0, .12 * delayScale) - v);
      w += -std::expm1(-dt / .3039) * (1.1471 * delayedAxis(1, .14 * delayScale) - w);
      const double midYaw = geometry(2) + .5 * dt * w;
      geometry(0) += dt * v * std::cos(midYaw);
      geometry(1) += dt * v * std::sin(midYaw);
      geometry(2) += dt * w;
      geometry.segment(3, 6) += dt * command.tail(6);
      while (history.size() > 1 && history[1].first <= time - .14 * delayScale - dt) { history.pop_front(); }
    }
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
