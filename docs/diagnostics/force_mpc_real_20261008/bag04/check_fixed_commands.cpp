// Offline only: links neither ROS nor the JAKA SDK. Send/stop are counters.
#include "ArmPositionCommand.h"
#include "jaka_hardware_interface/servo_command_stream.hpp"
#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
  if (argc != 2) { return 2; }
  std::ifstream input(argv[1]);
  if (!input) { return 2; }
  std::string line;
  std::getline(input, line);
  jaka_hardware_interface::ServoCommandStream stream;
  stream.setLimits(.0532, .4);
  std::vector<double> previous, target;
  double last_time = 0, max_step = 0, first_fault = -1, first_settled = -1;
  int sends = 0, stops = 0, rows = 0;
  while (std::getline(input, line)) {
    std::stringstream row(line);
    std::string field;
    std::vector<double> values;
    while (std::getline(row, field, ',')) { values.push_back(std::stod(field)); }
    if (values.size() != 21) { return 3; }
    const double receipt = values[0], time = values[1];
    const bool hold = values[2] != 0;
    std::vector<double> command(values.begin()+3, values.begin()+9);
    const std::vector<double> feedback(values.begin()+9, values.begin()+15);
    const std::vector<double> velocity(values.begin()+15, values.end());
    if (hold) {
      if (target.empty()) { target = command; }
      const double dt = std::clamp(time-last_time, 0.0, .05);
      if (!wbmm::boundedArmHoldCommand(target, previous, dt, .2, command)) { return 4; }
      for (std::size_t i = 0; i < 6; ++i) {
        const double step = std::abs(command[i]-previous[i]);
        max_step = std::max(max_step, step);
        if (step > dt*.2 + 1e-12) { return 5; }
      }
      if (first_settled < 0 && command == target) { first_settled = receipt; }
    }
    const bool ok = stream.update(command, feedback, velocity,
                                 [&] { ++sends; return true; },
                                 [&] { ++stops; return true; });
    if (!ok && first_fault < 0) {
      if (!hold) { std::cerr << "Unexpected guard trip before recorded force fault\n"; return 6; }
      first_fault = receipt;
    }
    previous = command; last_time = time; ++rows;
  }
  if (first_fault < 0 || stops != 1 || first_settled < 0) { return 7; }
  std::cout << "{\"rows\":" << rows << ",\"accepted_send_callbacks\":" << sends
            << ",\"stop_callbacks\":" << stops << ",\"hold_max_step_rad\":" << max_step
            << ",\"hold_target_reached_receipt_s\":" << first_settled
            << ",\"guard_first_trip_receipt_s\":" << first_fault
            << ",\"guard_reason\":\"" << stream.faultReason() << "\"}\n";
}
