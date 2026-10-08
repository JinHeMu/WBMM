// Mechanism regression, not a firmware emulator or a real-robot stability test.
// The synthetic actuator retains packet-derived velocity until the next packet,
// then filters it with the tau fitted from bag 04. No SDK/network/ROS is linked.
#include "ArmPositionCommand.h"
#include "jaka_hardware_interface/servo_command_stream.hpp"
#include <algorithm>
#include <cmath>
#include <deque>
#include <iostream>
#include <vector>

struct Result { double peak_speed = 0, peak_motion = 0, peak_step = 0; int packets = 0; };

Result simulate(bool fixed) {
  constexpr double dt = .008, tau = .08349;
  const double jump = .011603928165202504;
  std::vector<double> previous(6, 0), target(6, 0), feedback(6, 0), velocity(6, 0);
  target[1] = jump;
  double packet_position = 0, requested_velocity = -.0876, measured_velocity = -.0876, motion = 0;
  std::deque<double> delay(3, requested_velocity);
  jaka_hardware_interface::ServoCommandStream stream;
  // Disable numeric guard limits so the causal fix, rather than a protective
  // trip, must suppress runaway. Dispatch failure protection remains present.
  Result result;
  for (int tick = 0; tick < 125; ++tick) {
    std::vector<double> command = target;
    if (fixed && !wbmm::boundedArmHoldCommand(target, previous, dt, .2, command)) { std::exit(2); }
    result.peak_step = std::max(result.peak_step, std::abs(command[1]-previous[1]));
    const auto send = [&] {
      requested_velocity = (command[1]-packet_position)/dt;
      packet_position = command[1]; ++result.packets;
      return true;
    };
    if (fixed) {
      if (!stream.update(command, feedback, velocity, send, [] { return true; })) { std::exit(3); }
    } else if (std::abs(command[1]-packet_position) > 1e-5) {
      send(); // Original hardware writer suppressed every identical hold.
    }
    delay.push_back(requested_velocity);
    const double delayed_velocity = delay.front(); delay.pop_front();
    measured_velocity += (1-std::exp(-dt/tau))*(delayed_velocity-measured_velocity);
    motion += dt*measured_velocity;
    result.peak_speed = std::max(result.peak_speed, std::abs(measured_velocity));
    result.peak_motion = std::max(result.peak_motion, std::abs(motion));
    previous = command;
  }
  return result;
}

int main() {
  const auto old = simulate(false), fixed = simulate(true);
  if (old.peak_speed < 1 || old.peak_motion < .1 || old.packets != 1 ||
      fixed.peak_speed > .2+1e-12 || fixed.peak_motion > .02 ||
      fixed.peak_step > .0016+1e-12 || fixed.packets != 125) { return 1; }
  std::cout << "{\"synthetic_model_only\":true,\"guards_disabled\":true,"
            << "\"old\":{\"peak_speed_rad_s\":" << old.peak_speed
            << ",\"peak_motion_rad\":" << old.peak_motion << ",\"packets\":" << old.packets
            << "},\"fixed\":{\"peak_speed_rad_s\":" << fixed.peak_speed
            << ",\"peak_motion_rad\":" << fixed.peak_motion << ",\"max_step_rad\":" << fixed.peak_step
            << ",\"packets\":" << fixed.packets << "}}\n";
}
