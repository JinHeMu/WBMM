#include "wbmm_search/kino_astar.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
struct Scenario
{
  std::string name;
  wbmm::core::BaseState start;
  wbmm::core::BaseState goal;
};

std::ofstream openCsv(
  const std::filesystem::path & path, const wbmm::search::BaseSearchResult & result,
  const Scenario & scenario)
{
  std::ofstream file(path);
  file.exceptions(std::ios::badbit | std::ios::failbit);
  file << std::setprecision(17)
       << "# scenario=" << scenario.name << '\n'
       << "# frame_id=" << result.header.frame_id << '\n'
       << "# clock=" << static_cast<int>(result.header.clock) << '\n'
       << "# stamp=" << result.header.stamp << '\n'
       << "# collision_checked=" << (result.collision_checked ? 1 : 0) << '\n'
       << "# goal_x=" << scenario.goal.x << '\n'
       << "# goal_y=" << scenario.goal.y << '\n'
       << "# goal_yaw=" << scenario.goal.yaw << '\n';
  return file;
}

void exportResult(
  const std::filesystem::path & directory, const wbmm::search::BaseSearchResult & result,
  const Scenario & scenario)
{
  auto path = openCsv(directory / (scenario.name + "_path.csv"), result, scenario);
  auto primitives = openCsv(directory / (scenario.name + "_primitives.csv"), result, scenario);
  auto rollout = openCsv(directory / (scenario.name + "_rollout.csv"), result, scenario);
  path << "index,time,x,y,yaw,v,omega\n";
  primitives << "index,start_time,v,omega,duration\n";
  rollout << "time,x,y,yaw,v,omega\n";
  double time = 0.0;
  for (std::size_t i = 0; i < result.path.size(); ++i) {
    const auto & state = result.path[i];
    path << i << ',' << time << ',' << state.x << ',' << state.y << ',' << state.yaw << ','
         << state.linear_velocity << ',' << state.yaw_rate << '\n';
    if (i == result.primitives.size()) {break;}
    const auto & input = result.primitives[i];
    primitives << i << ',' << time << ',' << input.v << ',' << input.omega << ',' << input.duration << '\n';
    // Export the exact same rollout used by the planner. The plotting script
    // consumes these samples instead of maintaining a second motion model.
    const auto samples = static_cast<std::size_t>(std::ceil(input.duration / 0.025));
    for (std::size_t sample = 0; sample <= samples; ++sample) {
      const double local_time = input.duration *
        (static_cast<double>(sample) / static_cast<double>(samples));
      const auto point = wbmm::search::propagate(state, input, local_time);
      rollout << time + local_time << ',' << point.x << ',' << point.y << ',' << point.yaw << ','
              << input.v << ',' << input.omega << '\n';
    }
    time += input.duration;
  }
  if (result.primitives.empty()) {
    const auto & state = result.path.front();
    rollout << "0," << state.x << ',' << state.y << ',' << state.yaw << ",0,0\n";
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  if (argc > 2 || (argc == 2 && std::string(argv[1]) == "--help")) {
    std::cout << "Usage: kino_astar_demo [output_directory]\n"
              << "Default output: /tmp/wbmm_kino_astar_demo\n"
              << "Offline only; collision checking is explicitly DISABLED.\n";
    return argc > 2 ? 1 : 0;
  }
  try {
    const std::filesystem::path output = argc == 2 ? argv[1] : "/tmp/wbmm_kino_astar_demo";
    std::filesystem::create_directories(output);
    wbmm::search::KinoAstarConfig config;
    config.collision_mode = wbmm::search::CollisionMode::kDisabled;
    const wbmm::search::KinoAstar planner(config);
    wbmm::core::RobotLimits limits;
    limits.max_base_speed = 0.5;
    limits.max_base_yaw_rate = 1.0;
    const wbmm::core::Header header{"odom", 0.0, wbmm::core::ClockDomain::kSimulation};
    const std::vector<Scenario> scenarios{
      {"forward", {}, {2.0, 0.0, 0.0}},
      {"reverse", {}, {-2.0, 0.0, 0.0}},
      {"rotate", {}, {0.0, 0.0, 1.5707963267948966}}};
    std::cout << "Offline base search: collision checking DISABLED (未检查碰撞).\n";
    bool all_successful = true;
    for (const auto & scenario : scenarios) {
      const auto result = planner.search(header, scenario.start, scenario.goal, limits);
      std::cout << scenario.name << ": " << wbmm::search::statusName(result.status)
                << ", length=" << result.path_length << " m, cost=" << result.total_cost
                << ", solve_time=" << result.solve_time << " s, nodes=" << result.generated_nodes
                << "\n  " << result.message << '\n';
      if (!result.success) {all_successful = false; continue;}
      exportResult(output, result, scenario);
    }
    std::cout << "CSV directory: " << std::filesystem::absolute(output) << '\n';
    return all_successful ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << "Demo failed: " << error.what() << '\n';
    return 1;
  }
}
