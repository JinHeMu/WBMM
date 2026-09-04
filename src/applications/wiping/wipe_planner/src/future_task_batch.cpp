#include "wipe_planner/planner.hpp"

#include <Eigen/Core>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
struct Options
{
  std::string urdf;
  std::string task;
  std::string output{"future_task_batch_results"};
  int samples{50};
  int sequence_offset{2026};
};

struct Result
{
  int id{0};
  bool success{false};
  Eigen::VectorXd entry;
  double entry_position_error{std::numeric_limits<double>::quiet_NaN()};
  double entry_normal_error{std::numeric_limits<double>::quiet_NaN()};
  double planning_wall_time{0.0};
  double nominal_execution_time{std::numeric_limits<double>::quiet_NaN()};
  double approach_time{std::numeric_limits<double>::quiet_NaN()};
  double contact_time{std::numeric_limits<double>::quiet_NaN()};
  double base_travel{std::numeric_limits<double>::quiet_NaN()};
  double arm_travel{std::numeric_limits<double>::quiet_NaN()};
  std::size_t points{0};
  wipe_planner::PlanReport report;
  wipe_planner::TrajectoryMetrics metrics;
  std::vector<wipe_planner::Waypoint> trajectory;
  std::string error;
};

std::string valueAfter(const std::string & argument, const std::string & name)
{
  const std::string prefix = "--" + name + "=";
  return argument.rfind(prefix, 0) == 0 ? argument.substr(prefix.size()) : "";
}

Options parseOptions(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "--help") {
      std::cout
        << "Usage: wipe_future_task_batch --urdf=FILE --task=FILE [options]\n"
        << "  --output=DIR          CSV/Markdown output directory\n"
        << "  --samples=N           number of whole-body pre-contact states (default 50)\n"
        << "  --sequence-offset=N   deterministic Halton selection offset\n";
      std::exit(0);
    }
    if (const auto value = valueAfter(argument, "urdf"); !value.empty()) {
      options.urdf = value;
    } else if (const auto value = valueAfter(argument, "task"); !value.empty()) {
      options.task = value;
    } else if (const auto value = valueAfter(argument, "output"); !value.empty()) {
      options.output = value;
    } else if (const auto value = valueAfter(argument, "samples"); !value.empty()) {
      options.samples = std::stoi(value);
    } else if (const auto value = valueAfter(argument, "sequence-offset"); !value.empty()) {
      options.sequence_offset = std::stoi(value);
    } else {
      throw std::runtime_error("Unknown or incomplete argument: " + argument);
    }
  }
  if (options.urdf.empty() || options.task.empty()) {
    throw std::runtime_error("--urdf and --task are required (use --help)");
  }
  if (options.samples <= 0) {
    throw std::runtime_error("--samples must be positive");
  }
  return options;
}

double halton(int index, int base)
{
  double result = 0.0;
  double fraction = 1.0;
  while (index > 0) {
    fraction /= static_cast<double>(base);
    result += fraction * static_cast<double>(index % base);
    index /= base;
  }
  return result;
}

std::vector<Eigen::VectorXd> selectEntries(
  const std::vector<Eigen::VectorXd> & available, int requested, int offset)
{
  if (static_cast<int>(available.size()) < requested) {
    throw std::runtime_error(
      "Only " + std::to_string(available.size()) +
      " valid whole-body pre-contact states were found; requested " +
      std::to_string(requested));
  }
  std::vector<Eigen::VectorXd> selected;
  selected.reserve(static_cast<std::size_t>(requested));
  std::set<std::size_t> used;
  for (int sequence = std::max(1, offset);
       static_cast<int>(selected.size()) < requested; ++sequence)
  {
    const std::size_t candidate = std::min(
      available.size() - 1,
      static_cast<std::size_t>(halton(sequence, 2) * available.size()));
    if (used.insert(candidate).second) {
      selected.push_back(available[candidate]);
    }
  }
  return selected;
}

std::string csvText(const std::string & value)
{
  std::string escaped = value;
  std::size_t position = 0;
  while ((position = escaped.find('"', position)) != std::string::npos) {
    escaped.insert(position, 1, '"');
    position += 2;
  }
  return '"' + escaped + '"';
}

struct Stats
{
  double minimum{std::numeric_limits<double>::quiet_NaN()};
  double mean{std::numeric_limits<double>::quiet_NaN()};
  double maximum{std::numeric_limits<double>::quiet_NaN()};
  double standard_deviation{std::numeric_limits<double>::quiet_NaN()};
};

std::vector<double> successfulValues(
  const std::vector<Result> & results,
  const std::function<double(const Result &)> & selector)
{
  std::vector<double> values;
  for (const Result & result : results) {
    if (result.success) {
      values.push_back(selector(result));
    }
  }
  return values;
}

Stats stats(const std::vector<double> & values)
{
  if (values.empty()) {
    return {};
  }
  Stats result;
  const auto limits = std::minmax_element(values.begin(), values.end());
  result.minimum = *limits.first;
  result.maximum = *limits.second;
  result.mean = std::accumulate(values.begin(), values.end(), 0.0) / values.size();
  result.standard_deviation = 0.0;
  for (const double value : values) {
    result.standard_deviation += std::pow(value - result.mean, 2);
  }
  result.standard_deviation = std::sqrt(
    result.standard_deviation / static_cast<double>(values.size()));
  return result;
}

std::pair<double, double> wilson95(int successes, int total)
{
  constexpr double z = 1.959963984540054;
  const double n = total;
  const double p = static_cast<double>(successes) / n;
  const double denominator = 1.0 + z * z / n;
  const double center = (p + z * z / (2.0 * n)) / denominator;
  const double radius = z * std::sqrt(
    p * (1.0 - p) / n + z * z / (4.0 * n * n)) / denominator;
  return {std::max(0.0, center - radius), std::min(1.0, center + radius)};
}

void calculateMotion(Result & result)
{
  result.base_travel = 0.0;
  result.arm_travel = 0.0;
  for (std::size_t index = 1; index < result.trajectory.size(); ++index) {
    const auto & first = result.trajectory[index - 1].state;
    const auto & second = result.trajectory[index].state;
    result.base_travel += (second.head<2>() - first.head<2>()).norm();
    result.arm_travel += (second.tail(6) - first.tail(6)).norm();
  }
}

void writeCsv(const std::filesystem::path & path, const std::vector<Result> & results)
{
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("Cannot write " + path.string());
  }
  stream << std::setprecision(12)
    << "run_id,success,base_x,base_y,base_yaw,q1,q2,q3,q4,q5,q6,"
    << "entry_position_error_m,entry_normal_error_rad,planning_wall_time_s,"
    << "nominal_execution_time_s,approach_time_s,contact_time_s,trajectory_points,"
    << "base_travel_m,arm_travel_rad,min_joint_margin_rad,"
    << "min_joint_margin_normalized,min_manipulability,"
    << "min_self_collision_clearance_m,max_joint_speed_rad_s,"
    << "hybrid_expanded_nodes,ik_rejections,collision_rejections,error\n";
  for (const Result & result : results) {
    stream << result.id << ',' << (result.success ? 1 : 0);
    for (Eigen::Index index = 0; index < result.entry.size(); ++index) {
      stream << ',' << result.entry[index];
    }
    stream << ',' << result.entry_position_error
           << ',' << result.entry_normal_error
           << ',' << result.planning_wall_time
           << ',' << result.nominal_execution_time
           << ',' << result.approach_time
           << ',' << result.contact_time
           << ',' << result.points
           << ',' << result.base_travel
           << ',' << result.arm_travel
           << ',' << result.metrics.min_joint_margin
           << ',' << result.metrics.min_normalized_joint_margin
           << ',' << result.metrics.min_manipulability
           << ',' << result.metrics.min_self_collision_clearance
           << ',' << result.metrics.max_joint_speed
           << ',' << result.report.hybrid_expanded_nodes
           << ',' << result.report.reachability_rejections
           << ',' << result.report.collision_rejections
           << ',' << csvText(result.error) << '\n';
  }
}

void writeTrajectories(
  const std::filesystem::path & path, const wipe_planner::Planner & planner,
  const std::vector<Result> & results)
{
  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("Cannot write " + path.string());
  }
  stream << std::setprecision(12)
    << "run_id,waypoint,time_s,in_contact,base_x,base_y,base_yaw,"
    << "q1,q2,q3,q4,q5,q6,tool_x,tool_y,tool_z\n";
  for (const Result & result : results) {
    for (std::size_t index = 0; index < result.trajectory.size(); ++index) {
      const auto & waypoint = result.trajectory[index];
      const Eigen::Vector3d tool = planner.framePosition(waypoint.state, "tool0");
      stream << result.id << ',' << index << ',' << waypoint.time << ','
             << (waypoint.in_contact ? 1 : 0);
      for (Eigen::Index state = 0; state < waypoint.state.size(); ++state) {
        stream << ',' << waypoint.state[state];
      }
      stream << ',' << tool.x() << ',' << tool.y() << ',' << tool.z() << '\n';
    }
  }
}

void printMetric(
  std::ostream & stream, const std::string & name, const std::string & unit,
  const Stats & value)
{
  stream << "| " << name << " " << unit << " | " << value.minimum << " | "
         << value.mean << " | " << value.maximum << " | "
         << value.standard_deviation << " |\n";
}

void writeSummary(
  const std::filesystem::path & path, const Options & options,
  std::size_t available, const std::vector<Result> & results)
{
  const int successful = static_cast<int>(std::count_if(
    results.begin(), results.end(), [](const Result & result) {return result.success;}));
  const auto interval = wilson95(successful, static_cast<int>(results.size()));
  const auto execution = stats(successfulValues(results,
    [](const Result & result) {return result.nominal_execution_time;}));
  const auto approach = stats(successfulValues(results,
    [](const Result & result) {return result.approach_time;}));
  const auto contact = stats(successfulValues(results,
    [](const Result & result) {return result.contact_time;}));
  const auto base = stats(successfulValues(results,
    [](const Result & result) {return result.base_travel;}));
  const auto arm = stats(successfulValues(results,
    [](const Result & result) {return result.arm_travel;}));
  const auto joint = stats(successfulValues(results,
    [](const Result & result) {return result.metrics.min_normalized_joint_margin;}));
  const auto manipulability = stats(successfulValues(results,
    [](const Result & result) {return result.metrics.min_manipulability;}));
  const auto clearance = stats(successfulValues(results,
    [](const Result & result) {return result.metrics.min_self_collision_clearance;}));

  std::ofstream stream(path);
  if (!stream) {
    throw std::runtime_error("Cannot write " + path.string());
  }
  stream << std::fixed << std::setprecision(6)
    << "# Pre-contact-conditioned future-task rollout summary\n\n"
    << "- Experiment: `q_pre(i) -> direct surface-normal approach -> full wipe rollout`.\n"
    << "- No measured-state alignment or return to a canonical entry is inserted.\n"
    << "- Candidate pool: " << available
    << " collision-free whole-body IK states satisfying the same pre-contact tool pose.\n"
    << "- Selected: " << results.size() << " deterministic Halton entries (offset "
    << options.sequence_offset << ").\n"
    << "- Success: " << successful << '/' << results.size() << " ("
    << 100.0 * successful / results.size() << "%).\n"
    << "- Wilson 95% interval: [" << interval.first << ", " << interval.second << "].\n\n"
    << "## Successful rollout metrics (min / mean / max / std)\n\n"
    << "| Metric | Min | Mean | Max | Std |\n"
    << "|---|---:|---:|---:|---:|\n";
  printMetric(stream, "nominal execution time", "[s]", execution);
  printMetric(stream, "direct normal approach time", "[s]", approach);
  printMetric(stream, "contact coverage time", "[s]", contact);
  printMetric(stream, "base travel", "[m]", base);
  printMetric(stream, "accumulated arm travel", "[rad]", arm);
  printMetric(stream, "min normalized joint margin", "", joint);
  printMetric(stream, "min manipulability", "", manipulability);
  printMetric(stream, "min self-collision clearance", "[m]", clearance);
  stream
    << "\n## Interpretation boundary\n\n"
    << "Every rollout starts exactly at its CSV 9D entry state. Metrics cover only the "
       "normal approach and future wipe task. This remains an offline planning-level test; "
       "environment ESDF clearance, force tracking and closed-loop execution quality are not claimed.\n";
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parseOptions(argc, argv);
    std::filesystem::create_directories(options.output);
    wipe_planner::Planner planner(options.urdf, "tool0", options.task);
    const auto available = planner.precontactCandidates();
    const auto entries = selectEntries(
      available, options.samples, options.sequence_offset);

    std::vector<Result> results;
    results.reserve(entries.size());
    for (std::size_t index = 0; index < entries.size(); ++index) {
      Result result;
      result.id = static_cast<int>(index);
      result.entry = entries[index];
      result.entry_position_error = (
        planner.framePosition(result.entry, "tool0") - planner.precontactTarget()).norm();
      const Eigen::Vector3d tool_z =
        planner.frameRotation(result.entry, "tool0").col(2);
      result.entry_normal_error = std::acos(std::clamp(
        tool_z.dot(-planner.surfaceNormal()), -1.0, 1.0));
      const auto begin = std::chrono::steady_clock::now();
      try {
        result.trajectory = planner.planFromPrecontact(result.entry, result.report);
        if (result.trajectory.empty() ||
            !result.trajectory.front().state.isApprox(result.entry, 1.0e-12))
        {
          throw std::runtime_error("Rollout did not preserve q_pre as its exact first frame");
        }
        const auto contact = std::find_if(
          result.trajectory.begin(), result.trajectory.end(),
          [](const wipe_planner::Waypoint & waypoint) {return waypoint.in_contact;});
        if (contact == result.trajectory.end()) {
          throw std::runtime_error("Rollout never entered contact coverage");
        }
        result.approach_time = contact->time;
        result.nominal_execution_time = result.trajectory.back().time;
        result.contact_time = result.nominal_execution_time - result.approach_time;
        result.metrics = planner.evaluateTrajectory(result.trajectory);
        result.points = result.trajectory.size();
        calculateMotion(result);
        result.success = result.metrics.min_joint_margin >= -1.0e-8 &&
          result.metrics.min_self_collision_clearance >= -1.0e-8 &&
          result.metrics.evaluated_points == result.trajectory.size();
        if (!result.success) {
          result.error = "Post-check rejected joint margin or self-collision clearance";
        }
      } catch (const std::exception & error) {
        result.error = error.what();
        result.trajectory.clear();
      }
      result.planning_wall_time = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - begin).count();
      results.push_back(std::move(result));
      const Result & printed = results.back();
      std::cout << '[' << (index + 1) << '/' << entries.size() << "] "
                << (printed.success ? "SUCCESS" : "FAIL")
                << " base=(" << std::fixed << std::setprecision(2)
                << printed.entry[0] << ',' << printed.entry[1] << ','
                << printed.entry[2] << ") compute="
                << printed.planning_wall_time << " s, task="
                << printed.nominal_execution_time << " s";
      if (!printed.error.empty()) {
        std::cout << ", error=" << printed.error;
      }
      std::cout << std::endl;
    }

    const std::filesystem::path output(options.output);
    writeCsv(output / "rollouts.csv", results);
    writeTrajectories(output / "trajectories.csv", planner, results);
    writeSummary(output / "summary.md", options, available.size(), results);
    const int successful = static_cast<int>(std::count_if(
      results.begin(), results.end(), [](const Result & result) {return result.success;}));
    std::cout << "Completed: " << successful << '/' << results.size()
              << " successful from " << available.size()
              << " valid pre-contact candidates. Results: " << output << std::endl;
    return successful == static_cast<int>(results.size()) ? 0 : 2;
  } catch (const std::exception & error) {
    std::cerr << "Batch experiment failed: " << error.what() << std::endl;
    return 1;
  }
}
