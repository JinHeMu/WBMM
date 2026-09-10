#include "ta_wbmp/planner.hpp"

#include <Eigen/Core>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

namespace
{
struct Options
{
  std::string urdf;
  std::string task;
  std::string output;
};

std::string valueAfter(const std::string & argument, const std::string & name)
{
  const std::string prefix = "--" + name + "=";
  return argument.rfind(prefix, 0) == 0 ? argument.substr(prefix.size()) : "";
}

Options parseOptions(int argc, char ** argv)
{
  Options result;
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (const std::string value = valueAfter(argument, "urdf"); !value.empty()) {
      result.urdf = value;
    } else if (const std::string value = valueAfter(argument, "task"); !value.empty()) {
      result.task = value;
    } else if (const std::string value = valueAfter(argument, "output"); !value.empty()) {
      result.output = value;
    } else {
      throw std::runtime_error("Unknown or incomplete argument: " + argument);
    }
  }
  if (result.urdf.empty() || result.task.empty() || result.output.empty()) {
    throw std::runtime_error(
      "Usage: ta_wbmp_scenario_runner --urdf=FILE --task=FILE --output=DIR");
  }
  return result;
}

void writeState(std::ostream & stream, const Eigen::VectorXd & state)
{
  for (Eigen::Index index = 0; index < state.size(); ++index) {
    stream << (index == 0 ? "" : ",") << state[index];
  }
}

void writeTask(const std::filesystem::path & output,
               const ta_wbmp::TaskTrajectory & task)
{
  std::ofstream stream(output / "task_trajectory.csv");
  stream << std::setprecision(12)
         << "index,progress,x,y,z,qx,qy,qz,qw,nx,ny,nz,tx,ty,tz,"
            "nominal_speed,contact,label\n";
  for (std::size_t index = 0; index < task.points.size(); ++index) {
    const auto & point = task.points[index];
    stream << index << ',' << point.progress << ','
           << point.position.x() << ',' << point.position.y() << ','
           << point.position.z() << ',' << point.orientation.x() << ','
           << point.orientation.y() << ',' << point.orientation.z() << ','
           << point.orientation.w() << ',' << point.surface_normal.x() << ','
           << point.surface_normal.y() << ',' << point.surface_normal.z() << ','
           << point.tangent.x() << ',' << point.tangent.y() << ','
           << point.tangent.z() << ',' << point.nominal_speed << ','
           << (point.contact ? 1 : 0) << ',' << point.label << '\n';
  }
}

void writeCandidates(const std::filesystem::path & output,
                     const std::vector<ta_wbmp::CandidateMetrics> & values)
{
  std::ofstream stream(output / "candidates.csv");
  stream << std::setprecision(12)
         << "candidate_id,feasible,standoff,longitudinal_offset,yaw_offset,"
            "score,max_position_error,max_axis_error,min_joint_margin,"
            "min_manipulability,min_sigma,base_path_length,arm_path_length,"
            "navigation_cost_estimate,failure_reason\n";
  for (const auto & value : values) {
    stream << value.candidate_id << ',' << (value.feasible ? 1 : 0) << ','
           << value.standoff << ',' << value.longitudinal_offset << ','
           << value.yaw_offset << ',' << value.score << ','
           << value.max_position_error << ',' << value.max_axis_error << ','
           << value.min_joint_margin << ',' << value.min_manipulability << ','
           << value.min_sigma << ',' << value.base_path_length << ','
           << value.arm_path_length << ',' << value.navigation_cost_estimate
           << ',' << value.failure_reason << '\n';
  }
}

void writeWholeBody(const std::filesystem::path & output,
                    const ta_wbmp::Plan & plan)
{
  std::ofstream stream(output / "whole_body_trajectory.csv");
  stream << std::setprecision(12)
         << "index,time,phase,base_x,base_y,base_yaw,q1,q2,q3,q4,q5,q6,"
            "target_x,target_y,target_z,has_task_target\n";
  for (std::size_t index = 0; index < plan.waypoints.size(); ++index) {
    const auto & waypoint = plan.waypoints[index];
    stream << index << ',' << waypoint.time << ',' << waypoint.phase << ',';
    writeState(stream, waypoint.state);
    stream << ',' << waypoint.task_target.x() << ',' << waypoint.task_target.y()
           << ',' << waypoint.task_target.z() << ','
           << (waypoint.has_task_target ? 1 : 0) << '\n';
  }
}

void writeSummary(const std::filesystem::path & output,
                  const ta_wbmp::Plan & plan,
                  const Options & options)
{
  std::ofstream stream(output / "summary.yaml");
  const auto & report = plan.report;
  stream << std::setprecision(12)
         << "task_file: " << options.task << '\n'
         << "urdf_file: " << options.urdf << '\n'
         << "task_name: " << report.task_name << '\n'
         << "task_type: " << report.task_type << '\n'
         << "success: " << (report.success ? "true" : "false") << '\n'
         << "task_points: " << plan.task_trajectory.points.size() << '\n'
         << "whole_body_points: " << plan.waypoints.size() << '\n'
         << "duration_s: " << report.duration << '\n'
         << "candidate_count: " << report.candidate_count << '\n'
         << "feasible_candidate_count: " << report.feasible_candidate_count << '\n'
         << "selected_candidate_id: " << report.selected_candidate_id << '\n'
         << "selected_score: " << report.selected_future_task_score << '\n'
         << "min_joint_margin: " << report.minimum_joint_margin << '\n'
         << "min_manipulability: " << report.minimum_manipulability << '\n'
         << "min_sigma: " << report.minimum_sigma << '\n'
         << "task_base_path_length_m: " << report.task_base_path_length << '\n'
         << "max_contact_position_error_m: "
         << report.max_contact_position_error << '\n'
         << "max_tool_axis_error_rad: " << report.max_tool_axis_error << '\n'
         << "max_lateral_velocity_mps: " << report.max_lateral_velocity << '\n'
         << "execution_contract:\n"
         << "  navigation_backend: REMANI\n"
         << "  tracking_backend: OCS2_MPC\n"
         << "  remani_goal_9d: [";
  writeState(stream, plan.remani_navigation_goal);
  stream << "]\n  task_entry_9d: [";
  writeState(stream, plan.task_entry_state);
  stream << "]\n  execution_start_index: "
         << plan.execution_start_index
         << "\n  task_start_index: " << plan.task_start_index << '\n';
}
}  // namespace

int main(int argc, char ** argv)
{
  try {
    const Options options = parseOptions(argc, argv);
    const std::filesystem::path output(options.output);
    std::filesystem::create_directories(output);
    ta_wbmp::TaskAwarePlanner planner(options.urdf, "tool0", options.task);
    const ta_wbmp::Plan plan = planner.plan();
    writeTask(output, plan.task_trajectory);
    writeCandidates(output, plan.candidate_evaluations);
    writeWholeBody(output, plan);
    writeSummary(output, plan, options);
    std::cout << "PASS " << plan.report.task_name << " task_points="
              << plan.task_trajectory.points.size() << " whole_body_points="
              << plan.waypoints.size() << " selected_candidate="
              << plan.report.selected_candidate_id << " output=" << output
              << std::endl;
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "FAIL " << error.what() << std::endl;
    return 1;
  }
}
