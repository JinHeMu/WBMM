#include "ta_wbmp/task_trajectory.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ta_wbmp
{
namespace
{
Eigen::Vector3d vector3(const YAML::Node & node, const std::string & name)
{
  if (!node || node.size() != 3U) {
    throw std::runtime_error(name + " must contain three values");
  }
  return Eigen::Vector3d(
    node[0].as<double>(), node[1].as<double>(), node[2].as<double>());
}

Eigen::Vector2d vector2(const YAML::Node & node, const std::string & name)
{
  if (!node || node.size() != 2U) {
    throw std::runtime_error(name + " must contain two values");
  }
  return Eigen::Vector2d(node[0].as<double>(), node[1].as<double>());
}

Eigen::Matrix3d rotationMatrix(const YAML::Node & node)
{
  if (!node || node.size() != 9U) {
    throw std::runtime_error(
      "task.constraints.desired_tool_rotation must contain nine values");
  }
  Eigen::Matrix3d result;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      result(row, column) = node[row * 3 + column].as<double>();
    }
  }
  return result;
}

template<typename T>
T valueOr(const YAML::Node & node, const char * key, const T & fallback)
{
  return node && node[key] ? node[key].as<T>() : fallback;
}

void parseExecution(
  const YAML::Node & task, TaskExecutionConfig & execution)
{
  const YAML::Node root = task["execution"];
  const YAML::Node mpc = root ? root["mpc"] : YAML::Node();
  execution.mpc.reference_rate = valueOr(
    mpc, "reference_rate", execution.mpc.reference_rate);
  execution.mpc.reference_horizon = valueOr(
    mpc, "reference_horizon", execution.mpc.reference_horizon);
  execution.mpc.reference_dt = valueOr(
    mpc, "reference_dt", execution.mpc.reference_dt);
  execution.mpc.tracking_slow_squared_tolerance = valueOr(
    mpc, "tracking_slow_squared_tolerance",
    execution.mpc.tracking_slow_squared_tolerance);
  execution.mpc.tracking_stop_squared_tolerance = valueOr(
    mpc, "tracking_stop_squared_tolerance",
    execution.mpc.tracking_stop_squared_tolerance);

  const YAML::Node force = root ? root["force_control"] : YAML::Node();
  execution.force.enabled = valueOr(
    force, "enabled", execution.force.enabled);
  execution.force.mode = valueOr(
    force, "mode", execution.force.mode);
  execution.force.wrench_topic = valueOr(
    force, "wrench_topic", execution.force.wrench_topic);
  execution.force.force_axis = valueOr(
    force, "force_axis", execution.force.force_axis);
  execution.force.absolute_force = valueOr(
    force, "absolute_force", execution.force.absolute_force);
  execution.force.desired_force = valueOr(
    force, "desired_force", execution.force.desired_force);
  execution.force.mass = valueOr(force, "mass", execution.force.mass);
  execution.force.damping = valueOr(
    force, "damping", execution.force.damping);
  execution.force.stiffness = valueOr(
    force, "stiffness", execution.force.stiffness);
  execution.force.filter_alpha = valueOr(
    force, "filter_alpha", execution.force.filter_alpha);
  execution.force.max_offset = valueOr(
    force, "max_offset", execution.force.max_offset);
  execution.force.max_velocity = valueOr(
    force, "max_velocity", execution.force.max_velocity);
  execution.force.base_share = valueOr(
    force, "base_share", execution.force.base_share);
  execution.force.max_base_delta = valueOr(
    force, "max_base_delta", execution.force.max_base_delta);
  execution.force.max_joint_delta = valueOr(
    force, "max_joint_delta", execution.force.max_joint_delta);
  execution.force.sensor_timeout = valueOr(
    force, "sensor_timeout", execution.force.sensor_timeout);
  execution.force.progress_full_speed_error = valueOr(
    force, "progress_full_speed_error",
    execution.force.progress_full_speed_error);
  execution.force.progress_pause_error = valueOr(
    force, "progress_pause_error", execution.force.progress_pause_error);
  execution.force.progress_min_scale = valueOr(
    force, "progress_min_scale", execution.force.progress_min_scale);
  execution.force.hard_limit = valueOr(
    force, "hard_limit", execution.force.hard_limit);
  execution.force.spike_rejection_n = valueOr(
    force, "spike_rejection_n", execution.force.spike_rejection_n);
  execution.force.spike_confirm_samples = std::max<int>(1, valueOr(
    force, "spike_confirm_samples", execution.force.spike_confirm_samples));

  if (execution.force.mode != "admittance" &&
      execution.force.mode != "constant_force" &&
      execution.force.mode != "force_follow")
  {
    throw std::runtime_error(
      "task.execution.force_control.mode must be admittance, "
      "constant_force, or force_follow");
  }
  if (execution.force.force_axis != "x" &&
      execution.force.force_axis != "y" &&
      execution.force.force_axis != "z")
  {
    throw std::runtime_error(
      "task.execution.force_control.force_axis must be x, y, or z");
  }
}

void normalizePlane(TaskGeometry & geometry)
{
  if (geometry.normal.norm() < 1.0e-9 || geometry.axis_u.norm() < 1.0e-9) {
    throw std::runtime_error("Task surface normal and axis_u must be non-zero");
  }
  geometry.normal.normalize();
  geometry.axis_u -= geometry.normal * geometry.normal.dot(geometry.axis_u);
  if (geometry.axis_u.norm() < 1.0e-9) {
    throw std::runtime_error("Task surface axis_u must be tangent to the surface");
  }
  geometry.axis_u.normalize();
  geometry.axis_v = geometry.normal.cross(geometry.axis_u).normalized();
}

Eigen::Matrix3d orientationForNormal(
  const Eigen::Matrix3d & reference, const Eigen::Vector3d & normal,
  const Eigen::Vector3d & reference_normal)
{
  if ((normal - reference_normal).norm() < 1.0e-9) {
    return reference;
  }
  const Eigen::Quaterniond alignment = Eigen::Quaterniond::FromTwoVectors(
    reference_normal, normal);
  return alignment.toRotationMatrix() * reference;
}

std::vector<Eigen::Vector2d> rasPolyline()
{
  std::vector<Eigen::Vector2d> points;
  points.emplace_back(0.00, 0.00);
  auto line = [&points](double u, double v) {points.emplace_back(u, v);};
  auto cubic = [&points](const Eigen::Vector2d & c1,
                         const Eigen::Vector2d & c2,
                         const Eigen::Vector2d & end) {
      const Eigen::Vector2d start = points.back();
      constexpr int kSamples = 12;
      for (int sample = 1; sample <= kSamples; ++sample) {
        const double t = static_cast<double>(sample) / kSamples;
        const double s = 1.0 - t;
        points.push_back(
          s * s * s * start + 3.0 * s * s * t * c1 +
          3.0 * s * t * t * c2 + t * t * t * end);
      }
    };
  line(0.00, 1.00);
  line(0.18, 1.00);
  cubic({0.31, 1.00}, {0.31, 0.52}, {0.18, 0.52});
  line(0.00, 0.52);
  line(0.30, 0.00);
  line(0.47, 1.00);
  line(0.55, 0.42);
  line(0.39, 0.42);
  line(0.55, 0.42);
  line(0.62, 0.00);
  line(0.68, 0.00);
  cubic({0.88, 0.00}, {1.00, 0.04}, {1.00, 0.23});
  cubic({1.00, 0.44}, {0.83, 0.49}, {0.71, 0.52});
  cubic({0.63, 0.59}, {0.64, 0.76}, {0.73, 0.81});
  cubic({0.80, 0.94}, {0.92, 1.00}, {1.00, 1.00});
  return points;
}

void densify(
  const std::vector<Eigen::Vector3d> & positions,
  const TaskGeometry & geometry, const Eigen::Matrix3d & rotation,
  double spacing, double speed, const std::string & label,
  std::vector<TaskWaypoint> & output)
{
  if (positions.empty()) {
    return;
  }
  const auto append = [&](const Eigen::Vector3d & position,
                          const Eigen::Vector3d & tangent) {
      TaskWaypoint waypoint;
      waypoint.position = position;
      waypoint.orientation = Eigen::Quaterniond(rotation);
      waypoint.surface_normal = geometry.normal;
      waypoint.tangent = tangent.norm() > 1.0e-9 ?
        tangent.normalized() : geometry.axis_u;
      waypoint.nominal_speed = speed;
      waypoint.contact = true;
      waypoint.label = label;
      output.push_back(std::move(waypoint));
    };
  if (output.empty()) {
    const Eigen::Vector3d tangent = positions.size() > 1U ?
      positions[1] - positions[0] : geometry.axis_u;
    append(positions.front(), tangent);
  }
  for (std::size_t index = 1; index < positions.size(); ++index) {
    const Eigen::Vector3d delta = positions[index] - positions[index - 1];
    const int count = std::max(
      1, static_cast<int>(std::ceil(delta.norm() / std::max(0.01, spacing))));
    for (int sample = 1; sample <= count; ++sample) {
      append(
        positions[index - 1] +
        static_cast<double>(sample) / count * delta, delta);
    }
  }
}
}  // namespace

TaskTrajectoryGenerator::TaskTrajectoryGenerator(std::string task_file)
: task_file_(std::move(task_file))
{
}

TaskTrajectory TaskTrajectoryGenerator::generate() const
{
  const YAML::Node task = YAML::LoadFile(task_file_)["task"];
  if (!task) {
    throw std::runtime_error("Task YAML must contain a 'task' mapping");
  }
  TaskTrajectory result;
  result.name = task["name"].as<std::string>();
  result.frame_id = task["frame_id"].as<std::string>();
  parseExecution(task, result.execution);

  const YAML::Node surface = task["surface"];
  result.geometry.surface_type = surface["type"] ?
    surface["type"].as<std::string>() : "planar";
  result.geometry.center = vector3(surface["center"], "task.surface.center");
  if (result.geometry.surface_type == "cylindrical") {
    result.geometry.radius = surface["radius"].as<double>();
    result.geometry.u_limits = vector2(
      surface["angle_limits"], "task.surface.angle_limits");
    result.geometry.v_limits = vector2(
      surface["z_limits"], "task.surface.z_limits");
    result.geometry.normal = Eigen::Vector3d(0.0, -1.0, 0.0);
    result.geometry.axis_u = Eigen::Vector3d::UnitX();
    result.geometry.axis_v = Eigen::Vector3d::UnitZ();
  } else {
    result.geometry.normal = vector3(
      surface["normal_into_room"], "task.surface.normal_into_room");
    if (surface["axis_u"] && surface["u_limits"] && surface["v_limits"]) {
      result.geometry.local_coordinates = true;
      result.geometry.axis_u = vector3(surface["axis_u"], "task.surface.axis_u");
      result.geometry.u_limits = vector2(surface["u_limits"], "task.surface.u_limits");
      result.geometry.v_limits = vector2(surface["v_limits"], "task.surface.v_limits");
      normalizePlane(result.geometry);
    } else {
      // Backward-compatible world-X/world-Z planar task schema.
      result.geometry.axis_u = Eigen::Vector3d::UnitX();
      result.geometry.axis_v = Eigen::Vector3d::UnitZ();
      result.geometry.u_limits = vector2(surface["x_limits"], "task.surface.x_limits");
      result.geometry.v_limits = vector2(surface["z_limits"], "task.surface.z_limits");
    }
  }

  const YAML::Node pattern = task["pattern"];
  const std::string pattern_type = pattern["type"].as<std::string>();
  const double spacing = pattern["sample_spacing"].as<double>();
  const double tangential_speed = pattern["tangential_speed"].as<double>();
  const double corner_speed = pattern["row_change_speed"] ?
    pattern["row_change_speed"].as<double>() : tangential_speed;
  const Eigen::Matrix3d reference_rotation = rotationMatrix(
    task["constraints"]["desired_tool_rotation"]);

  const auto point_on_surface = [&](double u, double v) {
      if (result.geometry.surface_type == "cylindrical") {
        return Eigen::Vector3d(
          result.geometry.center.x() + result.geometry.radius * std::sin(u),
          result.geometry.center.y() - result.geometry.radius * std::cos(u), v);
      }
      return result.geometry.local_coordinates ?
        result.geometry.center + u * result.geometry.axis_u +
        v * result.geometry.axis_v : Eigen::Vector3d(
        u, result.geometry.center.y(), v);
    };
  const auto normal_at = [&](double u) {
      return result.geometry.surface_type == "cylindrical" ?
        Eigen::Vector3d(std::sin(u), -std::cos(u), 0.0) :
        result.geometry.normal;
    };

  if (pattern_type == "raster" || pattern_type == "cylindrical_raster") {
    result.type = TaskTrajectoryType::SURFACE_RASTER;
    const int rows = std::max(1, pattern["rows"].as<int>());
    const int columns = std::max(2, pattern["columns"].as<int>());
    std::vector<Eigen::Vector2d> keys;
    for (int row = 0; row < rows; ++row) {
      const double row_ratio = rows == 1 ? 0.5 :
        static_cast<double>(row) / static_cast<double>(rows - 1);
      const double v = result.geometry.v_limits[0] + row_ratio *
        (result.geometry.v_limits[1] - result.geometry.v_limits[0]);
      for (int column = 0; column < columns; ++column) {
        const int ordered = row % 2 == 0 ? column : columns - 1 - column;
        const double ratio = static_cast<double>(ordered) /
          static_cast<double>(columns - 1);
        const double u = result.geometry.u_limits[0] + ratio *
          (result.geometry.u_limits[1] - result.geometry.u_limits[0]);
        keys.emplace_back(u, v);
      }
    }
    for (std::size_t index = 0; index < keys.size(); ++index) {
      const Eigen::Vector3d position = point_on_surface(keys[index].x(), keys[index].y());
      const Eigen::Vector3d normal = normal_at(keys[index].x()).normalized();
      const Eigen::Matrix3d rotation = orientationForNormal(
        reference_rotation, normal, result.geometry.normal);
      if (index == 0U) {
        TaskWaypoint first;
        first.position = position;
        first.orientation = Eigen::Quaterniond(rotation);
        first.surface_normal = normal;
        first.tangent = result.geometry.axis_u;
        first.nominal_speed = tangential_speed;
        first.label = "raster";
        result.points.push_back(std::move(first));
        continue;
      }
      const Eigen::Vector3d previous = point_on_surface(
        keys[index - 1].x(), keys[index - 1].y());
      const Eigen::Vector3d delta = position - previous;
      const int count = std::max(
        1, static_cast<int>(std::ceil(delta.norm() / std::max(0.01, spacing))));
      const bool row_change = std::abs(keys[index].y() - keys[index - 1].y()) > 1.0e-9;
      for (int sample = 1; sample <= count; ++sample) {
        const double ratio = static_cast<double>(sample) / count;
        const double u = keys[index - 1].x() + ratio *
          (keys[index].x() - keys[index - 1].x());
        const double v = keys[index - 1].y() + ratio *
          (keys[index].y() - keys[index - 1].y());
        const Eigen::Vector3d sample_normal = normal_at(u).normalized();
        TaskWaypoint waypoint;
        waypoint.position = result.geometry.surface_type == "cylindrical" ?
          point_on_surface(u, v) : previous + ratio * delta;
        waypoint.orientation = Eigen::Quaterniond(orientationForNormal(
          reference_rotation, sample_normal, result.geometry.normal));
        waypoint.surface_normal = sample_normal;
        waypoint.tangent = delta.normalized();
        waypoint.nominal_speed = row_change ? corner_speed : tangential_speed;
        waypoint.label = row_change ? "raster_transition" : "raster";
        result.points.push_back(std::move(waypoint));
      }
    }
  } else if (pattern_type == "ras") {
    result.type = TaskTrajectoryType::RAS_DRAWING;
    const double width = pattern["width"].as<double>();
    const double height = pattern["height"].as<double>();
    std::vector<Eigen::Vector3d> positions;
    for (const Eigen::Vector2d & point : rasPolyline()) {
      positions.push_back(
        result.geometry.center + (point.x() - 0.5) * width *
        result.geometry.axis_u + (point.y() - 0.5) * height *
        result.geometry.axis_v);
    }
    densify(
      positions, result.geometry, reference_rotation, spacing,
      tangential_speed, "ras", result.points);
  } else if (pattern_type == "waypoint_sequence") {
    result.type = TaskTrajectoryType::WAYPOINT_SEQUENCE;
    for (const YAML::Node & value : pattern["waypoints"]) {
      TaskWaypoint waypoint;
      waypoint.position = vector3(value["position"], "task.pattern.waypoint.position");
      waypoint.surface_normal = value["normal"] ?
        vector3(value["normal"], "task.pattern.waypoint.normal").normalized() :
        result.geometry.normal;
      waypoint.orientation = value["orientation_xyzw"] ?
        Eigen::Quaterniond(
        value["orientation_xyzw"][3].as<double>(),
        value["orientation_xyzw"][0].as<double>(),
        value["orientation_xyzw"][1].as<double>(),
        value["orientation_xyzw"][2].as<double>()) :
        Eigen::Quaterniond(reference_rotation);
      waypoint.nominal_speed = value["speed"] ?
        value["speed"].as<double>() : tangential_speed;
      waypoint.contact = value["contact"] ? value["contact"].as<bool>() : false;
      waypoint.label = value["label"] ? value["label"].as<std::string>() : "waypoint";
      if (!result.points.empty()) {
        waypoint.tangent = (waypoint.position - result.points.back().position).normalized();
      }
      result.points.push_back(std::move(waypoint));
    }
  } else {
    throw std::runtime_error("Unsupported task pattern type: " + pattern_type);
  }

  if (result.points.empty()) {
    throw std::runtime_error("Task trajectory generator produced no waypoints");
  }
  double length = 0.0;
  for (std::size_t index = 1; index < result.points.size(); ++index) {
    length += (result.points[index].position - result.points[index - 1].position).norm();
    result.points[index].progress = length;
  }
  if (length > 1.0e-9) {
    for (TaskWaypoint & waypoint : result.points) {
      waypoint.progress /= length;
    }
  }
  return result;
}

std::string toString(TaskTrajectoryType type)
{
  switch (type) {
    case TaskTrajectoryType::SURFACE_RASTER: return "surface_raster";
    case TaskTrajectoryType::RAS_DRAWING: return "ras_drawing";
    case TaskTrajectoryType::WAYPOINT_SEQUENCE: return "waypoint_sequence";
  }
  return "unknown";
}

}  // namespace ta_wbmp
