#pragma once

#include <Eigen/Core>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace wbmm::environment
{

// SI units. origin is the lower grid boundary in frame_id.
struct MapInfo
{
  std::string frame_id;
  Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
  double voxel_size{0.0};
  Eigen::Vector3i shape{Eigen::Vector3i::Zero()};
};

// xyz axes, C order: index = (ix * shape.y() + iy) * shape.z() + iz.
// An unknown voxel is identified by observed, not by the sign of esdf.
struct EsdfGridData
{
  MapInfo info;
  std::vector<float> esdf;
  std::vector<std::uint8_t> occupancy;
  std::vector<std::uint8_t> observed;
};

enum class QueryStatus
{
  kNotImplemented = 0,
  kSuccess,
  kInvalidInput,
  kFrameMismatch,
  kOutOfBounds,
  kUnknown,
};

struct DistanceQuery
{
  QueryStatus status{QueryStatus::kNotImplemented};
  double distance{std::numeric_limits<double>::quiet_NaN()};
  // d(distance)/d(position), expressed in the map frame.
  Eigen::Vector3d gradient{Eigen::Vector3d::Constant(
      std::numeric_limits<double>::quiet_NaN())};
  bool gradient_valid{false};
  std::string message{"TBD: ESDF query is not implemented"};
};

}  // namespace wbmm::environment
