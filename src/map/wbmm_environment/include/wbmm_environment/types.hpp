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
  // Map-level policy for voxels that were never observed; producers such as
  // nvblox record this in the archive. It only matters when a consumer opts
  // into conservative handling. The ESDF payload itself is defined everywhere
  // (the deployed map1 stores the clamp maximum in every unobserved voxel).
  bool unknown_is_occupied{false};
};

// xyz axes, C order: index = (ix * shape.y() + iy) * shape.z() + iz.
// observed is provenance metadata. It does NOT invalidate the esdf payload:
// every voxel carries a distance, and unobserved voxels are expected to hold
// the clamp maximum (measured: 100% of them do on maps/map1/site_remani.npz).
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
  // Retained for consumers that apply an explicit conservative policy. query()
  // itself never returns this: an unobserved corner is reported through
  // DistanceQuery::fully_observed instead of being rejected.
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
  // False when at least one interpolation corner was never observed. The
  // distance and gradient are still returned and still valid; rejecting on
  // this flag would discard almost the whole deployed map1, where only ~11% of
  // voxels have a fully observed 8-corner stencil.
  bool fully_observed{true};
  std::string message{"TBD: ESDF query is not implemented"};
};

}  // namespace wbmm::environment
