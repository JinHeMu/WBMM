#include "wbmm_environment/esdf_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace wbmm::environment
{
namespace
{

std::size_t address(
  const Eigen::Vector3i & index, const Eigen::Vector3i & shape) noexcept
{
  return (static_cast<std::size_t>(index.x()) *
            static_cast<std::size_t>(shape.y()) +
          static_cast<std::size_t>(index.y())) *
           static_cast<std::size_t>(shape.z()) +
         static_cast<std::size_t>(index.z());
}

bool isFinite(const Eigen::Vector3d & value)
{
  return value.array().isFinite().all();
}

bool hasObservedData(const EsdfGridData & data) noexcept
{
  return data.observed.size() == data.esdf.size();
}

}  // namespace

EsdfGrid::EsdfGrid(EsdfGridData data) : data_(std::move(data)) {}

const MapInfo & EsdfGrid::info() const noexcept {return data_.info;}

const EsdfGridData & EsdfGrid::data() const noexcept {return data_;}

DistanceQuery EsdfGrid::query(
  const std::string & frame_id, const Eigen::Vector3d & position) const
{
  DistanceQuery result;

  const auto & info = data_.info;
  if (info.shape.x() <= 0 || info.shape.y() <= 0 || info.shape.z() <= 0 ||
    !(info.voxel_size > 0.0) || !std::isfinite(info.voxel_size) ||
    !isFinite(info.origin) ||
    data_.esdf.size() != static_cast<std::size_t>(info.shape.x()) *
        static_cast<std::size_t>(info.shape.y()) *
        static_cast<std::size_t>(info.shape.z())) {
    result.status = QueryStatus::kInvalidInput;
    result.message = "ESDF grid metadata or payload is invalid.";
    return result;
  }

  if (frame_id != info.frame_id) {
    result.status = QueryStatus::kFrameMismatch;
    result.message =
      "Query frame '" + frame_id + "' does not match ESDF frame '" +
      info.frame_id + "'.";
    return result;
  }

  if (!isFinite(position)) {
    result.status = QueryStatus::kInvalidInput;
    result.message = "Query position contains NaN or Inf.";
    return result;
  }

  const Eigen::Vector3d lower_bound = info.origin;
  const Eigen::Vector3d upper_bound =
    info.origin + info.voxel_size * info.shape.cast<double>();
  constexpr double kBoundsTolerance = 1e-9;
  for (int axis = 0; axis < 3; ++axis) {
    if (position(axis) < lower_bound(axis) - kBoundsTolerance ||
      position(axis) > upper_bound(axis) + kBoundsTolerance) {
      result.status = QueryStatus::kOutOfBounds;
      result.message = "Query point is outside the ESDF grid bounds.";
      return result;
    }
  }

  // Grid samples are located at voxel centers:
  //   center(i) = origin + (i + 0.5) * voxel_size
  // Shift the query point by half a voxel to find the lower corner index.
  // Points in the half-voxel boundary layer are handled by clamping the
  // index and interpolation fraction, matching the REMANI static-ESDF query.
  const Eigen::Vector3d shifted =
    position - 0.5 * info.voxel_size * Eigen::Vector3d::Ones();

  Eigen::Vector3i index;
  for (int axis = 0; axis < 3; ++axis) {
    const double coordinate =
      (shifted(axis) - info.origin(axis)) / info.voxel_size;
    const int raw_index = static_cast<int>(std::floor(coordinate));
    index(axis) = std::clamp(raw_index, 0, info.shape(axis) - 1);
  }

  const Eigen::Vector3d lower_corner_center =
    info.origin +
    (index.cast<double>() + 0.5 * Eigen::Vector3d::Ones()) *
      info.voxel_size;
  Eigen::Vector3d fraction =
    (position - lower_corner_center) / info.voxel_size;
  for (int axis = 0; axis < 3; ++axis) {
    fraction(axis) = std::clamp(fraction(axis), 0.0, 1.0);
  }

  double values[2][2][2];
  bool observed = true;
  const bool has_observed = hasObservedData(data_);

  for (int x = 0; x < 2; ++x) {
    for (int y = 0; y < 2; ++y) {
      for (int z = 0; z < 2; ++z) {
        Eigen::Vector3i corner = index + Eigen::Vector3i(x, y, z);
        for (int axis = 0; axis < 3; ++axis) {
          corner(axis) = std::clamp(corner(axis), 0, info.shape(axis) - 1);
        }
        const std::size_t id = address(corner, info.shape);
        values[x][y][z] = static_cast<double>(data_.esdf[id]);
        if (!std::isfinite(values[x][y][z])) {
          result.status = QueryStatus::kInvalidInput;
          result.message = "ESDF contains non-finite values.";
          return result;
        }
        if (has_observed && data_.observed[id] == 0U) {
          observed = false;
        }
      }
    }
  }

  const double v00 =
    (1.0 - fraction(0)) * values[0][0][0] +
    fraction(0) * values[1][0][0];
  const double v01 =
    (1.0 - fraction(0)) * values[0][0][1] +
    fraction(0) * values[1][0][1];
  const double v10 =
    (1.0 - fraction(0)) * values[0][1][0] +
    fraction(0) * values[1][1][0];
  const double v11 =
    (1.0 - fraction(0)) * values[0][1][1] +
    fraction(0) * values[1][1][1];

  const double v0 = (1.0 - fraction(1)) * v00 + fraction(1) * v10;
  const double v1 = (1.0 - fraction(1)) * v01 + fraction(1) * v11;
  result.distance = (1.0 - fraction(2)) * v0 + fraction(2) * v1;

  const double inverse_voxel = 1.0 / info.voxel_size;
  result.gradient(2) = (v1 - v0) * inverse_voxel;
  result.gradient(1) =
    ((1.0 - fraction(2)) * (v10 - v00) +
     fraction(2) * (v11 - v01)) *
    inverse_voxel;
  result.gradient(0) =
    (1.0 - fraction(2)) * (1.0 - fraction(1)) *
      (values[1][0][0] - values[0][0][0]) +
    (1.0 - fraction(2)) * fraction(1) *
      (values[1][1][0] - values[0][1][0]) +
    fraction(2) * (1.0 - fraction(1)) *
      (values[1][0][1] - values[0][0][1]) +
    fraction(2) * fraction(1) *
      (values[1][1][1] - values[0][1][1]);
  result.gradient(0) *= inverse_voxel;

  if (!observed) {
    result.status = QueryStatus::kUnknown;
    result.message = "At least one interpolation corner is unobserved.";
    result.gradient.setConstant(std::numeric_limits<double>::quiet_NaN());
    result.gradient_valid = false;
    return result;
  }

  if (!std::isfinite(result.distance) || !isFinite(result.gradient)) {
    result.status = QueryStatus::kInvalidInput;
    result.message = "Interpolated ESDF distance or gradient is non-finite.";
    return result;
  }

  result.status = QueryStatus::kSuccess;
  result.gradient_valid = true;
  result.message = "ok";
  return result;
}

}  // namespace wbmm::environment
