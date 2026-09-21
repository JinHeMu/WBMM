#pragma once

#include "wbmm_environment/types.hpp"

namespace wbmm::environment
{

// Concrete grid boundary. No backend hierarchy until a second implementation exists.
class EsdfGrid
{
public:
  explicit EsdfGrid(EsdfGridData data = {});

  [[nodiscard]] const MapInfo & info() const noexcept;
  [[nodiscard]] const EsdfGridData & data() const noexcept;

  // position is in frame_id; no implicit TF conversion. Currently NotImplemented.
  [[nodiscard]] DistanceQuery query(
    const std::string & frame_id, const Eigen::Vector3d & position) const;

private:
  EsdfGridData data_;
};

}  // namespace wbmm::environment
