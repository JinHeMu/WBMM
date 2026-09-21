#pragma once

#include "wbmm_environment/esdf_grid.hpp"

#include <memory>

namespace wbmm::environment
{

enum class LoadStatus {kNotImplemented = 0, kSuccess, kIoError, kInvalidData};

struct EsdfLoadResult
{
  LoadStatus status{LoadStatus::kNotImplemented};
  std::shared_ptr<const EsdfGrid> grid;
  std::string message{"TBD: NPZ ESDF loading is not implemented"};
};

class NpzEsdfLoader
{
public:
  // Path is supplied by the application; map1 and frame names are not hard-coded.
  [[nodiscard]] static EsdfLoadResult load(const std::string & path);
};

}  // namespace wbmm::environment
