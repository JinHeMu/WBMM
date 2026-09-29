#pragma once

#include <wbmm_core/robot_model.hpp>
#include <wbmm_core/trajectory.hpp>
#include <wbmm_core/types.hpp>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace wbmm::search
{

enum class ArmSeedStatus
{
  kSuccess,
  kInvalidInput,
  kMissingCollisionChecker,
  kCollisionCheckerError,
  kNoFeasibleSeed,
  kTimeout,
};

struct ArmSeedConfig
{
  // Arc-length spacing between base path points that get their own arm
  // configuration. Finer spacing tracks the base better and costs more checks.
  double waypoint_spacing{0.25};

  // Candidate configurations tried per waypoint, including the carry-over
  // candidate. The carry-over is always tried first so the seed stays as
  // continuous as possible and the arm is not re-shuffled without reason.
  std::size_t candidates_per_waypoint{64};

  // Samples taken between the previous accepted configuration and a candidate
  // when checking the swept motion, so the whole transition is validated and
  // not just its endpoints.
  std::size_t interpolation_steps{8};

  // The search must be reproducible: it uses a deterministic low-discrepancy
  // sequence, never a random engine seeded from the clock.
  std::uint64_t sequence_offset{0};

  double max_search_time{1.0};

  // Maximum joint travel from the previous accepted configuration to a
  // candidate. Bounds how far the arm may move between adjacent base points.
  double max_joint_step{0.6};
};

// true means the whole-body state is collision free.
using WholeBodyCollisionChecker = std::function<bool(
  const wbmm::core::Header &, const wbmm::core::WholeBodyState &)>;

struct ArmSeedResult
{
  bool success{false};
  ArmSeedStatus status{ArmSeedStatus::kInvalidInput};
  std::string message;

  // One configuration per resampled base path point. Empty unless successful.
  std::vector<wbmm::core::JointState> arm_seed;
  // Base path resampled at ArmSeedConfig::waypoint_spacing. Same length as
  // arm_seed on success.
  std::vector<wbmm::core::BaseState> base_path;

  std::size_t waypoints{0};
  std::size_t checked_candidates{0};
  double solve_time{0.0};

  // Fills a wbmm::core::SearchResult with this seed. Base search output, if
  // any, is preserved.
  [[nodiscard]] wbmm::core::SearchResult toSearchResult(
    wbmm::core::SearchResult base) const;
};

// Finds a collision-free arm configuration sequence along a base path.
//
// This is the WBMM redesign of the REMANI whole-body sampling stage. REMANI
// uses a bidirectional RRT* (about 2500 lines) whose complexity buys sampling
// optimality, which this problem does not need: what is required is a feasible,
// continuous configuration sequence along a known base path. A deterministic
// low-discrepancy sweep over the joint ranges is smaller, reproducible, and
// easier to debug. If probabilistic completeness is ever needed, an RRT backend
// can implement the same interface.
//
// The base path is resampled at waypoint_spacing. At each waypoint the previous
// configuration is tried first; if it is not collision free, candidates are
// drawn from the low-discrepancy sequence and the whole transition is checked,
// not just the endpoint.
[[nodiscard]] ArmSeedResult searchArmSeed(
  const wbmm::core::Header & header,
  const std::vector<wbmm::core::BaseState> & base_path,
  const wbmm::core::JointState & initial_joints,
  const wbmm::core::RobotLimits & limits,
  const WholeBodyCollisionChecker & collision_checker,
  const ArmSeedConfig & config = {});

}  // namespace wbmm::search
