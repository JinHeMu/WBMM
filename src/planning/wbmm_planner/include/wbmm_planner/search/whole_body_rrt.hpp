#pragma once

#include <optional>
#include <wbmm_planner/search/arm_seed_search.hpp>
#include <wbmm_planner/search/kino_astar.hpp>

namespace wbmm::search {

struct WholeBodyRrtConfig {
  std::size_t max_nodes{12000};
  std::size_t max_iterations{24000};
  double max_search_time{3.0};
  double translation_step{0.4};
  double joint_step{0.6};
  double collision_translation_step{0.02};
  double collision_angle_step{0.03};
  std::uint32_t random_seed{1};
};

struct WholeBodyRrtResult {
  bool success{false};
  // Invalid input/checker exceptions must not be hidden by another backend.
  bool fatal{false};
  std::string message;
  wbmm::core::SearchResult search;
  // Exactly one constant-control differential-drive edge per adjacent pair.
  // An edge with v=omega=0 permits arm reconfiguration at a fixed base.
  std::vector<MotionPrimitive> primitives;
  double solve_time{0.0};
  std::size_t generated_nodes{0};
};

// A tree in (base-path index, joints). It retains alternative parents and can
// reconfigure at a fixed index. Unlike a greedy sweep, a failed continuation
// does not discard previously discovered branches. This is feasible RRT, not RRT*.
WholeBodyRrtResult
sampleArmRrt(const wbmm::core::Header &, const BaseSearchResult &, const wbmm::core::JointState &,
             const wbmm::core::RobotLimits &, const WholeBodyCollisionChecker &,
             const WholeBodyRrtConfig & = {},
             const std::optional<wbmm::core::JointState> &goal_joints = std::nullopt);

// Fallback tree in (x,y,yaw,joints). Local steering uses exact differential
// rotate/translate/rotate connections and checks all swept states. Bounds and
// goal tolerances come from the existing KinoAstarConfig contract.
WholeBodyRrtResult
searchWholeBodyRrt(const wbmm::core::Header &, const wbmm::core::BaseState &start,
                   const wbmm::core::BaseState &goal, const wbmm::core::JointState &,
                   const wbmm::core::RobotLimits &, const BaseCollisionChecker &,
                   const WholeBodyCollisionChecker &, const KinoAstarConfig &,
                   const WholeBodyRrtConfig & = {},
                   const std::optional<wbmm::core::JointState> &goal_joints = std::nullopt);

} // namespace wbmm::search
