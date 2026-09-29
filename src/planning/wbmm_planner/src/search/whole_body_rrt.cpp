#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <set>
#include <stdexcept>
#include <wbmm_planner/search/whole_body_rrt.hpp>

namespace wbmm::search {
namespace {
using namespace wbmm::core;
using Clock = std::chrono::steady_clock;
constexpr std::size_t none = std::numeric_limits<std::size_t>::max();
double angle(double x) { return std::remainder(x, 2.0 * M_PI); }
struct Node {
  BaseState base;
  JointState joints;
  std::size_t parent{none};
  std::size_t layer{0};
  MotionPrimitive edge;
};
double jointDistance(const JointState &a, const JointState &b) {
  double d = 0;
  for (std::size_t j = 0; j < a.positions.size(); ++j)
    d = std::max(d, std::abs(a.positions[j] - b.positions[j]));
  return d;
}
JointState blend(const JointState &a, const JointState &b, double t) {
  auto q = a;
  for (std::size_t j = 0; j < q.positions.size(); ++j) {
    q.positions[j] += t * (b.positions[j] - a.positions[j]);
    q.velocities[j] = 0;
  }
  return q;
}
bool finitePose(const BaseState &s) {
  return std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.yaw);
}
struct Search {
  Header header;
  RobotLimits limits;
  WholeBodyCollisionChecker checker;
  WholeBodyRrtConfig config;
  Search(Header h, RobotLimits l, WholeBodyCollisionChecker c, WholeBodyRrtConfig cfg)
      : header(std::move(h)), limits(std::move(l)), checker(std::move(c)), config(cfg) {}
  Clock::time_point started{Clock::now()};
  std::vector<Node> nodes;
  WholeBodyRrtResult result;
  bool timedOut() const {
    return std::chrono::duration<double>(Clock::now() - started).count() >= config.max_search_time;
  }
  void validate(const JointState &q, const std::optional<JointState> &goal) {
    if (header.frame_id.empty() || !checker || q.positions.empty() ||
        q.positions.size() != q.names.size() || q.positions.size() != q.velocities.size() ||
        limits.joint_min.size() != q.positions.size() ||
        limits.joint_max.size() != q.positions.size() || config.max_nodes < 2 ||
        config.max_iterations == 0)
      throw std::invalid_argument("Invalid RRT contract/budget.");
    for (double v : {config.max_search_time, config.translation_step, config.joint_step,
                     config.collision_translation_step, config.collision_angle_step})
      if (!std::isfinite(v) || v <= 0)
        throw std::invalid_argument("Invalid RRT resolution/time.");
    std::set<std::string> names;
    for (std::size_t j = 0; j < q.names.size(); ++j)
      if (q.names[j].empty() || !names.insert(q.names[j]).second || !std::isfinite(q.velocities[j]))
        throw std::invalid_argument("Invalid initial joint names/velocities.");
    const auto checkQ = [&](const JointState &state) {
      if (state.names != q.names || state.positions.size() != q.positions.size())
        throw std::invalid_argument("Goal joint names/order/size mismatch.");
      for (std::size_t j = 0; j < q.positions.size(); ++j)
        if (!std::isfinite(limits.joint_min[j]) || !std::isfinite(limits.joint_max[j]) ||
            !std::isfinite(state.positions[j]) || state.positions[j] < limits.joint_min[j] ||
            state.positions[j] > limits.joint_max[j])
          throw std::invalid_argument("Invalid RRT joints/limits.");
    };
    checkQ(q);
    if (goal)
      checkQ(*goal);
  }
  bool free(const BaseState &b, const JointState &q) {
    WholeBodyState s;
    s.header = header;
    s.base_model = BaseModel::kDifferentialDrive;
    s.base = b;
    s.joints = q;
    return checker(header, s);
  }
  bool edgeFree(const Node &a, const JointState &q, const MotionPrimitive &p) {
    const double count =
        std::max({1.0, std::ceil(std::abs(p.v) * p.duration / config.collision_translation_step),
                  std::ceil(std::abs(p.omega) * p.duration / config.collision_angle_step),
                  std::ceil(jointDistance(a.joints, q) / config.collision_angle_step)});
    if (!std::isfinite(count) || count > 100000)
      throw std::invalid_argument("RRT edge check budget exceeded.");
    for (std::size_t k = 1; k <= static_cast<std::size_t>(count); ++k) {
      if (timedOut())
        return false;
      const double t = k / count;
      if (!free(propagate(a.base, p, t * p.duration), blend(a.joints, q, t)))
        return false;
    }
    return true;
  }
  std::size_t add(std::size_t parent, const MotionPrimitive &p, const JointState &q,
                  std::size_t layer) {
    if (nodes.size() >= config.max_nodes || !edgeFree(nodes[parent], q, p))
      return none;
    Node n;
    n.base = propagate(nodes[parent].base, p, p.duration);
    n.joints = q;
    n.parent = parent;
    n.layer = layer;
    n.edge = p;
    nodes.push_back(n);
    return nodes.size() - 1;
  }
  WholeBodyRrtResult finish(std::size_t tip = none,
                            std::string message = "RRT exhausted its search budget.") {
    result.solve_time = std::chrono::duration<double>(Clock::now() - started).count();
    result.generated_nodes = nodes.size();
    result.message = std::move(message);
    if (tip != none) {
      for (auto i = tip; i != none; i = nodes[i].parent) {
        result.search.base_path.push_back(nodes[i].base);
        result.search.arm_seed.push_back(nodes[i].joints);
        if (nodes[i].parent != none)
          result.primitives.push_back(nodes[i].edge);
      }
      std::reverse(result.search.base_path.begin(), result.search.base_path.end());
      std::reverse(result.search.arm_seed.begin(), result.search.arm_seed.end());
      std::reverse(result.primitives.begin(), result.primitives.end());
      for (auto &p : result.primitives)
        result.search.path_length += std::abs(p.v) * p.duration;
      result.search.phases.assign(result.search.base_path.size(), ExecutionPhase::kNavigate);
      result.success = result.search.success = true;
      result.search.solve_time = result.solve_time;
      result.message = "ok";
    }
    return result;
  }
};
JointState draw(const JointState &initial, const RobotLimits &limits, std::mt19937 &rng) {
  auto q = initial;
  for (std::size_t j = 0; j < q.positions.size(); ++j)
    q.positions[j] =
        std::uniform_real_distribution<double>(limits.joint_min[j], limits.joint_max[j])(rng);
  return q;
}
JointState steer(const JointState &from, const JointState &to, double step) {
  const double d = jointDistance(from, to);
  return blend(from, to, d > step ? step / d : 1.0);
}
} // namespace

WholeBodyRrtResult sampleArmRrt(const Header &h, const BaseSearchResult &path,
                                const JointState &initial, const RobotLimits &limits,
                                const WholeBodyCollisionChecker &checker,
                                const WholeBodyRrtConfig &config,
                                const std::optional<JointState> &goal) {
  Search s{h, limits, checker, config};
  try {
    s.validate(initial, goal);
    if (path.path.empty() || path.primitives.size() + 1 != path.path.size())
      throw std::invalid_argument("Base path and primitive sizes disagree.");
    for (std::size_t i = 0; i < path.path.size(); ++i) {
      if (!finitePose(path.path[i]))
        throw std::invalid_argument("Non-finite base path.");
      if (i > 0) {
        const auto b =
            propagate(path.path[i - 1], path.primitives[i - 1], path.primitives[i - 1].duration);
        if (std::hypot(b.x - path.path[i].x, b.y - path.path[i].y) > 1e-6 ||
            std::abs(angle(b.yaw - path.path[i].yaw)) > 1e-6)
          throw std::invalid_argument("Base primitive does not reproduce path.");
      }
    }
    s.nodes.push_back({path.path.front(), initial, none, 0, {}});
    if (!s.free(path.path.front(), initial))
      return s.finish(none, "Arm seeding failed: start is in collision.");
    std::mt19937 rng(config.random_seed);
    auto extend = [&](std::size_t index) {
      // Carry a newly found branch forward as far as possible. Failed edges
      // leave all parents in the tree for later random reconfiguration.
      while (s.nodes[index].layer + 1 < path.path.size() && !s.timedOut()) {
        const auto layer = s.nodes[index].layer;
        auto next = s.add(index, path.primitives[layer], s.nodes[index].joints, layer + 1);
        if (next == none)
          break;
        index = next;
      }
      return index;
    };
    auto reached = [&](std::size_t i) {
      return s.nodes[i].layer + 1 == path.path.size() &&
             (!goal || jointDistance(s.nodes[i].joints, *goal) < 1e-8);
    };
    auto tip = extend(0);
    if (reached(tip))
      return s.finish(tip);
    for (std::size_t it = 0;
         it < config.max_iterations && s.nodes.size() < config.max_nodes && !s.timedOut(); ++it) {
      const auto layer =
          s.nodes[std::uniform_int_distribution<std::size_t>(0, s.nodes.size() - 1)(rng)].layer;
      auto target = (goal && it % 4 == 0) ? *goal : draw(initial, limits, rng);
      std::size_t near = none;
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < s.nodes.size(); ++i)
        if (s.nodes[i].layer == layer) {
          const double d = jointDistance(s.nodes[i].joints, target);
          if (d < best) {
            best = d;
            near = i;
          }
        }
      if (near == none || best < 1e-9)
        continue;
      const auto q = steer(s.nodes[near].joints, target, config.joint_step);
      // Both stationary-arm and simultaneous base/arm extensions are tried.
      auto next = s.add(near, {0, 0, 1}, q, layer);
      if (next != none) {
        tip = extend(next);
        if (reached(tip))
          return s.finish(tip);
      }
      if (layer + 1 < path.path.size()) {
        next = s.add(near, path.primitives[layer], q, layer + 1);
        if (next != none) {
          tip = extend(next);
          if (reached(tip))
            return s.finish(tip);
        }
      }
    }
    return s.finish();
  } catch (const std::exception &e) {
    s.result.fatal = true;
    return s.finish(none, e.what());
  } catch (...) {
    s.result.fatal = true;
    return s.finish(none, "RRT collision checker exception.");
  }
}

WholeBodyRrtResult
searchWholeBodyRrt(const Header &h, const BaseState &start, const BaseState &goal_base,
                   const JointState &initial, const RobotLimits &limits,
                   const BaseCollisionChecker &base_checker,
                   const WholeBodyCollisionChecker &checker, const KinoAstarConfig &bounds,
                   const WholeBodyRrtConfig &config, const std::optional<JointState> &goal) {
  const auto valid = [&](const Header &header, const WholeBodyState &state) {
    const auto &b = state.base;
    return b.x >= bounds.min_x && b.x <= bounds.max_x && b.y >= bounds.min_y &&
           b.y <= bounds.max_y && base_checker(header, b) && checker(header, state);
  };
  Search s{h, limits, valid, config};
  try {
    if (!base_checker || !checker || !finitePose(start) || !finitePose(goal_base) ||
        !std::isfinite(bounds.min_x) || !std::isfinite(bounds.max_x) ||
        bounds.min_x >= bounds.max_x || !std::isfinite(bounds.min_y) ||
        !std::isfinite(bounds.max_y) || bounds.min_y >= bounds.max_y ||
        !std::isfinite(limits.max_base_speed) || limits.max_base_speed <= 0 ||
        !std::isfinite(limits.max_base_yaw_rate) || limits.max_base_yaw_rate <= 0 ||
        !std::isfinite(bounds.position_tolerance) || bounds.position_tolerance <= 0 ||
        !std::isfinite(bounds.yaw_tolerance) || bounds.yaw_tolerance <= 0)
      throw std::invalid_argument("Invalid whole-body RRT bounds/checkers/base limits.");
    s.validate(initial, goal);
    s.nodes.push_back({start, initial, none, 0, {}});
    if (!s.free(start, initial))
      return s.finish(none, "Arm seeding failed: start is in collision.");
    auto reached = [&](std::size_t i) {
      const auto &n = s.nodes[i];
      return std::hypot(n.base.x - goal_base.x, n.base.y - goal_base.y) <=
                 bounds.position_tolerance &&
             std::abs(angle(n.base.yaw - goal_base.yaw)) <= bounds.yaw_tolerance &&
             (!goal || jointDistance(n.joints, *goal) < 1e-8);
    };
    if (reached(0))
      return s.finish(0);
    std::mt19937 rng(config.random_seed);
    for (std::size_t it = 0;
         it < config.max_iterations && s.nodes.size() < config.max_nodes && !s.timedOut(); ++it) {
      BaseState target = goal_base;
      const bool biased = it % 4 == 0;
      if (!biased) {
        target.x = std::uniform_real_distribution<double>(bounds.min_x, bounds.max_x)(rng);
        target.y = std::uniform_real_distribution<double>(bounds.min_y, bounds.max_y)(rng);
        target.yaw = std::uniform_real_distribution<double>(-M_PI, M_PI)(rng);
      }
      auto q = biased ? (goal ? *goal : initial) : draw(initial, limits, rng);
      std::size_t near = 0;
      double best = std::numeric_limits<double>::infinity();
      for (std::size_t i = 0; i < s.nodes.size(); ++i) {
        const auto &n = s.nodes[i];
        const double d = std::hypot(n.base.x - target.x, n.base.y - target.y) +
                         0.15 * std::abs(angle(n.base.yaw - target.yaw)) +
                         0.1 * jointDistance(n.joints, q);
        if (d < best) {
          best = d;
          near = i;
        }
      }
      const auto from = s.nodes[near];
      q = steer(from.joints, q, config.joint_step);
      double distance = std::hypot(target.x - from.base.x, target.y - from.base.y);
      const bool clipped = distance > config.translation_step;
      distance = std::min(distance, config.translation_step);
      double heading = distance > 1e-9 ? std::atan2(target.y - from.base.y, target.x - from.base.x)
                                       : from.base.yaw;
      double turn = angle(heading - from.base.yaw);
      const double gear = std::abs(turn) > M_PI / 2 ? -1.0 : 1.0;
      if (gear < 0)
        heading = angle(heading + M_PI);
      turn = angle(heading - from.base.yaw);
      auto tip = near;
      if (std::abs(turn) > 1e-9)
        tip = s.add(tip,
                    {0, std::copysign(limits.max_base_yaw_rate, turn),
                     std::abs(turn) / limits.max_base_yaw_rate},
                    from.joints, 0);
      if (tip == none)
        continue;
      if (distance > 1e-9)
        tip = s.add(tip, {gear * limits.max_base_speed, 0, distance / limits.max_base_speed}, q, 0);
      else if (jointDistance(from.joints, q) > 1e-9)
        tip = s.add(tip, {0, 0, 1}, q, 0);
      if (tip == none)
        continue;
      if (!clipped) {
        turn = angle(target.yaw - s.nodes[tip].base.yaw);
        if (std::abs(turn) > 1e-9)
          tip = s.add(tip,
                      {0, std::copysign(limits.max_base_yaw_rate, turn),
                       std::abs(turn) / limits.max_base_yaw_rate},
                      q, 0);
      }
      if (tip != none && reached(tip))
        return s.finish(tip);
    }
    return s.finish();
  } catch (const std::exception &e) {
    s.result.fatal = true;
    return s.finish(none, e.what());
  } catch (...) {
    s.result.fatal = true;
    return s.finish(none, "RRT collision checker exception.");
  }
}
} // namespace wbmm::search
