#include <chrono>
#include <iostream>
#include <wbmm_collision/environment_collision_checker.hpp>
#include <wbmm_collision/urdf_collision_model.hpp>
#include <wbmm_environment/esdf_loader.hpp>
#include <wbmm_pinocchio/pinocchio_robot_model.hpp>
#include <wbmm_planner/whole_body_planner.hpp>
int main(int argc, char **argv) {
  const bool enabled = argc > 1 && std::string(argv[1]) == "--optimize";
  const std::string urdf = WBMM_BENCHMARK_URDF;
  auto robot =
      std::make_shared<wbmm::pinocchio::PinocchioRobotModel>(urdf, .5, 1.0);
  auto env = wbmm::environment::NpzEsdfLoader::load(WBMM_BENCHMARK_ESDF).grid;
  auto spheres = wbmm::collision::loadUrdfCollisionModel(
      urdf, robot->jointNames(), "base_link");
  wbmm::collision::CollisionModel model;
  model.spheres = spheres.base.spheres;
  for (auto &g : spheres.arm)
    model.spheres.insert(model.spheres.end(), g.spheres.begin(),
                         g.spheres.end());
  wbmm::collision::CollisionCheckOptions options;
  options.treat_unknown_as_occupied = false;
  wbmm::collision::EnvironmentCollisionChecker checker(robot, env, model,
                                                       options);
  for (auto goal :
       {std::pair<double, double>{.8, 0}, {1.5, 0}, {1, .5}, {2, -.6}}) {
    wbmm::planning::PlanRequest r;
    r.header.frame_id = "map";
    r.goal.x = goal.first;
    r.goal.y = goal.second;
    r.start_joints.names = robot->jointNames();
    r.start_joints.positions = {-.515, 1.5707, -1.5707, 1.5707, 1.5707, .254};
    r.start_joints.velocities.assign(6, 0);
    r.limits = robot->limits();
    r.environment_revision = r.collision_model_revision = 1;
    wbmm::planning::PlannerConfig c;
    c.base_search.position_tolerance = .075;
    c.base_search.yaw_tolerance = .15;
    c.base_search.position_resolution = .15;
    c.base_search.max_search_time = 2;
    c.sample_rrt.max_search_time = 2;
    std::size_t bc = 0, wc = 0;
    double bt = 0, wt = 0;
    auto b = [&](auto &h, auto &s) {
      auto t = std::chrono::steady_clock::now();
      ++bc;
      bool f = checker.checkBase(h, s).isFree();
      bt += std::chrono::duration<double>(std::chrono::steady_clock::now() - t)
                .count();
      return f;
    };
    auto w = [&](auto &, auto &s) {
      auto t = std::chrono::steady_clock::now();
      ++wc;
      bool f =
          checker.check(s, wbmm::collision::CheckScope::kWholeBody).isFree();
      wt += std::chrono::duration<double>(std::chrono::steady_clock::now() - t)
                .count();
      return f;
    };
    c.enable_optimization = enabled;
    auto optimize = [&](const wbmm::traj_opt::OptimizerInput &input,
                        const wbmm::traj_opt::OptimizerConfig &config) {
      wbmm::traj_opt::WholeBodyOptimizer optimizer(config, env, spheres);
      if (!optimizer.prepare(input)) {
        wbmm::traj_opt::OptimizerResult failure;
        failure.message = optimizer.message();
        return failure;
      }
      return optimizer.optimize();
    };
    auto t = std::chrono::steady_clock::now();
    auto p = wbmm::planning::WholeBodyPlanner(c).plan(r, b, w, optimize);
    std::cout << "{\"goal\":[" << goal.first << "," << goal.second
              << "],\"success\":" << (p.success ? "true" : "false")
              << ",\"total\":"
              << std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - t)
                     .count()
              << ",\"base\":" << p.base_search_time
              << ",\"arm\":" << p.arm_seed_time
              << ",\"rrt\":" << p.whole_body_rrt_time
              << ",\"build\":" << p.build_time << ",\"base_checks\":" << bc
              << ",\"whole_checks\":" << wc << ",\"base_check_s\":" << bt
              << ",\"whole_check_s\":" << wt
              << ",\"optimization_s\":" << p.optimization_time
              << ",\"optimization_applied\":"
              << (p.optimization_applied ? "true" : "false") << ",\"duration\":"
              << (p.trajectory.points.empty()
                      ? 0
                      : p.trajectory.points.back().time_from_start)
              << ",\"message\":\"" << p.message << "\"}" << std::endl;
  }
}
