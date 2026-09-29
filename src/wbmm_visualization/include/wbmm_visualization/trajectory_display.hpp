#pragma once
#include <ocs2_msgs/msg/mpc_flattened_controller.hpp>
#include <set>
#include <wbmm_planner_ros/msg/whole_body_trajectory.hpp>
#include <wbmm_visualization/robot_visual_model.hpp>

namespace wbmm::visualization {
struct DisplayOptions {
  double sample_interval{0.5};
  std::size_t max_robot_poses{12};
  std::size_t max_markers{5000};
  VisualStyle style;
  bool show_base_path{false};
  bool show_ee_path{false};
  std::string ee_frame;
  double line_width{0.015};
};

// Only displayed state/time fields are used. Inputs, controller gains, task
// phases and OCS2 plan_target_trajectories do not affect robot ghost poses.
wbmm::core::WholeBodyTrajectory
fromPlanner(const wbmm_planner_ros::msg::WholeBodyTrajectory &,
            std::size_t max_points = 100000);
wbmm::core::WholeBodyTrajectory
fromMpc(const ocs2_msgs::msg::MpcFlattenedController &,
        const std::string &frame, const std::vector<std::string> &joint_names,
        std::size_t max_points = 100000);

// Select original samples by time, with both endpoints and a bounded count.
// No interpolation/replanning; arm-only motion and in-place turns remain
// visible.
std::vector<std::size_t> selectSamples(const wbmm::core::WholeBodyTrajectory &,
                                       double interval, std::size_t max_poses);
visualization_msgs::msg::MarkerArray
renderTrajectory(const RobotVisualModel &,
                 const wbmm::core::WholeBodyTrajectory &,
                 const std::string &layer, const DisplayOptions &);

// Produces full current snapshots plus explicit DELETEs for obsolete ids.
// Does not use DELETEALL, which could erase other displays' namespaces.
class MarkerLayer {
public:
  visualization_msgs::msg::MarkerArray
  replace(visualization_msgs::msg::MarkerArray current);

private:
  std::map<std::pair<std::string, int>, std_msgs::msg::Header> previous_;
};
} // namespace wbmm::visualization
