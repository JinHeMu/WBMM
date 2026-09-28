#pragma once
#include <Eigen/Geometry>
#include <map>
#include <string>
#include <urdf/model.h>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <wbmm_core/trajectory.hpp>

namespace wbmm::visualization {

struct VisualStyle {
  // Uniform tint distinguishes planner and MPC ghosts. URDF materials can be
  // restored when the original REMANI mesh appearance is preferred.
  double red{0.1}, green{0.8}, blue{0.65}, alpha{0.17};
  bool use_urdf_materials{false};
  bool use_embedded_materials{false};
};

// Display geometry and link placement only: no dynamics, limits enforcement,
// collision queries, commands or planning. Consumes the existing WBMM state.
// The current wbmm_pinocchio RobotModel is six-joint-specific. This visual-only
// URDF tree supports arbitrary scalar joint counts without modifying that
// model.
class RobotVisualModel {
public:
  RobotVisualModel(const std::string &xml,
                   const std::vector<std::string> &joint_names,
                   std::string base_frame = {},
                   std::map<std::string, double> default_joint_positions = {},
                   std::string mesh_directory = {});
  const std::vector<std::string> &jointNames() const { return joint_names_; }
  std::size_t visualCount() const { return visuals_.size(); }
  const std::string &baseFrame() const { return base_frame_; }
  bool hasLink(const std::string &name) const;
  void validateState(const wbmm::core::WholeBodyState &state) const;
  std::map<std::string, Eigen::Isometry3d>
  linkPoses(const wbmm::core::WholeBodyState &) const;
  visualization_msgs::msg::MarkerArray
  markers(const wbmm::core::WholeBodyState &, const std::string &ns,
          const VisualStyle &) const;

private:
  struct Visual {
    std::string link;
    Eigen::Isometry3d origin;
    visualization_msgs::msg::Marker prototype;
    bool has_material{false};
  };
  urdf::Model model_;
  std::vector<std::string> joint_names_;
  std::map<std::string, double> defaults_;
  std::string base_frame_;
  std::vector<Visual> visuals_;
};
} // namespace wbmm::visualization
