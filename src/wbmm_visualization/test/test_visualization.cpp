#include <cmath>
#include <gtest/gtest.h>
#include <limits>
#include <wbmm_robot_model/wbmm_robot_model.hpp>
#include <wbmm_visualization/trajectory_display.hpp>

#include <map>
#include <memory>
#include <stdexcept>
#include <vector>

namespace {
using namespace wbmm::visualization;
const std::string robot = R"(<robot name="display_test">
<link name="root"/>
<link name="base"><visual><origin xyz="0 0 .1"/><geometry><box size="1 .5 .2"/></geometry></visual></link>
<joint name="root_mount" type="fixed"><parent link="root"/><child link="base"/><origin xyz="0 0 .3"/></joint>
<link name="arm">
<visual><origin xyz="1 0 0"/><geometry><sphere radius=".2"/></geometry><material name="red"><color rgba="1 0 0 .5"/></material></visual>
<visual><origin xyz="2 0 0"/><geometry><mesh filename="part.stl" scale=".1 .2 .3"/></geometry></visual></link>
<joint name="hinge" type="revolute"><parent link="base"/><child link="arm"/><origin xyz="0 0 1"/><axis xyz="0 0 1"/><limit lower="-3.14" upper="3.14" effort="10" velocity="1"/></joint>
<link name="tip"><visual><geometry><cylinder radius=".1" length=".4"/></geometry></visual></link>
<joint name="slide" type="prismatic"><parent link="arm"/><child link="tip"/><origin xyz="1 0 0"/><axis xyz="1 0 0"/><limit lower="0" upper="1" effort="10" velocity="1"/></joint>
<link name="finger"><visual><geometry><box size=".1 .1 .2"/></geometry></visual></link>
<joint name="follower" type="continuous"><parent link="tip"/><child link="finger"/><axis xyz="0 0 1"/><mimic joint="hinge" multiplier="-1" offset=".1"/></joint>
</robot>)";
RobotVisualModel makeModel(
    const std::string &xml, std::vector<std::string> joints,
    const std::string &base_frame,
    std::map<std::string, double> locked = {},
    std::string mesh_directory = "/tmp/assets") {
  auto description = std::make_shared<const wbmm::robot_model::RobotDescription>(
      wbmm::robot_model::parseRobotDescription(xml, "display_test"));
  auto config = wbmm::robot_model::RobotModelConfig::defaultsFor(*description);
  config.state_base_frame = base_frame.empty() ? description->root_link : base_frame;
  config.controlled_joints = std::move(joints);
  config.locked_joints = std::move(locked);
  return RobotVisualModel(description, std::move(config),
                          std::move(mesh_directory));
}
RobotVisualModel model() {
  return makeModel(robot, {"hinge", "slide"}, "base");
}
wbmm::core::WholeBodyState state() {
  wbmm::core::WholeBodyState s;
  s.header.frame_id = "map";
  s.base.x = 2;
  s.base.y = 3;
  s.base.yaw = M_PI / 2;
  s.joints.names = {"slide", "hinge"};
  s.joints.positions = {.4, M_PI / 2};
  return s;
}
wbmm_planner_ros::msg::WholeBodyTrajectory plan(std::size_t n = 5) {
  wbmm_planner_ros::msg::WholeBodyTrajectory m;
  m.header.frame_id = "map";
  m.joint_names = {"hinge", "slide"};
  for (std::size_t i = 0; i < n; ++i) {
    m.time_from_start.push_back(i * .2);
    m.base_x.push_back(i * .1);
    m.base_y.push_back(0);
    m.base_yaw.push_back(0);
    m.joint_positions.insert(m.joint_positions.end(), {i * .1, .2});
  }
  return m;
}
TEST(VisualModel, GeometryOriginsScaleAndNamedJoints) {
  auto m = model();
  auto markers = m.markers(state(), "planner/robot_0", {});
  ASSERT_EQ(markers.markers.size(), 5U);
  bool mesh = false, sphere = false;
  for (const auto &v : markers.markers) {
    EXPECT_EQ(v.header.frame_id, "map");
    EXPECT_EQ(v.ns, "planner/robot_0");
    if (v.type == v.MESH_RESOURCE) {
      mesh = true;
      EXPECT_EQ(v.mesh_resource, "file:///tmp/assets/part.stl");
      EXPECT_DOUBLE_EQ(v.scale.x, .1);
      EXPECT_DOUBLE_EQ(v.scale.y, .2);
      EXPECT_DOUBLE_EQ(v.scale.z, .3);
      EXPECT_NEAR(v.pose.position.x, 0, 1e-10);
      EXPECT_NEAR(v.pose.position.y, 3, 1e-10);
      EXPECT_NEAR(v.pose.position.z, 1, 1e-10);
    }
    if (v.type == v.SPHERE) {
      sphere = true;
      EXPECT_DOUBLE_EQ(v.scale.x, .4);
      EXPECT_NEAR(v.pose.position.x, 1, 1e-10);
    }
  }
  EXPECT_TRUE(mesh);
  EXPECT_TRUE(sphere);
}
TEST(VisualModel, FixedBaseOffsetPrismaticAndMimic) {
  auto poses = model().linkPoses(state());
  EXPECT_NEAR(poses.at("base").translation().z(), 0, 1e-10);
  EXPECT_NEAR(poses.at("root").translation().z(), -.3, 1e-10);
  EXPECT_NEAR(poses.at("tip").translation().x(), .6, 1e-10);
  const auto r = poses.at("finger").linear();
  EXPECT_NEAR(std::atan2(r(1, 0), r(0, 0)), M_PI / 2 + .1, 1e-10);
}
TEST(VisualModel, ExplicitAuxiliaryDefaultsAndNoSixJointAssumption) {
  EXPECT_THROW(makeModel(robot, {"hinge"}, "base", {}, "/tmp"),
               std::invalid_argument);
  RobotVisualModel m = makeModel(robot, {"hinge"}, "base", {{"slide", .4}}, "/tmp");
  auto s = state();
  s.joints.names = {"hinge"};
  s.joints.positions = {M_PI / 2};
  EXPECT_NEAR(m.linkPoses(s).at("tip").translation().x(), .6, 1e-10);
}
TEST(VisualModel, MaterialsAndAlpha) {
  VisualStyle style;
  style.use_urdf_materials = true;
  style.alpha = .2;
  style.use_embedded_materials = true;
  for (const auto &marker : model().markers(state(), "x", style).markers)
    if (marker.type == marker.SPHERE) {
      EXPECT_FLOAT_EQ(marker.color.r, 1);
      EXPECT_FLOAT_EQ(marker.color.g, 0);
      EXPECT_FLOAT_EQ(marker.color.a, .1);
    }
}
TEST(VisualModel, RejectsMissingOrDuplicateJointData) {
  auto m = model();
  auto s = state();
  s.joints.names = {"hinge", "hinge"};
  EXPECT_THROW(m.markers(s, "x", {}), std::invalid_argument);
  s = state();
  s.joints.positions[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(m.markers(s, "x", {}), std::invalid_argument);
  EXPECT_THROW(makeModel(robot, {"hinge", "slide"}, "arm", {}, "/tmp"),
               std::invalid_argument);
}
TEST(VisualModel, RejectsMimicCyclesAndRelativeMeshWithoutBase) {
  auto cycle = robot;
  auto at = cycle.find("mimic joint=\"hinge\"");
  cycle.replace(at, std::string("mimic joint=\"hinge\"").size(),
                "mimic joint=\"follower\"");
  EXPECT_THROW(makeModel(cycle, {"hinge", "slide"}, "base", {}, "/tmp"),
               std::invalid_argument);
  EXPECT_THROW(makeModel(robot, {"hinge", "slide"}, "base", {}, ""),
               std::invalid_argument);
}
TEST(Adapters, PlannerUsesDisplayFieldsAndRetainsFrame) {
  auto p = fromPlanner(plan());
  ASSERT_EQ(p.points.size(), 5U);
  EXPECT_EQ(p.points[0].state.header.frame_id, "map");
  EXPECT_EQ(p.points[0].state.joints.names,
            (std::vector<std::string>{"hinge", "slide"}));
  EXPECT_FALSE(p.points[0].feedforward_input.has_value());
}
TEST(Adapters, MalformedUnsampledPlannerPointIsRejected) {
  auto p = plan();
  p.base_x[2] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(fromPlanner(p), std::invalid_argument);
  p = plan();
  p.joint_positions.pop_back();
  EXPECT_THROW(fromPlanner(p), std::invalid_argument);
  p = plan();
  p.time_from_start[2] = 0;
  EXPECT_THROW(fromPlanner(p), std::invalid_argument);
  p = plan();
  EXPECT_THROW(fromPlanner(p, 2), std::invalid_argument);
}
TEST(Adapters, MpcUsesPredictionNotCostTargetAndAllowsEventDuplicates) {
  ocs2_msgs::msg::MpcFlattenedController p;
  p.time_trajectory = {100, 100, 101};
  for (int i = 0; i < 3; ++i) {
    ocs2_msgs::msg::MpcState x;
    x.value = {float(i), 0, 0, .1f, .2f};
    p.state_trajectory.push_back(x);
  }
  ocs2_msgs::msg::MpcState target;
  target.value = {99, 99, 99, 0, 0, 0, 1};
  p.plan_target_trajectories.state_trajectory = {target};
  auto t = fromMpc(p, "odom", {"hinge", "slide"});
  ASSERT_EQ(t.points.size(), 3U);
  EXPECT_DOUBLE_EQ(t.points.back().state.base.x, 2);
  EXPECT_DOUBLE_EQ(t.points.front().time_from_start, 0);
  EXPECT_EQ(t.points[0].state.header.frame_id, "odom");
  p.state_trajectory[0].value.push_back(0);
  EXPECT_THROW(fromMpc(p, "odom", {"hinge", "slide"}), std::invalid_argument);
}
TEST(Display, BoundedTimeSamplingKeepsBothEnds) {
  auto t = fromPlanner(plan(101));
  auto indices = selectSamples(t, .05, 8);
  EXPECT_EQ(indices.front(), 0U);
  EXPECT_EQ(indices.back(), 100U);
  EXPECT_LE(indices.size(), 8U);
  EXPECT_THROW(selectSamples(t, 0, 8), std::invalid_argument);
}
TEST(Display, ArmOnlyAndTurningTrajectoriesAreNotDropped) {
  auto p = plan();
  std::fill(p.base_x.begin(), p.base_x.end(), 0);
  p.base_yaw = {0, .2, .4, .6, .8};
  DisplayOptions options;
  options.sample_interval = .1;
  auto result = renderTrajectory(model(), fromPlanner(p), "planner", options);
  EXPECT_EQ(result.markers.size(), 25U);
}
TEST(Display, LimitsGeometryBudgetAndRemovesObsoleteModels) {
  DisplayOptions options;
  options.sample_interval = .1;
  auto m = model();
  auto all = renderTrajectory(m, fromPlanner(plan()), "planner", options);
  MarkerLayer layer;
  EXPECT_EQ(layer.replace(all).markers.size(), 25U);
  options.max_robot_poses = 2;
  auto smaller = layer.replace(
      renderTrajectory(m, fromPlanner(plan()), "planner", options));
  std::size_t removed = 0;
  for (const auto &marker : smaller.markers)
    if (marker.action == marker.DELETE) {
      ++removed;
      EXPECT_EQ(marker.header.frame_id, "map");
    }
  EXPECT_EQ(removed, 15U);
  EXPECT_EQ(
      layer.replace(visualization_msgs::msg::MarkerArray{}).markers.size(),
      10U);
  EXPECT_TRUE(
      layer.replace(visualization_msgs::msg::MarkerArray{}).markers.empty());
  options.max_markers = 1;
  EXPECT_THROW(renderTrajectory(m, fromPlanner(plan()), "planner", options),
               std::invalid_argument);
}
} // namespace
