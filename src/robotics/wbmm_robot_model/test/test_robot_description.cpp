#include "wbmm_robot_model/robot_description_loader.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace
{

constexpr char kSyntheticUrdf[] = R"(<?xml version="1.0"?>
<robot name="synthetic">
  <link name="base_link">
    <collision><origin xyz="0.1 0 0.05"/><geometry><sphere radius="0.10"/></geometry></collision>
    <collision><origin xyz="-0.1 0 0.05"/><geometry><sphere radius="0.10"/></geometry></collision>
  </link>
  <link name="shoulder_mount">
    <collision><origin xyz="0 0 0.30"/><geometry><sphere radius="0.08"/></geometry></collision>
    <collision><origin xyz="0.25 0 0.30"/><geometry><sphere radius="0.08"/></geometry></collision>
  </link>
  <link name="link_a">
    <visual><origin xyz="0 0 0.1"/><geometry><box size="0.1 0.2 0.3"/></geometry></visual>
    <collision><origin xyz="0 0 0.05"/><geometry><sphere radius="0.06"/></geometry></collision>
  </link>
  <link name="link_b">
    <collision><origin xyz="0 0 0"/><geometry><sphere radius="0.04"/></geometry></collision>
    <collision><origin xyz="0 0 0.2"/><geometry><cylinder radius="0.03" length="0.1"/></geometry></collision>
  </link>
  <link name="tool_fixed">
    <collision><origin xyz="0 0 0.10"/><geometry><sphere radius="0.03"/></geometry></collision>
  </link>

  <joint name="base_to_mount" type="fixed">
    <parent link="base_link"/><child link="shoulder_mount"/>
    <origin xyz="0 0 0.20" rpy="0 0 0"/>
  </joint>
  <joint name="joint_a" type="revolute">
    <parent link="shoulder_mount"/><child link="link_a"/>
    <origin xyz="0 0 0.30" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <limit lower="-1.0" upper="1.0" effort="1" velocity="1"/>
  </joint>
  <joint name="joint_b" type="continuous">
    <parent link="link_a"/><child link="link_b"/>
    <origin xyz="0.2 0 0" rpy="0 1.5708 0"/><axis xyz="0 0 1"/>
  </joint>
  <joint name="tool_joint" type="fixed">
    <parent link="link_b"/><child link="tool_fixed"/>
    <origin xyz="0 0.1 0" rpy="0 0 0"/>
  </joint>
</robot>
)";

std::string writeSyntheticUrdf()
{
  const std::string path = "wbmm_robot_model_synthetic.urdf";
  std::ofstream stream(path);
  stream << kSyntheticUrdf;
  stream.close();
  return path;
}

TEST(RobotDescription, ParsesLinksJointsAndGeometry)
{
  const std::string path = writeSyntheticUrdf();
  const auto result = wbmm::robot_model::RobotDescriptionLoader::fromFile(path);
  ASSERT_TRUE(result.success) << result.message;
  const auto & description = result.description;

  EXPECT_EQ(description.name, "synthetic");
  EXPECT_EQ(description.root_link, "base_link");
  EXPECT_EQ(description.links.size(), 5U);
  EXPECT_EQ(description.joints.size(), 4U);
  EXPECT_NE(description.contentId(), 0U);

  const auto * link_a = description.findLink("link_a");
  ASSERT_NE(link_a, nullptr);
  ASSERT_EQ(link_a->visuals.size(), 1U);
  EXPECT_EQ(link_a->visuals[0].geometry.type,
            wbmm::robot_model::GeometryType::kBox);
  EXPECT_TRUE(link_a->visuals[0].geometry.size.isApprox(
    Eigen::Vector3d(0.1, 0.2, 0.3), 1e-12));

  const auto * joint_a = description.findJoint("joint_a");
  ASSERT_NE(joint_a, nullptr);
  EXPECT_EQ(joint_a->type, wbmm::robot_model::JointType::kRevolute);
  EXPECT_EQ(joint_a->parent_link, "shoulder_mount");
  EXPECT_EQ(joint_a->child_link, "link_a");
  EXPECT_TRUE(joint_a->isScalar());
  EXPECT_TRUE(joint_a->has_position_limit);
  EXPECT_NEAR(joint_a->lower, -1.0, 1e-12);
  EXPECT_NEAR(joint_a->upper, 1.0, 1e-12);
  EXPECT_TRUE(joint_a->has_velocity_limit);
  EXPECT_NEAR(joint_a->velocity, 1.0, 1e-12);

  // Continuous joint: velocity limit present, position limit not applicable.
  const auto * joint_b = description.findJoint("joint_b");
  ASSERT_NE(joint_b, nullptr);
  EXPECT_FALSE(joint_b->has_position_limit);

  EXPECT_EQ(description.findJointByChildLink("link_b")->name, "joint_b");

  std::remove(path.c_str());
}

TEST(RobotDescription, RejectsUnsupportedMimicJoint)
{
  constexpr char kMimicUrdf[] = R"(<?xml version="1.0"?>
<robot name="mimic">
  <link name="root"/>
  <link name="child"/>
  <link name="mesh_link"/>
  <joint name="driver" type="revolute">
    <parent link="root"/><child link="child"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 1 0"/>
    <limit lower="-2" upper="2" effort="1" velocity="1"/>
  </joint>
  <joint name="source" type="revolute">
    <parent link="child"/><child link="mesh_link"/>
    <origin xyz="0 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-2" upper="2" effort="1" velocity="1"/>
    <mimic joint="driver" multiplier="2.0" offset="0.5"/>
  </joint>
</robot>
)";
  const auto result = wbmm::robot_model::RobotDescriptionLoader::fromXml(
    kMimicUrdf, "mimic");
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("Mimic joint"), std::string::npos);
}

TEST(RobotDescription, DecomposesFixedChainsInBothDirections)
{
  const std::string path = writeSyntheticUrdf();
  const auto result = wbmm::robot_model::RobotDescriptionLoader::fromFile(path);
  ASSERT_TRUE(result.success) << result.message;
  const auto & description = result.description;

  Eigen::Isometry3d transform;
  ASSERT_TRUE(wbmm::robot_model::fixedTransform(
    description, "base_link", "shoulder_mount", transform));
  EXPECT_TRUE(transform.translation().isApprox(
    Eigen::Vector3d(0, 0, 0.20), 1e-12));

  // Reverse direction must invert the same transform.
  Eigen::Isometry3d reverse;
  ASSERT_TRUE(wbmm::robot_model::fixedTransform(
    description, "shoulder_mount", "base_link", reverse));
  EXPECT_TRUE(reverse.translation().isApprox(
    Eigen::Vector3d(0, 0, -0.20), 1e-12));

  // A path crossing a movable joint must not be reported as fixed.
  Eigen::Isometry3d across;
  EXPECT_FALSE(wbmm::robot_model::fixedTransform(
    description, "base_link", "link_a", across));

  std::remove(path.c_str());
}

#ifdef WBMM_TEST_URDF
TEST(RobotDescription, MatchesTheDeployedRobotTopology)
{
  const auto result = wbmm::robot_model::RobotDescriptionLoader::fromFile(
    WBMM_TEST_URDF);
  ASSERT_TRUE(result.success) << result.message;
  const auto & description = result.description;

  EXPECT_EQ(description.root_link, "base_footprint");
  const auto scalar_joints = description.scalarJointNames();
  const std::vector<std::string> expected = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  ASSERT_EQ(scalar_joints.size(), expected.size());
  for (std::size_t i = 0U; i < expected.size(); ++i) {
    EXPECT_EQ(scalar_joints[i], expected[i]);
  }

  const auto * base_footprint = description.findLink("base_footprint");
  ASSERT_NE(base_footprint, nullptr);
  EXPECT_TRUE(base_footprint->visuals.empty());
  EXPECT_TRUE(base_footprint->collisions.empty());

  const auto * base_link = description.findLink("base_link");
  ASSERT_NE(base_link, nullptr);
  EXPECT_EQ(base_link->collisions.size(), 6U);
  for (const auto & collision : base_link->collisions) {
    EXPECT_EQ(collision.geometry.type,
              wbmm::robot_model::GeometryType::kSphere);
  }

  const auto * tool0 = description.findLink("tool0");
  ASSERT_NE(tool0, nullptr);
  EXPECT_NE(description.findJoint("tool0_joint"), nullptr);
}
#endif

}  // namespace
