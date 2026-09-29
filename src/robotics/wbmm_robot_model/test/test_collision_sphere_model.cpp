#include "wbmm_robot_model/collision_sphere_model.hpp"
#include "wbmm_robot_model/robot_description_loader.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <string>
#include <vector>

namespace
{

// A miniature robot that exercises every grouping rule the real URDF needs:
//   - a fixed subtree hanging off a movable joint's child link
//   - a shoulder sphere on the first joint's axis (the REMANI fold case)
//   - a sphere on a fixed link that is NOT on the axis (must be dropped, as
//     upstream does, because no joint group owns it)
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
  const std::string path = "wbmm_robot_model_spheres.urdf";
  std::ofstream stream(path);
  stream << kSyntheticUrdf;
  stream.close();
  return path;
}

wbmm::robot_model::RobotModelConfig syntheticConfig(
  const wbmm::robot_model::RobotDescription & description)
{
  auto config = wbmm::robot_model::RobotModelConfig::defaultsFor(description);
  config.base_collision_link = "base_link";
  config.controlled_joints = {"joint_a", "joint_b"};
  return config;
}

TEST(CollisionSphereModel, RejectsUnknownJointAndWrongType)
{
  const std::string path = writeSyntheticUrdf();
  const auto loaded = wbmm::robot_model::RobotDescriptionLoader::fromFile(path);
  ASSERT_TRUE(loaded.success) << loaded.message;
  auto config = syntheticConfig(loaded.description);

  config.controlled_joints = {"joint_a", "joint_missing"};
  auto missing = wbmm::robot_model::buildCollisionSphereModel(
    loaded.description, config);
  EXPECT_FALSE(missing.success);
  EXPECT_NE(missing.message.find("does not exist"), std::string::npos);

  config.controlled_joints = {"tool_joint"};
  auto fixed = wbmm::robot_model::buildCollisionSphereModel(
    loaded.description, config);
  EXPECT_FALSE(fixed.success);
  EXPECT_NE(fixed.message.find("revolute or continuous"), std::string::npos);

  std::remove(path.c_str());
}

TEST(CollisionSphereModel, RejectsMissingBaseCollisionLink)
{
  const std::string path = writeSyntheticUrdf();
  const auto loaded = wbmm::robot_model::RobotDescriptionLoader::fromFile(path);
  ASSERT_TRUE(loaded.success) << loaded.message;
  auto config = syntheticConfig(loaded.description);
  config.base_collision_link = "no_such_link";

  const auto result = wbmm::robot_model::buildCollisionSphereModel(
    loaded.description, config);
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("Base collision link"), std::string::npos);

  std::remove(path.c_str());
}

TEST(CollisionSphereModel, GroupsFixedSubtreeUnderItsMovableJoint)
{
  const std::string path = writeSyntheticUrdf();
  const auto loaded = wbmm::robot_model::RobotDescriptionLoader::fromFile(path);
  ASSERT_TRUE(loaded.success) << loaded.message;
  const auto result = wbmm::robot_model::buildCollisionSphereModel(
    loaded.description, syntheticConfig(loaded.description));
  ASSERT_TRUE(result.success) << result.message;

  ASSERT_EQ(result.arm.size(), 2U);
  ASSERT_EQ(result.arm_joint_names.size(), 2U);
  EXPECT_EQ(result.arm_joint_names[0], "joint_a");
  EXPECT_EQ(result.arm_joint_names[1], "joint_b");

  // joint_a owns link_a (1 sphere). The shoulder_mount sphere sits exactly on
  // joint_a's axis (x = y = 0) and is folded in; the other shoulder sphere is
  // 0.25 m off the axis and belongs to no joint group, so it is dropped.
  EXPECT_EQ(result.arm[0].spheres.size(), 2U);
  // joint_b owns link_b plus its fixed subtree (tool_fixed): 2 spheres.
  EXPECT_EQ(result.arm[1].spheres.size(), 2U);

  // Base group is exactly the declared base collision link's spheres.
  EXPECT_EQ(result.baseSphereCount(), 2U);

  for (const auto & sphere : result.base.spheres) {
    EXPECT_EQ(sphere.owner_link, "base_link");
    EXPECT_EQ(sphere.group_tags[0], "base");
  }
  for (const auto & sphere : result.arm[0].spheres) {
    EXPECT_EQ(sphere.owner_link, "link_a");
    EXPECT_EQ(sphere.group_tags[0], "arm");
    EXPECT_EQ(sphere.group_tags[1], "joint_a");
  }

  // The folded shoulder sphere must land at the joint origin, i.e. at zero in
  // link_a's frame.
  bool found_folded = false;
  for (const auto & sphere : result.arm[0].spheres) {
    if (std::abs(sphere.radius - 0.08) < 1e-12) {
      found_folded = true;
      EXPECT_LT(sphere.center_in_link.norm(), 1e-12);
    }
  }
  EXPECT_TRUE(found_folded);

  // The fixed-subtree sphere of joint_b is re-expressed in link_b's frame:
  // tool_joint sits at (0, 0.1, 0) in link_b and its sphere is at (0, 0, 0.10)
  // in tool_fixed, so in link_b it is (0, 0.1, 0.1).
  bool found_subtree = false;
  for (const auto & sphere : result.arm[1].spheres) {
    if (std::abs(sphere.radius - 0.03) < 1e-12) {
      found_subtree = true;
      EXPECT_NEAR(sphere.center_in_link.y(), 0.1, 1e-12);
      EXPECT_NEAR(sphere.center_in_link.z(), 0.1, 1e-12);
    }
  }
  EXPECT_TRUE(found_subtree);

  EXPECT_NEAR(result.max_arm_radius, 0.08, 1e-12);
  EXPECT_EQ(result.armSphereCount(), 4U);

  // The non-sphere collision of joint_b's subtree is skipped, not silently
  // forgotten.
  ASSERT_FALSE(result.skipped_geometries.empty());
  EXPECT_EQ(result.skipped_geometries[0], "link_b/collision_1");

  std::remove(path.c_str());
}

#ifdef WBMM_TEST_URDF
TEST(CollisionSphereModel, ResolvesTheArmMountFixedChain)
{
  const auto loaded = wbmm::robot_model::RobotDescriptionLoader::fromFile(
    WBMM_TEST_URDF);
  ASSERT_TRUE(loaded.success) << loaded.message;
  auto config = wbmm::robot_model::RobotModelConfig::defaultsFor(loaded.description);
  config.state_base_frame = "base_footprint";
  config.base_collision_link = "base_link";

  const auto result = wbmm::robot_model::buildCollisionSphereModel(
    loaded.description, config);
  ASSERT_TRUE(result.success) << result.message;

  EXPECT_EQ(result.root_link, "base_footprint");
  EXPECT_EQ(result.base_collision_link, "base_link");

  // base_footprint -> base_link is (0, 0, 0.147) with no rotation.
  EXPECT_TRUE(result.root_from_base_collision.translation().isApprox(
    Eigen::Vector3d(0.0, 0.0, 0.147), 1e-9));

  // base_link -> Link_0 is base_to_jaka (0, 0, 0.221, yaw -1.57) composed with
  // the identity jaka_base_link -> Link_0.
  EXPECT_TRUE(result.base_from_arm_mount.translation().isApprox(
    Eigen::Vector3d(0.0, 0.0, 0.221), 1e-9));
  const Eigen::Matrix3d expected_rotation =
    Eigen::AngleAxisd(-1.57, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE(result.base_from_arm_mount.linear().isApprox(expected_rotation, 1e-4));
}

// Regression lock against the real robot. The expected split (15 arm spheres in
// groups 2/2/5/1/1/4, 6 base spheres, max arm radius 0.085) matches what the
// running REMANI planner reported.
TEST(CollisionSphereModel, MatchesTheDeployedRobotSphereSplit)
{
  const auto loaded = wbmm::robot_model::RobotDescriptionLoader::fromFile(
    WBMM_TEST_URDF);
  ASSERT_TRUE(loaded.success) << loaded.message;
  auto config = wbmm::robot_model::RobotModelConfig::defaultsFor(loaded.description);
  config.state_base_frame = "base_footprint";
  config.base_collision_link = "base_link";

  const auto result = wbmm::robot_model::buildCollisionSphereModel(
    loaded.description, config);
  ASSERT_TRUE(result.success) << result.message;

  ASSERT_EQ(result.arm.size(), 6U);
  const std::vector<std::size_t> expected = {2U, 2U, 5U, 1U, 1U, 4U};
  for (std::size_t i = 0U; i < expected.size(); ++i) {
    EXPECT_EQ(result.arm[i].spheres.size(), expected[i]) << "joint " << i + 1;
  }
  EXPECT_EQ(result.armSphereCount(), 15U);
  EXPECT_EQ(result.baseSphereCount(), 6U);
  EXPECT_NEAR(result.max_arm_radius, 0.085, 1e-9);

  const std::vector<std::string> child_links = {
    "Link_1", "Link_2", "Link_3", "Link_4", "Link_5", "Link_6"};
  for (std::size_t i = 0U; i < child_links.size(); ++i) {
    EXPECT_EQ(result.arm_joint_names[i], "joint_" + std::to_string(i + 1));
    for (const auto & sphere : result.arm[i].spheres) {
      EXPECT_EQ(sphere.owner_link, child_links[i]);
    }
  }
}
#endif

}  // namespace
