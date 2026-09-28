#include "wbmm_collision/urdf_collision_model.hpp"

#include <Eigen/Geometry>

#include <gtest/gtest.h>

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
  const std::string path = "wbmm_collision_synthetic.urdf";
  std::ofstream stream(path);
  stream << kSyntheticUrdf;
  stream.close();
  return path;
}

TEST(UrdfCollisionModel, RejectsUnknownJointAndWrongType)
{
  const std::string path = writeSyntheticUrdf();

  const auto missing = wbmm::collision::loadUrdfCollisionModel(
    path, {"joint_a", "joint_missing"}, "base_link");
  EXPECT_FALSE(missing.success);
  EXPECT_NE(missing.message.find("does not exist"), std::string::npos);

  const auto fixed = wbmm::collision::loadUrdfCollisionModel(
    path, {"tool_joint"}, "base_link");
  EXPECT_FALSE(fixed.success);
  EXPECT_NE(fixed.message.find("revolute or continuous"), std::string::npos);

  std::remove(path.c_str());
}

TEST(UrdfCollisionModel, RejectsMissingBaseCollisionLink)
{
  const std::string path = writeSyntheticUrdf();
  const auto result = wbmm::collision::loadUrdfCollisionModel(
    path, {"joint_a"}, "no_such_link");
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("Base collision link"), std::string::npos);
  std::remove(path.c_str());
}

TEST(UrdfCollisionModel, GroupsFixedSubtreeUnderItsMovableJoint)
{
  const std::string path = writeSyntheticUrdf();
  const auto result = wbmm::collision::loadUrdfCollisionModel(
    path, {"joint_a", "joint_b"}, "base_link");
  ASSERT_TRUE(result.success) << result.message;

  ASSERT_EQ(result.joints.size(), 2U);
  ASSERT_EQ(result.arm.size(), 2U);

  // joint_a owns link_a (1 sphere). The shoulder_mount sphere sits exactly on
  // joint_a's axis (x = y = 0) and is folded in; the other shoulder sphere is
  // 0.25 m off the axis and belongs to no joint group, so it is dropped.
  EXPECT_EQ(result.arm[0].spheres.size(), 2U);
  // joint_b owns link_b plus its fixed subtree (tool_fixed): 2 spheres.
  EXPECT_EQ(result.arm[1].spheres.size(), 2U);

  // Base group is exactly the declared base collision link's spheres.
  EXPECT_EQ(result.baseSphereCount(), 2U);

  for (const auto & sphere : result.base.spheres) {
    EXPECT_EQ(sphere.link_name, "base_link");
    EXPECT_EQ(sphere.group, wbmm::collision::CollisionGroup::kBase);
  }
  for (const auto & sphere : result.arm[0].spheres) {
    EXPECT_EQ(sphere.link_name, "link_a");
    EXPECT_EQ(sphere.group, wbmm::collision::CollisionGroup::kArm);
  }
  for (const auto & sphere : result.arm[1].spheres) {
    EXPECT_EQ(sphere.link_name, "link_b");
  }

  // The folded shoulder sphere must land at the joint origin, i.e. at zero in
  // link_a's frame.
  bool found_folded = false;
  for (const auto & sphere : result.arm[0].spheres) {
    if (std::abs(sphere.radius - 0.08) < 1e-12) {
      found_folded = true;
      EXPECT_LT(sphere.center.norm(), 1e-12);
    }
  }
  EXPECT_TRUE(found_folded);

  // The fixed-subtree sphere of joint_b is re-expressed in link_b's frame:
  // tool_joint sits at (0, 0.1, 0) in link_b and its sphere is at (0, 0, 0.10)
  // in tool_fixed, so in link_b it is (0, 0.1, 0.1), i.e. sqrt(0.02) away.
  bool found_subtree = false;
  for (const auto & sphere : result.arm[1].spheres) {
    if (std::abs(sphere.radius - 0.03) < 1e-12) {
      found_subtree = true;
      EXPECT_NEAR(sphere.center.norm(), std::sqrt(0.02), 1e-12);
      EXPECT_NEAR(sphere.center.y(), 0.1, 1e-12);
      EXPECT_NEAR(sphere.center.z(), 0.1, 1e-12);
    }
  }
  EXPECT_TRUE(found_subtree);

  EXPECT_NEAR(result.max_arm_radius, 0.08, 1e-12);
  EXPECT_EQ(result.armSphereCount(), 4U);

  std::remove(path.c_str());
}

TEST(UrdfCollisionModel, ResolvesJointKinematicsForTheFkChain)
{
  const std::string path = writeSyntheticUrdf();
  const auto result = wbmm::collision::loadUrdfCollisionModel(
    path, {"joint_a", "joint_b"}, "base_link");
  ASSERT_TRUE(result.success) << result.message;

  const auto & a = result.joints[0];
  EXPECT_EQ(a.name, "joint_a");
  EXPECT_EQ(a.parent_link, "shoulder_mount");
  EXPECT_EQ(a.child_link, "link_a");
  EXPECT_NEAR(a.origin_translation.z(), 0.30, 1e-12);
  EXPECT_TRUE(a.axis.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));
  EXPECT_DOUBLE_EQ(a.lower, -1.0);
  EXPECT_DOUBLE_EQ(a.upper, 1.0);

  // joint_b has no <limit>; a continuous joint must stay usable.
  const auto & b = result.joints[1];
  EXPECT_EQ(b.name, "joint_b");
  EXPECT_TRUE(b.axis.isApprox(Eigen::Vector3d::UnitZ(), 1e-12));

  std::remove(path.c_str());
}

#ifdef WBMM_TEST_URDF
// The arm mount offset is what makes the arm spheres land in the right place.
// Dropping it would put the whole arm at the base origin.
TEST(UrdfCollisionModel, ResolvesTheArmMountFixedChain)
{
  const auto result = wbmm::collision::loadUrdfCollisionModel(
    WBMM_TEST_URDF,
    {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"},
    "base_link");
  ASSERT_TRUE(result.success) << result.message;

  EXPECT_EQ(result.root_link, "base_footprint");
  EXPECT_EQ(result.base_collision_link, "base_link");

  // base_footprint -> base_link is (0, 0, 0.147) with no rotation.
  const Eigen::Vector3d root_offset =
    result.root_from_base_collision.block<3, 1>(0, 3);
  EXPECT_TRUE(root_offset.isApprox((Eigen::Vector3d(0.0, 0.0, 0.147)), 1e-9));

  // base_link -> Link_0 is base_to_jaka (0, 0, 0.221, yaw -1.57) composed with
  // the identity jaka_base_link -> Link_0.
  const Eigen::Matrix4d & mount = result.base_from_arm_mount;
  const Eigen::Vector3d mount_offset = mount.block<3, 1>(0, 3);
  EXPECT_TRUE(mount_offset.isApprox((Eigen::Vector3d(0.0, 0.0, 0.221)), 1e-9));

  const Eigen::Matrix3d expected_rotation =
    Eigen::AngleAxisd(-1.57, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  EXPECT_TRUE((mount.block<3, 3>(0, 0).isApprox(expected_rotation, 1e-4)));

  // The joints must form a serial chain, which is what lets the consumer reach
  // group i by composing the mount with T_0 .. T_i.
  for (std::size_t i = 0U; i + 1U < result.joints.size(); ++i) {
    EXPECT_EQ(result.joints[i + 1].parent_link, result.joints[i].child_link)
      << "joint " << i + 2;
  }

  // Composing the mount with every joint placement must reproduce the URDF's
  // chain from base_link to Link_6.
  Eigen::Matrix4d composed = mount;
  for (const auto & joint : result.joints) {
    Eigen::Matrix4d placement = Eigen::Matrix4d::Identity();
    placement.block<3, 3>(0, 0) = joint.origin_rotation;
    placement.block<3, 1>(0, 3) = joint.origin_translation;
    composed = composed * placement;
  }
  // Link_6 ends up at z = 0.2276 for this URDF. The point of the check is the
  // mount: dropping base_from_arm_mount removes its 0.221 m and leaves ~0.007,
  // so a 0.2 threshold separates the two cases cleanly.
  EXPECT_NEAR((composed.block<3, 1>(0, 3).z()), 0.2276, 1e-3);
  EXPECT_GT((composed.block<3, 1>(0, 3).z()), 0.2);
}

// Regression lock against the real robot. The expected split (15 arm spheres in
// groups 2/2/5/1/1/4, 6 base spheres, max arm radius 0.085) matches what the
// running REMANI planner reported: "15 manipulator collision spheres and 6 base
// collision spheres; maximum arm radius=0.085 m".
TEST(UrdfCollisionModel, MatchesTheDeployedRobotSphereSplit)
{
  const auto result = wbmm::collision::loadUrdfCollisionModel(
    WBMM_TEST_URDF,
    {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"},
    "base_link");
  ASSERT_TRUE(result.success) << result.message;

  ASSERT_EQ(result.joints.size(), 6U);
  ASSERT_EQ(result.arm.size(), 6U);

  const std::vector<std::size_t> expected = {2U, 2U, 5U, 1U, 1U, 4U};
  for (std::size_t i = 0U; i < expected.size(); ++i) {
    EXPECT_EQ(result.arm[i].spheres.size(), expected[i]) << "joint " << i + 1;
  }
  EXPECT_EQ(result.armSphereCount(), 15U);
  EXPECT_EQ(result.baseSphereCount(), 6U);
  EXPECT_NEAR(result.max_arm_radius, 0.085, 1e-9);

  // Joint order and child links must line up with the WBMM joint contract.
  const std::vector<std::string> child_links = {
    "Link_1", "Link_2", "Link_3", "Link_4", "Link_5", "Link_6"};
  for (std::size_t i = 0U; i < child_links.size(); ++i) {
    EXPECT_EQ(result.joints[i].name, "joint_" + std::to_string(i + 1));
    EXPECT_EQ(result.joints[i].child_link, child_links[i]);
  }

  // Every arm sphere must be expressed in its own group's child link.
  for (std::size_t i = 0U; i < result.arm.size(); ++i) {
    for (const auto & sphere : result.arm[i].spheres) {
      EXPECT_EQ(sphere.link_name, child_links[i]);
    }
  }
}
#endif

}  // namespace
