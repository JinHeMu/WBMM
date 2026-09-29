#include "wbmm_robot_model/robot_description_loader.hpp"
#include "wbmm_robot_model/robot_model_description.hpp"
#include "wbmm_robot_model/robot_model_config_loader.hpp"

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace
{

constexpr char kSyntheticUrdf[] = R"(<?xml version="1.0"?>
<robot name="synthetic">
  <link name="base_link">
    <collision><origin xyz="0 0 0"/><geometry><sphere radius="0.1"/></geometry></collision>
  </link>
  <link name="link_a"/>
  <link name="link_b"/>
  <link name="tool0"/>
  <joint name="joint_a" type="revolute">
    <parent link="base_link"/><child link="link_a"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="joint_b" type="revolute">
    <parent link="link_a"/><child link="link_b"/>
    <origin xyz="0 0 0.1"/><axis xyz="0 1 0"/>
    <limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
  <joint name="tool_joint" type="fixed">
    <parent link="link_b"/><child link="tool0"/>
    <origin xyz="0 0 0.05"/>
  </joint>
</robot>
)";

const wbmm::robot_model::RobotDescription & syntheticDescription()
{
  static const auto description =
    wbmm::robot_model::parseRobotDescription(kSyntheticUrdf, "synthetic");
  return description;
}

TEST(RobotModelConfig, DefaultsComeFromTheDescription)
{
  const auto config =
    wbmm::robot_model::RobotModelConfig::defaultsFor(syntheticDescription());
  EXPECT_EQ(config.state_base_frame, "base_link");
  const std::vector<std::string> expected = {"joint_a", "joint_b"};
  EXPECT_EQ(config.controlled_joints, expected);
  EXPECT_TRUE(config.locked_joints.empty());
  ASSERT_EQ(config.end_effectors.count("tool"), 1U);
  EXPECT_EQ(config.end_effectors.at("tool"), "tool0");

  const auto validation =
    wbmm::robot_model::validate(config, syntheticDescription());
  EXPECT_TRUE(validation.ok) << validation.message;
}

TEST(RobotModelConfig, RejectsIncompleteOrConflictingConfiguration)
{
  auto config =
    wbmm::robot_model::RobotModelConfig::defaultsFor(syntheticDescription());

  config.controlled_joints = {"joint_a"};
  auto validation = wbmm::robot_model::validate(config, syntheticDescription());
  EXPECT_FALSE(validation.ok);
  EXPECT_NE(validation.message.find("neither controlled nor locked"),
            std::string::npos);

  config.locked_joints["joint_b"] = 0.0;
  validation = wbmm::robot_model::validate(config, syntheticDescription());
  EXPECT_TRUE(validation.ok) << validation.message;

  config.locked_joints["joint_a"] = 0.0;
  validation = wbmm::robot_model::validate(config, syntheticDescription());
  EXPECT_FALSE(validation.ok);
  EXPECT_NE(validation.message.find("both controlled and locked"),
            std::string::npos);

  auto missing = wbmm::robot_model::RobotModelConfig::defaultsFor(syntheticDescription());
  missing.controlled_joints = {"joint_a", "joint_missing"};
  validation = wbmm::robot_model::validate(missing, syntheticDescription());
  EXPECT_FALSE(validation.ok);
  EXPECT_NE(validation.message.find("does not exist"), std::string::npos);

  auto bad_frame = wbmm::robot_model::RobotModelConfig::defaultsFor(syntheticDescription());
  bad_frame.state_base_frame = "no_such_link";
  validation = wbmm::robot_model::validate(bad_frame, syntheticDescription());
  EXPECT_FALSE(validation.ok);
}

TEST(RobotModelConfig, YamlOverridesDefaultsAndPreservesJointOrder)
{
  constexpr char kYaml[] = R"(
state_base_frame: base_link
controlled_joints: [joint_b, joint_a]
locked_joints: {}
end_effectors: {tool: tool0}
base_collision_link: base_link
collision_groups: {arm: [link_a, link_b]}
)";
  const auto result = wbmm::robot_model::RobotModelConfigLoader::fromYaml(
    kYaml, syntheticDescription(), "inline");
  ASSERT_TRUE(result.success) << result.message;
  const std::vector<std::string> expected = {"joint_b", "joint_a"};
  EXPECT_EQ(result.config.controlled_joints, expected);
  EXPECT_EQ(result.config.base_collision_link, "base_link");
  ASSERT_EQ(result.config.collision_groups.count("arm"), 1U);
  EXPECT_EQ(result.config.collision_groups.at("arm").size(), 2U);

  const auto other = wbmm::robot_model::RobotModelConfigLoader::fromYaml(
    "controlled_joints: [joint_a, joint_b]", syntheticDescription(), "inline");
  ASSERT_TRUE(other.success) << other.message;
  EXPECT_NE(wbmm::robot_model::contentId(result.config),
            wbmm::robot_model::contentId(other.config));
}

TEST(RobotModelConfig, RejectsUnknownConfigField)
{
  const auto result = wbmm::robot_model::RobotModelConfigLoader::fromYaml(
    "sphere_approximation: {mode: bogus}", syntheticDescription(), "inline");
  EXPECT_FALSE(result.success);
  EXPECT_NE(result.message.find("sphere_approximation"), std::string::npos);
}

TEST(RobotModelConfig, ContentIdIsStableForIdenticalInput)
{
  const auto a =
    wbmm::robot_model::RobotModelConfig::defaultsFor(syntheticDescription());
  const auto b =
    wbmm::robot_model::RobotModelConfig::defaultsFor(syntheticDescription());
  EXPECT_EQ(wbmm::robot_model::contentId(a), wbmm::robot_model::contentId(b));
}

#ifdef WBMM_TEST_CONFIG
// The canonical robot config must stay loadable and consistent with the URDF.
TEST(RobotModelConfig, LoadsTheCanonicalRobotConfig)
{
  const auto description =
    wbmm::robot_model::loadRobotDescription(WBMM_TEST_URDF);
  const auto result = wbmm::robot_model::RobotModelConfigLoader::fromFile(
    WBMM_TEST_CONFIG, description);
  ASSERT_TRUE(result.success) << result.message;

  EXPECT_EQ(result.config.state_base_frame, "base_footprint");
  EXPECT_EQ(result.config.base_collision_link, "base_link");
  const std::vector<std::string> expected = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
  EXPECT_EQ(result.config.controlled_joints, expected);
  ASSERT_EQ(result.config.end_effectors.count("tool"), 1U);
  EXPECT_EQ(result.config.end_effectors.at("tool"), "tool0");

  // The canonical config must also produce the deployed sphere split.
  const auto built =
    wbmm::robot_model::buildRobotModelDescription(description, result.config);
  ASSERT_TRUE(built.success) << built.message;
  EXPECT_EQ(built.model.collision_spheres.baseSphereCount(), 6U);
  EXPECT_EQ(built.model.collision_spheres.armSphereCount(), 15U);
}
#endif

}  // namespace
