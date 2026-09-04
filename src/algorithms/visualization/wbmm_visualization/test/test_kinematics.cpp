#include "wbmm_visualization/whole_body_kinematics.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <string>

namespace wbmm_viz
{
namespace
{

std::string urdfFile()
{
  const std::string path = std::string(WBMM_VIZ_WORKSPACE_DIR) +
    "/src/robot/tracer_jaka_description/urdf/tracer_jaka_zu5.urdf";
  EXPECT_TRUE(std::filesystem::exists(path)) << path;
  return path;
}

TEST(WholeBodyKinematics, VisualGeometryFromUrdf)
{
  const WholeBodyKinematics kinematics(urdfFile());
  const Eigen::VectorXd state = Eigen::VectorXd::Zero(9);
  const auto geometry = kinematics.visualGeometry(state);
  // tracer_jaka_zu5.urdf: 29 <visual> entries.
  ASSERT_EQ(geometry.size(), 29U);
  for (const auto & visual : geometry) {
    EXPECT_FALSE(visual.name.empty());
    EXPECT_EQ(visual.mesh_path.substr(0, 7), "file://");
    const std::string path = visual.mesh_path.substr(7);
    EXPECT_TRUE(std::filesystem::exists(path)) << path;
    EXPECT_GT(visual.mesh_scale.x(), 0.0);
    EXPECT_GT(visual.mesh_scale.y(), 0.0);
    EXPECT_GT(visual.mesh_scale.z(), 0.0);
    EXPECT_TRUE(visual.position.allFinite());
    EXPECT_TRUE(std::isfinite(visual.orientation.x()));
    EXPECT_TRUE(std::isfinite(visual.orientation.y()));
    EXPECT_TRUE(std::isfinite(visual.orientation.z()));
    EXPECT_TRUE(std::isfinite(visual.orientation.w()));
    EXPECT_GE(visual.orientation.norm(), 1.0 - 1.0e-6);
    EXPECT_LE(visual.orientation.norm(), 1.0 + 1.0e-6);
    for (int channel = 0; channel < 4; ++channel) {
      EXPECT_GE(visual.color[channel], 0.0);
      EXPECT_LE(visual.color[channel], 1.0);
    }
  }
}

TEST(WholeBodyKinematics, FramePositionSemantics)
{
  const WholeBodyKinematics kinematics(urdfFile());
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(9);
  const Eigen::Vector3d at_zero = kinematics.eePosition(zero);
  EXPECT_TRUE(at_zero.allFinite());
  // Reproducible.
  EXPECT_TRUE(kinematics.eePosition(zero).isApprox(at_zero, 1.0e-12));

  // Moving the base by +1 X with yaw 0 shifts the EE by +1 X.
  Eigen::VectorXd shifted = zero;
  shifted[0] = 1.0;
  EXPECT_TRUE(kinematics.eePosition(shifted).isApprox(at_zero + Eigen::Vector3d::UnitX(), 1.0e-9));

  // eePosition equals framePosition("tool0") and the rotation is orthonormal.
  Eigen::VectorXd yawed = zero;
  yawed[2] = 0.7;
  EXPECT_TRUE(kinematics.eePosition(yawed).isApprox(
    kinematics.framePosition(yawed, "tool0"), 1.0e-12));
  const Eigen::Matrix3d rotation = kinematics.eeRotation(yawed);
  EXPECT_TRUE((rotation * rotation.transpose()).isIdentity(1.0e-9));
  EXPECT_NEAR(rotation.determinant(), 1.0, 1.0e-9);
}

TEST(WholeBodyKinematics, CollisionSpheres)
{
  const WholeBodyKinematics kinematics(urdfFile());
  const Eigen::VectorXd state = Eigen::VectorXd::Zero(9);
  const auto spheres = kinematics.collisionSpheres(state);
  // Parity with the TA-WBMP planner test (21 collision spheres).
  ASSERT_EQ(spheres.size(), 21U);
  for (const auto & sphere : spheres) {
    EXPECT_FALSE(sphere.name.empty());
    EXPECT_TRUE(sphere.center.allFinite());
    EXPECT_GT(sphere.radius, 0.0);
  }
}

}  // namespace
}  // namespace wbmm_viz
