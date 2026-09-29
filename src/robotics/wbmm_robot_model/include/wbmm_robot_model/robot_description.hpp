#pragma once

// ============================================================================
// RobotDescription 保存 urdfdom 解析出的只读 link、joint 和几何信息。
//
// 从文件读取时保留原始 XML，供 Pinocchio 建立运动学模型。
// urdfdom 和 Pinocchio 各自解析同一份 XML；此类型只负责描述，不做 FK。
// 长度单位 m，角度 rad，位姿使用 Eigen::Isometry3d。
// ============================================================================

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace wbmm::robot_model
{

// URDF 关节类型。kUnknown 表示 urdfdom 报出的未知类型，加载时不应静默忽略。
enum class JointType
{
  kRevolute = 0,
  kContinuous,
  kPrismatic,
  kFixed,
  kFloating,
  kPlanar,
  kUnknown,
};

[[nodiscard]] const char * toString(JointType type) noexcept;

enum class GeometryType
{
  kSphere = 0,
  kBox,
  kCylinder,
  kMesh,
  kUnknown,
};

[[nodiscard]] const char * toString(GeometryType type) noexcept;

struct InertialDescription
{
  Eigen::Isometry3d origin{Eigen::Isometry3d::Identity()};
  double mass{0.0};
  Eigen::Matrix3d inertia{Eigen::Matrix3d::Zero()};
};

// 几何尺寸只填写与 type 对应的字段：
//   kSphere   -> radius
//   kBox      -> size
//   kCylinder -> radius, length
//   kMesh     -> mesh_filename, mesh_scale
struct GeometryDescription
{
  GeometryType type{GeometryType::kUnknown};
  double radius{0.0};
  double length{0.0};
  Eigen::Vector3d size{Eigen::Vector3d::Zero()};
  std::string mesh_filename;
  Eigen::Vector3d mesh_scale{Eigen::Vector3d::Ones()};
};

struct VisualDescription
{
  std::string name;
  Eigen::Isometry3d origin{Eigen::Isometry3d::Identity()};
  GeometryDescription geometry;
  std::string material_name;
  bool has_color{false};
  Eigen::Vector4d color{Eigen::Vector4d::Ones()};
};

struct CollisionDescription
{
  std::string name;
  Eigen::Isometry3d origin{Eigen::Isometry3d::Identity()};
  GeometryDescription geometry;
  // 该 collision 在所属 link 的 collision 数组中的下标。
  // URDF 允许 collision 没有 name，稳定的几何 id 由 link + index 组成。
  std::size_t index{0};
};

struct JointDescription
{
  std::string name;
  JointType type{JointType::kUnknown};
  std::string parent_link;
  std::string child_link;
  Eigen::Isometry3d origin{Eigen::Isometry3d::Identity()};
  Eigen::Vector3d axis{Eigen::Vector3d::UnitZ()};
  bool has_position_limit{false};
  double lower{0.0};
  double upper{0.0};
  bool has_effort_limit{false};
  double effort{0.0};
  bool has_velocity_limit{false};
  double velocity{0.0};

  // revolute/continuous/prismatic 才是 1-DoF 标量关节。
  [[nodiscard]] bool isScalar() const noexcept;
  [[nodiscard]] bool isFixed() const noexcept;
};

struct LinkDescription
{
  std::string name;
  bool has_inertial{false};
  InertialDescription inertial;
  std::vector<VisualDescription> visuals;
  std::vector<CollisionDescription> collisions;
};

// 只读 URDF 描述。加载完成后不再修改；同进程内可安全共享
// （std::shared_ptr<const RobotDescription>）。
struct RobotDescription
{
  std::string name;
  std::string source;   // 文件路径，或 "<xml>"
  std::string root_link;
  // 展开后的 URDF 文本。Pinocchio 用同一份文本构建模型，避免二次读文件
  // 造成两个消费者看到不同内容。
  std::string xml;

  std::vector<LinkDescription> links;     // 按 link 名称排序，顺序稳定
  std::vector<JointDescription> joints;   // 按 joint 名称排序，顺序稳定

  std::map<std::string, std::size_t> link_index;
  std::map<std::string, std::size_t> joint_index;
  std::map<std::string, std::size_t> joint_by_child_link;

  [[nodiscard]] const LinkDescription * findLink(const std::string & name) const;
  [[nodiscard]] const JointDescription * findJoint(const std::string & name) const;
  // 返回以 link 为 child 的关节（每个 link 至多一个父关节）。
  [[nodiscard]] const JointDescription * findJointByChildLink(
    const std::string & link_name) const;

  // 全部 1-DoF（revolute/continuous/prismatic）关节名，按名称排序。
  [[nodiscard]] std::vector<std::string> scalarJointNames() const;

  [[nodiscard]] bool empty() const noexcept {return links.empty();}

  // 内容标识：展开后的 URDF 文本的稳定哈希。几何变化必然改变该值。
  // 注意：它只覆盖 URDF 本身，语义配置和球近似参数由
  // RobotModelDescription::contentId 一起纳入。
  [[nodiscard]] std::uint64_t contentId() const noexcept {return content_id;}

  std::uint64_t content_id{0};
};

// 64 位 FNV-1a，仅用于内容标识，不用于安全用途。
[[nodiscard]] std::uint64_t fnv1a64(const std::string & text) noexcept;

// 从固定关节链求 from -> to 的位姿；只允许经过 fixed 关节。
// 返回 false 表示两个 link 不由 fixed 关节链连通（或不存在）。
[[nodiscard]] bool fixedTransform(
  const RobotDescription & description, const std::string & from,
  const std::string & to, Eigen::Isometry3d & transform);

// 收集 link 及其 fixed 子树上的全部 collision；位姿重新表达在 link 系。
// 用于碰撞球分组（REMANI 兼容规则）。
struct LinkCollision
{
  std::string source_link;              // collision 原始所属 link
  std::string source_geometry;          // "<link>/<collision name|index>"
  Eigen::Isometry3d transform_in_link{Eigen::Isometry3d::Identity()};
  GeometryDescription geometry;
};

void collectFixedSubtreeCollisions(
  const RobotDescription & description, const std::string & link,
  std::vector<LinkCollision> & out);

}  // namespace wbmm::robot_model
